"""Physical constrained interval regression; experiment identity is configuration only."""
import argparse,sys,json,time
from pathlib import Path
import numpy as np
from scipy.linalg import qr
import cvxpy as cp
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'identification'))
from rigid import read,write,require
from experiment import physical,csvwrite

def pseudo(t):
    I=cp.bmat([[t[4],t[5],t[7]],[t[5],t[6],t[8]],[t[7],t[8],t[9]]])
    return cp.bmat([[cp.trace(I)*np.eye(3)/2-I,cp.reshape(t[1:4],(3,1),order='C')], [cp.reshape(t[1:4],(1,3),order='C'),cp.reshape(t[0],(1,1),order='C')]])

def design(c):
    scale=np.array(read(c['normalization'])['tau_scale'])
    xx=[];oo=[]
    for path in c['train_designs']:
        with np.load(path) as f:xx.append(f['X']);oo.append(f['offset'])
    X=np.concatenate(xx);o=np.concatenate(oo)
    w=np.tile(1/scale,len(o)//6)/np.sqrt(len(o))
    return X*w[:,None],-o*w

def precheck(c,out):
    require(read(Path(c['source'])/'build/metrics.json')['passed'],'Original positive control must pass')
    # A deterministic non-truth rigid body tests the parameter-to-pseudo-inertia mapping.
    m=2.;com=np.array([.1,-.2,.3]);Ic=np.diag([.2,.3,.4]);Io=Ic+m*((com@com)*np.eye(3)-np.outer(com,com))
    t=np.array([m,*(m*com),Io[0,0],Io[0,1],Io[1,1],Io[0,2],Io[1,2],Io[2,2]])
    J=pseudo(cp.Constant(t)).value
    expected=np.block([[np.trace(Io)*np.eye(3)/2-Io,(m*com)[:,None]],[(m*com)[None,:],np.array([[m]])]])
    np.testing.assert_allclose(J,expected,rtol=0,atol=1e-14)
    ph=physical(SimpleNamespace(njoints=7),np.r_[np.tile(t,6),np.ones(6)])
    require(ph['all_bodies_feasible'],'Synthetic physical mapping failed')
    A,y=design(c);norms=np.linalg.norm(A,axis=0);scales=np.where(norms>0,norms,1.)
    # Full augmented QR retains even exact-zero columns and the residual constant.
    Q,R=qr(np.column_stack([A/scales,y]),mode='economic');del Q
    rng=np.random.default_rng(0);probes=[np.zeros(66),np.ones(66),rng.normal(size=66),rng.normal(size=66)*.01]
    rows=[]
    for i,t in enumerate(probes):
        z=t*scales;raw=float(np.linalg.norm(A@t-y)**2);compact=float(np.linalg.norm(R[:,:66]@z-R[:,66])**2)
        rows.append(dict(probe=i,raw_loss=raw,compressed_loss=compact,absolute_error=abs(raw-compact),tolerance=1e-10*max(1,raw)))
    passed=all(x['absolute_error']<=x['tolerance'] for x in rows)
    write(out/'objective_equivalence.json',dict(passed=passed,probes=rows,zero_columns=np.flatnonzero(norms==0),compression='full augmented thin QR; no truncation'))
    require(passed,'QR objective mismatch')
    np.savez_compressed(out/'compressed.npz',R=R,scales=scales,norms=norms)
    return dict(passed=True,validation_used=False,test_read=False)

def solve(c,out,pre):
    require(read(pre/'metrics.json')['passed'],'Precheck required')
    f=np.load(pre/'compressed.npz');R=f['R'];scales=f['scales']
    z=cp.Variable(66);theta=cp.multiply(1/scales,z)
    constraints=[pseudo(theta[j*10:j*10+10]) >> c['margin']*np.eye(4) for j in range(6)]+[theta[60:]>=0]
    problem=cp.Problem(cp.Minimize(cp.sum_squares(R[:,:66]@z-R[:,66])),constraints)
    settings=dict(solver='CLARABEL',max_iter=200,tol_gap_abs=1e-9,tol_gap_rel=1e-9,tol_feas=1e-9,warm_start=False,verbose=True)
    write(out/'solver_config.json',dict(settings=settings,margin=c['margin'],mass_unit_kg=1,length_unit_m=1,regularizer=None,validation_used=False))
    value=problem.solve(**settings)
    status=dict(status=problem.status,objective=value,iterations=problem.solver_stats.num_iters,solve_time=problem.solver_stats.solve_time)
    write(out/'solver_return.json',status)
    require(problem.status=='optimal','Non-optimal solver status: '+problem.status)
    t=np.asarray(theta.value);require(np.isfinite(t).all(),'Nonfinite parameters')
    np.save(out/'theta.npy',t)
    A,y=design(c);res=A@t-y;status['original_train_loss']=float(res@res)
    status['compressed_loss_recomputed']=float(np.linalg.norm(R[:,:66]@(t*scales)-R[:,66])**2)
    status['dual_values']=[x.dual_value for x in constraints];write(out/'solver_status.json',status)
    ph=physical(SimpleNamespace(njoints=7),t)
    mins=np.array([min(x['pseudo_inertia_eigenvalues']) for x in ph['bodies']])
    ph['lmi_margin']=c['margin'];ph['lmi_violation']=np.maximum(c['margin']-mins,0)
    ph['friction_violation']=np.maximum(-t[60:],0)
    ph['independent_passed']=bool(ph['all_bodies_feasible'] and np.max(ph['lmi_violation'])<=1e-8 and np.min(t[60:])>=0)
    write(out/'physical_feasibility.json',ph)
    labels=c['labels'];csvwrite(out/'identified_parameters.csv',[dict(index=i,name=labels[i],value=float(t[i]),unit=('N m s / rad' if i>=60 else ('kg' if i%10==0 else 'kg m' if i%10<4 else 'kg m^2')),train_column_norm=float(f['norms'][i])) for i in range(66)])
    require(ph['independent_passed'],'Independent physical feasibility failed')
    return dict(passed=True,original_train_loss=status['original_train_loss'],physical_passed=True,validation_used=False,test_read=False)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--phase',required=True);p.add_argument('--output-dir',required=True);p.add_argument('--run-root',required=True);a=p.parse_args()
    c=read(a.config);out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=True);tick=time.monotonic()
    try:
        result=precheck(c,out) if a.phase=='precheck' else solve(c,out,Path(a.run_root)/'precheck')
        result['wall_seconds']=time.monotonic()-tick;write(out/'metrics.json',result)
    except Exception as e:
        write(out/'metrics.json',dict(passed=False,error=repr(e),wall_seconds=time.monotonic()-tick));raise
