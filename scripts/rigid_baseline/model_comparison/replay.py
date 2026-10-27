"""Common-covariance causal momentum KF; reference wrench is not an input."""
import argparse,time
from pathlib import Path
import numpy as np
from models import read,write,require,load,feedback,close

def terms(path):
 with np.load(path) as f:return {k:f[k].copy() for k in f.files}

def filter_run(cfg,f,m,ref,n=None):
 kernel=load(cfg['kf_source'],'kernel');o=cfg['observer'];Qp=np.diag(o['Qp_continuous_diag']);Qw=np.diag(o['Qw_continuous_diag']);Pw=np.diag(o['initial_wrench_variance'])
 previous=None;out={k:[] for k in ['wrench','joint_external_torque','momentum','nominal_input','innovation','covariance']}
 for i in range(len(f['t']) if n is None else n):
  v=f['v'][i];p=m['M'][i]@v;J=ref['J'][i];R=ref['R'][i]
  u=m['Mdot'][i]@v+f['tau'][i]-m['b'][i]*v-m['c'][i]-ref['G'][i]
  if previous is None:
   x=np.r_[p,np.zeros(6)];P=np.zeros((12,12));P[:6,:6]=R;P[6:,6:]=Pw;innovation=np.zeros(6)
  else:
   t0,u0,J0=previous;dt=f['t'][i]-t0;require(dt>0,'Nonmonotonic time');x,P,innovation=kernel.kf_step(x,P,p,u0,J0,dt,Qp,Qw,R)
  values=dict(wrench=x[6:].copy(),joint_external_torque=J.T@x[6:],momentum=p,nominal_input=u,innovation=innovation,covariance=P.copy())
  require(all(np.isfinite(v).all() for v in values.values()),'Nonfinite observer')
  require(np.linalg.eigvalsh(P).min()>0,'Nonpositive covariance')
  for k,val in values.items():out[k].append(val)
  previous=(f['t'][i],u,J)
 return {k:np.asarray(v) for k,v in out.items()}

def run(c,root,out):
 cfg=read(c['kf_config']);checks=[]
 # All oracle replays must pass before comparing any replacement model.
 for case in cfg['cases']:
  f=feedback(c,case);ref=terms(root/'rigid_terms'/case/'reference.npz');require(np.array_equal(f['t'],ref['t']),'Time mismatch')
  r=filter_run(cfg,f,ref,ref)
  with np.load(Path(c['logs'])/case/'estimates.npz') as old:comparison=close(r['wrench'],old['wrench'])
  row=dict(case=case,model='oracle',reference_replay=comparison);checks.append(row)
  write(out/(case+'_oracle_check.json'),row);require(comparison['passed'],'Oracle replay differs from archived result')
  dest=out/case;dest.mkdir();np.savez_compressed(dest/'oracle.npz',t=f['t'],**r)
 for case in cfg['cases']:
  f=feedback(c,case);ref=terms(root/'rigid_terms'/case/'reference.npz');oracle=terms(out/case/'oracle.npz')
  for name,path in [('oracle',None),('rigid',root/'rigid_terms'/case/'model_terms.npz'),('pinn',root/'pinn_terms'/case/'model_terms.npz')]:
   m=ref if path is None else terms(path);require(np.array_equal(m['t'],f['t']),'Model time mismatch')
   r=oracle if name=='oracle' else filter_run(cfg,f,m,ref)
   prefix=filter_run(cfg,{k:v[:101] for k,v in f.items()}, {k:v[:101] for k,v in m.items()}, {k:v[:101] for k,v in ref.items()})
   err=max(float(np.max(np.abs(r[k][:101]-prefix[k]))) for k in prefix)
   cov_err=float(np.max(np.abs(r['covariance']-oracle['covariance'])))
   row=dict(case=case,model=name,prefix_max_abs=err,shared_covariance_max_abs=cov_err,minimum_covariance_eigenvalue=float(np.linalg.eigvalsh(r['covariance']).min()),symmetry_max_abs=float(np.max(np.abs(r['covariance']-r['covariance'].transpose(0,2,1)))))
   checks.append(row);require(err==0 and cov_err==0,'Causality or common covariance failed')
   if name!='oracle':np.savez_compressed(out/case/(name+'.npz'),t=f['t'],**r)
  print('REPLAYED',case,flush=True)
 write(out/'replay_checks.json',dict(passed=True,checks=checks,shared_inputs=['feedback','G','J','R','Qp','Qw','P0'],reference_wrench_read=False))
 return dict(passed=True,test_read=False,simulation_steps=0)

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--run-root',required=True);p.add_argument('--output-dir',required=True);a=p.parse_args();out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=True);tick=time.monotonic()
 try:r=run(read(a.config),Path(a.run_root),out);r['wall_seconds']=time.monotonic()-tick;write(out/'metrics.json',r)
 except Exception as e:write(out/'metrics.json',dict(passed=False,error=repr(e),wall_seconds=time.monotonic()-tick));raise
