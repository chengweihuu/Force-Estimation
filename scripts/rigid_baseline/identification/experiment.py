import argparse,csv,json,time,filecmp
from pathlib import Path
import numpy as np
import mujoco
import pinocchio as pin
from scipy.linalg import svd
from rigid import read,write,require,make_model,parameters,apply_parameters,regressor,mass,Operator,packet


def check(a,b):
 a,b=np.asarray(a),np.asarray(b)
 den=float(np.linalg.norm(b))
 return dict(passed=bool(np.allclose(a,b,rtol=1e-8,atol=1e-9)),max_abs=float(np.abs(a-b).max()),relative_l2=float(np.linalg.norm(a-b)/den) if den else None)
def csvwrite(p,rows):
 with p.open('x',newline='') as f:
  w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def modelpath(cfg,spec):return Path(cfg['root'])/spec['folder']/'model_snapshot.mjb'
def refpath(cfg,spec):return Path(cfg['reference'])/f"reference_{spec['seed']}.npz"
def small(cfg,out):
 audits=[];names=None;first_model=None
 # Verify the declared physical meaning/order independently of FromDynamicParameters.
 probe_mass=2.;probe_com=np.array([.1,.2,.3]);probe_I=np.array([[.7,.02,.03],[.02,.8,.04],[.03,.04,.9]])
 origin=probe_I+probe_mass*((probe_com@probe_com)*np.eye(3)-np.outer(probe_com,probe_com))
 expected=np.r_[probe_mass,probe_mass*probe_com,origin[0,0],origin[0,1],origin[1,1],origin[0,2],origin[1,2],origin[2,2]]
 ordering=check(pin.Inertia(probe_mass,probe_com,probe_I).toDynamicParameters(),expected)
 write(out/'parameter_order_check.json',ordering);require(ordering['passed'],'Physical parameter order mismatch')
 for binding in cfg['bindings']:
  require(filecmp.cmp(binding['path'],binding['snapshot'],shallow=False),'Archived input changed')
 for spec in cfg['records']:
  model,m,mapping,names=make_model(modelpath(cfg,spec),False);truth,_,_,_=make_model(modelpath(cfg,spec),True)
  if first_model is None:first_model=m
  else:
   for field in ['body_mass','body_inertia','body_ipos','body_iquat','body_parentid','body_pos','body_quat','jnt_pos','jnt_axis','qpos0','dof_damping','dof_armature']:
    require(np.array_equal(getattr(m,field),getattr(first_model,field)),'Models differ: '+field)
  require(names==cfg['joints'],'Joint order differs');require(np.array_equal(m.dof_damping,cfg['old']['known_integrator_B']),'Damping differs')
  pi=parameters(truth);q,v,dv,tau=packet(cfg['root'],spec);td=truth.createData();data=model.createData();md=mujoco.MjData(m);zero=mujoco.MjData(m)
  indices=np.linspace(48000,207999,32).round().astype(int);rows=[]
  altered=model.copy()
  for j in range(1,altered.njoints):altered.inertias[j]=pin.Inertia(2.,np.ones(3)*.03,np.eye(3)*3)
  ad=altered.createData()
  for i in indices:
   md.qpos[:]=q[i];md.qvel[:]=v[i];mujoco.mj_forward(m,md);M=np.empty((6,6));mujoco.mj_fullM(m,M,md.qM)
   zero.qpos[:]=q[i];zero.qvel[:]=0;mujoco.mj_forward(m,zero);c=md.qfrc_bias-zero.qfrc_bias
   Y=regressor(model,data,q[i],v[i],dv[i]/cfg['old']['physics_dt_s'])
   checks=dict(mass=check(mass(truth,td,q[i]),M),velocity=check(pin.rnea(truth,td,q[i],v[i],np.zeros(6)),c),
    momentum=check(regressor(model,data,q[i],np.zeros(6),v[i])@pi,M@v[i]),
    inertia=check(regressor(model,data,q[i],np.zeros(6),dv[i])@pi,M@dv[i]),
    regressor_independence=check(Y,regressor(altered,ad,q[i],v[i],dv[i]/cfg['old']['physics_dt_s'])))
   # All 60 parameter basis columns must agree with direct spatial-inertia assembly.
   if i==indices[0]:
    basis_model=model.copy();bd=basis_model.createData();basis_error=0.;pi0=parameters(model);base=Y@pi0
    for col in range(60):
     unit=np.eye(60)[col];apply_parameters(basis_model,pi0+unit)
     value=pin.rnea(basis_model,bd,q[i],v[i],dv[i]/cfg['old']['physics_dt_s']).copy()
     basis_error=max(basis_error,float(np.abs(value-base-Y[:,col]).max()))
    checks['parameter_basis']=dict(passed=basis_error<=1e-9,max_abs=basis_error)
   rows.append(dict(index=int(i),checks=checks))
   write(out/f"probe_{spec['seed']}_{i}.json",rows[-1])
   require(all(x['passed'] for x in checks.values()),'Cross-platform true-parameter preflight failed')
  audits.append(dict(seed=spec['seed'],states=rows,mapping=mapping))
 fields=['mass','mc_x','mc_y','mc_z','Ixx_origin','Ixy_origin','Iyy_origin','Ixz_origin','Iyz_origin','Izz_origin']
 labels=[n+':'+f for n in names for f in fields]+['b:'+n for n in names]
 write(out/'parameter_schema.json',dict(labels=labels,units=(['kg']+['kg m']*3+['kg m^2']*6)*6+['N m s/rad']*6,per_joint_count=10,gravity_in_regressor=[0,0,0],truth_not_used_for_fit=True))
 write(out/'positive_control.json',audits)
 return dict(passed=True,simulation_steps=0,test_read=False)


def build(cfg,out,pre):
 require(read(pre/'metrics.json')['passed'],'Small positive control required')
 old=cfg['old'];h=old['physics_dt_s'];B=np.array(old['known_integrator_B']);op=Operator(old);checks=[]
 for spec in cfg['records']:
  model,m,_,_=make_model(modelpath(cfg,spec),False);truth,_,_,_=make_model(modelpath(cfg,spec),True);pi=parameters(truth);data=model.createData()
  q,v,dv,tau=packet(cfg['root'],spec);n=len(q)
  with np.load(refpath(cfg,spec)) as f:parts=f['parts'];saved=f['reference_residual']
  Z=np.empty((n,6,60));peak=np.zeros(6);velocity_peak=np.zeros(6);inertia_peak=np.zeros(6)
  for i in range(n):
   Z[i]=h*regressor(model,data,q[i],v[i],dv[i]/h)
   yc=regressor(model,data,q[i],v[i],np.zeros(6))@pi
   velocity_peak=np.maximum(velocity_peak,np.abs(yc-parts[i,2]/h))
   inertia_peak=np.maximum(inertia_peak,np.abs((Z[i]@pi-h*yc-parts[i,0]-parts[i,1])/h))
   if i and i%80000==0:print('BUILD',spec['seed'],i,flush=True)
  offset=parts[:,3]-h*tau+h*B*dv
  truth_impulse=np.einsum('ijk,k->ij',Z,pi)+h*v*B+offset
  peak=np.max(np.abs(truth_impulse/h),axis=0);closure=op(truth_impulse)
  row=dict(seed=spec['seed'],physical_peak_Nm=peak,velocity_peak_Nm=velocity_peak,inertia_equivalent_peak_Nm=inertia_peak,
   interval_peak_Nm=np.max(np.abs(closure),axis=0),reference_operator=check(op(parts.sum(1)-h*tau+h*B*dv),saved))
  write(out/f"positive_{spec['seed']}.json",row)
  require(np.max(peak)<=1e-8 and np.max(velocity_peak)<=1e-8 and np.max(inertia_peak)<=1e-8 and np.max(np.abs(closure))<=1e-8 and row['reference_operator']['passed'],'Full positive control failed')
  X=op(Z).reshape(-1,60);D=np.stack([op(h*v*np.eye(6)[j]).reshape(-1) for j in range(6)],1);y=op(offset).reshape(-1)
  design_closure=(X@pi+D@B+y).reshape(-1,6)
  design_check=dict(seed=spec['seed'],peak_Nm=np.max(np.abs(design_closure),axis=0),linearity=check(design_closure,closure))
  write(out/f"design_closure_{spec['seed']}.json",design_check)
  require(np.max(np.abs(design_closure))<=1e-8 and design_check['linearity']['passed'],'Column-wise interval closure failed')
  del Z
  np.savez_compressed(out/f"design_{spec['seed']}.npz",X=np.column_stack([X,D]),offset=y)
  checks.append(row)
 write(out/'positive_control.json',checks)
 return dict(passed=True,simulation_steps=0,test_read=False)


def spectrum(X,driver):
 s=svd(X,compute_uv=False,lapack_driver=driver);tol=np.finfo(float).eps*max(X.shape)*s[0]
 return dict(singular_values=s,rank=int(np.sum(s>tol)),threshold=tol,shape=list(X.shape))

def solve(cfg,out,pre,builddir):
 require(read(builddir/'metrics.json')['passed'],'Full truth positive control required')
 scale=np.array(read(Path(cfg['root'])/'learn/normalization.json')['tau_scale']);matrices=[];targets=[]
 for spec in cfg['records']:
  if spec['split']!='train':continue
  with np.load(builddir/f"design_{spec['seed']}.npz") as f:matrices.append(f['X']);targets.append(f['offset'])
 X=np.concatenate(matrices);y=np.concatenate(targets);weight=np.tile(1/scale,len(y)//6)/np.sqrt(len(y));A=X*weight[:,None];target=-y*weight
 norms=np.linalg.norm(A,axis=0);nonzero=norms>0;As=A[:,nonzero]/norms[nonzero]
 # Approved fixed-matrix driver comparison: returned decompositions must all pass.
 trials=[];decompositions={}
 for driver in ['gesdd','gesvd']:
  try:
   u,values,vt=svd(As.copy(order='F'),full_matrices=False,lapack_driver=driver)
  except np.linalg.LinAlgError as e:
   trials.append(dict(driver=driver,returned=False,error=repr(e)));continue
  cutoff=np.finfo(float).eps*max(As.shape)*values[0]
  reconstruction=float(np.linalg.norm((u*values)@vt-As)/np.linalg.norm(As))
  left=float(np.linalg.norm(u.T@u-np.eye(len(values)))/np.sqrt(len(values)))
  right=float(np.linalg.norm(vt@vt.T-np.eye(len(values)))/np.sqrt(len(values)))
  valid=bool(max(reconstruction,left,right)<=1e-9 and np.isfinite(values).all())
  trials.append(dict(driver=driver,returned=True,checks_passed=valid,reconstruction_relative=reconstruction,left_orthogonality_normalized=left,right_orthogonality_normalized=right,rank=int(np.sum(values>cutoff)),threshold=cutoff,singular_values=values))
  decompositions[driver]=(u,values,vt)
 write(out/'driver_comparison.json',dict(tolerance=1e-9,trials=trials))
 require(decompositions and all(t.get('checks_passed',True) for t in trials),'No valid decomposition or a returned decomposition failed checks')
 if len(decompositions)==2:
  require(trials[0]['rank']==trials[1]['rank'],'Driver rank conflict')
  k=trials[0]['rank'];difference=float(np.linalg.norm(decompositions['gesdd'][1][:k]-decompositions['gesvd'][1][:k])/np.linalg.norm(decompositions['gesdd'][1][:k]))
  write(out/'retained_spectrum_comparison.json',dict(relative_l2=difference,rank=k))
  require(difference<=1e-9,'Retained spectrum conflict')
 selected='gesdd' if 'gesdd' in decompositions else 'gesvd'
 U,s,Vt=decompositions[selected];tol=np.finfo(float).eps*max(As.shape)*s[0];keep=s>tol
 write(out/'solver_selection.json',dict(driver=selected,rule='gesdd if valid; otherwise gesvd only after predeclared fixed-matrix checks',validation_used=False))
 z=Vt[keep].T@((U[:,keep].T@target)/s[keep]);theta=np.zeros(66);theta[nonzero]=z/norms[nonzero]
 require(np.isfinite(theta).all(),'Nonfinite solution');np.save(out/'theta.npy',theta)
 schema=read(pre/'parameter_schema.json');labels=schema['labels'];normalized_vectors=np.zeros((len(s),66));normalized_vectors[:,nonzero]=Vt
 # Right vectors below threshold and exact zero-column directions span the numerical nullspace.
 np.savez_compressed(out/'svd_vectors.npz',right_vectors_normalized=normalized_vectors,singular_values=s,retained=keep,column_norms=norms,nonzero_columns=nonzero)
 rawdiag=spectrum(X,selected);weighteddiag=spectrum(A,selected)
 residual=A@theta-target
 write(out/'design_diagnostics.json',dict(unweighted=rawdiag,weighted=weighteddiag,normalized=dict(singular_values=s,rank=int(keep.sum()),threshold=tol),
    column_norms=norms,zero_columns=[labels[i] for i in np.flatnonzero(~nonzero)],nullity=66-int(keep.sum()),
    convention='Minimum Euclidean norm in training-column-normalized coordinates; no ridge; threshold eps*max(shape)*smax; validation not opened',
    original_train_loss=float(residual@residual),normal_equation_max=float(np.abs(A.T@residual).max())))
 csvwrite(out/'identified_parameters.csv',[dict(index=i,name=labels[i],value=theta[i],train_column_norm=norms[i]) for i in range(66)])
 csvwrite(out/'singular_values.csv',[dict(index=i,singular_value=v,retained=bool(keep[i]),threshold=tol) for i,v in enumerate(s)])
 rows=[]
 for i in list(np.flatnonzero(keep)[-10:])+list(np.flatnonzero(~keep)):
  for j in np.argsort(np.abs(normalized_vectors[i]))[-10:][::-1]:
   rows.append(dict(direction=int(i),singular_value=s[i],kind='weak_retained' if keep[i] else 'numerical_null',parameter=labels[j],normalized_coefficient=normalized_vectors[i,j],physical_direction_coefficient=normalized_vectors[i,j]/norms[j] if norms[j]>0 else 0))
 if rows:csvwrite(out/'weak_directions.csv',rows)
 return dict(passed=True,rank=int(keep.sum()),parameters=66,test_read=False,validation_used=False)


def physical(model,theta):
 result=[]
 for j in range(1,model.njoints):
  t=theta[(j-1)*10:j*10];m=float(t[0]);first=t[1:4]
  Iorigin=np.array([[t[4],t[5],t[7]],[t[5],t[6],t[8]],[t[7],t[8],t[9]]])
  second=np.eye(3)*np.trace(Iorigin)/2-Iorigin
  pseudo=np.block([[second,first[:,None]],[first[None,:],np.array([[m]])]])
  if m!=0:
   com=first/m;I=Iorigin-m*((com@com)*np.eye(3)-np.outer(com,com));eig=np.linalg.eigvalsh(I)
   triangle=np.linalg.eigvalsh(np.eye(3)*np.trace(I)/2-I)
  else:com=None;eig=None;triangle=None
  result.append(dict(joint=j,mass_kg=m,first_moment_kg_m=first,origin_inertia_kg_m2=Iorigin,com_m=com,inverse_mass_kg_inv=1/m if m!=0 else None,com_inertia_eigenvalues=eig,triangle_psd_eigenvalues=triangle,
    pseudo_inertia_eigenvalues=np.linalg.eigvalsh(pseudo),feasible=bool(m>0 and np.all(eig>0) and triangle.min()>=0),com_defined=bool(m!=0)))
 return dict(b=theta[-6:],friction_nonnegative=bool(np.all(theta[-6:]>=0)),bodies=result,all_bodies_feasible=all(x['feasible'] for x in result))


def evaluate(cfg,out,pre,builddir,solvedir):
 require(read(solvedir/'metrics.json')['passed'],'Fit required');theta=np.load(solvedir/'theta.npy');op=Operator(cfg['old']);h=cfg['old']['physics_dt_s']
 scale=np.array(read(Path(cfg['root'])/'learn/normalization.json')['tau_scale'])
 baseline=read(cfg['pinn_checkpoint']);pinnphysical=read(cfg['pinn_physical']);comparisons=[];six=[];physicalrows=[]
 for spec in cfg['records']:
  model,_,_,_=make_model(modelpath(cfg,spec),False);data=model.createData()
  if spec==cfg['records'][0]:write(out/'physical_feasibility.json',physical(model,theta))
  q,v,dv,tau=packet(cfg['root'],spec)
  with np.load(builddir/f"design_{spec['seed']}.npz") as f:r=(f['X']@theta+f['offset']).reshape(-1,6)
  with np.load(refpath(cfg,spec)) as f:parts=f['parts']
  c=np.empty_like(q)
  for i in range(len(q)):c[i]=regressor(model,data,q[i],v[i],np.zeros(6))@theta[:60]
  ce=op(h*c-parts[:,2]);del parts,c
  indices=np.arange(48000,208000,16);Ms=np.stack([np.column_stack([regressor(model,data,q[i],np.zeros(6),np.eye(6)[j])@theta[:60] for j in range(6)]) for i in indices])
  with np.load(Path(cfg['root'])/spec['folder']/'diagnostics_1kHz.npz') as f:Mt=f['M'][3000:13000]
  vv=v[indices];p=np.einsum('bij,bj->bi',Ms,vv);pt=np.einsum('bij,bj->bi',Mt,vv);eig=np.linalg.eigvalsh(Ms);err=Ms-Mt
  ph=dict(seed=spec['seed'],matrix_relative_frobenius_percent=float(100*np.linalg.norm(err)/np.linalg.norm(Mt)),
    matrix_absolute_frobenius=float(np.linalg.norm(err)),matrix_frobenius_rms_kg_m2=float(np.sqrt(np.mean(np.sum(err**2,axis=(1,2))))),matrix_entry_rmse_kg_m2=np.sqrt(np.mean(err**2,0)),
    mass_diagonal_rmse_kg_m2=np.sqrt(np.mean((np.diagonal(Ms,axis1=1,axis2=2)-np.diagonal(Mt,axis1=1,axis2=2))**2,0)),
    momentum_relative_frobenius_percent=float(100*np.linalg.norm(p-pt)/np.linalg.norm(pt)),momentum_rmse_Nm_s=np.sqrt(np.mean((p-pt)**2,0)),
    minimum_eigenvalue=float(eig.min()),nonpositive_eigenvalue_states=int(np.sum(eig[:,0]<=0)),symmetry_max_abs=float(np.abs(Ms-Ms.transpose(0,2,1)).max()))
  physicalrows.append(ph)
  pp=next(x for x in pinnphysical if x['updates']==200 and x['seed']==spec['seed']);ps=next(x for x in baseline['scores'] if x['seed']==spec['seed'])
  with np.load(Path(cfg['pinn_components'])/f"components_{spec['seed']}.npz") as f:pce=f['EC']
  for name,residual,vel,phy in [('rigid_unconstrained',r,ce,ph),('PINN_update200',None,pce,pp)]:
   loss=float(np.mean((residual/scale)**2)) if residual is not None else ps['loss'];rmse=np.sqrt(np.mean(residual**2,0)) if residual is not None else np.array(ps['rmse_Nm'])
   comparisons.append(dict(seed=spec['seed'],split=spec['split'],model=name,loss=loss,matrix_relative_frobenius_percent=phy['matrix_relative_frobenius_percent'],momentum_relative_frobenius_percent=phy['momentum_relative_frobenius_percent'],minimum_mass_eigenvalue=phy['minimum_eigenvalue'],matrix_error_mean_square=float(np.sum(np.array(phy['matrix_entry_rmse_kg_m2'])**2)),matrix_reference_mean_square=float(np.mean(np.sum(Mt**2,axis=(1,2)))),momentum_error_mean_square=float(np.sum(np.array(phy['momentum_rmse_Nm_s'])**2)),momentum_reference_mean_square=float(np.mean(np.sum(pt**2,axis=1)))))
   for j in range(6):six.append(dict(seed=spec['seed'],split=spec['split'],model=name,joint=j+1,residual_rmse_Nm=rmse[j],velocity_interval_rmse_Nm=np.sqrt(np.mean(vel[:,j]**2)),mass_diagonal_rmse_kg_m2=phy['mass_diagonal_rmse_kg_m2'][j],momentum_rmse_Nm_s=phy['momentum_rmse_Nm_s'][j]))
  print('EVALUATED',spec['seed'],flush=True)
 for name in ['rigid_unconstrained','PINN_update200']:
  rows=[x for x in comparisons if x['split']=='train' and x['model']==name]
  comparisons.append(dict(seed='train_joint',split='train',model=name,loss=float(np.mean([x['loss'] for x in rows])),matrix_relative_frobenius_percent=float(100*np.sqrt(sum(x['matrix_error_mean_square'] for x in rows)/sum(x['matrix_reference_mean_square'] for x in rows))),momentum_relative_frobenius_percent=float(100*np.sqrt(sum(x['momentum_error_mean_square'] for x in rows)/sum(x['momentum_reference_mean_square'] for x in rows))),minimum_mass_eigenvalue=min(x['minimum_mass_eigenvalue'] for x in rows),**{key:float(np.mean([x[key] for x in rows])) for key in ['matrix_error_mean_square','matrix_reference_mean_square','momentum_error_mean_square','momentum_reference_mean_square']}))
 for name in ['rigid_unconstrained','PINN_update200']:
  for j in range(1,7):
   rows=[x for x in six if x['split']=='train' and x['model']==name and x['joint']==j]
   row=dict(seed='train_joint',split='train',model=name,joint=j)
   for key in ['residual_rmse_Nm','velocity_interval_rmse_Nm','mass_diagonal_rmse_kg_m2','momentum_rmse_Nm_s']:
    row[key]=float(np.sqrt(np.mean([x[key]**2 for x in rows])))
   six.append(row)
 csvwrite(out/'model_comparison.csv',comparisons);csvwrite(out/'six_axis_metrics.csv',six);write(out/'physical_metrics.json',physicalrows)
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 fig,axs=plt.subplots(1,2,figsize=(11,4))
 diag=read(solvedir/'design_diagnostics.json');s=np.array(diag['normalized']['singular_values']);axs[0].semilogy(np.arange(len(s)),s,'o');axs[0].axhline(diag['normalized']['threshold'],color='r');axs[0].set(title='Training normalized singular spectrum',xlabel='Index',ylabel='Singular value')
 for name in ['rigid_unconstrained','PINN_update200']:
  rows=[x for x in six if x['seed']==20261020 and x['model']==name];axs[1].semilogy(range(1,7),[x['residual_rmse_Nm'] for x in rows],'o-',label=name)
 axs[1].set(title='Development validation residual',xlabel='Joint',ylabel='RMSE (N m)');axs[1].legend();fig.tight_layout();fig.savefig(out/'comparison.png',dpi=150);plt.close(fig)
 (out/'decision.md').write_text('# H-021 传统刚体联合辨识\n\nEXPLORE；无约束诊断，物理可行性必须单列，未运行KF。\n\n结果见model_comparison.csv、six_axis_metrics.csv、physical_feasibility.json以及../solve/design_diagnostics.json。\n')
 return dict(passed=True,pass_semantics='diagnostic completed; not physical model acceptance',test_read=False,simulation_steps=0)


if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--phase',required=True);p.add_argument('--output-dir',required=True);p.add_argument('--run-root',required=True);p.add_argument('--source-run-root');a=p.parse_args()
 cfg=read(a.config);out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=True);root=Path(a.run_root);source=Path(a.source_run_root) if a.source_run_root else root;tick=time.monotonic()
 try:
  if a.phase=='small':r=small(cfg,out)
  elif a.phase=='build':r=build(cfg,out,root/'small')
  elif a.phase=='solve':r=solve(cfg,out,source/'small',source/'build')
  else:r=evaluate(cfg,out,source/'small',source/'build',root/'solve')
  r['wall_seconds']=time.monotonic()-tick;write(out/'metrics.json',r);print(json.dumps(r),flush=True)
 except Exception as e:
  write(out/'metrics.json',dict(passed=False,error=repr(e),wall_seconds=time.monotonic()-tick));raise
