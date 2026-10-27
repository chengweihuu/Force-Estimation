import sys,importlib.util,argparse,json,time,csv
from pathlib import Path
import numpy as np
import mujoco
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / 'scripts/rigid_baseline/reference_kf'))
from oracle_kf import Model
from observer import MomentumResidual, UR5Observer
# Shared simulation utilities remain outside this paper-specific directory.
spec = importlib.util.spec_from_file_location('plant_helpers', ROOT / 'scripts/simulation/ur5_helpers.py')
core = importlib.util.module_from_spec(spec)
spec.loader.exec_module(core)
CONFIG_PATH = HERE / 'configs/actuator_loss.json'
def write(p,v):p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def packet(p):
 with np.load(p) as f:return {k:f[k] for k in f.files}
def csvwrite(p,rs):
 with p.open('w') as f:w=csv.DictWriter(f,fieldnames=list(rs[0]));w.writeheader();w.writerows(rs)
def config():
 c=json.loads(CONFIG_PATH.read_text())
 model=Path(c['model'])
 c['model']=str(model if model.is_absolute() else ROOT/model)
 return c

def loss_fraction(t,case):
 rho=np.zeros(6)
 if case!='motion' and 2<=t<3:rho[1]=.2
 if case=='J2_J5_loss' and 2.5<=t<3.5:rho[4]=.2
 return rho
def preflight(c,out):
 model=Model(c['model'],c['derivative_epsilon_s']);m=model.m;d=mujoco.MjData(m);ids=np.array([m.joint(name).id for name in c['joint_names']]);qa=m.jnt_qposadr[ids];va=m.jnt_dofadr[ids]
 core.require((m.nq,m.nv,m.nu)==(6,6,6) and np.array_equal(qa,np.arange(6)) and np.array_equal(va,np.arange(6)),'UR5 joint mapping');core.require(np.array_equal(m.dof_damping,c['damping']) and not np.any(m.dof_armature) and not np.any(m.dof_frictionloss),'model changes');core.require(m.opt.timestep==c['physics_dt_s'] and m.opt.integrator==mujoco.mjtIntegrator.mjINT_IMPLICITFAST,'integrator');core.require(not(m.opt.disableflags&(mujoco.mjtDisableBit.mjDSBL_CONTACT|mujoco.mjtDisableBit.mjDSBL_CONSTRAINT)),'contacts disabled')
 checks=[];mask=core.moving_mask(m)
 for tt in [0.,.25,.75,1.5,3.]:
  q,v,_=core.trajectory(tt,c,True);M,h,Md,J=model.evaluate(q,v);np.linalg.cholesky(M);d.qpos[qa]=q;d.qvel[va]=v;d.ctrl[:]=h+model.B*v;ext=np.array([.2,1.,0.,0.,-.5,0.]);d.qfrc_applied[va]=ext;mujoco.mj_forward(m,d);core.valid(m,d,mask,d.ctrl)
  alpha=h+model.B*v-Md@v;lhs=M@d.qacc+Md@v;rhs=d.qfrc_actuator-alpha+ext;err=float(abs(lhs-rhs).max());core.require(np.allclose(lhs,rhs,rtol=1e-8,atol=1e-9),'momentum balance');core.require(np.allclose(d.qfrc_passive,-model.B*v,rtol=1e-8,atol=1e-9),'passive sign')
  for f in [.5,2.]:core.require(np.allclose(model.evaluate(q,v,c['derivative_epsilon_s']*f)[2],Md,rtol=1e-4,atol=1e-9),'direction derivative')
  checks.append(dict(t=tt,balance_max_Nm=err))
 # Independent discrete identity: p increments from known input+disturbance,
 # observer never receives disturbance; includes nonzero p0 and both signs.
 obs=MomentumResidual(c['gain_s_inv']);p=np.arange(6)*.2;ref=np.zeros(6);last_b=last_f=None;maxerr=0.;dt=.001
 for k in range(1001):
  t=k*dt;b=np.arange(6)*.01*np.cos(t)
  if k:p=p+dt*(last_b+last_f);ref=ref+dt*50*(last_f-ref)
  r,ph=obs.update(t,p,b);maxerr=max(maxerr,float(abs(r-ref).max()));f=np.zeros(6)
  if .2<=t<.6:f[1]=1.;f[4]=-.5
  last_b,last_f=b,f
 core.require(np.allclose(r,ref,rtol=1e-8,atol=1e-9) and maxerr<1e-9,'synthetic recurrence');obs.reset();rr,ph=obs.update(0.,p,np.zeros(6));core.require(np.array_equal(ph,p) and np.array_equal(rr,np.zeros(6)),'reset')
 # Fault interface test: same physical momentum, nominal vs actual input.
 primary=MomentumResidual(c['gain_s_inv']);actual=MomentumResidual(c['gain_s_inv']);p=np.arange(6)*.2;ref=np.zeros(6);previous=None;fault_error=actual_error=0.
 for k in range(401):
  t=k*.001;u=np.arange(1,7)*np.cos(t);rho=np.zeros(6)
  if .1<=t<.3:rho[1]=.2
  if .2<=t<.35:rho[4]=.2
  ta=(1-rho)*u;alpha=np.arange(6)*.03
  if previous is not None:
   u0,ta0,a0=previous;p+=.001*(ta0-a0);ref+=.001*50*((u0-ta0)-ref)
  rr,_=primary.update(t,p,u-alpha);ra,_=actual.update(t,p,ta-alpha)
  fault_error=max(fault_error,float(abs(-rr-ref).max()));actual_error=max(actual_error,float(abs(ra).max()));previous=(u,ta,alpha)
 core.require(fault_error<1e-9 and actual_error<1e-9,'fault sign/input synthetic check')
 write(out/'fault_interface_checks.json',dict(fault_reference_max_Nm=fault_error,actual_input_max_Nm=actual_error,synthetic_calls=802))
 write(out/'config.json',c);write(out/'checks.json',dict(mapping_names=c['joint_names'],qpos_addresses=qa.tolist(),dof_addresses=va.tolist(),momentum_checks=checks,synthetic_max_Nm=maxerr,truth_not_observer_input=True));return dict(passed=True,simulation_steps=0,observer_calls=1804,static_states=15)
def simulate(c,root,out):
 core.require(json.loads((root/'preflight/metrics.json').read_text())['passed'],'preflight');total=calls=0
 for case,cc in c['cases'].items():
  dest=out/case;dest.mkdir();m=mujoco.MjModel.from_binary_path(c['model']);d=mujoco.MjData(m);control=Model(c['model'],c['derivative_epsilon_s']);obs=UR5Observer(Model(c['model'],c['derivative_epsilon_s']),c['gain_s_inv']);core.require(obs.nominal.m is not m and obs.nominal.d is not d,'shared plant');actual_obs=UR5Observer(Model(c['model'],c['derivative_epsilon_s']),c['gain_s_inv']);mask=core.moving_mask(m);d.qpos[:]=c['initial_q_rad'];dt=c['physics_dt_s'];stride=round(c['observer_dt_s']/dt);steps=round(cc['duration_s']/dt);kp=np.array(c['controller']['kp_Nm_rad']);kd=np.array(c['controller']['kd_Nm_s_rad']);data={k:[] for k in ['t','q','v','tau']};truth={k:[] for k in ['t','tau_ext','q_ref','command','rho','tau_missing']};history={};command=np.zeros(6);completed=0;balance=tracking=0.
  try:
   for n in range(steps+1):
    t=n*dt;d.qfrc_applied[:]=0;d.xfrc_applied[:]=0;mujoco.mj_forward(m,d);qr,vr,ar=core.trajectory(t,c,cc['motion'])
    if n%stride==0:
     M,h,_,_=control.evaluate(d.qpos.copy(),d.qvel.copy());command=M@ar+h+control.B*d.qvel+kp*(qr-d.qpos)+kd*(vr-d.qvel);d.ctrl[:]=command
    rho=loss_fraction(t,case);drive=(1-rho)*command;d.ctrl[:]=drive;ext=np.zeros(6);d.qfrc_applied[:]=ext;mujoco.mj_forward(m,d);core.valid(m,d,mask,drive);lim=m.actuator_ctrllimited.astype(bool);core.require(np.all(command[lim]>=m.actuator_ctrlrange[lim,0]) and np.all(command[lim]<=m.actuator_ctrlrange[lim,1]),'Nominal torque saturation');core.require(np.allclose(command-d.qfrc_actuator,rho*command,rtol=1e-8,atol=1e-9),'Missing torque identity');M=np.zeros((6,6));mujoco.mj_fullM(m,M,d.qM);err=M@d.qacc+d.qfrc_bias-d.qfrc_actuator-d.qfrc_passive-d.qfrc_applied;balance=max(balance,float(abs(err).max()));core.require(np.allclose(err,0,rtol=0,atol=1e-9),'plant balance');tracking=max(tracking,float(abs(d.qpos-qr).max()))
    if n%stride==0:
     q,v,tau=d.qpos.copy(),d.qvel.copy(),d.qfrc_actuator.copy();result=obs.update(t,q,v,command.copy());diagnostic=actual_obs.update(t,q,v,tau);result['actual_input_residual']=diagnostic['residual'];result['fault_residual']=-result['residual'];calls+=2;core.require(all(np.isfinite(a).all() for a in result.values()),'observer finite')
     for key,val in zip(data,[t,q,v,tau]):data[key].append(val)
     for key,val in zip(truth,[t,ext.copy(),qr,command.copy(),rho.copy(),command-tau]):truth[key].append(val)
     for key,val in result.items():history.setdefault(key,[]).append(val.copy())
    if n<steps:mujoco.mj_step(m,d);completed+=1
  except Exception as exc:write(dest/'failure.json',dict(error=repr(exc),time_s=float(d.time),completed_steps=completed));raise
  finally:
   np.savez_compressed(dest/'measurements.npz',**data);np.savez_compressed(dest/'references.npz',**truth);np.savez_compressed(dest/'estimates.npz',t=data['t'],**history)
  if case=='motion':
   checks=[]
   for file in ['measurements','estimates']:
    prior=packet((HERE / 'reference_data')/(file+'.npz'));now=packet(dest/(file+'.npz'))
    for key in prior:
     err=float(abs(now[key]-prior[key]).max());ok=bool(np.allclose(now[key],prior[key],rtol=1e-8,atol=1e-9));checks.append(dict(file=file,field=key,max_error=err,passed=ok));core.require(ok,'historical healthy mismatch')
   write(dest/'equivalence.json',checks)
  else:
   for file in ['measurements','estimates']:
    prior=packet(out/'motion'/(file+'.npz'));now=packet(dest/(file+'.npz'));core.require(all(np.array_equal(now[key][:2000],prior[key][:2000]) for key in now),'prefault history mismatch')
  write(dest/'checks.json',dict(passed=True,simulation_steps=completed,frames=len(data['t']),max_balance_Nm=balance,max_tracking_rad=tracking,contacts=0,clipping=0,limits=0,warnings=0));total+=completed;print(case,'passed',flush=True)
 core.require(total==288000 and calls==36006,'counts');return dict(passed=True,simulation_steps=total,observer_calls=calls,model_evaluations=3*calls//2)
def evaluate(c,root,out):
 spec=importlib.util.spec_from_file_location('evaluation',HERE / 'actuator_loss_evaluation.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);return mod.evaluate(c,root,out)
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--phase',required=True);a.add_argument('--run-root',type=Path,required=True);a.add_argument('--output-dir',type=Path,required=True);a.add_argument('--config',type=Path,default=CONFIG_PATH);a=a.parse_args();CONFIG_PATH=a.config.resolve();a.run_root=a.run_root.resolve();a.output_dir.mkdir(parents=True,exist_ok=True);st=time.monotonic()
 try:
  c=config();result=preflight(c,a.output_dir) if a.phase=='preflight' else simulate(c,a.run_root,a.output_dir) if a.phase=='simulate' else evaluate(c,a.run_root,a.output_dir);result['wall_seconds']=time.monotonic()-st;write(a.output_dir/'metrics.json',result)
 except Exception as exc:write(a.output_dir/'metrics.json',dict(passed=False,error=repr(exc),wall_seconds=time.monotonic()-st));raise
