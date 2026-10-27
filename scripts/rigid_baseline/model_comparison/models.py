"""State-only model queries and independent interface checks."""
import argparse,importlib.util,json,time
from pathlib import Path
import numpy as np

def load(p,name):
 s=importlib.util.spec_from_file_location(name,p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m

def read(p):return json.loads(Path(p).read_text())
def write(p,x):
 def default(v):
  if isinstance(v,np.ndarray):return v.tolist()
  if isinstance(v,np.generic):return v.item()
  raise TypeError(type(v).__name__)
 with Path(p).open('x') as f:json.dump(x,f,default=default,indent=2,allow_nan=False)
def require(x,msg):
 if not x:raise ValueError(msg)
def feedback(c,case):
 with np.load(Path(c['logs'])/case/'measurements.npz') as f:
  require(set(f.files)=={'t','q','v','tau'},'Feedback boundary');return {k:f[k].copy() for k in f.files}
def close(a,b,rtol=1e-8,atol=1e-9):
 return dict(passed=bool(np.allclose(a,b,rtol=rtol,atol=atol)),max_abs=float(np.max(np.abs(a-b))))

class Rigid:
 def __init__(self,c,cfg):
  self.api=load(c['rigid_code'],'rigid_api');self.model,_,_,names=self.api.make_model(cfg['model'],False)
  require(names==cfg['joint_names'],'Rigid joint order')
  self.theta=np.load(c['theta']);self.api.apply_parameters(self.model,self.theta[:60]);self.data=self.model.createData();self.b=self.theta[60:];self.ep=cfg['derivative_epsilon_s']
 def mass(self,q):return self.api.mass(self.model,self.data,q)
 def one(self,q,v,ep=None):
  ep=self.ep if ep is None else ep;M=self.mass(q);Md=(self.mass(q+ep*v)-self.mass(q-ep*v))/(2*ep)
  c=self.api.regressor(self.model,self.data,q,v,np.zeros(6))@self.theta[:60]
  return dict(M=M,Mdot=Md,c=c,b=self.b)
 def check(self,q,v):
  a=self.one(q,v);checks=[close(a['Mdot'],self.one(q,v,self.ep*f)['Mdot'],1e-4) for f in [.5,2.]]
  # Regression columns must agree with inertia model used for Mdot.
  Mr=np.column_stack([self.api.regressor(self.model,self.data,q,np.zeros(6),np.eye(6)[j])@self.theta[:60] for j in range(6)])
  checks.append(close(a['M'],Mr));return dict(passed=all(x['passed'] for x in checks),checks=checks)

class Pinn:
 def __init__(self,c,cfg):
  import torch
  torch.set_num_threads(1);self.torch=torch
  api=load(c['adapter'],'interval_adapter');cls=load(c['structured'],'structured').StructuredDynamics
  plan=read(c['old_plan']);norm=read(c['normalization']);self.net=api.LearnedDynamics(cls,plan,norm['q_center'],norm['q_scale'],norm['tau_mean'])
  self.net.load_state_dict(torch.load(c['weights'],map_location='cpu',weights_only=True),strict=True);self.net.eval()
  ck=read(c['checkpoint']);require(ck['updates']==200,'Wrong PINN checkpoint')
  self.b=torch.nn.functional.softplus(self.net.friction_raw).detach().numpy()
  require(np.allclose(self.b,ck['physical_friction_coefficients'],rtol=1e-8,atol=1e-9),'Friction checkpoint mismatch')
 def batch(self,q,v):
  t=self.torch
  with t.no_grad():
   r=self.net.network.dynamics(t.as_tensor(q,dtype=t.float64),t.as_tensor(v,dtype=t.float64))
  return dict(M=r['M'].numpy(),Mdot=r['Mdot'].numpy(),c=(r['h']-r['G']).numpy(),b=np.broadcast_to(self.b,(len(q),6)).copy())
 def one(self,q,v):return {k:x[0] for k,x in self.batch(q[None],v[None]).items()}
 def check(self,q,v):
  t=self.torch;x=t.tensor(q,dtype=t.float64,requires_grad=True);vel=t.tensor(v,dtype=t.float64)
  def mass(z):return self.net.network.mass_gravity(z[None])[0][0]
  jac=t.autograd.functional.jacobian(mass,x);Md=t.einsum('ijk,k->ij',jac,vel)
  gradT=.5*t.einsum('i,ijk,j->k',vel,jac,vel);cref=Md@vel-gradT
  r=self.one(q,v);checks=[close(r['M'],mass(x).detach().numpy()),close(r['Mdot'],Md.detach().numpy()),close(r['c'],cref.detach().numpy())]
  def masks(z):
   with t.no_grad():
    z=(z-self.net.network.input_center)/self.net.network.input_scale;out=[]
    for layer in self.net.network.layers[:-1]:z=layer(z);out.append(z>0);z=t.relu(z)
   return out
  mm=masks(x-1e-6*vel);mp=masks(x+1e-6*vel);cross=sum(int((a!=b).sum()) for a,b in zip(mm,mp))
  return dict(passed=all(x['passed'] for x in checks),checks=checks,relu_crossings_directional_epsilon_1e_6=cross,method='autograd physical-q Jacobian; ReLU zero derivative convention')

def run(c,kind,phase,out):
 cfg=read(c['kf_config']);api=load(c['oracle_code'],'oracle');oracle=api.Model(cfg['model'],cfg['derivative_epsilon_s']);model=Rigid(c,cfg) if kind=='rigid' else Pinn(c,cfg);checks=[]
 for case in cfg['cases']:
  f=feedback(c,case);q,v=f['q'],f['v'];require(np.array_equal(v[0],np.zeros(6)),'Nonzero initial velocity')
  if phase=='check':
   idx=np.unique(np.linspace(0,len(q)-1,32,dtype=int))
   for i in idx:
    r=model.check(q[i],v[i]);r.update(case=case,index=int(i));checks.append(r)
    if kind=='rigid':
     M,h,Md,J=oracle.evaluate(q[i],v[i]);r['oracle_derivative']=[close(Md,oracle.evaluate(q[i],v[i],cfg['derivative_epsilon_s']*fac)[2],1e-4) for fac in [.5,2.]];r['passed'] &= all(x['passed'] for x in r['oracle_derivative'])
   if kind=='pinn':
    bat=model.batch(q[idx],v[idx]);err=max(float(np.max(np.abs(bat[k][j]-model.one(q[i],v[i])[k]))) for j,i in enumerate(idx) for k in bat)
    checks.append(dict(case=case,batch_point_max_abs=err,passed=err<=1e-9))
  else:
   dest=out/case;dest.mkdir();terms={k:[] for k in ['M','Mdot','c','b']};ref={k:[] for k in ['M','Mdot','c','b','G','J','R']}
   if kind=='pinn':
    chunks=[model.batch(q[i:i+256],v[i:i+256]) for i in range(0,len(q),256)]
    terms={k:np.concatenate([b[k] for b in chunks]) for k in chunks[0]}
   else:
    Sv=np.diag(np.square(cfg['observer']['assumed_velocity_std_rad_s']))
    for qi,vi in zip(q,v):
     r=model.one(qi,vi)
     for k in terms:terms[k].append(r[k])
     M,h,Md,J=oracle.evaluate(qi,vi);G=oracle.evaluate(qi,np.zeros(6))[1]
     rr=dict(M=M,Mdot=Md,c=h-G,b=oracle.B,G=G,J=J,R=M@Sv@M.T)
     for k in ref:ref[k].append(rr[k])
    np.savez_compressed(dest/'reference.npz',t=f['t'],**{k:np.asarray(x) for k,x in ref.items()})
   np.savez_compressed(dest/'model_terms.npz',t=f['t'],**{k:np.asarray(x) for k,x in terms.items()})
  print(kind,phase,case,flush=True)
 if phase=='check':
  write(out/'checks.json',dict(passed=all(r['passed'] for r in checks),states=checks));require(all(r['passed'] for r in checks),'Model interface/derivative check failed')
 return dict(passed=True,kind=kind,phase=phase,test_read=False,simulation_steps=0)

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--kind',required=True);p.add_argument('--phase',required=True);p.add_argument('--output-dir',required=True);a=p.parse_args();out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=True);tick=time.monotonic()
 try:r=run(read(a.config),a.kind,a.phase,out);r['wall_seconds']=time.monotonic()-tick;write(out/'metrics.json',r)
 except Exception as e:write(out/'metrics.json',dict(passed=False,error=repr(e),wall_seconds=time.monotonic()-tick));raise
