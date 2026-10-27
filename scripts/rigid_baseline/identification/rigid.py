"""Geometry-only Pinocchio regression with separately constructed diagnostic truth."""
import json
from pathlib import Path
import mujoco
import numpy as np
import pinocchio as pin
from scipy.signal import butter,sosfiltfilt


def read(p):return json.loads(Path(p).read_text())
def write(p,x):
 def convert(v):
  if isinstance(v,np.ndarray):return v.tolist()
  if isinstance(v,np.generic):return v.item()
  raise TypeError(type(v).__name__)
 with Path(p).open('x') as f:json.dump(x,f,default=convert,indent=2,ensure_ascii=False,allow_nan=False)
def require(ok,msg):
 if not ok:raise RuntimeError(msg)
def rotation(quat):
 R=np.empty(9);mujoco.mju_quat2Mat(R,quat);return R.reshape(3,3)


def make_model(path,truth=False):
 m=mujoco.MjModel.from_binary_path(str(path));model=pin.Model();model.gravity.linear[:]=0
 require((m.nq,m.nv)==(6,6) and np.all(m.qpos0==0),'Unsupported coordinates')
 require(not np.any(m.dof_armature) and not np.any(m.dof_frictionloss),'Unexpected fixed armature/frictionloss')
 body_joint={0:0};placements={0:pin.SE3.Identity()};mapping=[];names=[]
 for b in range(1,m.nbody):
  parent=int(m.body_parentid[b]);T=placements[parent]*pin.SE3(rotation(m.body_quat[b]),m.body_pos[b])
  owner=body_joint[parent]
  require(m.body_jntnum[b]<=1,'Unsupported multi-joint body')
  if m.body_jntnum[b]:
   j=int(m.body_jntadr[b]);require(m.jnt_type[j]==mujoco.mjtJoint.mjJNT_HINGE,'Not hinge')
   require(m.jnt_qposadr[j]==len(names) and m.jnt_dofadr[j]==len(names),'Joint order mismatch')
   name=m.joint(j).name;names.append(name)
   owner=model.addJoint(owner,pin.JointModelRevoluteUnaligned(m.jnt_axis[j]),T*pin.SE3(np.eye(3),m.jnt_pos[j]),name)
   T=pin.SE3(np.eye(3),-m.jnt_pos[j])
  body_joint[b]=owner;placements[b]=T
  mapping.append(dict(body=m.body(b).name,joint=owner,placement=T.homogeneous))
  if truth and owner>0:
   R=rotation(m.body_iquat[b]);I=R@np.diag(m.body_inertia[b])@R.T
   model.appendBodyToJoint(owner,pin.Inertia(float(m.body_mass[b]),m.body_ipos[b],I),T)
 if not truth:
  # Arbitrary physical placeholders; the regression must be independent of them.
  for j in range(1,model.njoints):model.inertias[j]=pin.Inertia(1.,np.zeros(3),np.eye(3))
 require(model.nq==6 and model.nv==6 and model.njoints==7,'Unexpected reduced tree')
 return model,m,mapping,names


def parameters(model):return np.concatenate([model.inertias[j].toDynamicParameters() for j in range(1,model.njoints)])
def apply_parameters(model,pi):
 for j in range(1,model.njoints):model.inertias[j]=pin.Inertia.FromDynamicParameters(pi[(j-1)*10:j*10])
def regressor(model,data,q,v,a):return pin.computeJointTorqueRegressor(model,data,q,v,a).copy()
def mass(model,data,q):
 M=pin.crba(model,data,q).copy();return np.triu(M)+np.triu(M,1).T


class Operator:
 def __init__(self,plan):
  self.h=plan['physics_dt_s'];self.dt=plan['feedback_dt_s'];self.stride=round(self.dt/self.h)
  f=plan['filter'];require(f==dict(order=4,cutoff_Hz=20,padtype='odd',padlen=150,edge_order=2),'Operator differs')
  self.sos=butter(4,20,fs=1/self.dt,btype='lowpass',output='sos');self.lo,self.hi=[round(t/self.dt) for t in plan['core_interval_s']]
 def __call__(self,e):
  # Support arbitrary batch columns, each with exactly the same linear operation.
  block=e.reshape(-1,self.stride,*e.shape[1:]).sum(axis=1)
  nodes=np.concatenate([np.zeros((1,*e.shape[1:])),np.cumsum(block,axis=0)],axis=0)
  filtered=sosfiltfilt(self.sos,nodes,axis=0,padtype='odd',padlen=150)
  return np.gradient(filtered,self.dt,axis=0,edge_order=2)[self.lo:self.hi].copy()


def packet(root,spec):
 folder=Path(root)/spec['folder'];c=read(folder/'config.json')
 require(c['seed']==spec['seed'] and c['split']==spec['split'],'Data split mismatch')
 require(read(folder/'collection_checks.json')['passed'],'Collection failed')
 with np.load(folder/'training_packet.npz') as f:
  require(set(f.files)=={'t','q','v','tau_actuator','ctrl','control_update','control_index','feedback_sample','physics_dt_s'},'Schema mismatch')
  q=f['q'][:-1].copy();v=f['v'].copy();tau=f['tau_actuator'][:-1].copy()
  require(float(f['physics_dt_s'])==.0000625 and len(q)==272000,'Packet rate/size mismatch')
 return q,v[:-1],np.diff(v,axis=0),tau
