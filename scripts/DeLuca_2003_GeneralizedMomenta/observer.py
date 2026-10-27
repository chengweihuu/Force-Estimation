"""Causal positive external joint-torque residual; no KF or truth inputs."""
import numpy as np
class MomentumResidual:
 def __init__(self,gain):
  self.K=np.asarray(gain,dtype=float);self.reset()
 def reset(self):self.previous=None
 def update(self,t,p,b):
  p,b=np.asarray(p),np.asarray(b)
  assert p.shape==b.shape==self.K.shape and np.isfinite(p).all() and np.isfinite(b).all()
  if self.previous is None:self.phat=p.copy();self.r=np.zeros_like(p)
  else:
   t0,b0=self.previous;dt=t-t0
   assert dt>0 and np.all(dt*self.K<2)
   self.phat+=dt*(b0+self.r);self.r=self.K*(p-self.phat)
  self.previous=(t,b.copy())
  return self.r.copy(),self.phat.copy()
class UR5Observer:
 def __init__(self,model,gain):self.nominal=model;self.core=MomentumResidual(gain)
 def update(self,t,q,v,tau):
  M,h,Md,_=self.nominal.evaluate(q,v);p=M@v;alpha=h+self.nominal.B*v-Md@v;b=tau-alpha;r,phat=self.core.update(t,p,b)
  return dict(momentum=p,alpha=alpha,nominal_input=b,residual=r,estimated_momentum=phat)
