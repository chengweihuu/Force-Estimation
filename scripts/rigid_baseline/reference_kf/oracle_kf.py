"""Independent true-parameter model; estimator only accepts t, q, v, tau."""
import importlib.util
from pathlib import Path
import mujoco
import numpy as np


def load(path,name):
    s=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m


class Model:
    def __init__(self,path,epsilon):
        self.m=mujoco.MjModel.from_binary_path(str(path))
        self.d=mujoco.MjData(self.m);self.plus=mujoco.MjData(self.m);self.minus=mujoco.MjData(self.m)
        self.site=self.m.site('tool0').id;self.epsilon=epsilon;self.B=self.m.dof_damping.copy()
    def mass(self,q,d):
        d.qpos[:]=q;d.qvel[:]=0;mujoco.mj_forward(self.m,d)
        M=np.zeros((6,6));mujoco.mj_fullM(self.m,M,d.qM);return M
    def evaluate(self,q,v,epsilon=None):
        self.d.qpos[:]=q;self.d.qvel[:]=v;mujoco.mj_forward(self.m,self.d)
        M=np.zeros((6,6));mujoco.mj_fullM(self.m,M,self.d.qM)
        jp,jr=np.zeros((3,6)),np.zeros((3,6));mujoco.mj_jacSite(self.m,self.d,jp,jr,self.site)
        ep=self.epsilon if epsilon is None else epsilon
        Md=(self.mass(q+ep*v,self.plus)-self.mass(q-ep*v,self.minus))/(2*ep)
        return M,self.d.qfrc_bias.copy(),Md,np.vstack([jp,jr])


class Observer:
    def __init__(self,cfg):
        self.nominal=Model(cfg['model'],cfg['derivative_epsilon_s'])
        self.step=load(cfg['kf_source'],'kf_kernel').kf_step
        c=cfg['observer'];self.Qp=np.diag(c['Qp_continuous_diag']);self.Qw=np.diag(c['Qw_continuous_diag'])
        self.Sv=np.diag(np.square(c['assumed_velocity_std_rad_s']));self.Pw=np.diag(c['initial_wrench_variance']);self.previous=None
    def update(self,t,q,v,tau):
        assert all(np.asarray(x).shape==(6,) and np.isfinite(x).all() for x in [q,v,tau])
        M,h,Md,J=self.nominal.evaluate(q,v);p=M@v;R=M@self.Sv@M.T
        if self.previous is None:
            self.x=np.r_[p,np.zeros(6)];self.P=np.zeros((12,12));self.P[:6,:6]=R;self.P[6:,6:]=self.Pw;innovation=np.zeros(6)
        else:
            t0,u0,J0=self.previous;assert t>t0
            self.x,self.P,innovation=self.step(self.x,self.P,p,u0,J0,t-t0,self.Qp,self.Qw,R)
        u=Md@v+tau-self.nominal.B*v-h
        self.previous=(t,u,J)
        return dict(wrench=self.x[6:].copy(),joint_external_torque=J.T@self.x[6:],covariance=self.P.copy(),
                    innovation=innovation,momentum=p,nominal_input=u)
