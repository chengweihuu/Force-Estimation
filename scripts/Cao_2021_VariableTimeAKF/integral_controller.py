"""Model-feedforward + fixed scaled PD + bounded torque integral.
No experiment identity, wrench, event time, or observer inputs.
"""
import numpy as np


class IntegralController:
    def __init__(self, kp_old, kd_old, scale, integral_rate, torque_limits,
                 integral_limit_fraction, control_dt):
        self.kp=scale*np.asarray(kp_old,dtype=float)
        self.kd=np.sqrt(scale)*np.asarray(kd_old,dtype=float)
        self.ki=integral_rate*self.kp
        self.limits=np.asarray(torque_limits,dtype=float)
        self.imax=integral_limit_fraction*self.limits
        self.dt=float(control_dt)
        self.integral=np.zeros_like(self.kp)
        if any(x.shape!=(6,) or not np.isfinite(x).all() for x in [self.kp,self.kd,self.ki,self.limits,self.imax]):
            raise ValueError('Expected finite six-axis controller parameters')
        if not (scale>0 and integral_rate>=0 and self.dt>0 and np.all(self.limits>0) and np.all(self.imax>0)):
            raise ValueError('Invalid controller parameters')

    def update(self,q,v,q_ref,v_ref,a_ref,M,bias,damping):
        e=np.asarray(q)-np.asarray(q_ref);ev=np.asarray(v)-np.asarray(v_ref)
        p=-self.kp*e;d=-self.kd*ev
        ff=np.asarray(M)@np.asarray(a_ref)+np.asarray(bias)+np.asarray(damping)*np.asarray(v)
        used=self.integral.copy();command=ff+p+d+used
        proposed=used-self.dt*self.ki*e
        candidate=np.clip(proposed,-self.imax,self.imax)
        increment=candidate-used
        frozen=((command>=self.limits)&(increment>0))|((command<=-self.limits)&(increment<0))
        self.integral=np.where(frozen,used,candidate)
        return dict(command=command,feedforward=ff,P=p,D=d,I_used=used,I_next=self.integral.copy(),
                    integral_upper_hit=proposed>self.imax,integral_lower_hit=proposed<-self.imax,
                    antiwindup_frozen=frozen,command_exceeds_limit=abs(command)>self.limits)


def frozen_closed_loop(M,damping,kp,kd,ki,physics_dt,control_dt):
    """Frozen implicit velocity + semi-implicit position; held control torque."""
    n=6;eye=np.eye(n);zero=np.zeros((n,n));B=np.diag(damping)
    V=np.linalg.solve(M+physics_dt*B,M)
    W=np.linalg.solve(M+physics_dt*B,physics_dt*eye)
    F=np.block([[eye,physics_dt*V],[zero,V]])
    G=np.vstack([physics_dt*W,W])
    count=round(control_dt/physics_dt)
    if not np.isclose(count*physics_dt,control_dt,rtol=0,atol=1e-15):
        raise ValueError('Nonintegral controller/physics clock ratio')
    Fc=np.eye(12);Gc=np.zeros((12,6))
    for _ in range(count):
        Gc=F@Gc+G;Fc=F@Fc
    K=np.column_stack([-np.diag(kp),B-np.diag(kd)])
    bottom=np.column_stack([-control_dt*np.diag(ki),zero])
    return np.block([[Fc+Gc@K,Gc],[bottom,eye]])
