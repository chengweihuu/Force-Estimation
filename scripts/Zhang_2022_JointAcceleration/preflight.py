import inspect
import numpy as np
import mujoco
from common import Model,MomentumResidual,helper,write,load,ROOT
from acceleration import HODO,AccelerationBank

def run(c,out):
    # Existing verified model/GM preflight; no physical time integration.
    old=load(ROOT/'scripts/DeLuca_2003_GeneralizedMomenta/external_torque.py','verified_gmo')
    first=old.preflight(c,out)
    gains=np.asarray(c['hodo_gains']);rng=np.random.default_rng(1729)
    maxima=[];updates=first['observer_calls']
    for linear in [False,True]:
        g=gains.copy()
        if linear:g[[0,2,4]]=0
        bank=HODO(g);x=None;previous=None;t0=None;mx=0.
        for k in range(201):
            t=.001*k;v=.2+np.arange(6)*.03+.2*t+.15*t*t
            base=np.tile(.2+np.arange(6)*.01,(3,1))
            if x is None:
                x=np.zeros((3,3,6));x[:,0]=v
            else:
                before=x.copy()
                for method in range(3):
                    for layer in range(3):x[method,layer]=before[method,layer]+(t-t0)*previous[method,layer]
            x[1,2]=0
            deriv=np.zeros_like(x);us=np.zeros_like(x)
            for m in range(3):
                for j in range(6):
                    e=v[j]-x[m,0,j];sign=1. if e>0 else -1. if e<0 else 0.
                    us[m,0,j]=g[0,j]*np.sqrt(abs(e))*sign+g[1,j]*e
                    us[m,1,j]=g[2,j]*sign+g[3,j]*e
                    us[m,2,j]=g[4,j]*sign+g[5,j]*e if m!=1 else 0.
                    deriv[m,0,j]=base[m,j]+x[m,1,j]+us[m,0,j]
                    deriv[m,1,j]=x[m,2,j]+us[m,1,j]
                    deriv[m,2,j]=us[m,2,j]
            result=bank.update(t,v,base);updates+=3
            for a,b in [(result['state'],x),(result['acceleration'],deriv[:,0]),(result['correction'],us)]:
                mx=max(mx,float(abs(a-b).max()));np.testing.assert_allclose(a,b,rtol=1e-8,atol=1e-9)
            assert np.all(result['state'][1,2]==0) and np.all(result['correction'][1,2]==0)
            previous=deriv.copy();t0=t
        bank.reset();result=bank.update(0.,np.ones(6),np.zeros((3,6)));updates+=3
        assert np.array_equal(result['state'][:,0],np.ones((3,6)))
        assert np.all(result['state'][:,1:]==0) and np.all(result['acceleration']==0)
        maxima.append(dict(linear_diagnostic=linear,max_error=mx))
    omega=c['omega_s_inv'];A=np.array([[-3*omega,1,0],[-3*omega**2,0,1],[-omega**3,0,0.]])
    np.testing.assert_allclose(np.poly(A),[1,3*omega,3*omega**2,omega**3],rtol=1e-8,atol=1e-9)
    assert list(inspect.signature(AccelerationBank.update).parameters)==['self','t','q','v','tau']
    # An independent static force balance and acceleration-equivalent injection.
    model=Model(c['model'],c['derivative_epsilon_s']);d=mujoco.MjData(model.m);d.qpos[:]=c['initial_q_rad'];d.qvel[:]=np.arange(6)*.02
    M,h,Md,J=model.evaluate(d.qpos.copy(),d.qvel.copy());a=np.zeros(6);a[1]=1.
    d.ctrl[:]=h+model.B*d.qvel;d.qfrc_applied[:]=M@a;mujoco.mj_forward(model.m,d)
    np.testing.assert_allclose(d.qacc,a,rtol=1e-8,atol=1e-9)
    nominal=Model(c['model'],c['derivative_epsilon_s']);assert nominal.d is not d
    bank=AccelerationBank(nominal,MomentumResidual(c['gain_s_inv']),gains)
    q=d.qpos.copy();v=d.qvel.copy();tau=d.qfrc_actuator.copy();o=bank.update(0.,q,v,tau);updates+=6
    np.testing.assert_allclose(o['f'],np.linalg.solve(M,tau-h-model.B*v),rtol=1e-8,atol=1e-9)
    assert np.all(o['gm_residual']==0)
    write(out/'hodo_checks.json',dict(independent_update=maxima,linear_polynomial=np.poly(A),truth_free_interface=True,acceleration_injection_error=float(abs(d.qacc-a).max()),gain_units=c['gain_units']))
    return dict(passed=True,simulation_steps=0,observer_calls=updates,model_evaluations=17)
