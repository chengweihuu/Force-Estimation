"""Bounded checks of reused current/KF/controller code on new fixtures only."""
import json
from pathlib import Path
import numpy as np
import mujoco, scipy
from scipy.linalg import expm
from input_statistics import write, csvwrite, load, require
from runner import load as module
from akf_comparison import CaptureObserver, close
from current_interface import EquivalentCurrentSensor, CurrentObserver
from filter_checks import run_checks
from controlled_collection import make_controller
from integral_controller import frozen_closed_loop
import constant_speed_v2 as calibration


def units(cfg, base, out, ledger):
    checks = run_checks(dict(window=50, history_count=64, rtol=cfg['rtol'], atol=cfg['atol']))
    write(out/'filter_checks.json', checks)
    versions = dict(mujoco=mujoco.__version__, numpy=np.__version__, scipy=scipy.__version__)
    for key,value in base['expected_versions'].items(): require(versions[key]==value, 'Version mismatch: '+key)
    calibration.precheck(cfg['calibration'],out)
    class Sink:
        def update(self,t,q,v,tau): return dict(torque=tau)
    D=np.array(base['nonuniform_fixture_D_Nm_per_Aeq']); tau=np.array([-3,2,-1,.5,-.2,.1])
    adapter=CurrentObserver(Sink(),D)
    got=adapter.update(0.,np.zeros(6),np.zeros(6),EquivalentCurrentSensor(D).measure(tau))
    close(got['torque'],tau,cfg,'Signed current conversion')
    try: adapter.update(0.,np.zeros(6),np.zeros(6),tau)
    except ValueError: pass
    else: raise RuntimeError('Duplicate timestamp accepted')
    nominal=module(base['oracle_source'],'unit_nominal')
    model=nominal.Model(base['model'],base['derivative_epsilon_s'])
    kernel=module(base['kf_source'],'unit_kernel')
    M,h,Md,J=model.evaluate(np.array(base['initial_q_rad']),np.zeros(6))
    F=np.zeros((12,12)); F[:6,6:]=J.T; Qw=np.diag(base['observer']['Qw_continuous_diag'])
    sigma=np.full(6,.05**2); errs=[]
    for dt in [.004,.008,.016]:
        Qpc=dt*np.diag(sigma); A,B,Q=kernel.discretize(J,dt,Qpc,Qw)
        G=np.zeros((12,12));G[:6,:6]=Qpc;G[6:,6:]=Qw
        E=expm(np.block([[-F,G],[np.zeros_like(F),F.T]])*dt)
        errs.append(close(Q/dt,(E[12:,12:].T@E[:12,12:])/dt,cfg,'Exact frozen-coefficient Q'))
        close((Q[:6,:6]-dt**3/3*(J.T@Qw@J))/dt**2,np.diag(sigma),cfg,'T squared input covariance')
        close(Q[:6,6:]/dt**2,J.T@Qw/2,cfg,'Q cross block')
        require(np.linalg.eigvalsh(Q)[0]>0,'Q positive definiteness')
    original=nominal.Observer(base); fixed=CaptureObserver(base,True); dynamic=CaptureObserver(base,True)
    variance=np.array(base['observer']['Qp_continuous_diag'])/.008; previous=None; records=[]; maxerr=0.
    for k in range(151):
        t=k*.008; q=np.array(base['initial_q_rad'])+.002*np.sin(t)*np.arange(1,7)
        v=.002*np.cos(t)*np.arange(1,7); tau=.1*np.cos(t)*np.arange(1,7)
        a=original.update(t,q,v,tau); b=fixed.update(t,q,v,tau,variance)
        varying=sigma*(1.+.2*np.sin(t+np.arange(6))); c=dynamic.update(t,q,v,tau,varying)
        ledger['KF_calls']+=3
        for key in a.keys() & b.keys(): maxerr=max(maxerr,close(a[key],b[key],cfg,'Baseline equivalence '+key))
        if k:
            close(c['used_sigma'],previous,cfg,'Previous variance is used')
            close(c['input_source_t'],t-.008,cfg,'Previous input time')
            close(c['Q'][:6,:6]-.008**3/3*(previous_J.T@Qw@previous_J),.008**2*np.diag(previous),cfg,'Dynamic variance block')
        else:
            close(c['covariance'],b['covariance'],cfg,'Shared initial P')
            close(c['wrench'],np.zeros(6),cfg,'Shared initial wrench')
        require(np.linalg.eigvalsh(c['covariance'])[0]>0,'Dynamic P not PD')
        previous=varying.copy();previous_J=c['J'].copy();records.append(c['used_sigma'])
    require(ledger['KF_calls']<=600,'Precheck budget')
    write(out/'checks.json',dict(passed=True,versions=versions,filter_check_count=len(checks['checks']),KF_calls=ledger['KF_calls'],baseline_max_difference=maxerr,expm_scaled_Q_max=max(errs),variance_time='previous feedback sample',truth_inputs=False))
    return dict(filter_check_count=len(checks['checks']))


def controller(cfg,base,out,ledger):
    require(json.loads((Path(cfg['run_root'])/'path/metrics.json').read_text())['passed'],'Path not passed')
    def eq(a,b,label): return close(a,b,cfg,label)
    M=np.diag([3.,2.,1.,.1,.01,.0002]);B=np.asarray(base['damping']);z=np.zeros(6)
    q=np.arange(6)*.002;v=np.arange(6)*.003;qr=q+.001;vr=v-.002;ar=np.arange(6)*.004;h=np.arange(6)*.1
    original=make_controller(cfg,base,scale=1.,rate=0.)
    expected=M@ar+h+B*v+np.asarray(base['controller']['kp_Nm_rad'])*(qr-q)+np.asarray(base['controller']['kd_Nm_s_rad'])*(vr-v)
    eq(original.update(q,v,qr,vr,ar,M,h,B)['command'],expected,'Restores original PD');eq(original.integral,z,'Disabled integral')
    c=make_controller(cfg,base);e=np.full(6,.0001);r=c.update(e,z,z,z,z,M,z,B)
    eq(r['I_used'],z,'Causal I');eq(r['command'],-c.kp*e,'Old I in command');eq(r['I_next'],-.001*c.ki*e,'Integral sign')
    eq(c.update(e,z,z,z,z,M,z,B)['I_used'],r['I_next'],'I order')
    for sign in [-1.,1.]:
        c=make_controller(cfg,base);e=np.full(6,-sign*.0001);bias=sign*(c.limits+1)
        r=c.update(e,z,z,z,z,M,bias,B);require(np.all(r['antiwindup_frozen']),'Outward freeze');eq(c.integral,z,'Frozen I')
        c=make_controller(cfg,base);e=np.full(6,sign*.0001)
        r=c.update(e,z,z,z,z,M,bias+sign*c.kp*.0001,B);require(not np.any(r['antiwindup_frozen']) and np.all(sign*c.integral<0),'Recovery I')
        c=make_controller(cfg,base);e=np.full(6,-sign*1000);c.update(e,z,z,z,z,M,c.kp*e,B);eq(c.integral,sign*c.imax,'I cap')
    c=make_controller(cfg,base);initial=np.linspace(-1e-5,1e-5,18);c.integral=initial[12:].copy()
    r=c.update(initial[:6],initial[6:12],z,z,z,M,z,B);e=initial[:6].copy();vel=initial[6:12].copy();hp=base['physics_dt_s']
    for _ in range(16): vel=np.linalg.solve(M+hp*np.diag(B),M@vel+hp*r['command']);e=e+hp*vel
    eq(frozen_closed_loop(M,B,c.kp,c.kd,c.ki,hp,base['control_dt_s'])@initial,np.r_[e,vel,c.integral],'Explicit recurrence')
    m=mujoco.MjModel.from_binary_path(base['model']);d=mujoco.MjData(m)
    require(m.opt.integrator==mujoco.mjtIntegrator.mjINT_IMPLICITFAST and m.opt.timestep==hp,'Integrator/step')
    eq(m.dof_damping,B,'Damping');require(not np.any(m.dof_armature) and not np.any(m.dof_frictionloss),'Unexpected passive terms')
    tr=load(Path(cfg['run_root'])/'path/trajectory.npz');rows=[];c=make_controller(cfg,base)
    require(len(tr['q'][::8])==751,'Static population')
    for k,q in enumerate(tr['q'][::8]):
        require(ledger['static_states']<751,'Static cap');ledger['static_states']+=1
        d.qpos[:]=q;d.qvel[:]=0;mujoco.mj_forward(m,d);M=np.zeros((6,6));mujoco.mj_fullM(m,M,d.qM)
        require(np.linalg.eigvalsh(M)[0]>0,'Mass not PD')
        ev=np.linalg.eigvals(frozen_closed_loop(M,B,c.kp,c.kd,c.ki,hp,base['control_dt_s']))
        require(np.isfinite(ev).all(),'Nonfinite poles');rho=float(abs(ev).max())
        rows.append(dict(time_s=float(tr['t'][::8][k]),spectral_radius=rho,passed=bool(rho<1)))
    csvwrite(out/'local_stability.csv',rows)
    write(out/'checks.json',dict(passed=all(r['passed'] for r in rows),states=len(rows),max_spectral_radius=max(r['spectral_radius'] for r in rows),scope='Frozen-state screening, not nonlinear stability proof',unit_checks=True))
    require(all(r['passed'] for r in rows),'Frozen stability screen failed')
    return dict(static_population='new path only, 751 states')
