"""Bounded UR5 experiments with separate feedback, truth, and causal replay."""
import argparse
import csv
import json
from pathlib import Path
import time
import mujoco
import numpy as np
from oracle_kf import Model,Observer,load


def write(path,value):
    def convert(x):
        if isinstance(x,np.ndarray):return x.tolist()
        if isinstance(x,np.generic):return x.item()
        raise TypeError(type(x).__name__)
    with path.open('x') as f:json.dump(value,f,default=convert,ensure_ascii=False,indent=2,allow_nan=False)


def require(ok,msg):
    if not ok:raise RuntimeError(msg)


def trajectory(t,cfg,moving):
    q0=np.array(cfg['initial_q_rad']);A=np.array(cfg['trajectory']['amplitude_rad']);w=2*np.pi*cfg['trajectory']['frequency_Hz']
    if not moving:return q0,np.zeros(6),np.zeros(6)
    return q0+A*(1-np.cos(w*t)),A*w*np.sin(w*t),A*w*w*np.cos(w*t)


def wrench(t,cfg,kind):
    f=np.zeros(6);start=cfg['force']['start_s']
    if t>=start and kind!='none':f[2]=cfg['force']['amplitude_N']*(np.sin(2*np.pi*cfg['force']['frequency_Hz']*(t-start)) if kind=='sine' else 1)
    return f


def jacobian(m,d,site):
    jp,jr=np.zeros((3,6)),np.zeros((3,6));mujoco.mj_jacSite(m,d,jp,jr,site);return np.vstack([jp,jr])


def conditioning(J):
    scaled=J.copy();scaled[:3]/=.5;s=np.linalg.svd(scaled,compute_uv=False);tol=np.finfo(float).eps*6*s[0]
    rank=int(np.sum(s>tol));require(rank==6,'Jacobian rank below six')
    return rank,float(s[-1]),float(s[0]/s[-1])


def moving_mask(m):
    mask=np.zeros(m.nbody,dtype=bool)
    for b in range(1,m.nbody):mask[b]=mask[m.body_parentid[b]] or m.body_dofnum[b]>0
    return mask


def valid(m,d,mask,command):
    require(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all() and np.isfinite(d.qacc).all(),'Nonfinite state')
    require(not np.any(d.warning.number),'MuJoCo warning')
    limited=m.jnt_limited.astype(bool)
    require(np.all(d.qpos[limited]>m.jnt_range[limited,0]) and np.all(d.qpos[limited]<m.jnt_range[limited,1]),'Joint limit')
    lim=m.actuator_ctrllimited.astype(bool)
    require(np.all(command[lim]>=m.actuator_ctrlrange[lim,0]) and np.all(command[lim]<=m.actuator_ctrlrange[lim,1]),'Torque saturation')
    for i in range(d.ncon):
        c=d.contact[i]
        if c.geom1>=0 and c.geom2>=0 and (mask[m.geom_bodyid[c.geom1]] or mask[m.geom_bodyid[c.geom2]]):
            require(not (c.dist<=0 or c.efc_address>=0),'Moving link contact')
    require(np.max(np.abs(d.qfrc_constraint))<=1e-9,'Unexpected constraint torque')
    require(np.allclose(d.qfrc_actuator,command,rtol=1e-8,atol=1e-9),'Actual motor torque differs')


def preflight(cfg,out):
    model=Model(cfg['model'],cfg['derivative_epsilon_s']);m=model.m;d=mujoco.MjData(m);mask=moving_mask(m)
    names=[m.joint(i).name for i in range(m.njnt)]
    require(names==cfg['joint_names'] and (m.nq,m.nv,m.nu,m.na)==(6,6,6,0),'Not expected UR5')
    require(np.array_equal(m.dof_damping,cfg['damping']) and not np.any(m.dof_armature) and not np.any(m.dof_frictionloss),'Physical model differs')
    require(m.opt.timestep==cfg['physics_dt_s'],'Physical step differs')
    require(m.opt.integrator==mujoco.mjtIntegrator.mjINT_IMPLICITFAST,'Integrator differs')
    require(not (m.opt.disableflags & (mujoco.mjtDisableBit.mjDSBL_CONTACT|mujoco.mjtDisableBit.mjDSBL_CONSTRAINT)),'Contact disabled')
    checks=[]
    for t in [0.,.25,.75,1.5,3.]:
        q,v,a=trajectory(t,cfg,True);M,h,Md,J=model.evaluate(q,v)
        d.qpos[:]=q;d.qvel[:]=v;d.ctrl[:]=h+model.B*v;mujoco.mj_forward(m,d)
        valid(m,d,mask,d.ctrl);rank,sv,cond=conditioning(J)
        Mdchecks=[]
        for factor in [.5,2.]:
            alt=model.evaluate(q,v,cfg['derivative_epsilon_s']*factor)[2]
            err=float(np.max(np.abs(alt-Md)));passed=bool(np.allclose(alt,Md,rtol=1e-4,atol=1e-9))
            Mdchecks.append(dict(factor=factor,max_abs=err,passed=passed))
        row=dict(t=t,rank=rank,min_scaled_sv=sv,condition=cond,derivative=Mdchecks)
        checks.append(row);write(out/f'probe_{len(checks)}.json',row)
        require(all(c['passed'] for c in Mdchecks),'Mdot derivative stability failed')
        for j in range(6):
            w=np.eye(6)[j];d.qfrc_applied[:]=0
            mujoco.mj_applyFT(m,d,w[:3],w[3:],d.site_xpos[model.site],int(m.site_bodyid[model.site]),d.qfrc_applied)
            require(np.allclose(d.qfrc_applied,J.T@w,rtol=1e-8,atol=1e-9),'Wrench/Jacobian sign mismatch')
        d.qfrc_applied[:]=0;mujoco.mj_forward(m,d)
        require(np.allclose(M@d.qacc+h,d.qfrc_actuator+d.qfrc_passive,rtol=1e-8,atol=1e-9),'Dynamics balance failed')
        require(np.allclose(d.qfrc_passive,-model.B*v,rtol=1e-8,atol=1e-9),'Friction convention failed')
    # Independent matrix exponential check of the existing frozen-J discretization.
    from scipy.linalg import expm
    kernel=load(cfg['kf_source'],'kernel');dt=cfg['observer_dt_s'];Qp=np.diag(cfg['observer']['Qp_continuous_diag']);Qw=np.diag(cfg['observer']['Qw_continuous_diag'])
    A,B,Q=kernel.discretize(J,dt,Qp,Qw);F=np.zeros((12,12));F[:6,6:]=J.T
    augmented=np.zeros((18,18));augmented[:12,:12]=F;augmented[:6,12:]=np.eye(6)
    E=expm(augmented*dt);require(np.allclose(A,E[:12,:12],rtol=1e-8,atol=1e-9) and np.allclose(B,E[:12,12:],rtol=1e-8,atol=1e-9),'A/B discretization failed')
    spectral=np.zeros((12,12));spectral[:6,:6]=Qp;spectral[6:,6:]=Qw
    V=expm(np.block([[F,spectral],[np.zeros_like(F),-F.T]])*dt)
    require(np.allclose(Q,V[:12,12:]@A.T,rtol=1e-8,atol=1e-9),'Q discretization failed')
    write(out/'checks.json',dict(passed=True,joint_names=names,states=checks,discretization=True,simulation_steps=0))
    return dict(passed=True,simulation_steps=0,test_read=False)


def simulate(cfg,name,out):
    case=cfg['cases'][name];m=mujoco.MjModel.from_binary_path(cfg['model']);d=mujoco.MjData(m)
    nominal=Model(cfg['model'],cfg['derivative_epsilon_s']);mask=moving_mask(m);site=m.site('tool0').id;body=int(m.site_bodyid[site])
    d.qpos[:]=cfg['initial_q_rad'];kp=np.array(cfg['controller']['kp_Nm_rad']);kd=np.array(cfg['controller']['kd_Nm_s_rad'])
    dt=cfg['physics_dt_s'];stride=round(cfg['observer_dt_s']/dt);steps=round(case['duration_s']/dt)
    measurements={k:[] for k in ['t','q','v','tau']};refs={k:[] for k in ['t','wrench','tau_ext','qacc','q_ref','J','command']}
    max_balance=0.;max_tracking=0.;minsv=float('inf');maxcond=0.;command=np.zeros(6);completed=0
    try:
        for n in range(steps+1):
            t=n*dt;d.qfrc_applied[:]=0;d.xfrc_applied[:]=0;mujoco.mj_forward(m,d)
            qr,vr,ar=trajectory(t,cfg,case['motion'])
            if n%stride==0:
                M,h,_,_=nominal.evaluate(d.qpos.copy(),d.qvel.copy())
                command=M@ar+h+nominal.B*d.qvel+kp*(qr-d.qpos)+kd*(vr-d.qvel);d.ctrl[:]=command
            w=wrench(t,cfg,case['force'])
            mujoco.mj_applyFT(m,d,w[:3],w[3:],d.site_xpos[site],body,d.qfrc_applied);mujoco.mj_forward(m,d)
            valid(m,d,mask,command)
            M=np.zeros((6,6));mujoco.mj_fullM(m,M,d.qM)
            balance=M@d.qacc+d.qfrc_bias-d.qfrc_actuator-d.qfrc_passive-d.qfrc_applied
            max_balance=max(max_balance,float(np.abs(balance).max()));require(np.allclose(balance,0,rtol=0,atol=1e-9),'Dynamics balance failed')
            max_tracking=max(max_tracking,float(np.abs(d.qpos-qr).max()))
            if n%stride==0:
                J=jacobian(m,d,site);_,sv,cond=conditioning(J);minsv=min(minsv,sv);maxcond=max(maxcond,cond)
                require(np.allclose(J.T@w,d.qfrc_applied,rtol=1e-8,atol=1e-9),'Force map differs')
                for k,val in zip(measurements,[t,d.qpos.copy(),d.qvel.copy(),d.qfrc_actuator.copy()]):measurements[k].append(val)
                for k,val in zip(refs,[t,w,d.qfrc_applied.copy(),d.qacc.copy(),qr,J,command.copy()]):refs[k].append(val)
            if n<steps:mujoco.mj_step(m,d);completed+=1
        checks=dict(passed=True,simulation_steps=completed,max_balance_Nm=max_balance,max_tracking_rad=max_tracking,min_scaled_sv=minsv,max_scaled_condition=maxcond,
                    minimum_rank=6,moving_contacts=0,saturation_steps=0,joint_limit_steps=0,engine_warnings=0)
    except Exception as exc:
        write(out/'failure.json',dict(error=repr(exc),t=float(d.time),q=d.qpos,v=d.qvel,command=command,simulation_steps=completed));raise
    finally:
        np.savez_compressed(out/'measurements.npz',**measurements);np.savez_compressed(out/'references.npz',**refs)
    write(out/'checks.json',checks);return checks


def replay(cfg,measurement_path,out):
    # Only the feedback file and fixed model/config enter this function.
    with np.load(measurement_path) as f:packet={k:f[k] for k in f.files}
    require(set(packet)=={'t','q','v','tau'},'Feedback boundary violated')
    obs=Observer(cfg);records={};mineig=float('inf');sym=0
    for t,q,v,tau in zip(packet['t'],packet['q'],packet['v'],packet['tau']):
        result=obs.update(float(t),q,v,tau)
        require(all(np.isfinite(x).all() for x in result.values()),'Nonfinite estimate')
        eig=float(np.linalg.eigvalsh(result['covariance']).min());require(eig>0,'Non-positive covariance')
        mineig=min(mineig,eig);sym=max(sym,float(np.abs(result['covariance']-result['covariance'].T).max()))
        for k,val in result.items():records.setdefault(k,[]).append(val)
    prefix=Observer(cfg);err=0
    for i in range(min(101,len(packet['t']))):
        r=prefix.update(float(packet['t'][i]),packet['q'][i],packet['v'][i],packet['tau'][i]);err=max(err,float(np.abs(r['wrench']-records['wrench'][i]).max()))
    require(err==0,'Noncausal replay')
    np.savez_compressed(out/'estimates.npz',t=packet['t'],**records)
    write(out/'replay_checks.json',dict(passed=True,prefix_error=err,minimum_covariance_eigenvalue=mineig,symmetry_error=sym,feedback_fields=list(packet)))


def evaluate(cfg,name,out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    ref=np.load(out/'references.npz');est=np.load(out/'estimates.npz');t=ref['t'];error=est['wrench']-ref['wrench']
    steady=t>=(.2 if cfg['cases'][name]['force']=='none' else 2.)
    result=dict(rmse_full=np.sqrt(np.mean(error**2,0)),rmse_steady=np.sqrt(np.mean(error[steady]**2,0)),bias_steady=error[steady].mean(0),
        peak_error_steady=np.abs(error[steady]).max(0),joint_torque_rmse=np.sqrt(np.mean((est['joint_external_torque']-ref['tau_ext'])**2,0)),
        units=['N','N','N','N m','N m','N m'],steady_start_s=.2 if cfg['cases'][name]['force']=='none' else 2.)
    if name=='sine':
        w=2*np.pi*cfg['force']['frequency_Hz'];X=np.column_stack([np.sin(w*(t[steady]-1)),np.cos(w*(t[steady]-1)),np.ones(steady.sum())])
        a,b,c=np.linalg.lstsq(X,est['wrench'][steady,2],rcond=None)[0];phase=np.arctan2(b,a)
        result['sine']=dict(amplitude_N=float(np.hypot(a,b)),phase_deg=float(np.degrees(phase)),equivalent_delay_s=float(-phase/w),offset_N=float(c))
    if name=='step':
        response={}
        for frac in [.1,.9]:
            hits=np.flatnonzero((t>=1)&(est['wrench'][:,2]>=3*frac));response[f't{int(frac*100)}_s']=float(t[hits[0]]-1) if len(hits) else None
        response.update(steady_mean_N=float(est['wrench'][steady,2].mean()),peak_N=float(est['wrench'][t>=1,2].max()));result['step']=response
    fig,axs=plt.subplots(2,3,figsize=(12,6),sharex=True)
    for j,ax in enumerate(axs.flat):
        ax.plot(t,ref['wrench'][:,j],label='Applied truth');ax.plot(t,est['wrench'][:,j],label='Oracle momentum KF')
        ax.set(xlabel='Time (s)',ylabel=['Fx (N)','Fy (N)','Fz (N)','Mx (N m)','My (N m)','Mz (N m)'][j]);ax.grid(alpha=.3)
    axs[0,0].legend();fig.suptitle(name);fig.tight_layout();fig.savefig(out/'wrench.png',dpi=150);plt.close(fig)
    write(out/'evaluation.json',result);return result


def experiments(cfg,out):
    rows=[];results={};count=0
    for name in cfg['cases']:
        folder=out/name;folder.mkdir();checks=simulate(cfg,name,folder);count+=checks['simulation_steps']
        replay(cfg,folder/'measurements.npz',folder);result=evaluate(cfg,name,folder);results[name]=result
        for j in range(6):rows.append(dict(case=name,axis=['Fx','Fy','Fz','Mx','My','Mz'][j],unit=result['units'][j],rmse_full=result['rmse_full'][j],rmse_steady=result['rmse_steady'][j],bias_steady=result['bias_steady'][j],peak_error_steady=result['peak_error_steady'][j],joint=j+1,joint_torque_rmse_Nm=result['joint_torque_rmse'][j]))
        print('completed',name,flush=True)
    with (out/'six_axis_errors.csv').open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    write(out/'evaluation.json',results)
    (out/'decision.md').write_text('# H-020 UR5真实动力学KF基线\n\nEXPLORE；结果待审阅。四场景共享固定参数，无训练、无噪声扫描。\n\n'
        '反馈仅t、q、v、实际驱动力矩；独立模型提供已知动力学和阻尼，外力真值只供评价。物理16kHz，控制及KF均1kHz，采用上一间隔输入和J。\n\n'
        '仅世界Fz激励3N，六维输出用于检查串扰，不能视为六方向独立验收。真实模型仍存在隐式阻尼与左端输入离散近似。\n\n'
        '各场景目录保存独立反馈、真值、估计、曲线、工程检查及指标；汇总见six_axis_errors.csv和evaluation.json。\n')
    return dict(passed=True,pass_semantics='engineering execution checks only, not force accuracy acceptance',simulation_steps=count,training_updates=0,test_read=False)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--phase',required=True);p.add_argument('--output-dir',required=True);p.add_argument('--preflight-dir');a=p.parse_args()
    cfg=json.loads(Path(a.config).read_text());out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=True);tick=time.perf_counter()
    try:
        if a.phase=='preflight':result=preflight(cfg,out)
        else:
            require(json.loads((Path(a.preflight_dir)/'metrics.json').read_text())['passed'],'Preflight failed');result=experiments(cfg,out)
        result['wall_seconds']=time.perf_counter()-tick;write(out/'metrics.json',result);print(json.dumps(result),flush=True)
    except Exception as exc:
        write(out/'metrics.json',dict(passed=False,error=repr(exc),wall_seconds=time.perf_counter()-tick));raise
