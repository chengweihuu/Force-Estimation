"""Single-joint constant-speed current residual calibration; no KF."""
import argparse, json, time, traceback
from pathlib import Path
import numpy as np
import mujoco
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from runner import load as load_module, sample, save_series
from input_statistics import write, csvwrite, require, load
from msma import process, wma_weights, hma_weights


def trajectory(t, base, case):
    # One-second smooth velocity ramps, 2..7 s constant speed, all else fixed.
    v0=case['speed']; q=np.array(base['initial_q_rad'],dtype=float);v=np.zeros(6);a=np.zeros(6)
    j=case['joint']-1
    if t<1: x=vel=acc=0.
    elif t<2:
        u=t-1;x=v0*(u/2-np.sin(np.pi*u)/(2*np.pi));vel=v0*(1-np.cos(np.pi*u))/2;acc=v0*np.pi*np.sin(np.pi*u)/2
    elif t<7:x=v0*(.5+t-2);vel=v0;acc=0.
    else:
        u=min(t-7,1.);x=v0*(5.5+u/2+np.sin(np.pi*u)/(2*np.pi));vel=v0*(1+np.cos(np.pi*u))/2;acc=-v0*np.pi*np.sin(np.pi*u)/2
    q[j]+=x;v[j]=vel;a[j]=acc
    return q,v,a


def environment(cfg):
    base=json.loads(Path(cfg['base_config']).read_text())
    oracle=load_module(base['oracle_source'],'nominal_model')
    helpers=load_module(base['simulation_helpers_source'],'simulation_helpers')
    m=mujoco.MjModel.from_binary_path(base['model'])
    require(m.nq==m.nv==m.nu==6,'Six-axis direct drive required')
    require(abs(m.opt.timestep-base['physics_dt_s'])<1e-15,'Physical step mismatch')
    require(np.allclose(m.dof_damping,base['damping'],rtol=0,atol=1e-15),'Damping mismatch')
    require(np.array_equal(m.actuator_gear[:,0],np.ones(6)) and not np.any(m.dof_armature),'Drive/armature mismatch')
    require([m.joint(j).name for j in range(6)]==base['joint_names'],'Joint order mismatch')
    return base,oracle,helpers


def precheck(cfg,out):
    base,oracle,helpers=environment(cfg)
    # Fixed analytic trajectory continuity and derivative checks, no physical stepping.
    for case in cfg['cases']:
        for t in [0.,1.,2.,3.,6.,7.,8.]:
            q,v,a=trajectory(t,base,case);require(np.isfinite(q).all(),'Nonfinite trajectory')
            if t in [1.,2.,7.]:
                ql,vl,al=trajectory(t-1e-8,base,case);qr,vr,ar=trajectory(t+1e-8,base,case)
                require(np.max(abs(qr-ql))<2e-9 and np.max(abs(vr-vl))<2e-9 and np.max(abs(ar-al))<2e-8,'Ramp discontinuity')
        q,v,a=trajectory(4.,base,case)
        require(v[case['joint']-1]==case['speed'] and not np.any(a),'Constant speed plateau missing')
    require(cfg['history_count']>=max(cfg['windows'])+int(np.sqrt(max(cfg['windows'])))-1,'Insufficient HMA real history')
    write(out/'precheck.json',dict(versions=dict(mujoco=mujoco.__version__,numpy=np.__version__),cases=len(cfg['cases']),physics_dt_s=base['physics_dt_s'],feedback_dt_s=base['observer_dt_s'],reference='bias(q,v)+B*v, positive drive convention'))
    return dict(new_physics_steps=0)


def simulate(cfg,case,out,base,oracle,helpers,ledger):
    out.mkdir(parents=True,exist_ok=True)
    m=mujoco.MjModel.from_binary_path(base['model']);d=mujoco.MjData(m)
    controller=oracle.Model(base['model'],base['derivative_epsilon_s'])
    d.qpos[:]=base['initial_q_rad'];mask=helpers.moving_mask(m)
    kp=np.asarray(base['controller']['kp_Nm_rad']);kd=np.asarray(base['controller']['kd_Nm_s_rad']);B=m.dof_damping.copy()
    dt=base['physics_dt_s'];steps=round(cfg['duration_s']/dt);cs=round(base['control_dt_s']/dt);ss=round(base['observer_dt_s']/dt)
    records={};command=np.zeros(6);tracking=np.zeros(6);peak=np.zeros(6);balance_max=0.;ctrl_times=[]
    try:
        for k in range(steps+1):
            t=k*dt;qr,vr,ar=trajectory(t,base,case)
            d.qfrc_applied[:]=0;d.xfrc_applied[:]=0
            if k%cs==0:
                M,h,_,_=controller.evaluate(d.qpos.copy(),d.qvel.copy())
                command=M@ar+h+B*d.qvel+kp*(qr-d.qpos)+kd*(vr-d.qvel);d.ctrl[:]=command;ctrl_times.append(t)
            mujoco.mj_forward(m,d);helpers.valid(m,d,mask,command)
            tracking=np.maximum(tracking,abs(d.qpos-qr));peak=np.maximum(peak,abs(d.qfrc_actuator))
            if k%ss==0:
                M=np.zeros((6,6));mujoco.mj_fullM(m,M,d.qM);inertial=M@d.qacc
                reference=d.qfrc_bias+B*d.qvel
                balance=inertial+reference-d.qfrc_actuator
                require(np.allclose(balance,0,rtol=0,atol=cfg['atol']),'Instantaneous balance failed')
                balance_max=max(balance_max,float(np.max(abs(balance))))
                sample(records,t=t,q=d.qpos,v=d.qvel,qacc=d.qacc,qref=qr,vref=vr,
                    tau_act=d.qfrc_actuator,tau_ref_no_acc=reference,inertial=inertial,
                    current=d.qfrc_actuator/np.array(cfg['D']),ctrl=command)
            if k<steps:
                require(ledger['steps']<cfg['new_physics_cap'],'Physical step cap reached')
                mujoco.mj_step(m,d);ledger['steps']+=1
            if k and k%32000==0:print(json.dumps(dict(case=case['name'],time=t,steps=ledger['steps'])),flush=True)
    finally:
        save_series(out/'measurements.npz',records)
        write(out/'physics_ledger.json',ledger)
    require(np.allclose(np.diff(ctrl_times),base['control_dt_s'],rtol=cfg['rtol'],atol=cfg['atol']),'Control time mismatch')
    require(np.allclose(np.diff(records['t']),base['observer_dt_s'],rtol=cfg['rtol'],atol=cfg['atol']),'Feedback time mismatch')
    write(out/'guards.json',dict(passed=True,steps=steps,tracking_max_rad=tracking,peak_torque_Nm=peak,balance_max_Nm=balance_max,contacts=0,saturation=0))


def collect(cfg,out,pilot):
    base,oracle,helpers=environment(cfg);ledger=dict(steps=0)
    selected=[c for c in cfg['cases'] if (c['name']==cfg['pilot_case'])==pilot]
    try:
        for case in selected:simulate(cfg,case,out/case['name'],base,oracle,helpers,ledger)
    finally:write(out/'physics_ledger.json',ledger)
    return dict(new_physics_steps=ledger['steps'],cases=len(selected))


def scalar_stats(x):
    return dict(mean_Nm=float(x.mean()),variance_Nm2=float(x.var(ddof=0)),sample_variance_Nm2=float(x.var(ddof=1)),rmse_Nm=float(np.sqrt(np.mean(x*x))))


def fixed_apply(x,modes,lengths,N,history):
    y=np.zeros_like(x[history:]);kernel=hma_weights(N)
    for k in range(len(y)):
        for j in range(6):
            w=wma_weights(int(lengths[k,j])) if modes[k,j]==1 else kernel
            ix=history+k-np.arange(len(w));y[k,j]=w@x[ix,j]
    return y


def analyze(cfg,out):
    root=Path(cfg['data_root']);delta=json.loads(Path(cfg['threshold_file']).read_text())['threshold_Aeq']
    all_rows=[];checks=[];summaries=[]
    D=np.asarray(cfg['D']);h=cfg['history_count']
    for case in cfg['cases']:
        folder='pilot' if case['name']==cfg['pilot_case'] else 'collect'
        s=load(root/folder/case['name']/'measurements.npz');t=s['t'][h:]
        mask=(t>=cfg['core_start_s'])&(t<cfg['core_end_s']);require(mask.sum()==500,'Core sample mismatch')
        noise=np.random.default_rng(case['seed']).normal(size=s['current'].shape)*np.asarray(cfg['sigma_Nm'])/D
        noisy=s['current']+noise
        dest=out/case['name'];dest.mkdir();save_series(dest/'noise.npz',dict(noise=noise,t=s['t']))
        ref=s['tau_ref_no_acc'][h:];tau=s['tau_act'][h:];inertial=s['inertial'][h:]
        speederr=s['v'][h:]-s['vref'][h:]
        for j in range(6):
            checks.append(dict(case=case['name'],moving_joint=case['joint'],joint=j+1,
                speed_error_rms_rad_s=float(np.sqrt(np.mean(speederr[mask,j]**2))),
                qacc_rms_rad_s2=float(np.sqrt(np.mean(s['qacc'][h:][mask,j]**2))),
                omitted_inertia_rms_Nm=float(np.sqrt(np.mean(inertial[mask,j]**2))),
                reference_actuator_rmse_Nm=float(np.sqrt(np.mean((ref[mask,j]-tau[mask,j])**2)))))
        variants=[('raw',1)]+[(method,N) for N in cfg['windows'] for method in ['wma','hma','msma']]
        for method,N in variants:
            if method=='raw':f=noisy[h:];clean=s['current'][h:];prop=noise[h:];resets=np.zeros_like(f,dtype=bool)
            else:
                r=process(noisy,method,N,delta,h);f=r['current'];resets=r['reset']
                clean=fixed_apply(s['current'],r['mode'],r['length'],N,h)
                prop=fixed_apply(noise,r['mode'],r['length'],N,h)
                require(np.allclose(f,clean+prop,rtol=cfg['rtol'],atol=cfg['atol']),'Filter split failed')
            eps=ref-D*f;error=D*f-tau;dist=D*clean-tau;pn=D*prop
            require(np.allclose(eps,-inertial-error,rtol=cfg['rtol'],atol=cfg['atol']),'Paper residual/inertia relation failed')
            require(np.allclose(error,dist+pn,rtol=cfg['rtol'],atol=cfg['atol']),'Input split failed')
            require(np.allclose(np.mean(error[mask]**2,axis=0),np.mean(dist[mask]**2+pn[mask]**2+2*dist[mask]*pn[mask],axis=0),rtol=cfg['rtol'],atol=cfg['atol']),'MSE split failed')
            for j in range(6):
                row=dict(case=case['name'],moving_joint=case['joint'],speed_rad_s=case['speed'],joint=j+1,method=method,window=N,samples=int(mask.sum()),resets_in_core=int(resets[mask,j].sum()))
                row.update({'residual_'+k:v for k,v in scalar_stats(eps[mask,j]).items()})
                row.update(input_RMSE_Nm=float(np.sqrt(np.mean(error[mask,j]**2))),distortion_RMS_Nm=float(np.sqrt(np.mean(dist[mask,j]**2))),noise_RMS_Nm=float(np.sqrt(np.mean(pn[mask,j]**2))),cross_Nm2=float(2*np.mean(dist[mask,j]*pn[mask,j])))
                all_rows.append(row)
            if method=='msma':save_series(dest/f'msma_{N}.npz',dict(t=t,core=mask,filtered=f,mode=r['mode'],length=r['length'],reset=resets,residual=eps,distortion=dist,propagated_noise=pn))
        print(json.dumps(dict(analyzed=case['name'])),flush=True)
    csvwrite(out/'all_axes_windows.csv',all_rows);csvwrite(out/'constant_speed_checks.csv',checks)
    # Candidate minima are descriptive grid summaries, never default settings.
    fig,axes=plt.subplots(3,2,figsize=(12,10))
    for j in range(1,7):
        ax=axes.flat[j-1]
        for sign in [1,-1]:
            rows=[r for r in all_rows if r['moving_joint']==j and r['joint']==j and r['speed_rad_s']*sign>0 and r['method']=='msma']
            raw=next(r for r in all_rows if r['moving_joint']==j and r['joint']==j and r['speed_rad_s']*sign>0 and r['method']=='raw')
            vmin=min(rows,key=lambda r:r['residual_variance_Nm2']);rmin=min(rows,key=lambda r:r['residual_rmse_Nm']);fifty=next(r for r in rows if r['window']==50)
            summaries.append(dict(joint=j,speed_rad_s=sign*cfg['speed_abs'],raw_RMSE_Nm=raw['residual_rmse_Nm'],N50_RMSE_Nm=fifty['residual_rmse_Nm'],N50_bias_Nm=fifty['residual_mean_Nm'],N50_variance_Nm2=fifty['residual_variance_Nm2'],min_variance_window=vmin['window'],min_RMSE_window=rmin['window'],min_RMSE_Nm=rmin['residual_rmse_Nm']))
            label='+' if sign>0 else '-';ax.plot([r['window'] for r in rows],[r['residual_rmse_Nm'] for r in rows],marker='o',label=label+' RMSE')
            ax.plot([r['window'] for r in rows],[np.sqrt(r['residual_variance_Nm2']) for r in rows],ls='--',label=label+' centered SD')
        ax.set_title(f'J{j}: moving joint');ax.set_xlabel('Maximum window / samples');ax.set_ylabel('N m');ax.grid(alpha=.3);ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(out/'window_curves.png',dpi=150);plt.close(fig)
    csvwrite(out/'moving_joint_summary.csv',summaries)
    # Equal-weight positive/negative mean-square aggregate, not centered pooled variance.
    agg=[]
    for j in range(1,7):
        for N in cfg['windows']:
            rows=[r for r in all_rows if r['moving_joint']==j and r['joint']==j and r['window']==N and r['method']=='msma']
            agg.append(dict(joint=j,window=N,mean_within_variance_Nm2=float(np.mean([r['residual_variance_Nm2'] for r in rows])),paired_RMSE_Nm=float(np.sqrt(np.mean([r['residual_rmse_Nm']**2 for r in rows])))))
    csvwrite(out/'paired_directions.csv',agg)
    report=['# 单关节匀速窗口标定','',
        'EXPLORE；六轴逐个±0.05 rad/s，无外力。每段8 s，0–1 s静止，1–2 s平滑加速，2–7 s匀速，7–8 s减速。只评价[3,7) s的500帧，125 Hz。',
        '前128帧为真实历史，滤波从1.024 s开始；所有窗口在同一时间起步、评价同一核心段。保留加减速数据，但不计入标定。',
        '模型参考为反馈状态上的bias(q,v)+Bv，符号采用本项目正驱动力矩；不将指令速度代替实际速度。实际加速度仅用来检查忽略惯性项的误差。',
        '噪声0.05 A_eq、D=1 N·m/A_eq；各段固定种子，同段不同窗口/方法共用噪声。阈值沿用原N=50噪声标定值以单独考察窗口，不是作者的实测阈值；本轮不重新优化。',
        '', '|轴|速度rad/s|原残差RMSE|50点RMSE|50点均值|方差最低窗口|RMSE最低窗口|最低RMSE|', '|---|---:|---:|---:|---:|---:|---:|---:|']
    for r in summaries:
        report.append(f"|J{r['joint']}|{r['speed_rad_s']:+.2f}|{r['raw_RMSE_Nm']:.6g}|{r['N50_RMSE_Nm']:.6g}|{r['N50_bias_Nm']:.6g}|{r['min_variance_window']}|{r['min_RMSE_window']}|{r['min_RMSE_Nm']:.6g}|")
    report+=['','力矩幅值单位N·m，方差(N·m)²。最小值仅指已测网格上的描述性最小值，不是部署推荐，不据单次样本宣布最优。正负方向分开报告；paired_directions是方向等权均方汇总，避免正负偏置相消。',
        'all_axes_windows.csv覆盖所有静止和运动关节；constant_speed_checks.csv保存实际速度偏差、实际加速度及省略惯性力矩。',
        '该标定使用准确仿真模型和等效电流，不包括真实电流增益辨识或模型误差，不等价于实机标定。未运行KF，未在原动态日志上选择窗口或检查泛化。',
        '![窗口曲线](window_curves.png)','']
    (out/'decision.md').write_text('\n'.join(report))
    return dict(new_physics_steps=0,rows=len(all_rows),cases=len(cfg['cases']))


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--config',type=Path,required=True);ap.add_argument('--output-dir',type=Path,required=True);ap.add_argument('--mode',choices=['precheck','pilot','collect','analyze'],required=True)
    a=ap.parse_args();cfg=json.loads(a.config.read_text());out=a.output_dir;out.mkdir(parents=True,exist_ok=True);start=time.perf_counter()
    try:
        if a.mode=='precheck':result=precheck(cfg,out)
        elif a.mode in ['pilot','collect']:result=collect(cfg,out,a.mode=='pilot')
        else:result=analyze(cfg,out)
        write(out/'metrics.json',dict(passed=True,new_KF_calls=0,wall_seconds=time.perf_counter()-start,**result))
    except Exception as exc:
        write(out/'failure.json',dict(error=repr(exc),traceback=traceback.format_exc()))
        write(out/'metrics.json',dict(passed=False,wall_seconds=time.perf_counter()-start));raise
if __name__=='__main__':main()
