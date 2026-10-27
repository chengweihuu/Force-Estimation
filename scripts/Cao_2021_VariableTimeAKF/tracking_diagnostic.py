"""Read-only tracking diagnosis from stored measurement/control/diagnostic logs.
No MuJoCo import, physical step, KF update, or model evaluation.
"""
import argparse,json,time,traceback
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import csv


def write(p,x):
    with p.open('x') as f:json.dump(x,f,indent=2,ensure_ascii=False,allow_nan=False,default=lambda y:y.tolist() if isinstance(y,np.ndarray) else y.item())
def load(p):
    with np.load(p,allow_pickle=False) as f:return {k:f[k].copy() for k in f.files}
def csvwrite(p,rows):
    with p.open('x',newline='') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def require(ok,msg):
    if not ok:raise RuntimeError(msg)

def diagnose(cfg,out):
    source=Path(cfg['source_run']);m=load(source/'collect/measurements.npz');d=load(source/'collect/diagnostics.npz');c=load(source/'collect/control.npz');base=json.loads(Path(cfg['base_config']).read_text());old=json.loads((source/'collect/checks.json').read_text());t=m['t'];rtol=cfg['rtol'];atol=cfg['atol']
    def close(a,b,label):
        require(np.allclose(a,b,rtol=rtol,atol=atol),label);return float(np.max(abs(np.asarray(a)-np.asarray(b))))
    require(np.array_equal(t,d['t']),'Measurement/diagnostic times differ');require(len(t)==751 and len(c['t'])==6001,'Unexpected log lengths')
    ix=np.rint(t/.001).astype(int);close(t,c['t'][ix],'Feedback not a control update time');close(d['command'],c['command'][ix],'Command timestamp mismatch');close(d['q_ref'],c['q_ref'][ix],'Position target mismatch');close(d['v_ref'],c['v_ref'][ix],'Velocity target mismatch');close(m['v'],d['v_true'],'Velocity measurement differs from actual');close(d['tau_act'],d['command'],'Actual actuator differs from command')
    load_ix=int(np.flatnonzero(np.linalg.norm(d['wrench'],axis=1)>0)[0]);load_time=float(t[load_ix]);require(load_time==2.56,'Load time changed');close(np.einsum('nji,nj->ni',d['J'],d['wrench']),d['tau_ext'],'External torque mapping mismatch')
    eq=m['q']-d['q_ref'];ev=m['v']-d['v_ref'];kp=np.asarray(base['controller']['kp_Nm_rad']);kd=np.asarray(base['controller']['kd_Nm_s_rad']);p=-kp*eq;v=-kd*ev;pd=p+v;ext=d['tau_ext'];net=pd+ext;ff=d['command']-pd;static_e=ext/kp
    # ff is an algebraic remainder, not an independently recomputed rigid-body model.
    pos=d['tcp']-d['tcp_ref'];orient=d['orientation_error'];vel=d['tcp_velocity']-d['tcp_velocity_ref'];pn=np.linalg.norm(pos,axis=1);on=np.linalg.norm(orient,axis=1);vn=np.linalg.norm(vel,axis=1);core=(t>=2)&(t<5)
    close(pn[core].max(),old['position_peak_m'],'Prior position metric changed');close(on[core].max(),old['orientation_peak_rad'],'Prior orientation metric changed');close(np.sqrt(np.mean(vn[core]**2)),old['velocity_vector_rmse_m_s'],'Prior velocity metric changed')
    windows=cfg['windows'];phases=[];joint=[]
    for name,lo,hi in windows:
        mask=(t>=lo)&(t<hi) if name!='deceleration' else (t>=lo)&(t<=hi)
        require(mask.any(),'Empty phase');ids=np.flatnonzero(mask);pi=ids[np.argmax(pn[mask])];oi=ids[np.argmax(on[mask])];vi=ids[np.argmax(vn[mask])]
        phases.append(dict(phase=name,start_s=lo,end_s=hi,samples=int(mask.sum()),position_peak_m=float(pn[pi]),position_peak_time_s=float(t[pi]),orientation_peak_rad=float(on[oi]),orientation_peak_time_s=float(t[oi]),velocity_vector_rmse_m_s=float(np.sqrt(np.mean(vn[mask]**2))),velocity_error_peak_m_s=float(vn[vi]),velocity_peak_time_s=float(t[vi]),actual_vz_mean_m_s=float(d['tcp_velocity'][mask,2].mean()),actual_vz_min_m_s=float(d['tcp_velocity'][mask,2].min()),actual_vz_max_m_s=float(d['tcp_velocity'][mask,2].max()),position_mean_x_m=float(pos[mask,0].mean()),position_mean_y_m=float(pos[mask,1].mean()),position_mean_z_m=float(pos[mask,2].mean())))
        for j in range(6):
            joint.append(dict(phase=name,joint=j+1,kp_Nm_rad=float(kp[j]),kd_Nm_s_rad=float(kd[j]),q_error_mean_rad=float(eq[mask,j].mean()),q_error_peak_abs_rad=float(abs(eq[mask,j]).max()),velocity_error_rmse_rad_s=float(np.sqrt(np.mean(ev[mask,j]**2))),external_mean_Nm=float(ext[mask,j].mean()),P_mean_Nm=float(p[mask,j].mean()),D_mean_Nm=float(v[mask,j].mean()),net_PD_plus_external_mean_Nm=float(net[mask,j].mean()),net_PD_plus_external_rms_Nm=float(np.sqrt(np.mean(net[mask,j]**2))),external_over_kp_mean_rad=float(static_e[mask,j].mean()),offset_approx_gap_rms_rad=float(np.sqrt(np.mean((eq[mask,j]-static_e[mask,j])**2))),feedforward_remainder_mean_Nm=float(ff[mask,j].mean()),actuator_mean_Nm=float(d['tau_act'][mask,j].mean())))
    csvwrite(out/'phases.csv',phases);csvwrite(out/'joint_control.csv',joint)
    # Native sampling events, no interpolation or window selected from KF outcomes.
    events={'load_frame':load_ix,'next_feedback':load_ix+1,'core_position_peak':np.flatnonzero(core)[np.argmax(pn[core])],'core_orientation_peak':np.flatnonzero(core)[np.argmax(on[core])],'core_velocity_peak':np.flatnonzero(core)[np.argmax(vn[core])],'end_constant_speed':np.flatnonzero(core)[-1]}
    crossings={}
    for name,x,bound in [('position',pn,.002),('orientation',on,.01)]:
        hit=np.flatnonzero(core&(x>bound));crossings[name]=float(t[hit[0]]) if len(hit) else None
        if len(hit):events['first_'+name+'_violation']=int(hit[0])
    rows=[];joint_events=[]
    for name,k in events.items():
        rows.append(dict(event=name,time_s=float(t[k]),from_load_s=float(t[k]-load_time),position_error_m=float(pn[k]),orientation_error_rad=float(on[k]),velocity_error_m_s=float(vn[k]),actual_vz_m_s=float(d['tcp_velocity'][k,2]),target_vz_m_s=float(d['tcp_velocity_ref'][k,2])))
        for j in range(6):joint_events.append(dict(event=name,time_s=float(t[k]),joint=j+1,q_error_rad=float(eq[k,j]),velocity_error_rad_s=float(ev[k,j]),qacc_rad_s2=float(d['qacc'][k,j]),reference_acceleration_rad_s2=float(c['a_ref'][ix[k],j]),P_Nm=float(p[k,j]),D_Nm=float(v[k,j]),external_Nm=float(ext[k,j]),net_PD_plus_external_Nm=float(net[k,j]),actuator_Nm=float(d['tau_act'][k,j])))
    csvwrite(out/'events.csv',rows);csvwrite(out/'joint_events.csv',joint_events)
    limits=np.asarray(cfg['torque_limits_Nm']);torques=[]
    for j in range(6):
        k=int(np.argmax(abs(c['command'][:,j])));torques.append(dict(joint=j+1,limit_Nm=float(limits[j]),command_peak_abs_Nm=float(abs(c['command'][k,j])),peak_time_s=float(c['t'][k]),fraction_of_limit=float(abs(c['command'][k,j])/limits[j])))
    csvwrite(out/'torque_limits.csv',torques)
    np.savez_compressed(out/'tracking_components.npz',t=t,q_error=eq,velocity_error=ev,position_error=pos,orientation_error=orient,tcp_velocity_error=vel,P=p,D=v,external=ext,net=net,feedforward_remainder=ff,static_offset_approx=static_e)
    fig,ax=plt.subplots(4,1,figsize=(11,11),sharex=True)
    ax[0].plot(t,pn*1000,label='Position error');ax[0].axhline(2,color='red',ls='--');ax[0].set_ylabel('Position / mm')
    ax[1].plot(t,on,label='Orientation error');ax[1].axhline(.01,color='red',ls='--');ax[1].set_ylabel('Orientation / rad')
    ax[2].plot(t,d['tcp_velocity'][:,2],label='Actual vz');ax[2].plot(t,d['tcp_velocity_ref'][:,2],'k--',label='Target vz');ax[2].set_ylabel('Vz / m/s')
    for j in range(6):ax[3].plot(t,eq[:,j],label='J'+str(j+1))
    ax[3].set_ylabel('q - q_ref / rad');ax[3].set_xlabel('Time / s')
    for a in ax:a.axvline(load_time,c='gray',ls=':');a.grid(alpha=.2);a.legend(fontsize=8)
    fig.tight_layout();fig.savefig(out/'tracking_phases.png',dpi=140);plt.close(fig)
    fig,ax=plt.subplots(3,2,figsize=(13,10),sharex=True)
    for j,a in enumerate(ax.flat):
        for arr,label in [(ext,'External'),(p,'P feedback'),(v,'D feedback'),(net,'PD + external')]:a.plot(t,arr[:,j],label=label,lw=.9)
        a.axvline(load_time,c='gray',ls=':');a.set_ylabel('J'+str(j+1)+' / N m');a.grid(alpha=.2);a.legend(fontsize=7)
    fig.tight_layout();fig.savefig(out/'joint_torque_balance.png',dpi=140);plt.close(fig)
    summary=dict(load_time_s=load_time,crossings=crossings,phases=phases,events=rows,torque_limits=torques,core_metric_reproduction=True,velocity_measurement_noise_max=float(abs(m['v']-d['v_true']).max()),full_input_samples=len(t),control_samples=len(c['t']),source_snapshots_unchanged=True)
    write(out/'summary.json',summary)
    report=['# 竖直加载场景：只读运动跟踪诊断','','统计来自已保存751帧反馈和6001条控制命令；不运行MuJoCo、不求新模型量、不重放KF。原运动守门不修改。','','## 分阶段结果','','|阶段|位置峰值/mm|姿态峰值/rad|速度误差向量RMSE/(m/s)|平均实际Vz/(m/s)|','|---|---:|---:|---:|---:|']
    for r in phases:report.append(f"|{r['phase']}|{r['position_peak_m']*1000:.6g}|{r['orientation_peak_rad']:.6g}|{r['velocity_vector_rmse_m_s']:.6g}|{r['actual_vz_mean_m_s']:.6g}|")
    report+=['','## 连续控制关系','','代码控制器为准确模型前馈加PD，没有积分或未知外力前馈。设 $e=q-q_{ref}$，$\\dot e=v-v_{ref}$，则当前采样状态上：','','$$','\\tau_a=M a_{ref}+h+Bv-K_pe-K_d\\dot e.','$$','','结合采样状态上的连续动力学关系：','','$$','M(a-a_{ref})=-K_pe-K_d\\dot e+\\tau_{ext}.','$$','','右侧全部可由日志计算；左侧的M未保存在本日志，本轮未独立重算它，不把右侧构造称为新的完整动力学闭合验证。控制在1 ms区间内保持，以上采样状态关系不能取代整个区间的精确积分。采样帧与控制更新帧已核对重合。','','若加速度跟踪差和速度误差都较小，则 $e\\approx K_p^{-1}\\tau_{ext}$。该式只是局部准静态近似，不意味着固定姿态、不包含独立因果辨识，也不表示提高增益必然满足全部性能。','','## 持续加载匀速段3.56≤t<5 s','','|关节|平均位置误差/rad|平均外力矩/N·m|P反馈/N·m|D反馈/N·m|PD+外力矩/N·m|外力矩/Kp近似/rad|','|---|---:|---:|---:|---:|---:|---:|']
    for r in joint:
        if r['phase']=='loaded_steady':report.append('|J'+str(r['joint'])+'|'+'|'.join(f"{r[k]:.7g}" for k in ['q_error_mean_rad','external_mean_Nm','P_mean_Nm','D_mean_Nm','net_PD_plus_external_mean_Nm','external_over_kp_mean_rad'])+'|')
    report+=['','## 事件','','|事件|时间/s|距加载/s|位置误差/mm|姿态误差/rad|实际Vz/(m/s)|','|---|---:|---:|---:|---:|---:|']
    for r in rows:report.append(f"|{r['event']}|{r['time_s']:.3f}|{r['from_load_s']:.3f}|{r['position_error_m']*1000:.6g}|{r['orientation_error_rad']:.6g}|{r['actual_vz_m_s']:.6g}|")
    report+=['','所有组别的KF性能尚未评价；不能从本轮跟踪偏差认定MSMA失败。控制前馈余项由实际命令减PD得到，仅作代数分解，不冒充独立计算的重力/惯性参考。无负载对照新轨迹，不能严格分离时间、姿态变化与载荷的每一项独立作用。','','![跟踪阶段](tracking_phases.png)','','![逐轴力矩](joint_torque_balance.png)','','完整六轴分阶段、事件、限幅余量分别见joint_control.csv、joint_events.csv、torque_limits.csv。']
    (out/'decision.md').write_text('\n'.join(report)+'\n')
    return dict(passed=True,physics_steps=0,KF_calls=0,model_evaluations=0,feedback_samples=len(t),control_samples=len(c['t']))


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--config',type=Path,required=True);ap.add_argument('--output-dir',type=Path,required=True);a=ap.parse_args();cfg=json.loads(a.config.read_text());out=a.output_dir;out.mkdir(parents=True,exist_ok=True);start=time.perf_counter()
    try:
        root=Path(cfg['source_run']);s=json.loads((root/'workflow-state.json').read_text())
        for item in s['inputSnapshots']:require((Path('/workspace')/item['path']).read_bytes()==(root/item['snapshot']).read_bytes(),'Original input changed: '+item['path'])
        result=diagnose(cfg,out);write(out/'metrics.json',dict(**result,wall_seconds=time.perf_counter()-start))
    except Exception as e:
        write(out/'failure.json',dict(error=repr(e),traceback=traceback.format_exc()));write(out/'metrics.json',dict(passed=False,physics_steps=0,KF_calls=0,model_evaluations=0,wall_seconds=time.perf_counter()-start));raise
if __name__=='__main__':main()
