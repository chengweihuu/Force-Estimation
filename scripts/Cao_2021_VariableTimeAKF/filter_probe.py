"""Filter/noise probe and short KF integration, separate from original interface run."""
import argparse
import csv
import json
from pathlib import Path
import signal
import time
import traceback

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from current_interface import CurrentObserver, load_current_packet
from msma import process, linear_series, wma_weights, hma_weights
from filter_checks import run_checks
from runner import load, write, save_series


def require(ok, reason):
    if not ok:
        raise RuntimeError(reason)


def csv_write(path, rows):
    with Path(path).open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)


def stats(error):
    e=np.asarray(error)
    centered=e-e.mean(0)
    denominator=np.sum(centered*centered,axis=0)
    correlation=np.divide(np.sum(centered[1:]*centered[:-1],axis=0),denominator,
                          out=np.full_like(denominator,np.nan),where=denominator>0)
    return dict(rmse=np.sqrt(np.mean(e*e,0)), mean=np.mean(e,0),
                variance=np.var(e,axis=0,ddof=1),
                acf1=correlation)


def calibration(cfg,out):
    n,h=cfg['window'],cfg['history_count']; count=cfg['noise_calibration_samples']
    sigma=np.asarray(cfg['sigma_torque_Nm'])/np.asarray(cfg['D_Nm_per_Aeq'])
    x=np.random.default_rng(cfg['calibration_seed']).standard_normal((h+count,6))*sigma
    H=linear_series(x,hma_weights(n)); difference=x[h:]-H[h-1:-1]
    center=np.median(difference,axis=0)
    scale=1.4826*np.median(np.abs(difference-center),axis=0)
    delta=np.abs(center)+cfg['threshold_multiplier']*scale
    require(np.isfinite(delta).all() and np.all(delta>0),'Invalid calibrated threshold')
    write(out/'calibration.json',dict(threshold_Aeq=delta,median_Aeq=center,robust_scale_Aeq=scale,
          multiplier=cfg['threshold_multiplier'],window=n,seed=cfg['calibration_seed'],
          samples_per_axis=count,scope='known stationary white measurement noise only; not robot dynamic residual calibration'))
    rows=[]
    kernels=[('raw',1,np.ones(1))]+[('wma',k,wma_weights(k)) for k in range(1,n+1)]+[('hma',n,hma_weights(n))]
    for method,length,kernel in kernels:
        y=linear_series(x,kernel)[h:]*np.asarray(cfg['D_Nm_per_Aeq'])
        s=stats(y); theoretical=np.square(cfg['sigma_torque_Nm'])*np.sum(kernel*kernel)
        require(np.isfinite(s['variance']).all() and np.all(s['variance']>0),'Invalid noise variance')
        for j in range(6):
            rows.append(dict(method=method,length=length,joint=j+1,mean_Nm=s['mean'][j],
                sample_variance_Nm2=s['variance'][j],theory_variance_Nm2=theoretical[j],
                ratio_to_theory=s['variance'][j]/theoretical[j],lag1_autocorrelation=s['acf1'][j],
                proposed_Qp_continuous=s['variance'][j]*cfg['dt_s'],
                proposed_discrete_momentum_variance=s['variance'][j]*cfg['dt_s']**2))
    csv_write(out/'window_variances.csv',rows)
    z=np.random.default_rng(cfg['heldout_noise_seed']).standard_normal((h+count,6))*sigma
    holdout=[]
    for method in cfg['methods']:
        result=process(z,method,n,delta,h); s=stats(result['current']*np.asarray(cfg['D_Nm_per_Aeq']))
        for j in range(6):
            holdout.append(dict(method=method,joint=j+1,rmse_Nm=s['rmse'][j],variance_Nm2=s['variance'][j],
                lag1_autocorrelation=s['acf1'][j],resets=int(result['reset'][:,j].sum()),
                WMA_fraction=float(np.mean(result['mode'][:,j]==1)),HMA_fraction=float(np.mean(result['mode'][:,j]==2))))
    csv_write(out/'heldout_noise.csv',holdout)
    save_series(out/'noise_fixture.npz',dict(calibration_noise_Aeq=x,heldout_noise_Aeq=z))
    metrics=[]; resets=[]
    fig,ax=plt.subplots(6,2,figsize=(13,13),sharex=True)
    for column,case in enumerate(cfg['cases']):
        source=Path(cfg['source_run'])/'probe'/case
        packet=load_current_packet(source/'current_measurements.npz')
        clean=packet['current']
        noise=np.random.default_rng(cfg['measurement_seed']).standard_normal(clean.shape)*sigma
        noisy=clean+noise
        t=packet['t'][h:]
        path=out/case;path.mkdir()
        sequences=dict(t=t,q=packet['q'][h:],v=packet['v'][h:],clean_current=clean[h:],
                       noisy_current=noisy[h:],prehistory_t=packet['t'][:h],prehistory_noisy_current=noisy[:h])
        for method in cfg['methods']:
            noisy_result=process(noisy,method,n,delta,h)
            clean_result=process(clean,method,n,delta,h)
            estimate=noisy_result['current']
            D=np.asarray(cfg['D_Nm_per_Aeq'])
            error=(estimate-clean[h:])*D
            distortion=(clean_result['current']-clean[h:])*D
            noise_effect=(estimate-clean_result['current'])*D
            require(np.allclose(error,distortion+noise_effect,rtol=cfg['rtol'],atol=cfg['atol']),'Filter error decomposition failed')
            for key,value in noisy_result.items():sequences[method+'_'+key]=value
            sequences[method+'_clean_output']=clean_result['current']
            for window,mask in [('full',np.ones(len(t),dtype=bool)),('after_2s',t>=2)]:
                s=stats(error[mask]); l=stats(distortion[mask]); ne=stats(noise_effect[mask])
                for j in range(6):
                    mode=noisy_result['mode'][mask,j]
                    metrics.append(dict(case=case,window=window,method=method,joint=j+1,
                        samples=int(mask.sum()),start_s=t[mask][0],end_s=t[mask][-1],
                        total_torque_rmse_Nm=s['rmse'][j],bias_Nm=s['mean'][j],
                        clean_filter_distortion_rmse_Nm=l['rmse'][j],
                        noise_and_switching_change_rmse_Nm=ne['rmse'][j],
                        resets=int(noisy_result['reset'][mask,j].sum()),WMA_fraction=float(np.mean(mode==1)),
                        HMA_fraction=float(np.mean(mode==2))))
            for k,j in np.argwhere(noisy_result['reset']):
                resets.append(dict(case=case,method=method,t_s=t[k],joint=int(j+1),
                                   input_current_Aeq=noisy[h+k,j],output_current_Aeq=estimate[k,j]))
            for j in range(6):
                ax[j,column].plot(t,error[:,j],label=method,lw=.8,alpha=.85)
                ax[j,column].set_ylabel(f'J{j+1} error [N m]')
                ax[j,column].grid(alpha=.25)
        save_series(path/'filtered_sequences.npz',sequences)
        ax[0,column].set_title(case+'; filtered torque minus clean instantaneous torque')
        ax[-1,column].set_xlabel('Time [s]')
    ax[0,0].legend(ncol=4,fontsize=8)
    fig.tight_layout();fig.savefig(out/'current_errors.png',dpi=140);plt.close(fig)
    csv_write(out/'current_metrics.csv',metrics)
    if resets:csv_write(out/'reset_events.csv',resets)
    else:(out/'reset_events.csv').write_text('case,method,t_s,joint,input_current_Aeq,output_current_Aeq\n')
    write(out/'current_summary.json',dict(threshold_Aeq=delta,heldout=holdout,metrics=metrics,
          truth_usage='clean simulated current/torque used only for offline distortion/noise diagnostics; online filter has no truth input'))
    return dict(passed=True,threshold_Aeq=delta,calibration_samples=count,methods=len(cfg['methods']),
                processed_physical_feedback_samples=1502,KF_calls=0,physics_steps=0)


def integrate(cfg,out,calibration_dir,ledger):
    require(json.loads((calibration_dir/'metrics.json').read_text())['passed'],'Calibration step not passed')
    base=json.loads(Path(cfg['base_config']).read_text())
    oracle=load(base['oracle_source'],'retained_current_kf')
    with np.load(calibration_dir/'step/filtered_sequences.npz',allow_pickle=False) as f:
        seq={k:f[k].copy() for k in f.files}
    length=cfg['integration_samples']
    t,q,v=seq['t'][:length],seq['q'][:length],seq['v'][:length]
    require(length==151 and np.isclose(t[0],.512) and np.isclose(t[-1],1.712),'Wrong integration interval')
    # The following truth is used in evaluation only, outside every observer.
    with np.load(Path(cfg['source_run'])/'probe/step/diagnostics.npz',allow_pickle=False) as f:
        truth=f['wrench'][cfg['history_count']:cfg['history_count']+length].copy()
    outputs={}; reference_P=None; metrics=[]; eigmin=float('inf'); max_P_difference=0.
    for method in cfg['methods']:
        observer=CurrentObserver(oracle.Observer(base),cfg['D_Nm_per_Aeq'])
        records={}
        for k in range(length):
            require(ledger['cumulative_KF_calls']<cfg['cumulative_KF_cap'],'KF budget exhausted')
            ledger['cumulative_KF_calls']+=1;ledger['new_KF_calls']+=1
            record=observer.update(t[k],q[k],v[k],seq[method+'_current'][k])
            require(all(np.isfinite(x).all() for x in record.values()),'Nonfinite KF output')
            ev=float(np.linalg.eigvalsh(record['covariance'])[0]);eigmin=min(eigmin,ev)
            require(ev>0,'Non-positive covariance')
            for key,value in record.items():records.setdefault(key,[]).append(value)
        records={key:np.asarray(value) for key,value in records.items()};records['t']=t
        outputs[method]=records
        if reference_P is None:reference_P=records['covariance']
        diff=float(np.max(np.abs(records['covariance']-reference_P)));max_P_difference=max(max_P_difference,diff)
        require(np.allclose(records['covariance'],reference_P,rtol=cfg['rtol'],atol=cfg['atol']),'Q/R/init were not held fixed')
        save_series(out/(method+'_estimates.npz'),records)
        for window,mask in [('preload',t<1),('loaded',t>=1),('tail',t>=1.5)]:
            errors=records['wrench'][mask]-truth[mask]
            for j,name in enumerate(['Fx','Fy','Fz','Mx','My','Mz']):
                metrics.append(dict(method=method,window=window,axis=name,unit='N' if j<3 else 'N m',
                    samples=int(mask.sum()),start_s=t[mask][0],end_s=t[mask][-1],
                    rmse=float(np.sqrt(np.mean(errors[:,j]**2))),bias=float(np.mean(errors[:,j])),
                    max_absolute_error=float(np.max(np.abs(errors[:,j])))))
    responses=[]
    for method,record in outputs.items():
        mask=t>=1; y=record['wrench'][mask,2];times=t[mask]
        crossings=np.flatnonzero(y>=2.7)
        responses.append(dict(method=method,first_90pct_time_from_load_s=float(times[crossings[0]]-1) if len(crossings) else None,
            maximum_Fz_N=float(y.max()),minimum_Fz_N=float(y.min()),
            peak_Fx_abs_N=float(np.abs(record['wrench'][mask,0]).max()),
            overshoot_in_observed_window_pct=float(max(0,y.max()-3)/3*100)))
    csv_write(out/'six_axis_KF_metrics.csv',metrics);csv_write(out/'short_response.csv',responses)
    write(out/'integration_checks.json',dict(passed=True,initialization_time_s=t[0],end_s=t[-1],
        samples_per_method=length,minimum_covariance_eigenvalue=eigmin,
        maximum_covariance_difference_between_methods=max_P_difference,
        Q_R_unchanged=True,filter_reset_does_not_reset_KF=True,**ledger))
    fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
    for method,record in outputs.items():
        axes[0].plot(t,record['wrench'][:,2],label=method)
        axes[1].plot(t,record['wrench'][:,0],label=method)
    axes[0].plot(t,truth[:,2],'k--',label='reference')
    axes[1].plot(t,truth[:,0],'k--')
    for j in [1,2,5]:axes[2].step(t,seq['msma_length'][:length,j],where='post',label=f'J{j+1}')
    for ax in axes:ax.axvline(1,color='k',ls=':',lw=.7);ax.grid(alpha=.2);ax.legend(ncol=5)
    axes[0].set_ylabel('Fz [N]');axes[1].set_ylabel('Fx [N]');axes[2].set_ylabel('MSMA window [samples]');axes[2].set_xlabel('Sampling time [s]')
    fig.suptitle('Short integration check; same Q/R; not a complete AKF performance comparison')
    fig.tight_layout();fig.savefig(out/'short_KF_response.png',dpi=140);plt.close(fig)
    summary=json.loads((calibration_dir/'current_summary.json').read_text())
    report=['# 变窗口电流滤波与短段接入', '', 'EXPLORE；保留既有等效电流与准确名义动力学。', '',
        '本轮实现论文图 2 的 WMA 增长、HMA 和阈值重置；没有把离线窗口方差表接入 Q。四种处理沿用同一 KF 参数，因此以下不是论文完整 AKF 与 CKF 的性能验收。', '',
        '## 固定配置', '',f"125 Hz；N={cfg['window']}；64 个真实过去样本预热缓存；六轴阈值分别为 "+', '.join(f'{x:.6g}' for x in summary['threshold_Aeq'])+' A_eq。', '',
        '噪声标准差各轴 0.05 N·m（当前单位映射下 0.05 A_eq），仅为仿真假设。标定/留出/反馈噪声种子分别 8501/8502/8503。阈值来自静态白噪声夹具，并非真实电机或运动工况标定。', '',
        '## 独立白噪声检查', '',
        '| 处理 | 六轴平均 RMS／N·m | 六轴平均 lag-1 相关 | 六轴合计重置次数 |', '|---|---:|---:|---:|']
    for method in cfg['methods']:
        rows=[r for r in summary['heldout'] if r['method']==method]
        report.append(f"| {method} | {np.mean([r['rmse_Nm'] for r in rows]):.8g} | {np.mean([r['lag1_autocorrelation'] for r in rows]):.6g} | {sum(r['resets'] for r in rows)} |")
    report+=['', 'RMS 为六个相同单位关节指标的算术平均，不是末端向量误差。各轴、每个 WMA 窗口、HMA 核理论方差见 calibration/window_variances.csv。', '',
        '## 实际运动电流', '', '| 场景 | 处理 | 六轴平均总 RMSE／N·m | 干净滤波失真 RMS 均值／N·m | 重置次数 |', '|---|---|---:|---:|---:|']
    for case in cfg['cases']:
        for method in cfg['methods']:
            rows=[r for r in summary['metrics'] if r['case']==case and r['method']==method and r['window']=='full']
            report.append(f"| {case} | {method} | {np.mean([r['total_torque_rmse_Nm'] for r in rows]):.8g} | {np.mean([r['clean_filter_distortion_rmse_Nm'] for r in rows]):.8g} | {sum(r['resets'] for r in rows)} |")
    report+=['', '评价为 0.512–6 s 全段，全部六轴与 2–6 s 窗见 calibration/current_metrics.csv；MSMA 的带噪/干净输出差包括模式变化，不能当作纯加性噪声。', '',
        '![六轴电流误差](../calibration/current_errors.png)', '', '## 短段 KF 接入', '',
        '共同从 t=0.512 s 初始化，到 1.712 s；每组 151 帧。同一 q/v、带噪电流、Q/R/P0，不重置 KF。原 t=0 初始化的旧结果不作为本表基线。', '',
        '| 处理 | 加载后 Fz RMSE／N | Fx 峰值／N | 首次 90%／ms | 观测窗内超调／% |', '|---|---:|---:|---:|---:|']
    for r in responses:
        fz=next(x['rmse'] for x in metrics if x['method']==r['method'] and x['window']=='loaded' and x['axis']=='Fz')
        latency='未达到' if r['first_90pct_time_from_load_s'] is None else f"{r['first_90pct_time_from_load_s']*1000:.6g}"
        report.append(f"| {r['method']} | {fz:.8g} | {r['peak_Fx_abs_N']:.8g} | {latency} | {r['overshoot_in_observed_window_pct']:.6g} |")
    report+=['', '加载窗实际为 1–1.712 s。末尾 1.5–1.712 s 只标为 tail，不称稳态；首次越阈可能来自噪声或超调，不当作稳定收敛。六轴绝对误差见 six_axis_KF_metrics.csv。', '',
        '![短段输出](short_KF_response.png)', '',
        '## 已实现与尚未验证', '',
        '- 已实现因果 WMA/HMA、原图增长期间不重置的规则、逐轴状态、连续历史缓存，并完成独立核与边界检查。',
        '- 白噪声方差下降不保证运动中的总输入误差下降；固定长窗可滞后，HMA 的负权重可产生过冲。模式切换还会改变统计条件。',
        '- 方差表只包含测量白噪声，未包含模型误差、运动滤波滞后或条件选择偏差；不是论文 §3.2 的恒速机械臂残差标定。',
        '- 若后续采用样本保持近似，过程输入的离散动量方差为 Ts² Sigma_tau，连续 Qp 对应 Ts Sigma_tau；本轮未将其用于 KF。移动平均相关性也不能被逐点方差完全表示。',
        '- 尚未完成完整 AKF 的协方差配置、D/摩擦标定及整段多种子估力比较，不判断论文方法整体优于原 KF。', '',
        '## 计算与下一步', '',
        f"本轮 KF 调用 {ledger['new_KF_calls']}，任务累计 {ledger['cumulative_KF_calls']}/4000；新增物理步 0，累计 192000。旧 Run 墙钟 12.272 s；新 Run 实际成本见 ../workflow-state.json。", '',
        '下一步应先检查这些窗口方差是否能代表实际运动时的输入误差，再决定如何接入时变过程协方差；不得仅凭静态白噪声降噪率宣称完整 AKF 有效。本轮停止，不调参或扩展。', '']
    (out/'decision.md').write_text('\n'.join(report))
    write(out/'summary.json',dict(passed=True,responses=responses,metrics=metrics,**ledger))
    return dict(passed=True,new_KF_calls=ledger['new_KF_calls'],cumulative_KF_calls=ledger['cumulative_KF_calls'],
                physics_steps=0,samples_per_method=length,maximum_covariance_difference=max_P_difference)


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--mode',choices=['checks','calibration','integration'],required=True)
    ap.add_argument('--config',type=Path,required=True);ap.add_argument('--output-dir',type=Path,required=True)
    ap.add_argument('--calibration-dir',type=Path)
    args=ap.parse_args();cfg=json.loads(args.config.read_text());out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
    old=json.loads(Path(cfg['source_run']+'/workflow-state.json').read_text())
    require(old['status']=='completed','Original run not completed')
    ledger=dict(new_KF_calls=0,cumulative_KF_calls=cfg['prior_KF_calls'],new_physics_steps=0)
    begin=time.perf_counter()
    def timeout(signum,frame):raise TimeoutError('Process wall ceiling exceeded')
    signal.signal(signal.SIGALRM,timeout);signal.alarm({'checks':55,'calibration':115,'integration':235}[args.mode])
    try:
        write(out/'config.json',cfg)
        if args.mode=='checks':
            result=run_checks(cfg);write(out/'checks.json',result)
        elif args.mode=='calibration':result=calibration(cfg,out)
        else:
            require(args.calibration_dir is not None,'Missing calibration directory')
            result=integrate(cfg,out,args.calibration_dir,ledger)
        result['wall_seconds']=time.perf_counter()-begin
        write(out/'metrics.json',result)
        print(json.dumps(result,default=lambda x: x.tolist() if isinstance(x,np.ndarray) else x.item()),flush=True)
    except Exception as e:
        write(out/'failure.json',dict(error=repr(e),traceback=traceback.format_exc(),ledger=ledger))
        write(out/'metrics.json',dict(passed=False,error=repr(e),wall_seconds=time.perf_counter()-begin,**ledger))
        raise
    finally:
        signal.alarm(0)
        write(out/'budget_ledger.json',dict(**ledger,wall_seconds=time.perf_counter()-begin))


if __name__=='__main__':main()
