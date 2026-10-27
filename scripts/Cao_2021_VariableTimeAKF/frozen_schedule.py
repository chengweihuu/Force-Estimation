"""Offline algebra under the observed adaptive schedule, not a new estimator."""
import argparse, json, time, traceback
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from msma import wma_weights, hma_weights
from input_statistics import load, write, csvwrite, require, correlation, phases


def setup(cfg):
    data={}; checks={}
    for case in cfg['cases']:
        s=load(Path(cfg['filter_run'])/'calibration'/case/'filtered_sequences.npz')
        original=load(Path(cfg['current_run'])/'probe'/case/'current_measurements.npz')
        pre=len(s['prehistory_t']); clean=original['current']
        noisy=np.concatenate([s['prehistory_noisy_current'],s['noisy_current']])
        require(np.array_equal(original['t'][:pre],s['prehistory_t']),'History timestamp mismatch')
        require(np.array_equal(original['t'][pre:],s['t']),'Evaluation timestamp mismatch')
        require(np.array_equal(clean[pre:],s['clean_current']),'Clean reference mismatch')
        require(clean.shape==noisy.shape and np.isfinite(noisy).all(),'Input shape/value mismatch')
        noise=noisy-clean
        fc=np.zeros_like(s['clean_current']); fn=np.zeros_like(fc); reconstructed=np.zeros_like(fc)
        sums=[]
        for k in range(len(fc)):
            idx=pre+k
            for j in range(6):
                mode=int(s['msma_mode'][k,j]); n=int(s['msma_length'][k,j])
                require(mode in (1,2) and 1<=n<=cfg['window'],'Unexpected mode/length')
                if mode==2: require(n==cfg['window'],'HMA length mismatch')
                w=wma_weights(n) if mode==1 else hma_weights(cfg['window'])
                require(idx-len(w)+1>=0,'Insufficient real history')
                ix=idx-np.arange(len(w))
                fc[k,j]=w@clean[ix,j]; fn[k,j]=w@noise[ix,j]
                reconstructed[k,j]=w@noisy[ix,j]; sums.append(w.sum())
        D=np.asarray(cfg['D'])
        e=(s['msma_current']-clean[pre:])*D
        distortion=(fc-clean[pre:])*D; propagated=fn*D
        close=lambda a,b: np.allclose(a,b,rtol=cfg['rtol'],atol=cfg['atol'])
        require(close(reconstructed,s['msma_current']),'Frozen-weight output differs from saved MSMA; stop attribution')
        require(close(fc+fn,reconstructed),'Linear split failure')
        require(close(e,distortion+propagated),'Error closure failure')
        require(close(sums,np.ones(len(sums))),'DC weight sum failure')
        terms=np.mean(distortion**2,axis=0)+np.mean(propagated**2,axis=0)+2*np.mean(distortion*propagated,axis=0)
        require(close(np.mean(e**2,axis=0),terms),'MSE closure failure')
        # Difference from the earlier clean-input run includes its different schedule.
        schedule_delta=(fc-s['msma_clean_output'])*D
        prior_n=(s['msma_current']-s['msma_clean_output'])*D
        require(close(prior_n,propagated+schedule_delta),'Prior decomposition correspondence failure')
        checks[case]=dict(samples=len(fc),reconstruction_max_Aeq=float(np.max(abs(reconstructed-s['msma_current']))),
            linear_split_max_Aeq=float(np.max(abs(fc+fn-reconstructed))),error_closure_max_Nm=float(np.max(abs(e-distortion-propagated))),
            mse_closure_max_Nm2=float(np.max(abs(np.mean(e**2,axis=0)-terms))))
        data[case]=dict(s=s,e=e,d=distortion,n=propagated,raw=noise[pre:]*D,schedule_delta=schedule_delta)
    return data,checks


def stats(x):
    return dict(mean_Nm=float(np.mean(x)),rms_Nm=float(np.sqrt(np.mean(x*x))),variance_Nm2=float(np.var(x)))


def diagnose(cfg,out,data,checks):
    overall=[];groups=[];acf=[]
    fig,axes=plt.subplots(6,2,figsize=(13,12),sharex=True)
    for cidx,(case,d) in enumerate(data.items()):
        s=d['s'];N=len(s['t']);allmask=np.ones(N,dtype=bool)
        np.savez_compressed(out/(case+'_split.npz'),t=s['t'],error=d['e'],distortion=d['d'],propagated_noise=d['n'],raw_noise=d['raw'],schedule_delta=d['schedule_delta'],mode=s['msma_mode'],length=s['msma_length'])
        for j in range(6):
            e,dist,noise,raw,delta=(d[k][:,j] for k in ['e','d','n','raw','schedule_delta'])
            row=dict(case=case,joint=j+1,samples=N)
            for name,x in [('total',e),('distortion',dist),('noise',noise),('raw',raw),('schedule_delta',delta)]:
                row.update({name+'_'+k:v for k,v in stats(x).items()})
            row.update(distortion_MSE_Nm2=float(np.mean(dist**2)),noise_MSE_Nm2=float(np.mean(noise**2)),cross_Nm2=float(2*np.mean(dist*noise)),total_MSE_Nm2=float(np.mean(e**2)))
            row['noise_RMS_reduction_percent']=100*(1-row['noise_rms_Nm']/row['raw_rms_Nm'])
            overall.append(row)
            phase=phases(s['msma_mode'][:,j],s['msma_length'][:,j],s['msma_reset'][:,j])
            masks=[('all',allmask),('WMA',s['msma_mode'][:,j]==1),('HMA',s['msma_mode'][:,j]==2)]
            masks += [(str(p),phase==p) for p in np.unique(phase)]
            for group,mask in masks:
                for name,x in [('distortion',dist),('noise',noise),('total',e),('raw',raw)]:
                    groups.append(dict(case=case,joint=j+1,group=group,component=name,samples=int(mask.sum()),**stats(x[mask])))
                    for lag in cfg['lags']:
                        count,rho=correlation(x,mask,lag)
                        acf.append(dict(case=case,joint=j+1,group=group,component=name,lag_frames=lag,pairs=count,correlation=rho))
            ax=axes[j,cidx]
            ax.plot(s['t'],e,label='Total',lw=.9,color='black');ax.plot(s['t'],dist,label='Motion distortion',lw=.9);ax.plot(s['t'],noise,label='Propagated noise',lw=.8)
            ax.set_ylabel(f'J{j+1} / N m');ax.grid(alpha=.2)
            if j==0:ax.set_title(case);ax.legend(fontsize=7)
            if j==5:ax.set_xlabel('Time / s')
    fig.tight_layout();fig.savefig(out/'frozen_split.png',dpi=150);plt.close(fig)
    csvwrite(out/'six_axis_split.csv',overall);csvwrite(out/'conditional_moments.csv',groups);csvwrite(out/'autocorrelation.csv',acf)
    write(out/'summary.json',dict(checks=checks,overall=overall))
    report=['# 固定实际窗口序列：电流误差分离','',
        '125 Hz；复用0.512–6 s全部687帧及真实历史；两场景、六轴。所有幅值已用既有D换成N·m。',
        '记录的带噪MSMA模式和窗口完全固定，没有重新运行模式判定。令W为这个已观测时变算子，i为干净电流、η为已记录噪声：','',
        '$$','e=D[W(i+η)-i]=D(Wi-i)+DWη=d+n.','$$','',
        '固定算子下严格线性分解；窗口由原噪声参与选择，不能据此假定W与η独立或把n视为白噪声。',
        '## 六轴结果','',
        '|场景|轴|原噪声RMS|传播噪声RMS|运动失真RMS|总误差RMSE|噪声RMS降低|',
        '|---|---|---:|---:|---:|---:|---:|']
    for r in overall:
        report.append(f"|{r['case']}|J{r['joint']}|{r['raw_rms_Nm']:.6f}|{r['noise_rms_Nm']:.6f}|{r['distortion_rms_Nm']:.6f}|{r['total_rms_Nm']:.6f}|{r['noise_RMS_reduction_percent']:.1f}%|")
    report += ['', '各RMS不能相加；精确MSE及带符号交叉项见six_axis_split.csv。conditional_moments.csv给出模式/增长阶段条件均值与方差，autocorrelation.csv仅使用连续组内配对。',
        'schedule_delta是当前固定窗口下干净输出与此前独立干净运行输出之差；用于核对旧分解，不等于已证明的独立因果效应。',
        '', '## 边界','',
        '仅两个短段及一次已记录噪声；不作置信区间、不拟合新方差表、不评价完整AKF。本轮没有定义电流真值阶跃，因此不从外力加载时刻推造电流延迟指标。',
        '纯噪声表与实际残差标定不是同一对象；本结果不否定论文的匀速模型残差标定。运动失真不应被等同于独立零均值测量噪声。',
        '', '![六轴同步分解](frozen_split.png)','']
    (out/'decision.md').write_text('\n'.join(report))
    return dict(rows=len(overall))


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--config',type=Path,required=True);ap.add_argument('--output-dir',type=Path,required=True);ap.add_argument('--mode',choices=['precheck','diagnose'],required=True)
    a=ap.parse_args();cfg=json.loads(a.config.read_text());a.output_dir.mkdir(parents=True,exist_ok=True);start=time.perf_counter()
    try:
        data,checks=setup(cfg);write(a.output_dir/'closure.json',checks)
        extra=diagnose(cfg,a.output_dir,data,checks) if a.mode=='diagnose' else {}
        write(a.output_dir/'metrics.json',dict(passed=True,new_KF_calls=0,new_physics_steps=0,new_adaptive_filter_calls=0,wall_seconds=time.perf_counter()-start,**extra))
    except Exception as exc:
        write(a.output_dir/'failure.json',dict(error=repr(exc),traceback=traceback.format_exc()))
        write(a.output_dir/'metrics.json',dict(passed=False,wall_seconds=time.perf_counter()-start));raise
if __name__=='__main__':main()
