"""Paired transfer of fixed calibrated windows; causal finite-sum MSMA evaluator."""
import argparse,json,time,traceback
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from msma import wma_weights,hma_weights,process
from input_statistics import write,csvwrite,load,require


def causal_msma(x,windows,threshold,history):
    # Only compute HMA when requested. Initial growing WMA needs no invented history.
    size=len(x)-history;y=np.empty((size,6));mode=np.empty_like(y,dtype=int);length=np.empty_like(mode)
    reset=np.zeros_like(y,dtype=bool);checked=np.zeros_like(reset);start=np.zeros(6,dtype=int);used=[]
    kernels={N:hma_weights(N) for N in set(windows)}
    weights={n:wma_weights(n) for n in range(1,max(windows)+1)}
    for k in range(size):
        idx=history+k;row=[]
        for j,N in enumerate(windows):
            age=k-start[j]+1
            if age<=N:md,n=1,age
            else:
                checked[k,j]=True
                if abs(x[idx,j]-y[k-1,j])<=threshold[j]:md,n=2,N
                else:md,n=1,1;start[j]=k;reset[k,j]=True
            w=weights[n] if md==1 else kernels[N]
            require(idx+1>=len(w),'HMA requested before real history is available')
            y[k,j]=w@x[idx-np.arange(len(w)),j];mode[k,j]=md;length[k,j]=n;row.append(w)
        used.append(row)
    return dict(current=y,mode=mode,length=length,reset=reset,threshold_checked=checked,weights=used)


def apply_weights(x,weights,history):
    out=np.empty((len(weights),6))
    for k,row in enumerate(weights):
        for j,w in enumerate(row):out[k,j]=w@x[history+k-np.arange(len(w)),j]
    return out


def setup(cfg):
    delta=json.loads(Path(cfg['threshold_file']).read_text())['threshold_Aeq'];data={};checks={};h=cfg['history_count']
    close=lambda a,b:np.allclose(a,b,rtol=cfg['rtol'],atol=cfg['atol'])
    for case in cfg['cases']:
        s=load(Path(cfg['filter_run'])/'calibration'/case/'filtered_sequences.npz')
        old=load(Path(cfg['current_run'])/'probe'/case/'current_measurements.npz')
        require(np.array_equal(old['t'][:h],s['prehistory_t']) and np.array_equal(old['t'][h:],s['t']),'Time mismatch')
        require(np.array_equal(old['current'][h:],s['clean_current']),'Clean reference mismatch')
        clean=old['current'];noisy=np.concatenate([s['prehistory_noisy_current'],s['noisy_current']]);noise=noisy-clean
        require(np.array_equal(s['raw_current'],noisy[h:]),'Raw reference mismatch')
        base=causal_msma(noisy,[50]*6,delta,h)
        for key in ['current','mode','length','reset','threshold_checked']:
            require(close(base[key],s['msma_'+key]) if key=='current' else np.array_equal(base[key],s['msma_'+key]),'Baseline reproduction failed: '+key)
        cand=causal_msma(noisy,cfg['candidate_windows'],delta,h)
        prefix=causal_msma(noisy[:cfg['prefix_frames']],cfg['candidate_windows'],delta,h)
        for key in ['current','mode','length','reset','threshold_checked']:
            require(np.array_equal(prefix[key],cand[key][:len(prefix[key])]),'Causal prefix failure: '+key)
        require(np.array_equal(cand['current'][0],noisy[h]),'First WMA sample must equal raw')
        # Original implementation requires a full HMA cache at prime. At a common
        # later start with 128 past frames, cross-check N=30 and N=100 with it.
        max_ref_error=0.
        for N in sorted(set(cfg['candidate_windows'])):
            ref=process(noisy,'msma',N,delta,128);finite=causal_msma(noisy,[N]*6,delta,128)
            for key in ['current','mode','length','reset','threshold_checked']:
                require(close(ref[key],finite[key]) if key=='current' else np.array_equal(ref[key],finite[key]),'Finite-sum vs retained filter mismatch')
            max_ref_error=max(max_ref_error,float(np.max(abs(ref['current']-finite['current']))))
        variants={};mseclose=0.;errorclose=0.
        rawrec=dict(current=noisy[h:],mode=np.zeros_like(noisy[h:],dtype=int),length=np.ones_like(noisy[h:],dtype=int),reset=np.zeros_like(noisy[h:],dtype=bool))
        for name,rec in [('raw',rawrec),('N50',base),('calibrated',cand)]:
            fc=clean[h:] if name=='raw' else apply_weights(clean,rec['weights'],h)
            pn=noise[h:] if name=='raw' else apply_weights(noise,rec['weights'],h)
            e=(rec['current']-clean[h:])*cfg['D'];d=(fc-clean[h:])*cfg['D'];n=pn*cfg['D']
            require(close(e,d+n),'Error split failure')
            require(close(np.mean(e*e,axis=0),np.mean(d*d+n*n+2*d*n,axis=0)),'MSE closure failure')
            errorclose=max(errorclose,float(np.max(abs(e-d-n))))
            mseclose=max(mseclose,float(np.max(abs(np.mean(e*e-d*d-n*n-2*d*n,axis=0)))))
            variants[name]=dict(rec=rec,e=e,d=d,n=n)
        checks[case]=dict(saved_baseline_max_Aeq=float(np.max(abs(base['current']-s['msma_current']))),reference_filter_max_Aeq=max_ref_error,prefix_frames=len(prefix['current']),error_closure_Nm=errorclose,mse_closure_Nm2=mseclose)
        data[case]=dict(t=s['t'],truth=clean[h:]*cfg['D'],variants=variants)
    return data,checks


def analyze(cfg,out,data,checks):
    rows=[];events=[];comparisons=[];fig,axes=plt.subplots(6,2,figsize=(13,12),sharex=True)
    for ci,(case,c) in enumerate(data.items()):
        t=c['t'];windows={'full':np.ones(len(t),dtype=bool),'after_2s':t>=2,'load_neighborhood':(t>=cfg['event_start_s'])&(t<=cfg['event_end_s'])}
        if case=='step':windows['loaded_short']=(t>=1)&(t<=1.712)
        series=dict(t=t,truth_Nm=c['truth'])
        for name,v in c['variants'].items():
            for key in ['e','d','n']:series[name+'_'+key]=v[key]
            for key in ['current','mode','length','reset']:series[name+'_'+key]=v['rec'][key]
            for window,mask in windows.items():
                for j in range(6):
                    e=v['e'][mask,j];d=v['d'][mask,j];n=v['n'][mask,j];local=int(np.argmax(abs(e)));indices=np.flatnonzero(mask)
                    row=dict(case=case,method=name,window=window,joint=j+1,samples=len(e),rmse_Nm=float(np.sqrt(np.mean(e*e))),bias_Nm=float(e.mean()),peak_abs_Nm=float(np.max(abs(e))),peak_signed_Nm=float(e[local]),peak_time_s=float(t[indices[local]]),distortion_RMS_Nm=float(np.sqrt(np.mean(d*d))),noise_RMS_Nm=float(np.sqrt(np.mean(n*n))),cross_Nm2=float(2*np.mean(d*n)),resets=int(v['rec']['reset'][mask,j].sum()))
                    rows.append(row)
            for j in range(6):
                ax=axes[j,ci];ax.plot(t,v['e'][:,j],label=name,lw=.8,alpha=.6 if name=='raw' else .95)
                if case=='step':
                    eventmask=windows['load_neighborhood'];peakidx=np.flatnonzero(eventmask)[np.argmax(abs(v['e'][eventmask,j]))]
                    for kind,k in [('first_loaded',int(np.flatnonzero(t>=1)[0])),('peak_in_neighborhood',int(peakidx))]:
                        events.append(dict(method=name,joint=j+1,event=kind,t_s=float(t[k]),truth_Nm=float(c['truth'][k,j]),filtered_Nm=float(v['rec']['current'][k,j]*cfg['D'][j]),error_Nm=float(v['e'][k,j]),mode=int(v['rec']['mode'][k,j]),length=int(v['rec']['length'][k,j]),reset=bool(v['rec']['reset'][k,j])))
                ax.set_ylabel(f'J{j+1} / N m');ax.grid(alpha=.2)
                if j==0:ax.set_title(case);ax.legend(fontsize=8)
                if j==5:ax.set_xlabel('Time / s')
        np.savez_compressed(out/(case+'_series.npz'),**series)
        for window in windows:
            for j in range(1,7):
                get=lambda name:next(r for r in rows if r['case']==case and r['window']==window and r['joint']==j and r['method']==name)
                a,b,raw=get('calibrated'),get('N50'),get('raw');row=dict(case=case,window=window,joint=j)
                for key in ['rmse_Nm','bias_Nm','peak_abs_Nm','distortion_RMS_Nm','noise_RMS_Nm']:
                    row['candidate_minus_N50_'+key]=a[key]-b[key];row['candidate_minus_raw_'+key]=a[key]-raw[key]
                comparisons.append(row)
    fig.tight_layout();fig.savefig(out/'full_errors.png',dpi=150);plt.close(fig)
    # Signed torque waveform around external-force onset; no fabricated current-step delay.
    c=data['step'];t=c['t'];mask=(t>=cfg['event_start_s'])&(t<=cfg['event_end_s']);fig,axes=plt.subplots(3,2,figsize=(12,9),sharex=True)
    for j,ax in enumerate(axes.flat):
        ax.plot(t[mask],c['truth'][mask,j],label='Clean torque',color='black',lw=1.5)
        for name,v in c['variants'].items():ax.plot(t[mask],v['rec']['current'][mask,j]*cfg['D'][j],label=name,lw=.9)
        ax.axvline(1,color='gray',ls=':');ax.set_title(f'J{j+1}');ax.set_ylabel('N m');ax.grid(alpha=.2);ax.legend(fontsize=7)
    axes[2,0].set_xlabel('Time / s');axes[2,1].set_xlabel('Time / s');fig.tight_layout();fig.savefig(out/'load_waveforms.png',dpi=150);plt.close(fig)
    csvwrite(out/'metrics_all.csv',rows);csvwrite(out/'paired_differences.csv',comparisons);csvwrite(out/'load_events.csv',events)
    write(out/'summary.json',dict(checks=checks,full=[r for r in rows if r['window']=='full']))
    report=['# 匀速标定窗口的动态迁移检查','',
        '候选[J1..J6]=[100,30,100,100,100,100]；原始输入与统一50点MSMA为对照。阈值、已记录噪声、q/v/力矩真值和125 Hz时间戳不变。',
        '所有方法从原0.512 s起点开始评价，共687帧至6 s。MSMA从WMA(1)增长，HMA只在历史足够且规则要求时计算；没有补零或未来数据。100点较长启动是其行为的一部分，after_2s另列以区分启动影响。',
        '原50点输出及mode/length/reset/threshold_checked重构通过；30/100点有限和实现与原实现还在128帧共同历史下交叉核对，候选通过因果前缀检查。',
        '', '|场景|轴|原始RMSE|50点RMSE|候选RMSE|候选−50点|', '|---|---|---:|---:|---:|---:|']
    for case in cfg['cases']:
        for j in range(1,7):
            rr={r['method']:r for r in rows if r['case']==case and r['joint']==j and r['window']=='full'}
            report.append(f"|{case}|J{j}|{rr['raw']['rmse_Nm']:.6g}|{rr['N50']['rmse_Nm']:.6g}|{rr['calibrated']['rmse_Nm']:.6g}|{rr['calibrated']['rmse_Nm']-rr['N50']['rmse_Nm']:+.6g}|")
    report += ['', '单位N·m。误差为滤后实际驱动力矩减同帧干净实际驱动力矩；没有用匀速零加速度参考评价动态轨迹。',
        '全段、2 s后、加载邻域[0.8,1.712] s及加载短段[1,1.712] s见metrics_all.csv；偏置、原生峰值、峰值时刻、分解和窗口重置全部保留。',
        '未对非阶跃的真实电流轨迹套用90%响应时间；外力加载不代表每轴电流都发生理想阶跃。load_events及同步波形给出实际变化，不作事后移位对齐。',
        '这是已看过日志的探索性迁移检查，不是独立测试；不以单个改善指标替代全轴取舍判断，不重新选择窗口、不运行KF。',
        '', '![全程误差](full_errors.png)','', '![加载邻域真实和处理后力矩](load_waveforms.png)','']
    (out/'decision.md').write_text('\n'.join(report))
    return dict(rows=len(rows),comparisons=len(comparisons))


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--config',required=True,type=Path);ap.add_argument('--output-dir',required=True,type=Path);ap.add_argument('--mode',required=True,choices=['precheck','analyze']);a=ap.parse_args()
    out=a.output_dir;out.mkdir(parents=True,exist_ok=True);cfg=json.loads(a.config.read_text());start=time.perf_counter()
    try:
        data,checks=setup(cfg);write(out/'checks.json',checks)
        extra=analyze(cfg,out,data,checks) if a.mode=='analyze' else {}
        write(out/'metrics.json',dict(passed=True,new_physics_steps=0,new_KF_calls=0,wall_seconds=time.perf_counter()-start,**extra))
    except Exception as exc:
        write(out/'failure.json',dict(error=repr(exc),traceback=traceback.format_exc()));write(out/'metrics.json',dict(passed=False,wall_seconds=time.perf_counter()-start));raise
if __name__=='__main__':main()
