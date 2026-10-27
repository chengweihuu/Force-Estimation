"""Native-sample metrics; paired seeds and fixed example selection."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from input_statistics import write,csvwrite,load,require
from current_interface import load_current_packet
from akf_comparison import close
from paper_compare import filter_inputs,replay,AXES


def masks(t):
    return dict(preload=(t>=2)&(t<2.56),transient=(t>=2.56)&(t<3.06),steady=(t>=3.56)&(t<5),loaded=(t>=2.56)&(t<5))


def summarize(rows,keys,fields):
    groups={}
    for r in rows:groups.setdefault(tuple(r[k] for k in keys),[]).append(r)
    result=[]
    for key,rr in groups.items():
        require(len(rr)==3,'Expected three paired seeds per summary')
        row=dict(zip(keys,key));row['seeds']=3
        for field in fields:
            values=[r[field] for r in rr if r[field] is not None]
            row[field+'_valid_seeds']=len(values)
            # Missing response is not zero; aggregate only if all three exist.
            row[field+'_mean']=float(np.mean(values)) if len(values)==3 else None
            row[field+'_sd']=float(np.std(values,ddof=1)) if len(values)==3 else None
        result.append(row)
    return result


def compare(cfg,base,out,ledger):
    root=Path(cfg['run_root'])
    for step in ['units','path','precheck','calibrate','collect']:
        require(json.loads((root/step/'metrics.json').read_text())['passed'],'Upstream gate failed: '+step)
    prior=json.loads((root/'units/metrics.json').read_text())['KF_calls'];require(prior+6183<=6783,'Cumulative KF budget')
    meas=load_current_packet(root/'collect/measurements.npz');require(len(meas['t'])==751,'751 shared feedback frames')
    s={k:meas[k][64:] for k in ['t','q','v']};t=s['t'];require(len(t)==687 and t[0]==.512 and t[-1]==6,'Evaluation frame range')
    close(np.diff(t),np.full(686,.008),cfg,'125 Hz time base')
    # Save all estimates before evaluation loads physical truth.
    for seed in cfg['seeds']:
        dest=out/str(seed);dest.mkdir()
        standard=np.random.default_rng(seed).normal(size=meas['current'].shape)
        noise=standard*.05/np.asarray(cfg['D']);noisy=meas['current']+noise;groups=filter_inputs(cfg,noisy)
        np.savez_compressed(dest/'current_inputs.npz',t=meas['t'],standard_noise=standard,noise=noise,noisy_current=noisy)
        results={}
        for name,g in groups.items():
            r=replay(cfg,base,s,g,ledger);results[name]=r
            np.savez_compressed(dest/(name+'.npz'),**r,current=g['current'],mode=g['mode'],length=g['length'],reset=g['reset'],threshold_checked=g['threshold_checked'])
        for name in ['HMA','MSMA']:
            for key in ['R']:
                close(results[name][key],results['RAW'][key],cfg,'Shared '+key)
            close(results[name]['covariance'][0],results['RAW']['covariance'][0],cfg,'Shared P0')
            close(results[name]['Q'][:,6:,:],results['RAW']['Q'][:,6:,:],cfg,'Shared Qw and cross blocks')
        print('paired KF complete',seed,flush=True)
    require(ledger['KF_calls']==6183,'Main KF call count')
    diag=load(root/'collect/diagnostics.npz');require(np.array_equal(t,diag['t'][64:]),'Truth time alignment');truth=diag['wrench'][64:]
    metrics=[];responses=[];inputs=[];switches=[];closure=0.
    for seed in cfg['seeds']:
        dest=out/str(seed);results={name:load(dest/(name+'.npz')) for name in ['RAW','HMA','MSMA']}
        for name,r in results.items():
            for window,mask in masks(t).items():
                for j,axis in enumerate(AXES):
                    e=r['wrench'][mask,j]-truth[mask,j];mse=float(np.mean(e**2));bias=float(e.mean());var=float(e.var(ddof=0));ix=int(np.argmax(abs(e)))
                    closure=max(closure,close(mse,bias**2+var,cfg,'MSE=bias^2+variance'))
                    metrics.append(dict(seed=seed,group=name,window=window,axis=axis,unit='N' if j<3 else 'N m',samples=len(e),rmse=np.sqrt(mse),bias=bias,sd=np.sqrt(var),variance=var,peak_abs=float(abs(e[ix])),peak_signed=float(e[ix]),peak_time_s=float(t[mask][ix])))
                    ie=(r['current'][mask,j]-meas['current'][64:][mask,j])*cfg['D'][j]
                    inputs.append(dict(seed=seed,group=name,window=window,joint=j+1,unit='N m',rmse=float(np.sqrt(np.mean(ie**2))),bias=float(ie.mean()),sd=float(ie.std(ddof=0))))
            mask=masks(t)['loaded'];tt=t[mask]
            for j in [2,3]:
                amp=abs(cfg['wrench'][j]);z=np.sign(cfg['wrench'][j])*r['wrench'][mask,j]
                def cross(frac):
                    hit=np.flatnonzero(z>=frac*amp);return float(tt[hit[0]]-2.56) if len(hit) else None
                good=np.logical_and.accumulate((abs(z-amp)<=.05*amp)[::-1])[::-1];hit=np.flatnonzero(good);t10=cross(.1);t90=cross(.9)
                responses.append(dict(seed=seed,group=name,axis=AXES[j],unit='N' if j<3 else 'N m',t10_s=t10,t90_s=t90,rise_10_90_s=t90-t10 if t90 is not None and t10 is not None else None,overshoot=float(max(0,z.max()-amp)),overshoot_percent=float(max(0,z.max()-amp)/amp*100),peak_time_s=float(tt[int(np.argmax(z))]),settle_5pct_s=float(tt[hit[0]]-2.56) if len(hit) else None,remaining_in_band_s=float(tt[-1]-tt[hit[0]]) if len(hit) else None))
        r=results['MSMA']
        for window,mask in dict(full=np.ones(len(t),bool),**masks(t)).items():
            for j in range(6):
                idx=np.flatnonzero(mask);mode=r['mode'][:,j];length=r['length'][:,j];trans=np.r_[False,mode[1:]!=mode[:-1]]
                switches.append(dict(seed=seed,window=window,joint=j+1,resets=int(r['reset'][mask,j].sum()),mode_changes=int(trans[mask].sum()),threshold_checks=int(r['threshold_checked'][mask,j].sum()),wma_frames=int((mode[mask]==1).sum()),hma_frames=int((mode[mask]==2).sum()),length_min=int(length[mask].min()),length_max=int(length[mask].max()),length_mean=float(length[mask].mean())))
        if seed==cfg['plot_seed']:
            fig,axs=plt.subplots(6,1,figsize=(11,12),sharex=True)
            for j,ax in enumerate(axs):
                for name,r in results.items():ax.plot(t,r['wrench'][:,j],lw=.8,label=name)
                ax.plot(t,truth[:,j],'k--',lw=.9,label='truth');ax.axvline(2.56,color='gray',ls=':');ax.grid(alpha=.2);ax.set_ylabel(AXES[j]+(' / N' if j<3 else ' / N m'))
            axs[0].legend(ncol=4);axs[-1].set_xlabel('Sample time / s');fig.tight_layout();fig.savefig(out/'six_axes_seed8801.png',dpi=140);plt.close(fig)
            fig,axs=plt.subplots(2,2,figsize=(12,7))
            for row,j in enumerate([2,3]):
                for col,(lo,hi) in enumerate([(2.4,3.25),(3.56,5)]):
                    mask=(t>=lo)&(t<hi);ax=axs[row,col]
                    for name,r in results.items():ax.plot(t[mask],r['wrench'][mask,j],label=name,lw=1)
                    ax.plot(t[mask],truth[mask,j],'k--');ax.grid(alpha=.2);ax.set_ylabel(AXES[j]+(' / N' if j<3 else ' / N m'));ax.set_xlabel('Sample time / s');ax.legend()
            fig.tight_layout();fig.savefig(out/'primary_response_seed8801.png',dpi=150);plt.close(fig)
    fields=['rmse','bias','sd','variance','peak_abs','peak_signed','peak_time_s'];rfields=['t10_s','t90_s','rise_10_90_s','overshoot','overshoot_percent','peak_time_s','settle_5pct_s','remaining_in_band_s']
    summary=summarize(metrics,['group','window','axis','unit'],fields);rsummary=summarize(responses,['group','axis','unit'],rfields)
    paired=[];response_pairs=[]
    index={(r['seed'],r['group'],r['window'],r['axis']):r for r in metrics}
    ri={(r['seed'],r['group'],r['axis']):r for r in responses}
    for seed in cfg['seeds']:
        for candidate,baseline in [('HMA','RAW'),('MSMA','RAW'),('MSMA','HMA')]:
            for window in masks(t):
                for axis in AXES:
                    a=index[seed,candidate,window,axis];b=index[seed,baseline,window,axis]
                    paired.append(dict(seed=seed,comparison=candidate+'-'+baseline,window=window,axis=axis,unit=a['unit'],**{'delta_'+f:a[f]-b[f] for f in fields}))
            for axis in ['Fz','Mx']:
                a=ri[seed,candidate,axis];b=ri[seed,baseline,axis]
                response_pairs.append(dict(seed=seed,comparison=candidate+'-'+baseline,axis=axis,unit=a['unit'],**{'delta_'+f:a[f]-b[f] if a[f] is not None and b[f] is not None else None for f in rfields}))
    for filename,rows in [('metrics_six_axes',metrics),('response',responses),('input_metrics',inputs),('msma_switches',switches),('summary',summary),('response_summary',rsummary),('paired_differences',paired),('response_paired',response_pairs),('paired_summary',summarize(paired,['comparison','window','axis','unit'],['delta_'+f for f in fields])),('response_paired_summary',summarize(response_pairs,['comparison','axis','unit'],['delta_'+f for f in rfields])),('input_summary',summarize(inputs,['group','window','joint','unit'],['rmse','bias','sd']))]:csvwrite(out/(filename+'.csv'),rows)
    write(out/'checks.json',dict(passed=True,main_KF_calls=ledger['KF_calls'],precheck_KF_calls=prior,total_KF_calls=prior+ledger['KF_calls'],truth_read_after_all_estimates_saved=True,same_R_P0_Qw=True,MSE_closure_max=closure,seeds=cfg['seeds'],units='force N; moment N m; no combined magnitude',statistics='within-window SD ddof=0; across-seed SD ddof=1; native 125 Hz'))
    lines=['# 电流滤波与配套输入协方差：有限场景方法复现','','三组共用一次6 s物理轨迹；三种子8801/8802/8803配对。电流系数与摩擦已知；等效电流加噪0.05 N·m。不是电气电机仿真或完整作者实验。','','标定只用独立正反匀速数据；窗口50和阈值固定。每组使用对应残差方差，自行递推协方差；保留外力过程噪声交叉块。假设残差作为一步独立输入误差，仅匹配边际方差，没有显式建模滤波后的时间相关性。','','以下为三种子的均值 ± 种子间样本SD。三次噪声实现共享物理轨迹，不代表三个独立工况。','','|窗口/轴/指标|RAW|HMA|MSMA|','|---|---:|---:|---:|']
    for window in ['preload','transient','steady','loaded']:
        for axis in ['Fz','Mx']:
            rr=[next(x for x in summary if x['group']==g and x['window']==window and x['axis']==axis) for g in ['RAW','HMA','MSMA']]
            lines.append('|'+window+'/'+axis+'/RMSE ('+rr[0]['unit']+')|'+'|'.join(f"{r['rmse_mean']:.6g} ± {r['rmse_sd']:.3g}" for r in rr)+'|')
    lines+=['','|轴/响应|RAW|HMA|MSMA|','|---|---:|---:|---:|']
    for axis in ['Fz','Mx']:
        rr=[next(x for x in rsummary if x['group']==g and x['axis']==axis) for g in ['RAW','HMA','MSMA']]
        for field,label,scale in [('t90_s','首次90% / ms',1000),('overshoot_percent','超调 / %',1),('settle_5pct_s','进入并保持5%带 / ms',1000)]:
            vals=[f"{r[field+'_mean']*scale:.5g} ± {r[field+'_sd']*scale:.3g}" if r[field+'_mean'] is not None else '并非所有种子达到' for r in rr]
            lines.append('|'+axis+'/'+label+'|'+'|'.join(vals)+'|')
    lines+=['','## 使用边界','','- 不对不同单位合并排名；主要响应指标与持续误差并列判断，不以平滑程度代替估力精度。','- 超调按输入符号统一后计算；不扣除基线偏置。5%带只保证到4.992 s，remaining_in_band_s另存。','- 全部六轴、三种子、四窗口及配对差见CSV；绘图种子在计算前固定为8801。','- MuJoCo准确模型、无速度噪声、单一姿态路径与载荷，结论不外推真机或所有装配。','- 仅完成预先批准的比较；不自动调参、延长或替换默认方法。']
    (out/'decision.md').write_text('\n'.join(lines)+'\n')
    return dict(seeds=3,groups=3,estimated_sequences=9,rows_six_axes=len(metrics),paired_rows=len(paired),MSE_closure_max=closure)
