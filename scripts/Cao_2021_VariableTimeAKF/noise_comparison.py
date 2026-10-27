"""Paired saved-noise scaling, direct residual recalibration, and fixed KF replay."""
import argparse,csv,json,sys,time,traceback
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from input_statistics import load,write,csvwrite,require
from akf_comparison import read_table,close
from paper_compare import filter_inputs,replay,input_split,masks,GROUPS,AXES
from msma import wma_weights,hma_weights,linear_series


def recalibrate(cfg,out):
    source=Path(cfg['source_run']);oldstate=json.loads((source/'workflow-state.json').read_text())
    for x in oldstate['inputSnapshots']:require((Path('/workspace')/x['path']).read_bytes()==(source/x['snapshot']).read_bytes(),'Source input changed: '+x['path'])
    cc=json.loads(Path(cfg['constant_config']).read_text());D=np.array(cfg['D']);kernels=[('raw',1,np.ones(1))]+[('wma',n,wma_weights(n)) for n in range(1,51)]+[('hma',50,hma_weights(50))];tables={};stats={}
    for level,factor in [('low',1.),('high',cfg['noise_factor'])]:
        dest=out/level;dest.mkdir();rows=[];table=[]
        for case in cc['cases']:
            folder='pilot' if case['name']==cc['pilot_case'] else 'collect';s=load(Path(cfg['constant_run'])/folder/case['name']/'measurements.npz');n=load(Path(cfg['constant_analysis'])/case['name']/'noise.npz')
            require(np.array_equal(s['t'],n['t']),'Calibration noise time mismatch');noise=factor*n['noise'];x=s['current']+noise;core=(s['t']>=3)&(s['t']<7);j=case['joint']-1
            require(core.sum()==500 and np.flatnonzero(core)[0]>=55,'Calibration core/history mismatch')
            for method,length,w in kernels:
                y=linear_series(x,w)*D;e=s['tau_ref_no_acc'][core,j]-y[core,j]
                rows.append(dict(case=case['name'],joint=j+1,method=method,length=length,samples=len(e),mean_Nm=float(e.mean()),sample_variance_Nm2=float(e.var(ddof=1)),rmse_Nm=float(np.sqrt(np.mean(e*e)))))
        for j in range(1,7):
            for method,length,w in kernels:
                pair=[r for r in rows if r['joint']==j and r['method']==method and r['length']==length];require(len(pair)==2,'Signed pair missing');pos=next(r for r in pair if r['case'].endswith('positive'));neg=next(r for r in pair if r['case'].endswith('negative'))
                variance=float(np.mean([r['sample_variance_Nm2'] for r in pair]));require(np.isfinite(variance) and variance>0,'Invalid recalibrated variance')
                table.append(dict(joint=j,method=method,length=length,variance_Nm2=variance,positive_mean_Nm=pos['mean_Nm'],negative_mean_Nm=neg['mean_Nm'],pooled_MSE_Nm2=float(np.mean([r['rmse_Nm']**2 for r in pair]))))
        csvwrite(dest/'direction_statistics.csv',rows);csvwrite(dest/'residual_variance_table.csv',table);tables[level]=table;stats[level]=len(rows)
    original=read_table(dict(table_dir=cfg['original_table_dir']));diff=[];maxerr=0.
    for low,high in zip(tables['low'],tables['high']):
        key=low['method'],low['length'],low['joint'];maxerr=max(maxerr,close(low['variance_Nm2'],original[key],cfg,'Original residual table not reproduced'))
        diff.append(dict(joint=low['joint'],method=low['method'],length=low['length'],low_variance_Nm2=low['variance_Nm2'],high_variance_Nm2=high['variance_Nm2'],high_over_low=high['variance_Nm2']/low['variance_Nm2'],high_minus_nine_low_Nm2=high['variance_Nm2']-9*low['variance_Nm2']))
    csvwrite(out/'variance_changes.csv',diff);write(out/'checks.json',dict(passed=True,original_table_max_error=maxerr,entries_per_level=312,direction_rows=stats,noise_source='factor times each saved calibration noise; no RNG, no empirical restandardization',original_snapshots_checked=len(oldstate['inputSnapshots'])))
    return dict(entries_per_level=312,original_table_max_error=maxerr)


def inputs(cfg,level):
    root=Path(cfg['source_run']);s=load(root/'collect/measurements.npz');saved=load(root/'compare/current_inputs.npz');require(np.array_equal(s['t'],saved['t']) and s['current'].shape==(751,6),'Evaluation saved noise alignment failed')
    close(saved['noisy_current'],s['current']+saved['noise'],cfg,'Saved noisy current does not reconstruct')
    factor=1. if level=='low' else cfg['noise_factor'];noise=factor*saved['noise'];noisy=s['current']+noise
    local=dict(cfg,table_dir=str(Path(cfg['run_root'])/'calibrate'/level));g=filter_inputs(local,noisy)
    return s,noise,noisy,g


def precheck(cfg,base,out,ledger):
    require(json.loads((Path(cfg['run_root'])/'calibrate/metrics.json').read_text())['passed'],'Calibration prerequisite failed')
    s,noise,noisy,g=inputs(cfg,'low');ss={k:s[k][64:] for k in ['t','q','v']};errors={}
    for name in GROUPS:
        old=load(Path(cfg['source_run'])/'compare'/(name+'.npz'))
        close(g[name]['current'],old['current'],cfg,'Original input changed '+name)
        for key in ['mode','length','reset','threshold_checked']:require(np.array_equal(g[name][key],old[key]),'Original schedule changed '+name+key)
        r=replay(cfg,base,ss,g[name],ledger,151);errors[name]=max(close(r[key],old[key][:151],cfg,'Low baseline changed '+name+key) for key in ['wrench','covariance','gain','Q','R','momentum','nominal_input','innovation','selected_sigma'])
        np.savez_compressed(out/(name+'.npz'),**r)
    _,highnoise,highnoisy,_=inputs(cfg,'high');close(highnoise,3*noise,cfg,'High noise not exactly scaled saved sequence');close(highnoisy,s['current']+highnoise,cfg,'High noisy current wrong')
    import mujoco,scipy,platform
    ver=dict(mujoco=mujoco.__version__,numpy=np.__version__,scipy=scipy.__version__,python=platform.python_version())
    for name,value in base['expected_versions'].items():require(ver[name]==value,'Version mismatch '+name)
    write(out/'checks.json',dict(passed=True,versions=ver,baseline_recovery_max=errors,paired_saved_noise=True,precheck_KF_calls=ledger['KF_calls']))
    return dict(baseline_recovery_max=errors)


def evaluate(cfg,t,truth,results,level):
    rows=[];responses=[]
    for name,r in results.items():
        for window,mask in masks(t).items():
            for j,axis in enumerate(AXES):
                e=r['wrench'][mask,j]-truth[mask,j];bias=float(e.mean());var=float(e.var());mse=float(np.mean(e*e));close(mse,var+bias*bias,cfg,'Output MSE closure');i=int(np.argmax(abs(e)))
                rows.append(dict(level=level,sigma_Nm=.05 if level=='low' else .15,group=name,window=window,axis=axis,unit='N' if j<3 else 'N m',samples=len(e),rmse=float(np.sqrt(mse)),bias=bias,sd=float(np.sqrt(var)),variance=var,peak_abs=float(abs(e[i])),peak_signed=float(e[i]),peak_time_s=float(t[mask][i])))
        mask=masks(t)['loaded'];tt=t[mask]
        for j in [2,3]:
            amp=abs(cfg['wrench'][j]);z=np.sign(cfg['wrench'][j])*r['wrench'][mask,j]
            def cross(frac):
                hit=np.flatnonzero(z>=frac*amp);return float(tt[hit[0]]-2.56) if len(hit) else None
            t10=cross(.1);t90=cross(.9);hit=np.flatnonzero(np.logical_and.accumulate((abs(z-amp)<=.05*amp)[::-1])[::-1])
            responses.append(dict(level=level,group=name,axis=AXES[j],unit='N' if j<3 else 'N m',t10_s=t10,t90_s=t90,rise_10_90_s=None if t10 is None or t90 is None else t90-t10,overshoot=float(max(0,z.max()-amp)),overshoot_percent=float(max(0,z.max()-amp)/amp*100),settle_5pct_s=float(tt[hit[0]]-2.56) if len(hit) else None,remaining_in_band_s=float(tt[-1]-tt[hit[0]]) if len(hit) else None))
    return rows,responses


def mode_stats(cfg,out,t,low,high):
    rows=[];events=[];changes=[]
    for label,s in [('low',low),('high',high)]:
        for window,mask in masks(t).items():
            for j in range(6):
                checked=s['threshold_checked'][:,j].astype(bool);reset=s['reset'][:,j].astype(bool);mode=s['mode'][:,j];n=s['length'][:,j];transition=np.r_[False,mode[1:]!=mode[:-1]];count=int((checked&mask).sum());num=int((reset&mask).sum())
                require(not np.any(reset&~checked),'Reset without threshold check');require(np.all(n[reset]==1),'Reset not WMA1')
                rows.append(dict(level=label,window=window,joint=j+1,samples=int(mask.sum()),threshold_checks=count,resets=num,resets_per_check=None if count==0 else num/count,WMA_frames=int(((mode==1)&mask).sum()),HMA_frames=int(((mode==2)&mask).sum()),length_mean=float(n[mask].mean()),length_min=int(n[mask].min()),length_max=int(n[mask].max()),mode_transitions=int((transition&mask).sum())))
        for j in range(6):
            for k in np.flatnonzero(s['reset'][:,j]):events.append(dict(level=label,joint=j+1,time_s=float(t[k]),low_reset_same_frame=bool(low['reset'][k,j]),high_reset_same_frame=bool(high['reset'][k,j])))
    for window,mask in masks(t).items():
        for j in range(6):
            changes.append(dict(window=window,joint=j+1,additional_high_reset_frames=int((mask&high['reset'][:,j]&~low['reset'][:,j]).sum()),low_only_reset_frames=int((mask&low['reset'][:,j]&~high['reset'][:,j]).sum()),different_mode_frames=int((mask&(high['mode'][:,j]!=low['mode'][:,j])).sum())))
    csvwrite(out/'mode_summary.csv',rows);csvwrite(out/'reset_events.csv',events);csvwrite(out/'mode_changes.csv',changes)
    fig,axes=plt.subplots(6,1,figsize=(11,12),sharex=True)
    for j,ax in enumerate(axes):
        for label,s in [('low',low),('high',high)]:
            ax.step(t,s['length'][:,j],where='post',label=label,lw=.9);ix=np.flatnonzero(s['reset'][:,j]);ax.scatter(t[ix],s['length'][ix,j],s=9)
        ax.set_ylabel('J'+str(j+1)+' window');ax.axvline(2.56,c='gray',ls=':');ax.grid(alpha=.2)
    axes[0].legend();axes[-1].set_xlabel('Time / s');fig.tight_layout();fig.savefig(out/'window_changes.png',dpi=140);plt.close(fig)
    return rows


def compare(cfg,base,out,ledger):
    root=Path(cfg['run_root']);pre=json.loads((root/'precheck/metrics.json').read_text());require(pre['passed'] and pre['KF_calls']+2061<=cfg['KF_cap'],'Precheck/budget failed')
    s,noise,noisy,groups=inputs(cfg,'high');ss={k:s[k][64:] for k in ['t','q','v']};t=ss['t'];high={};low={};np.savez_compressed(out/'current_inputs.npz',t=s['t'],noise=noise,noisy_current=noisy)
    for name in GROUPS:
        r=replay(cfg,base,ss,groups[name],ledger);high[name]=r;np.savez_compressed(out/(name+'.npz'),**r,**{k:groups[name][k] for k in ['current','mode','length','reset','threshold_checked']})
        old=load(Path(cfg['source_run'])/'compare'/(name+'.npz'));low[name]=old;require(np.array_equal(t,old['t']),'Low/high sample time differs')
        close(r['R'],old['R'],cfg,'R changed');close(r['covariance'][0],old['covariance'][0],cfg,'P0 changed');close(r['wrench'][0],old['wrench'][0],cfg,'Initial wrench changed');close(r['J'],old['J'],cfg,'J changed')
        close(r['Q'][:,6:,:],old['Q'][:,6:,:],cfg,'Qw/cross lower block changed');close(r['Q'][:,:6,6:],old['Q'][:,:6,6:],cfg,'Q cross upper changed')
        dt=r['dt'][1:,None,None];diff=(r['Q'][1:,:6,:6]-old['Q'][1:,:6,:6])/dt**2;expect=np.array([np.diag(a-b) for a,b in zip(r['used_sigma'][1:],old['used_sigma'][1:])]);close(diff,expect,cfg,'Q input block not T-squared variance mapping')
        print('Completed high-noise KF',name,flush=True)
    # Evaluation reference is read after all new estimator outputs have been saved.
    diag=load(Path(cfg['source_run'])/'collect/diagnostics.npz');require(np.array_equal(t,diag['t'][64:]),'Truth time mismatch');truth=diag['wrench'][64:]
    metrics=[];responses=[]
    for level,result in [('low',low),('high',high)]:
        rows,resp=evaluate(cfg,t,truth,result,level);metrics+=rows;responses+=resp
    original=list(csv.DictReader((Path(cfg['source_run'])/'compare/metrics_six_axes.csv').open()))
    for r in metrics:
        if r['level']=='low':
            old=next(x for x in original if all(x[k]==r[k] for k in ['group','window','axis']))
            for key in ['rmse','bias','sd','variance','peak_abs','peak_signed','peak_time_s']:close(r[key],float(old[key]),cfg,'Prior metric not recovered '+key)
    csvwrite(out/'metrics_six_axes.csv',metrics);csvwrite(out/'response.csv',responses);differences=[]
    for r in metrics:
        comparisons=[]
        if r['group']!='RAW':comparisons.append(('method_minus_RAW',next(x for x in metrics if x['level']==r['level'] and x['group']=='RAW' and x['window']==r['window'] and x['axis']==r['axis'])))
        if r['level']=='high':comparisons.append(('high_minus_low',next(x for x in metrics if x['level']=='low' and x['group']==r['group'] and x['window']==r['window'] and x['axis']==r['axis'])))
        for label,old in comparisons:differences.append(dict(level=r['level'],group=r['group'],window=r['window'],axis=r['axis'],unit=r['unit'],comparison=label,**{'delta_'+k:r[k]-old[k] for k in ['rmse','bias','sd','variance','peak_abs']}))
    csvwrite(out/'differences.csv',differences)
    mode_stats(cfg,out,t,low['MSMA'],groups['MSMA']);input_split(cfg,out,s['current'],noise,groups,t)
    previous=list(csv.DictReader((Path(cfg['source_run'])/'compare/input_split.csv').open()));current=list(csv.DictReader((out/'input_split.csv').open()));csvwrite(out/'input_split_both_levels.csv',[dict(level='low',**x) for x in previous]+[dict(level='high',**x) for x in current])
    fig,axes=plt.subplots(6,2,figsize=(14,14),sharex=True)
    for col,(level,result) in enumerate([('low',low),('high',high)]):
        for j,ax in enumerate(axes[:,col]):
            for name,r in result.items():ax.plot(t,r['wrench'][:,j],label=name,lw=.8)
            ax.plot(t,truth[:,j],'k--',lw=.9);ax.axvline(2.56,c='gray',ls=':');ax.set_ylabel(AXES[j]+(' / N' if j<3 else ' / N m'));ax.grid(alpha=.2)
            if j==0:ax.set_title(level+' noise');ax.legend(fontsize=7)
    fig.tight_layout();fig.savefig(out/'six_axes_levels.png',dpi=140);plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(12,7))
    for row,j in enumerate([2,3]):
        for col,window in enumerate([(2.4,3.1),(3.56,5.)]):
            mask=(t>=window[0])&(t<window[1]);ax=axes[row,col]
            for name,r in high.items():ax.plot(t[mask],r['wrench'][mask,j],label=name)
            ax.plot(t[mask],truth[mask,j],'k--');ax.set_ylabel(AXES[j]+(' / N' if j<3 else ' / N m'));ax.set_xlabel('Time / s');ax.legend(fontsize=8);ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(out/'high_noise_response.png',dpi=150);plt.close(fig)
    write(out/'checks.json',dict(passed=True,calls=ledger['KF_calls'],paired_saved_noise=True,unchanged_R_P0_Qw_J=True,T_squared_input_block=True,old_metrics_reproduced=True,truth_after_new_outputs=True))
    report=['# 3倍电流噪声：配对重标与三组比较','','物理日志不变；标准差0.05→0.15 N·m；各自重标输入协方差，固定窗口/阈值/Qw/R/P0。低等级输出复用，高等级三组重新递推。','']
    for level in ['low','high']:
        for window in ['transient','steady']:
            report += ['## '+level+' / '+window,'','各单元为RMSE / 偏置 / SD；Fx–Fz单位N，Mx–Mz单位N·m。','','|轴|RAW|HMA|MSMA|','|---|---:|---:|---:|']
            for axis in AXES:
                entries=[]
                for name in GROUPS:
                    r=next(x for x in metrics if x['level']==level and x['window']==window and x['axis']==axis and x['group']==name);entries.append(' / '.join(f'{r[k]:.6g}' for k in ['rmse','bias','sd']))
                report.append('|'+axis+'|'+'|'.join(entries)+'|')
    report+=['','## 主轴响应','','|等级/轴/指标|RAW|HMA|MSMA|','|---|---:|---:|---:|']
    for level in ['low','high']:
        for axis in ['Fz','Mx']:
            for key in ['t10_s','t90_s','rise_10_90_s','overshoot_percent','settle_5pct_s','remaining_in_band_s']:
                vals=[next(x[key] for x in responses if x['level']==level and x['axis']==axis and x['group']==name) for name in GROUPS];report.append('|'+level+' '+axis+' '+key+'|'+'|'.join('未达到' if v is None else f'{v:.6g}' for v in vals)+'|')
    report+=['','零目标方向不定义百分比超调与90%响应；串扰见六轴峰值/带符号事件时刻CSV。完整同等级方法差与高低等级差见differences.csv。模式检查分母、重置与转换次数见mode_summary.csv；重置时刻见reset_events.csv。','','![响应](high_noise_response.png)','','![窗口](window_changes.png)','','![全部六轴](six_axes_levels.png)']
    (out/'decision.md').write_text('\n'.join(report)+'\n')
    return dict(metric_rows=len(metrics),KF_calls=ledger['KF_calls'])


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--config',type=Path,required=True);ap.add_argument('--output-dir',type=Path,required=True);ap.add_argument('--mode',choices=['calibrate','precheck','compare'],required=True);a=ap.parse_args();cfg=json.loads(a.config.read_text());base=json.loads(Path(cfg['base_config']).read_text());out=a.output_dir;out.mkdir(parents=True,exist_ok=True);start=time.perf_counter();ledger=dict(KF_calls=0)
    try:
        result=recalibrate(cfg,out) if a.mode=='calibrate' else (precheck if a.mode=='precheck' else compare)(cfg,base,out,ledger)
        result.update(passed=True,physics_steps=0,KF_calls=ledger['KF_calls'],wall_seconds=time.perf_counter()-start);write(out/'metrics.json',result)
    except Exception as exc:
        write(out/'failure.json',dict(error=repr(exc),traceback=traceback.format_exc()));write(out/'metrics.json',dict(passed=False,physics_steps=0,KF_calls=ledger['KF_calls'],wall_seconds=time.perf_counter()-start));raise
    finally:write(out/'ledger.json',ledger)
if __name__=='__main__':main()
