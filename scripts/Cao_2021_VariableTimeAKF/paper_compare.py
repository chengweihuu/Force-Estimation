"""Paired raw/HMA/MSMA comparison; fixed table and no outcome-based tuning."""
import argparse,json,sys,time,traceback
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from akf_comparison import CaptureObserver,read_table,close
from input_statistics import load,write,csvwrite,require
from msma import process,linear_series,wma_weights,hma_weights

GROUPS={'RAW':'raw','HMA':'hma','MSMA':'msma'}
AXES=['Fx','Fy','Fz','Mx','My','Mz']


def filter_inputs(cfg,noisy):
    thresholds=json.loads(Path(cfg['threshold_file']).read_text())['threshold_Aeq'];table=read_table(cfg);groups={}
    for g,method in GROUPS.items():
        s=process(noisy,method,50,thresholds,64);var=np.zeros_like(s['current'])
        for k in range(len(var)):
            for j in range(6):var[k,j]=table[{0:'raw',1:'wma',2:'hma'}[int(s['mode'][k,j])],int(s['length'][k,j]),j+1]
        s['sigma']=var;groups[g]=s
    close(groups['HMA']['current'],linear_series(noisy,hma_weights(50))[64:],cfg,'HMA direct kernel mismatch')
    close(groups['RAW']['current'],noisy[64:],cfg,'RAW input mismatch')
    return groups


def replay(cfg,base,s,g,ledger,N=None):
    N=len(s['t']) if N is None else N;obs=CaptureObserver(base,True);records={}
    for k in range(N):
        require(ledger['KF_calls']<cfg['KF_cap'],'KF cap exceeded');ledger['KF_calls']+=1
        r=obs.update(s['t'][k],s['q'][k],s['v'][k],g['current'][k]*cfg['D'],g['sigma'][k])
        require(all(np.isfinite(x).all() for x in r.values()),'Nonfinite KF result')
        require(np.linalg.eigvalsh(r['covariance'])[0]>0,'P not positive definite')
        if k:require(np.linalg.eigvalsh(r['Q'])[0]>0,'Q not positive definite')
        for key,val in r.items():records.setdefault(key,[]).append(np.array(val,copy=True))
    r={k:np.asarray(v) for k,v in records.items()};close(r['used_sigma'][1:],r['selected_sigma'][:-1],cfg,'Variance timestamp mismatch');close(r['input_source_t'][1:],r['t'][:-1],cfg,'Input timestamp mismatch')
    return r


def precheck(cfg,base,out,ledger):
    old=load(Path(cfg['filter_run'])/'calibration/step/filtered_sequences.npz');noisy=np.concatenate([old['prehistory_noisy_current'],old['noisy_current']]);groups=filter_inputs(cfg,noisy)
    close(groups['MSMA']['current'],old['msma_current'],cfg,'MSMA input changed')
    require(np.array_equal(groups['MSMA']['mode'],old['msma_mode']) and np.array_equal(groups['MSMA']['length'],old['msma_length']),'MSMA schedule changed')
    source={'RAW':Path(cfg['raw_run'])/'compare/step_D.npz','MSMA':Path(cfg['akf_run'])/'compare/step_C.npz'};records={};errors={}
    for name in GROUPS:
        r=replay(cfg,base,old,groups[name],ledger,151);records[name]=r
        if name in source:
            saved=load(source[name]);errors[name]=max(close(r[key],saved[key][:151],cfg,name+' old output changed '+key) for key in ['wrench','covariance','gain','Q','R','nominal_input','innovation'])
        np.savez_compressed(out/(name+'.npz'),**r)
    for name in ['HMA','MSMA']:
        close(records[name]['R'],records['RAW']['R'],cfg,'R differs');close(records[name]['covariance'][0],records['RAW']['covariance'][0],cfg,'P0 differs');close(records[name]['Q'][:,6:,:],records['RAW']['Q'][:,6:,:],cfg,'Qw/cross blocks differ')
    # Package versions are binding assumptions, not scientific outcomes.
    import mujoco,scipy,platform
    ver=dict(mujoco=mujoco.__version__,numpy=np.__version__,scipy=scipy.__version__,python=platform.python_version())
    for key,val in base['expected_versions'].items():require(ver[key]==val,'Version mismatch '+key)
    write(out/'checks.json',dict(passed=True,old_baseline_max=errors,versions=ver,HMA_direct_kernel=True,same_R_P0_Qw=True))
    return dict(precheck_calls=ledger['KF_calls'])


def masks(t):
    return dict(preload=(t>=2)&(t<2.56),transient=(t>=2.56)&(t<3.06),steady=(t>=3.56)&(t<5),loaded=(t>=2.56)&(t<5),full=np.ones(len(t),bool))


def input_split(cfg,out,clean,noise,groups,t):
    rows=[]
    for name,g in groups.items():
        fc=np.zeros_like(g['current']);fn=fc.copy()
        for k in range(len(fc)):
            idx=64+k
            for j in range(6):
                mode=int(g['mode'][k,j]);n=int(g['length'][k,j]);w=np.ones(1) if mode==0 else (wma_weights(n) if mode==1 else hma_weights(50));ix=idx-np.arange(len(w));fc[k,j]=w@clean[ix,j];fn[k,j]=w@noise[ix,j]
        close(fc+fn,g['current'],cfg,'Frozen input decomposition failed');dist=(fc-clean[64:])*cfg['D'];prop=fn*cfg['D'];e=(g['current']-clean[64:])*cfg['D'];close(e,dist+prop,cfg,'Input error split failed')
        np.savez_compressed(out/(name+'_input_split.npz'),t=t,distortion=dist,noise=prop,total=e,mode=g['mode'],length=g['length'],reset=g['reset'])
        for window,mask in masks(t).items():
            for j in range(6):
                a=float(np.mean(dist[mask,j]**2));b=float(np.mean(prop[mask,j]**2));cross=float(2*np.mean(dist[mask,j]*prop[mask,j]));total=float(np.mean(e[mask,j]**2));close(total,a+b+cross,cfg,'Input MSE closure')
                rows.append(dict(group=name,window=window,joint=j+1,unit='N m',total_rmse=np.sqrt(total),distortion_rms=np.sqrt(a),noise_rms=np.sqrt(b),cross_Nm2=cross))
    csvwrite(out/'input_split.csv',rows)


def compare(cfg,base,out,ledger):
    root=Path(cfg['run_root']);require(json.loads((root/'collect/metrics.json').read_text())['passed'],'Collection gate not passed');require(json.loads((root/'precheck/metrics.json').read_text())['KF_calls']+2061<=cfg['KF_cap'],'Cumulative KF budget exceeded')
    meas=load(root/'collect/measurements.npz');require(len(meas['t'])==751,'Wrong sample count');noise=np.random.default_rng(cfg['seed']).normal(0,.05,(751,6))/cfg['D'];noisy=meas['current']+noise;groups=filter_inputs(cfg,noisy);s={k:meas[k][64:] for k in ['t','q','v']};results={}
    np.savez_compressed(out/'current_inputs.npz',t=meas['t'],noisy_current=noisy,noise=noise)
    for name,g in groups.items():
        r=replay(cfg,base,s,g,ledger);results[name]=r;np.savez_compressed(out/(name+'.npz'),**r,current=g['current'],mode=g['mode'],length=g['length'],reset=g['reset'],threshold_checked=g['threshold_checked']);print('KF complete',name,flush=True)
    for name in ['HMA','MSMA']:
        close(results[name]['R'],results['RAW']['R'],cfg,'R differs');close(results[name]['covariance'][0],results['RAW']['covariance'][0],cfg,'P0 differs');close(results[name]['Q'][:,6:,:],results['RAW']['Q'][:,6:,:],cfg,'Qw/cross differs')
    # Truth is read only after all estimates have been saved.
    diag=load(root/'collect/diagnostics.npz');t=s['t'];require(np.array_equal(t,diag['t'][64:]),'Truth alignment error');truth=diag['wrench'][64:];metrics=[];responses=[]
    for name,r in results.items():
        for window,mask in masks(t).items():
            for j,axis in enumerate(AXES):
                e=r['wrench'][mask,j]-truth[mask,j];var=float(e.var());bias=float(e.mean());mse=float(np.mean(e*e));close(mse,var+bias*bias,cfg,'RMSE variance closure');ix=int(np.argmax(abs(e)))
                metrics.append(dict(group=name,window=window,axis=axis,unit='N' if j<3 else 'N m',samples=len(e),rmse=np.sqrt(mse),bias=bias,variance=var,sd=np.sqrt(var),peak_abs=float(abs(e[ix])),peak_signed=float(e[ix]),peak_time_s=float(t[mask][ix])))
        mask=masks(t)['loaded'];tt=t[mask];
        for j in [2,3]:
            amp=abs(cfg['wrench'][j]);z=np.sign(cfg['wrench'][j])*r['wrench'][mask,j]
            def cross(frac):
                hit=np.flatnonzero(z>=frac*amp);return float(tt[hit[0]]-2.56) if len(hit) else None
            good=np.logical_and.accumulate((abs(z-amp)<=.05*amp)[::-1])[::-1];hit=np.flatnonzero(good)
            responses.append(dict(group=name,axis=AXES[j],unit='N' if j<3 else 'N m',t10_s=cross(.1),t90_s=cross(.9),overshoot=float(max(0,z.max()-amp)),overshoot_percent=float(max(0,z.max()-amp)/amp*100),settle_5pct_s=float(tt[hit[0]]-2.56) if len(hit) else None,remaining_in_band_s=float(tt[-1]-tt[hit[0]]) if len(hit) else None))
    csvwrite(out/'metrics_six_axes.csv',metrics);csvwrite(out/'response.csv',responses)
    differences=[]
    for r in metrics:
        if r['group']=='RAW':continue
        old=next(x for x in metrics if x['group']=='RAW' and x['window']==r['window'] and x['axis']==r['axis'])
        differences.append(dict(group=r['group'],window=r['window'],axis=r['axis'],unit=r['unit'],delta_rmse=r['rmse']-old['rmse'],delta_variance=r['variance']-old['variance'],delta_bias=r['bias']-old['bias']))
    csvwrite(out/'differences.csv',differences);input_split(cfg,out,meas['current'],noise,groups,t)
    fig,axes=plt.subplots(6,1,figsize=(11,13),sharex=True)
    for j,ax in enumerate(axes):
        for name,r in results.items():ax.plot(t,r['wrench'][:,j],label=name,lw=.9)
        ax.plot(t,truth[:,j],'k--',label='truth');ax.axvline(2.56,c='gray',ls=':');ax.set_ylabel(AXES[j]+(' / N' if j<3 else ' / N m'));ax.grid(alpha=.2)
    axes[0].legend();axes[-1].set_xlabel('Time / s');fig.tight_layout();fig.savefig(out/'six_axes.png',dpi=140);plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(12,7));
    for row,j in enumerate([2,3]):
        for col,window in enumerate([(2.4,3.25),(3.56,5.)]):
            ax=axes[row,col];mask=(t>=window[0])&(t<window[1]);
            for name,r in results.items():ax.plot(t[mask],r['wrench'][mask,j],label=name)
            ax.plot(t[mask],truth[mask,j],'k--');ax.set_ylabel(AXES[j]+(' / N' if j<3 else ' / N m'));ax.grid(alpha=.2);ax.set_xlabel('Time / s');ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(out/'loaded_response.png',dpi=150);plt.close(fig)
    write(out/'checks.json',dict(passed=True,calls=ledger['KF_calls'],same_R_P0_Qw=True,truth_read_after_estimation=True,measurement_velocity_noise=0,R_is_assumed=True))
    report=['# 固定竖直运动与偏置载荷：三组对照','','探索性机制复现：同一物理日志与噪声种子8601；不是完整夹持接合或真机复现。R仍为共同假设，D与摩擦已知。','','|窗口/指标|RAW|HMA|MSMA|','|---|---:|---:|---:|']
    for window in ['preload','loaded','steady']:
        for axis in ['Fz','Mx']:
            for field in ['rmse','variance','bias']:
                vals=[next(x[field] for x in metrics if x['group']==g and x['window']==window and x['axis']==axis) for g in GROUPS];unit='N' if axis=='Fz' else 'N·m';unit='('+unit+')²' if field=='variance' else unit;report.append('|'+window+' '+axis+' '+field+' / '+unit+'|'+'|'.join(f'{v:.9g}' for v in vals)+'|')
    report+=['','|轴/响应|RAW|HMA|MSMA|','|---|---:|---:|---:|']
    for axis in ['Fz','Mx']:
        for field in ['t90_s','overshoot_percent','settle_5pct_s','remaining_in_band_s']:
            vals=[next(x[field] for x in responses if x['group']==g and x['axis']==axis) for g in GROUPS];report.append('|'+axis+' '+field+'|'+'|'.join('未达到' if v is None else f'{v:.9g}' for v in vals)+'|')
    report+=['','全六轴见metrics_six_axes.csv，配对绝对差见differences.csv；带内剩余时长不能省略。单一种子不支持显著性判断。后续不自动调参或追加运行。','','![六轴](six_axes.png)','','![响应](loaded_response.png)']
    (out/'decision.md').write_text('\n'.join(report)+'\n');return dict(metric_rows=len(metrics))


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--config',type=Path,required=True);ap.add_argument('--output-dir',type=Path,required=True);ap.add_argument('--mode',choices=['precheck','compare'],required=True);a=ap.parse_args();cfg=json.loads(a.config.read_text());base=json.loads(Path(cfg['base_config']).read_text());out=a.output_dir;out.mkdir(parents=True,exist_ok=True);start=time.perf_counter();ledger=dict(KF_calls=0,physics_steps=0)
    try:
        r=(precheck if a.mode=='precheck' else compare)(cfg,base,out,ledger);write(out/'metrics.json',dict(passed=True,wall_seconds=time.perf_counter()-start,**ledger,**r))
    except Exception as exc:
        write(out/'failure.json',dict(error=repr(exc),traceback=traceback.format_exc()));write(out/'metrics.json',dict(passed=False,wall_seconds=time.perf_counter()-start,**ledger));raise
    finally:write(out/'ledger.json',ledger)
if __name__=='__main__':main()
