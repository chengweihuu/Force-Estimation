"""Read-only diagnostics of stored instantaneous drive-torque input errors."""
import argparse,csv,json,signal,time,traceback
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def write(path,obj):
    with Path(path).open('x') as f: json.dump(obj,f,indent=2,ensure_ascii=False,allow_nan=False,default=lambda x:x.tolist() if isinstance(x,np.ndarray) else x.item())

def csvwrite(path,rows):
    with Path(path).open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)

def require(ok,msg):
    if not ok:raise RuntimeError(msg)

def load(path):
    with np.load(path,allow_pickle=False) as f:return {k:f[k].copy() for k in f.files}

def setup(cfg):
    root=Path(cfg['filter_run'])/'calibration'
    table={(r['method'],int(r['length']),int(r['joint'])):float(r['sample_variance_Nm2']) for r in csv.DictReader((root/'window_variances.csv').open())}
    data={}; closure={}
    for case in cfg['cases']:
        s=load(root/case/'filtered_sequences.npz')
        old=load(Path(cfg['current_run'])/'probe'/case/'current_measurements.npz')
        require(np.array_equal(s['t'],old['t'][64:]),'Time correspondence failed')
        require(np.array_equal(s['clean_current'],old['current'][64:]),'Clean current reference mismatch')
        require(np.allclose(np.diff(s['t']),.008,rtol=cfg['rtol'],atol=cfg['atol']),'Not 125 Hz')
        require(np.array_equal(s['raw_current'],s['noisy_current']),'Raw input mismatch')
        for method in cfg['methods']:
            for key in ['current','clean_output','mode','length','reset']:
                require(s[method+'_'+key].shape==(687,6),'Unexpected sequence shape')
            e=(s[method+'_current']-s['clean_current'])*cfg['D']
            l=(s[method+'_clean_output']-s['clean_current'])*cfg['D']
            n=(s[method+'_current']-s[method+'_clean_output'])*cfg['D']
            require(np.isfinite(e).all(),'Nonfinite error')
            require(np.allclose(e,l+n,rtol=cfg['rtol'],atol=cfg['atol']),'e=l+n closure failed')
            sigma=np.zeros_like(e)
            for k in range(len(e)):
                for j in range(6):
                    mode=int(s[method+'_mode'][k,j]);length=int(s[method+'_length'][k,j])
                    name={0:'raw',1:'wma',2:'hma'}[mode]
                    sigma[k,j]=table[name,length,j+1]
            require(np.isfinite(sigma).all() and np.all(sigma>0),'Invalid/missing calibration variance')
            mse=np.mean(e*e,axis=0)
            a=np.mean(l*l,axis=0);b=np.mean(n*n,axis=0);cross=2*np.mean(l*n,axis=0)
            require(np.allclose(mse,a+b+cross,rtol=cfg['rtol'],atol=cfg['atol']),'MSE component closure failed')
            bias=np.mean(e,axis=0);var=np.var(e,axis=0,ddof=0)
            require(np.allclose(mse,bias*bias+var,rtol=cfg['rtol'],atol=cfg['atol']),'Bias/variance closure failed')
            closure[case+'_'+method]=dict(error_max=float(np.max(np.abs(e-l-n))),mse_max=float(np.max(np.abs(mse-a-b-cross))),bias_variance_max=float(np.max(np.abs(mse-bias*bias-var))))
            data[case,method]=dict(e=e,l=l,n=n,sigma=sigma,s=s)
    return data,closure


KEYS=['mean_Nm','rmse_Nm','variance_population_Nm2','variance_sample_Nm2','bias_energy_fraction',
      'table_rms_Nm','MSE_over_table_variance','centered_variance_over_table','standardized_mean',
      'standardized_MSE','fraction_within_2sigma','clean_distortion_MSE_Nm2',
      'noise_and_switching_MSE_Nm2','cross_term_Nm2']

def moments(e,l,n,v,mask):
    count=int(mask.sum());result=dict(samples=count,**{k:None for k in KEYS})
    if not count:return result
    a,b,c,d=e[mask],l[mask],n[mask],v[mask]
    mean=float(a.mean());mse=float(np.mean(a*a));variance=float(np.var(a));z=a/np.sqrt(d)
    result.update(mean_Nm=mean,rmse_Nm=mse**.5,variance_population_Nm2=variance,
        variance_sample_Nm2=float(np.var(a,ddof=1)) if count>1 else None,
        bias_energy_fraction=mean*mean/mse if mse else None,table_rms_Nm=float(np.sqrt(d.mean())),
        MSE_over_table_variance=mse/float(d.mean()),centered_variance_over_table=variance/float(d.mean()),
        standardized_mean=float(z.mean()),standardized_MSE=float(np.mean(z*z)),fraction_within_2sigma=float(np.mean(np.abs(z)<=2)),
        clean_distortion_MSE_Nm2=float(np.mean(b*b)),noise_and_switching_MSE_Nm2=float(np.mean(c*c)),cross_term_Nm2=float(2*np.mean(b*c)))
    return result


def correlation(e,mask,lag):
    # Require every sample within the lag span to remain in the group.
    bad=np.r_[0,np.cumsum(~mask)]
    k=np.arange(lag,len(mask));valid=(bad[k+1]-bad[k-lag])==0;k=k[valid]
    if not len(k):return 0,None
    mu=np.mean(e[mask]);a=e[k]-mu;b=e[k-lag]-mu
    norm=float(np.sqrt(np.sum(a*a)*np.sum(b*b)))
    return len(k),float(a@b/norm) if norm>0 else None


def phases(mode,length,reset):
    phase=np.full(len(mode),'other',dtype=object);hma_age=0
    for k in range(len(mode)):
        if mode[k]==1:
            hma_age=0
            phase[k]='startup_growth' if k<50 else ('reset_frame' if reset[k] else 'postreset_growth')
        elif mode[k]==2:
            hma_age=hma_age+1 if k and mode[k-1]==2 else 1
            phase[k]='HMA_entry_6' if hma_age<=6 else 'HMA_later'
        else:phase[k]='raw'
    return phase


def diagnose(cfg,out,data,closure):
    overall=[];groups=[];acf=[];blocks=[];events=[]
    fig,axes=plt.subplots(6,2,figsize=(13,13),sharex=True)
    for case in cfg['cases']:
        for method in cfg['methods']:
            d=data[case,method];s=d['s'];t=s['t'];size=len(t)
            for j in range(6):
                base=dict(case=case,method=method,joint=j+1)
                e,l,n,v=(d[k][:,j] for k in ['e','l','n','sigma'])
                full=np.ones(size,dtype=bool)
                row={**base,**moments(e,l,n,v,full)};overall.append(row)
                mode=s[method+'_mode'][:,j];length=s[method+'_length'][:,j];reset=s[method+'_reset'][:,j]
                masks=[('full','full',full)]+[('mode',name,mode==code) for code,name in [(0,'raw'),(1,'WMA'),(2,'HMA')]]
                masks += [('WMA_length',str(k),(mode==1)&(length==k)) for k in range(1,51)]
                if method=='msma':
                    phase=phases(mode,length,reset)
                    masks += [('phase',name,phase==name) for name in ['startup_growth','reset_frame','postreset_growth','HMA_entry_6','HMA_later']]
                for kind,name,mask in masks:
                    groups.append(dict(**base,kind=kind,group=name,**moments(e,l,n,v,mask)))
                    for lag in cfg['lags']:
                        count,r=correlation(e,mask,lag);_,rz=correlation(e/np.sqrt(v),mask,lag)
                        acf.append(dict(**base,kind=kind,group=name,lag_frames=lag,lag_s=.008*lag,continuous_pairs=count,
                            correlation=r,standardized_correlation=rz))
                for start in range(0,size,50):
                    mask=full.copy();mask[:start]=False;mask[start+50:]=False
                    blocks.append(dict(**base,start_s=float(t[start]),end_s=float(t[min(start+49,size-1)]),
                        **moments(e,l,n,v,mask)))
                if method=='msma':
                    for k in np.flatnonzero(reset):
                        for label,lo,hi in [('preceding50',k-50,k),('reset',k,k+1),('growth_next49',k+1,k+50),('following6',k+50,k+56)]:
                            left=max(0,lo);right=min(size,hi);mask=np.zeros(size,dtype=bool)
                            if right>left:mask[left:right]=True
                            events.append(dict(**base,event_t_s=float(t[k]),segment=label,requested_samples=hi-lo,
                                truncated=int(mask.sum())!=hi-lo,other_resets=int(np.sum(reset[mask]))-int(mask[k]),
                                HMA_samples=int(np.sum(mode[mask]==2)),**moments(e,l,n,v,mask)))
                    column=cfg['cases'].index(case)
                    ax=axes[j,column];ax.plot(t,e,lw=.9,label='input error')
                    ax.fill_between(t,-2*np.sqrt(v),2*np.sqrt(v),color='orange',alpha=.4,label='+/- 2 table sigma')
                    for k in np.flatnonzero(reset):ax.axvline(t[k],color='red',ls=':',lw=.6)
                    ax.set_ylabel(f'J{j+1} [N m]');ax.grid(alpha=.2)
                    if j==0:ax.set_title(case+' / MSMA');ax.legend(fontsize=8)
                    if j==5:ax.set_xlabel('Time [s]')
    for name,rows in [('overall',overall),('groups',groups),('autocorrelation',acf),('blocks',blocks),('reset_events',events)]:csvwrite(out/(name+'.csv'),rows)
    fig.tight_layout();fig.savefig(out/'MSMA_error_and_static_band.png',dpi=140);plt.close(fig)
    write(out/'summary.json',dict(overall=overall,closure=closure))
    report=['# 实际运动输入误差的统计描述','',
      '只读 EXPLORE，四种处理、两场景、所有六轴；使用既有一次噪声实现。没有更新 KF 或重新滤波。', '',
      '## 定义与边界','',
      'e 为带噪滤波输出减干净瞬时驱动力矩，单位 N·m。表中统计覆盖 0.512–6 s 的全部687帧。方差表按每帧实际模式、窗口长度、关节匹配。',
      'MSE/表方差为 mean(e²)/mean(查表方差)；中心方差采用总体二阶矩以保证MSE=均值²+方差。样本方差另列。',
      '本误差是电流预处理对输入的扰动，不是包含动力学区间保持缺陷的完整动量预测残差。2σ带仅作描述，不代表高斯置信区间。', '',
      '## MSMA 六轴结果','',
      '| 场景 | 轴 | 均值／N·m | RMSE／N·m | 查表RMS／N·m | MSE/表方差 | 中心方差/表方差 | 连续lag1 |',
      '|---|---|---:|---:|---:|---:|---:|---:|']
    for r in overall:
        if r['method']=='msma':
            a=next(x for x in acf if x['case']==r['case'] and x['method']=='msma' and x['joint']==r['joint'] and x['kind']=='full' and x['lag_frames']==1)
            report.append('| '+r['case']+' | J'+str(r['joint'])+' | '+' | '.join(f"{r[k]:.6g}" for k in ['mean_Nm','rmse_Nm','table_rms_Nm','MSE_over_table_variance','centered_variance_over_table'])+f" | {a['correlation']:.6g} |")
    report += ['', '全部 raw/WMA/HMA/MSMA 结果见 overall.csv；各模式和WMA长度见 groups.csv；连续配对自相关见 autocorrelation.csv。', '',
      '## 总体均值与局部偏置','', '| 场景 | 轴 | 全段均值／N·m | 50帧分块最小均值／N·m | 最大均值／N·m |', '|---|---|---:|---:|---:|']
    for r in overall:
        if r['method']=='msma':
            b=[x['mean_Nm'] for x in blocks if x['case']==r['case'] and x['method']=='msma' and x['joint']==r['joint']]
            report.append(f"| {r['case']} | J{r['joint']} | {r['mean_Nm']:.6g} | {min(b):.6g} | {max(b):.6g} |")
    report += ['', '分块为原序列每50帧（0.4 s）不重叠分段，最后37帧单列；局部均值可能受趋势及样本波动影响，不是已辨识的永久偏置。', '',
      '## 过渡与窗口统计','',
      'MSMA阶段分为初次增长、重置当帧、重置后增长、每次进入HMA的前6帧和后续HMA。6由外层7点缓存长度确定；不是经验优化阈值。',
      '每个WMA固定长度通常只有初次增长及少数重置各一个点，不能据此可靠标定该长度的条件方差。未访问的组保留0样本和空统计，单样本不报告样本方差。',
      '分组相关只使用整个滞后跨度均在同组的连续样本对，不拼接不相邻时刻；没有连续对时相关留空，不能当作零相关。',
      '逐重置前50帧、当帧、随后49帧、之后6帧见reset_events.csv。最后约5.968 s的重置缺少后续数据；事件窗口可能重叠或包含下一次重置，已标记，不作为独立重复。', '',
      '![MSMA残差与静态方差带](MSMA_error_and_static_band.png)', '',
      '## 误差组成和闭合','',
      'e=l+n，其中l为干净输入的滤波失真，n为带噪与干净滤波输出之差。MSMA的n还包含模式选择变化，不能解释为独立传感噪声。',
      'overall.csv包含mean(l²)、mean(n²)、2mean(ln)，它们相加重构mean(e²)；不能直接相加RMSE。代数闭合见checks/summary，不等于因果机制完全分离。', '',
      '## 决策边界','',
      '依据以上数值判断静态方差表能否描述运动残差；不把比值或2σ覆盖比例当作正式统计检验，不通过增大Q或扣除偏置重新拟合结果。',
      '后续若接入协方差，需要区分运动相关失真与随机输入不确定性，并显式处理或评估跨周期相关性；本轮不选择具体修复。', '']
    (out/'decision.md').write_text('\n'.join(report))
    return dict(passed=True,overall_rows=len(overall),group_rows=len(groups),acf_rows=len(acf),new_KF_calls=0,new_physics_steps=0,new_filter_calls=0)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--config',type=Path,required=True);ap.add_argument('--output-dir',type=Path,required=True);ap.add_argument('--mode',choices=['precheck','diagnose'],required=True)
    args=ap.parse_args();cfg=json.loads(args.config.read_text());out=args.output_dir;out.mkdir(exist_ok=True,parents=True);start=time.perf_counter()
    def timeout(a,b):raise TimeoutError('Diagnostic wall limit')
    signal.signal(signal.SIGALRM,timeout);signal.alarm(25 if args.mode=='precheck' else 140)
    try:
        data,closure=setup(cfg)
        if args.mode=='precheck':
            write(out/'closure.json',closure);result=dict(passed=True,sequences=len(data),new_KF_calls=0,new_physics_steps=0,new_filter_calls=0)
        else:result=diagnose(cfg,out,data,closure)
        result['wall_seconds']=time.perf_counter()-start;write(out/'metrics.json',result);print(json.dumps(result),flush=True)
    except Exception as exc:
        write(out/'failure.json',dict(error=repr(exc),traceback=traceback.format_exc()))
        write(out/'metrics.json',dict(passed=False,error=repr(exc),wall_seconds=time.perf_counter()-start));raise
    finally:signal.alarm(0)

if __name__=='__main__':main()
