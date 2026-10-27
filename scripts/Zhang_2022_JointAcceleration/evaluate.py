import json
import numpy as np
from common import packet,write,csvwrite,core_mask
from acceleration import METHODS

def error_metrics(e):
    return dict(rmse=float(np.sqrt(np.mean(e*e))),bias=float(np.mean(e)),sd=float(np.std(e)),peak=float(abs(e).max()))

def run(c,root,out):
    import matplotlib.pyplot as plt
    records=json.loads((root/'replay/replay_index.json').read_text())
    rows=[];harmonics=[];noise_rows=[];sampling=[];closures=[];gmrows=[];x2rows=[]
    clean={};summary=[]
    for record in records:
        case=record['case'];mode=record['mode'];scene=c['cases'][case]
        h=packet(root/'replay'/mode/case/'history.npz');ref=packet(root/'collect'/case/'reference.npz')
        stride=round(1./record['rate_hz']/c['record_dt_s']);ref={key:val[::stride] for key,val in ref.items()};t=h['t'];assert np.array_equal(t,ref['t'])
        A=h['acceleration'];target=ref['qacc'];cm=core_mask(t,case)
        base=np.stack((h['f'],h['fstar'],h['fstar']),axis=1)
        closed=A[:,2:]-base-h['state'][:,:,1,:]-h['correction'][:,:,0,:]
        np.testing.assert_allclose(closed,0,rtol=0,atol=1e-9)
        assert np.all(h['state'][:,1,2]==0) and np.all(h['correction'][:,1,2]==0)
        closures.append(dict(mode=mode,case=case,acceleration_identity_max_rad_s2=float(abs(closed).max())))
        for window,mask in [('full',np.ones(len(t),dtype=bool)),('core',cm)]:
            for m,name in enumerate(METHODS):
                for j in range(6):
                    e=A[mask,m,j]-target[mask,j];metric=error_metrics(e);k=int(np.flatnonzero(mask)[np.argmax(abs(e))])
                    rows.append(dict(mode=mode,case=case,seed=record['seed'],window=window,method=name,joint=j+1,**metric,peak_time_s=float(t[k]),unit='rad/s^2'))
        for j in range(6):
            gmrows.append(dict(mode=mode,case=case,joint=j+1,**error_metrics((h['gm_residual']-ref['tau_ext'])[cm,j]),unit='N*m'))
            for mi,name in enumerate(METHODS[2:]):
                w=target-(h['f'] if mi==0 else h['fstar'])
                x2rows.append(dict(mode=mode,case=case,method=name,joint=j+1,**error_metrics((h['state'][:,mi,1]-w)[cm,j]),unit='rad/s^2'))
        freq=scene.get('frequency_Hz')
        if freq is not None:
            X=np.column_stack((np.sin(2*np.pi*freq*t[cm]),np.cos(2*np.pi*freq*t[cm]),np.ones(cm.sum())))
            trcoef=np.linalg.lstsq(X,target[cm],rcond=None)[0];refamp=np.hypot(trcoef[0],trcoef[1]);refphase=np.arctan2(trcoef[1],trcoef[0])
            for m,name in enumerate(METHODS):
                co=np.linalg.lstsq(X,A[cm,m],rcond=None)[0];amp=np.hypot(co[0],co[1]);phase=np.arctan2(co[1],co[0])
                for j in range(6):
                    valid=refamp[j]>=1e-8;pd=float(np.angle(np.exp(1j*(phase[j]-refphase[j])))*180/np.pi)
                    harmonics.append(dict(mode=mode,case=case,seed=record['seed'],method=name,joint=j+1,frequency_Hz=freq,reference_amplitude_rad_s2=float(refamp[j]),estimated_amplitude_rad_s2=float(amp[j]),amplitude_ratio=float(amp[j]/refamp[j]) if valid else None,phase_error_deg=pd if valid else None))
        if mode=='clean':
            clean[case]=dict(A=A.copy(),t=t.copy())
            for mi,name in enumerate(METHODS):summary.append(dict(case=case,method=name,J2_rmse_rad_s2=float(np.sqrt(np.mean((A[cm,mi,1]-target[cm,1])**2))),six_axis_vector_rmse_rad_s2=float(np.sqrt(np.mean(np.sum((A[cm,mi]-target[cm])**2,axis=1)))),max_axis_rmse_rad_s2=float(np.sqrt(np.mean((A[cm,mi]-target[cm])**2,axis=0)).max())))
            fig,axs=plt.subplots(3,2,figsize=(12,8),sharex=True)
            for j,ax in enumerate(axs.flat):
                ax.plot(t,target[:,j],'k',lw=1,label='MuJoCo qacc')
                for mi,name in enumerate(METHODS):ax.plot(t,A[:,mi,j],lw=.65,label=name,alpha=.8)
                ax.set(xlabel='Time s',ylabel=f'J{j+1} rad/s2');ax.grid(alpha=.2)
            axs[0,0].legend(fontsize=6,ncol=2);fig.tight_layout();fig.savefig(out/(case+'_six_axes.png'),dpi=120);plt.close(fig)
        elif mode.startswith('noise'):
            assert np.array_equal(t,clean[case]['t'])
            delta=A-clean[case]['A']
            for mi,name in enumerate(METHODS):
                for j in range(6):noise_rows.append(dict(mode=mode,case=case,seed=record['seed'],method=name,joint=j+1,**error_metrics(delta[cm,mi,j]),unit='rad/s^2'))
        elif mode=='rate2k':
            assert np.array_equal(t[::2],clean[case]['t'])
            mask=cm[::2];low=clean[case]['A'];high=A[::2];truth=target[::2]
            for mi,name in enumerate(METHODS):
                for j in range(6):
                    sampling.append(dict(case=case,method=name,joint=j+1,rmse_1k_rad_s2=float(np.sqrt(np.mean((low[mask,mi,j]-truth[mask,j])**2))),rmse_2k_rad_s2=float(np.sqrt(np.mean((high[mask,mi,j]-truth[mask,j])**2))),output_difference_rmse_rad_s2=float(np.sqrt(np.mean((high[mask,mi,j]-low[mask,mi,j])**2)))))
    # All seed outcomes retained, without pooling frames as independent repeats.
    aggreg=[]
    for case in c['cases']:
        for method in METHODS:
            for j in range(1,7):
                group=[x for x in rows if x['mode'].startswith('noise') and x['case']==case and x['window']=='core' and x['method']==method and x['joint']==j]
                expected=sum(m['name'].startswith('noise') and case in m.get('cases',c['cases']) for m in c['replay_modes'])
                assert len(group)==expected
                if not group:continue
                nr=[x for x in noise_rows if x['case']==case and x['method']==method and x['joint']==j]
                aggreg.append(dict(case=case,method=method,joint=j,rmse_mean_rad_s2=float(np.mean([x['rmse'] for x in group])),rmse_seed_sd_rad_s2=float(np.std([x['rmse'] for x in group],ddof=1)),noise_increment_rmse_mean_rad_s2=float(np.mean([x['rmse'] for x in nr])),temporal_error_sd_mean_rad_s2=float(np.mean([x['sd'] for x in group]))))
    # Whole six-axis trade-off list, relative to each structural comparator.
    tradeoffs=[]
    for mode in ['clean','friction']:
        for case in c['cases']:
            for j in range(1,7):
                g={x['method']:x for x in rows if x['mode']==mode and x['case']==case and x['joint']==j and x['window']=='core'}
                if not g:continue
                for baseline in METHODS[:-1]:
                    diff=g['gm_hodo']['rmse']-g[baseline]['rmse']
                    tradeoffs.append(dict(mode=mode,case=case,joint=j,baseline=baseline,full_rmse_rad_s2=g['gm_hodo']['rmse'],baseline_rmse_rad_s2=g[baseline]['rmse'],delta_rad_s2=diff,comparison='higher' if diff>1e-9+1e-8*abs(g[baseline]['rmse']) else 'lower' if diff<-(1e-9+1e-8*abs(g[baseline]['rmse'])) else 'numerically_equal'))
    for file,data in [('six_axis_errors.csv',rows),('harmonics.csv',harmonics),('noise_increments.csv',noise_rows),('noise_summary.csv',aggreg),('sampling.csv',sampling),('clean_summary.csv',summary),('gm_errors.csv',gmrows),('residual_state_errors.csv',x2rows),('all_tradeoffs.csv',tradeoffs)]:csvwrite(out/file,data)
    write(out/'closure_checks.json',closures)
    frequency_summary(c,rows,harmonics,out)
    # Save primary table and interpretation boundary now; interpretation added after reading values.
    text=['# Zhang 2022：UR5关节加速度固定参数复现','','本轮是方法适配与结构消融，未使用作者未公开的完整增益，也未做等噪声整定。主要输出为原始加速度，不加事后低通。','','## 干净模型：J2核心窗口 RMSE（rad/s²）','','| 场景 | 名义 | GM | HODO | GM＋两层 | GM＋三层 |','|---|---:|---:|---:|---:|---:|']
    for case in c['cases']:
        vals={x['method']:x['J2_rmse_rad_s2'] for x in summary if x['case']==case};text.append('| '+case+' | '+' | '.join(f'{vals[m]:.6g}' for m in METHODS)+' |')
    text += ['','核心窗口：静止[0.2,2]s，其余[2,8]s；全时段六轴误差和峰值另存，未移除不利通道。六轴向量RMSE与逐轴RMSE分列。','', '方法定义：名义动力学f；GM补偿fstar；HODO无GM；GM加两层（第三层始终清零）；GM加三层。完整输出包含u1。力矩与加速度单位不混用。所有方法共享参数与同一测量，每个HODO有独立状态。','', '模型和参考：Plant16kHz、控制1kHz，主要观测1kHz。即时qacc只用于评价；施力器读取Plant惯性以构造J2加速度等效扰动，估计器未读取真值。既有隐式阻尼与离散时间对应问题仍存在，不将反馈速度差分直接等同瞬时qacc。','', '本次实际重放组：'+', '.join(dict.fromkeys(r['mode'] for r in records))+'。噪声、摩擦失配和采样对照仅在配置启用时运行；各组定义见配置快照，不能把未运行的组列为已完成。','', '全部变差项见all_tradeoffs.csv，完整六轴、全窗口和频率幅相见CSV。连续有限时间证明未独立验证；未做真机、KF、控制闭环或调参。']
    (out/'decision.md').write_text('\n'.join(text)+'\n')
    return dict(passed=True,engineering_checks_passed=True,performance_acceptance='not asserted; fixed-parameter descriptive ablation',rows=len(rows),simulation_steps=0,observer_calls=0,model_evaluations=0)


def frequency_summary(c,rows,harmonics,out):
    """Report current-run frequencies; no dependency on historical output directories."""
    import matplotlib.pyplot as plt
    main=[];paired=[]
    scenes=sorted(((v['frequency_Hz'],k) for k,v in c['cases'].items() if 'frequency_Hz' in v))
    for frequency,case in scenes:
        for method in METHODS:
            row=next(x for x in rows if x['mode']=='clean' and x['case']==case and x['method']==method and x['joint']==2 and x['window']=='core')
            harm=next(x for x in harmonics if x['mode']=='clean' and x['case']==case and x['method']==method and x['joint']==2)
            main.append(dict(frequency_Hz=frequency,method=method,rmse_rad_s2=row['rmse'],amplitude_ratio=harm['amplitude_ratio'],phase_error_deg=harm['phase_error_deg']))
        for joint in range(1,7):
            group={x['method']:x['rmse'] for x in rows if x['mode']=='clean' and x['case']==case and x['joint']==joint and x['window']=='core'}
            delta=group['gm_hodo']-group['gm_two_layer'];tol=1e-9+1e-8*abs(group['gm_two_layer'])
            paired.append(dict(frequency_Hz=frequency,joint=joint,two_layer_rmse_rad_s2=group['gm_two_layer'],three_layer_rmse_rad_s2=group['gm_hodo'],three_minus_two_rad_s2=delta,comparison='three_lower' if delta < -tol else 'two_lower' if delta > tol else 'numerically_equal'))
    if not main:return
    csvwrite(out/'J2_frequency_comparison.csv',main);csvwrite(out/'paired_six_axis.csv',paired)
    fig,axes=plt.subplots(1,3,figsize=(13,4))
    for method in METHODS:
        points=[x for x in main if x['method']==method]
        for ax,key in zip(axes,['rmse_rad_s2','amplitude_ratio','phase_error_deg']):
            ax.scatter([x['frequency_Hz'] for x in points],[x[key] for x in points],s=20,label=method)
            ax.set(xlabel='Input frequency Hz',ylabel=key);ax.grid(alpha=.2)
    axes[0].set_yscale('log');axes[0].legend(fontsize=6)
    axes[1].axhline(1,color='k',ls=':',lw=.7);axes[2].axhline(0,color='k',ls=':',lw=.7)
    fig.tight_layout();fig.savefig(out/'J2_frequency_comparison.png',dpi=130);plt.close(fig)
