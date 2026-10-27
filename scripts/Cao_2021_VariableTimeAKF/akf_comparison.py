"""Residual-table calibration and three paired momentum-KF input treatments."""
import argparse,json,time,traceback
from pathlib import Path
import numpy as np
from scipy.linalg import expm
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from input_statistics import write,csvwrite,load,require
from runner import load as load_module
from msma import wma_weights,hma_weights,linear_series


def close(a,b,cfg,label):
    require(np.allclose(a,b,rtol=cfg['rtol'],atol=cfg['atol']),label)
    return float(np.max(abs(np.asarray(a)-np.asarray(b))))


def calibrate(cfg,out):
    cc=json.loads(Path(cfg['constant_config']).read_text());rows=[];table=[];D=np.asarray(cfg['D'])
    kernels=[('raw',1,np.ones(1))]+[('wma',n,wma_weights(n)) for n in range(1,51)]+[('hma',50,hma_weights(50))]
    for case in cc['cases']:
        folder='pilot' if case['name']==cc['pilot_case'] else 'collect'
        s=load(Path(cfg['constant_run'])/folder/case['name']/'measurements.npz')
        nz=load(Path(cfg['constant_analysis'])/case['name']/'noise.npz')
        require(np.array_equal(s['t'],nz['t']),'Noise/measurement time mismatch')
        x=s['current']+nz['noise'];mask=(s['t']>=3)&(s['t']<7);j=case['joint']-1
        require(mask.sum()==500 and np.flatnonzero(mask)[0]>=55,'Invalid full-history core')
        for method,n,w in kernels:
            filtered=linear_series(x,w)*D
            residual=s['tau_ref_no_acc'][mask,j]-filtered[mask,j]
            rows.append(dict(case=case['name'],joint=j+1,method=method,length=n,samples=len(residual),mean_Nm=float(residual.mean()),sample_variance_Nm2=float(residual.var(ddof=1)),rmse_Nm=float(np.sqrt(np.mean(residual**2)))))
    for j in range(1,7):
        for method,n,_ in kernels:
            pair=[r for r in rows if r['joint']==j and r['method']==method and r['length']==n];require(len(pair)==2,'Missing signed calibration pair')
            value=float(np.mean([r['sample_variance_Nm2'] for r in pair]));require(np.isfinite(value) and value>0,'Nonpositive/nonfinite table entry')
            table.append(dict(joint=j,method=method,length=n,variance_Nm2=value,positive_mean_Nm=pair[0]['mean_Nm'],negative_mean_Nm=pair[1]['mean_Nm'],pooled_MSE_Nm2=float(np.mean([r['rmse_Nm']**2 for r in pair]))))
    csvwrite(out/'direction_statistics.csv',rows);csvwrite(out/'residual_variance_table.csv',table)
    # Independent saved fixed-HMA50 statistics from the preceding calibration.
    import csv
    old=list(csv.DictReader((Path(cfg['constant_analysis'])/'all_axes_windows.csv').open()))
    maxdiff=0.
    for r in rows:
        if r['method']=='hma':
            prior=next(x for x in old if x['case']==r['case'] and int(x['joint'])==r['joint'] and x['method']=='hma' and x['window']=='50')
            maxdiff=max(maxdiff,close(r['sample_variance_Nm2'],float(prior['residual_sample_variance_Nm2']),cfg,'HMA calibration does not reproduce old result'))
    write(out/'checks.json',dict(entries=len(table),direction_rows=len(rows),old_HMA50_max_variance_difference=maxdiff,formula='equal mean of two within-direction ddof=1 variances; no bias compensation'))
    return dict(new_KF_calls=0,entries=len(table))


def read_table(cfg):
    import csv
    table={(r['method'],int(r['length']),int(r['joint'])):float(r['variance_Nm2']) for r in csv.DictReader((Path(cfg['table_dir'])/'residual_variance_table.csv').open())}
    require(len(table)==312 and all(np.isfinite(x) and x>0 for x in table.values()),'Incomplete/nonfinite table')
    return table


def dynamic(cfg,case,table):
    s=load(Path(cfg['filter_run'])/'calibration'/case/'filtered_sequences.npz')
    require(len(s['t'])==687 and s['t'][0]==.512 and s['t'][-1]==6,'Wrong dynamic range')
    close(np.diff(s['t']),np.full(686,.008),cfg,'Non-125Hz packet')
    sigma=np.empty_like(s['msma_current'])
    for k in range(len(sigma)):
        for j in range(6):
            mode=int(s['msma_mode'][k,j]);n=int(s['msma_length'][k,j]);require(mode in (1,2),'Unexpected mode')
            if s['msma_reset'][k,j]:require(mode==1 and n==1,'Reset did not select WMA1')
            sigma[k,j]=table['wma' if mode==1 else 'hma',n,j+1]
    return s,sigma


class CaptureObserver:
    """Independent dynamic model. Only measurement and calibrated variance enter."""
    def __init__(self,base,adaptive):
        self.model=load_module(base['oracle_source'],'retained_nominal').Model(base['model'],base['derivative_epsilon_s'])
        self.kernel=load_module(base['kf_source'],'retained_kernel');c=base['observer']
        self.Qp=np.diag(c['Qp_continuous_diag']);self.Qw=np.diag(c['Qw_continuous_diag']);self.Sv=np.diag(np.square(c['assumed_velocity_std_rad_s']));self.Pw=np.diag(c['initial_wrench_variance']);self.previous=None;self.adaptive=adaptive

    def update(self,t,q,v,tau,sigma):
        M,h,Md,J=self.model.evaluate(q,v);p=M@v;R=M@self.Sv@M.T
        K=np.zeros((12,6));Q=np.zeros((12,12));used=np.zeros(6);source_t=t;dt=0.;xp=np.r_[p,np.zeros(6)];innovation=np.zeros(6)
        if self.previous is None:
            self.x=xp.copy();self.P=np.zeros((12,12));self.P[:6,:6]=R;self.P[6:,6:]=self.Pw
        else:
            source_t,u0,J0,s0=self.previous;dt=t-source_t;require(dt>0,'Nonpositive dt');used=s0.copy()
            Qpc=dt*np.diag(s0) if self.adaptive else self.Qp
            A,B,Q=self.kernel.discretize(J0,dt,Qpc,self.Qw)
            xp=A@self.x+B@u0;Pp=A@self.P@A.T+Q;innovation=p-xp[:6]
            S=Pp[:6,:6]+R;K=np.linalg.solve(S,Pp[:,:6].T).T;C=np.eye(6,12);left=np.eye(12)-K@C
            self.x=xp+K@innovation;P=left@Pp@left.T+K@R@K.T;self.P=(P+P.T)/2
        u=Md@v+tau-self.model.B*v-h
        self.previous=(t,u.copy(),J.copy(),sigma.copy())
        return dict(t=t,wrench=self.x[6:].copy(),covariance=self.P.copy(),gain=K,Q=Q,R=R,momentum=p,nominal_input=u,innovation=innovation,predicted_state=xp,J=J,selected_sigma=sigma.copy(),used_sigma=used,input_source_t=source_t,dt=dt)


def run_filter(cfg,base,s,sigma,group,ledger,length=None,override=None):
    N=len(s['t']) if length is None else length;obs=CaptureObserver(base,group=='C');records={}
    currents=s['raw_current'] if group=='A' else s['msma_current']
    for k in range(N):
        require(ledger['calls']<cfg['new_KF_cap'],'KF call budget exceeded');ledger['calls']+=1
        var=(sigma[k] if group=='C' else np.asarray(base['observer']['Qp_continuous_diag'])/.008) if override is None else override
        r=obs.update(s['t'][k],s['q'][k],s['v'][k],currents[k]*cfg['D'],var)
        require(all(np.isfinite(v).all() for v in r.values()),'Nonfinite recursive result')
        require(np.linalg.eigvalsh(r['covariance'])[0]>0,'Nonpositive covariance')
        if k:require(np.linalg.eigvalsh(r['Q'])[0]>0,'Nonpositive process covariance')
        for key,value in r.items():records.setdefault(key,[]).append(np.array(value,copy=True))
    r={k:np.asarray(v) for k,v in records.items()}
    close(r['used_sigma'][1:],r['selected_sigma'][:-1],cfg,'Variance uses wrong sample index')
    close(r['input_source_t'][1:],r['t'][:-1],cfg,'Input uses wrong sample timestamp')
    return r


def precheck(cfg,out,ledger):
    base=json.loads(Path(cfg['base_config']).read_text());table=read_table(cfg);s,sigma=dynamic(cfg,'step',table)
    kernel=load_module(base['kf_source'],'precheck_kernel');nominal=load_module(base['oracle_source'],'precheck_nominal').Model(base['model'],base['derivative_epsilon_s']);_,_,_,J=nominal.evaluate(s['q'][0],s['v'][0]);Qw=np.diag(base['observer']['Qw_continuous_diag'])
    F=np.zeros((12,12));F[:6,6:]=J.T;errs=[]
    for dt in [.004,.008,.016]:
        Qpc=dt*np.diag(sigma[0]);A,B,Q=kernel.discretize(J,dt,Qpc,Qw)
        G=np.zeros((12,12));G[:6,:6]=Qpc;G[6:,6:]=Qw
        block=np.block([[-F,G],[np.zeros_like(F),F.T]]);E=expm(block*dt);via_expm=E[12:,12:].T@E[:12,12:]
        errs.append(close(Q/dt,via_expm/dt,cfg,'Closed form vs matrix exponential'))
        close((Q[:6,:6]-dt**3/3*(J.T@Qw@J))/dt**2,np.diag(sigma[0]),cfg,'Input variance is not T squared')
        close(Q[:6,6:]/dt**2,(J.T@Qw)/2,cfg,'Cross block incorrect/removed')
        require(np.linalg.eigvalsh(Q)[0]>0,'Q not positive definite')
    rec={name:run_filter(cfg,base,s,sigma,name,ledger,151) for name in ['A','B','C']}
    baseline_variance=np.asarray(base['observer']['Qp_continuous_diag'])/.008
    fixed=run_filter(cfg,base,s,sigma,'C',ledger,151,override=baseline_variance)
    baseline_max=0.
    for name,method in [('A','raw'),('B','msma')]:
        old=load(Path(cfg['filter_run'])/'integration'/(method+'_estimates.npz'))
        for key in ['wrench','covariance','momentum','nominal_input','innovation']:
            baseline_max=max(baseline_max,close(rec[name][key],old[key],cfg,'Saved baseline mismatch: '+name+key))
    fixedmax=close(fixed['wrench'],rec['B']['wrench'],cfg,'Baseline-equivalent table output mismatch')
    close(fixed['covariance'],rec['B']['covariance'],cfg,'Baseline-equivalent table P mismatch')
    close(rec['A']['covariance'],rec['B']['covariance'],cfg,'A/B P differs');close(rec['A']['gain'],rec['B']['gain'],cfg,'A/B K differs')
    close(rec['B']['nominal_input'],rec['C']['nominal_input'],cfg,'B/C input differs')
    write(out/'checks.json',dict(passed=True,calls=ledger['calls'],saved_baseline_max=baseline_max,fixed_table_wrench_max=fixedmax,expm_scaled_Q_max=max(errs),variance_timestamp='previous sample',bias_removed=False,cross_blocks_retained=True))
    return dict(new_KF_calls=ledger['calls'])


def metric_rows(case,group,t,y,truth):
    rows=[];windows={'full':np.ones(len(t),dtype=bool),'after_2s':t>=2}
    if case=='step':windows.update(preload=t<1,loaded=t>=1,loaded_short=(t>=1)&(t<=1.712))
    for window,mask in windows.items():
        e=y[mask]-truth[mask];tt=t[mask]
        for j,axis in enumerate(['Fx','Fy','Fz','Mx','My','Mz']):
            z=e[:,j];peak=int(np.argmax(abs(z)))
            rows.append(dict(case=case,group=group,window=window,axis=axis,unit='N' if j<3 else 'N m',samples=len(z),rmse=float(np.sqrt(np.mean(z*z))),bias=float(z.mean()),sd=float(z.std(ddof=0)),peak_abs=float(abs(z[peak])),peak_signed=float(z[peak]),peak_time_s=float(tt[peak])))
    return rows


def evaluate(cfg,out,ledger):
    base=json.loads(Path(cfg['base_config']).read_text());table=read_table(cfg);metrics=[];responses=[];summaries=[];results={};check={}
    for case in cfg['cases']:
        s,sigma=dynamic(cfg,case,table);t=s['t'];rec={}
        for group in ['A','B','C']:
            r=run_filter(cfg,base,s,sigma,group,ledger);rec[group]=r
            np.savez_compressed(out/(case+'_'+group+'.npz'),**r,mode=s['msma_mode'],length=s['msma_length'],reset=s['msma_reset'])
        close(rec['A']['gain'],rec['B']['gain'],cfg,'Paired A/B gain mismatch');close(rec['A']['covariance'],rec['B']['covariance'],cfg,'Paired A/B P mismatch');close(rec['B']['nominal_input'],rec['C']['nominal_input'],cfg,'Paired B/C input mismatch')
        # Evaluation-only truth is loaded after all observer outputs are fixed.
        truth_record=load(Path(cfg['current_run'])/'probe'/case/'diagnostics.npz');require(np.array_equal(t,truth_record['t'][64:]),'Truth timestamp mismatch');truth=truth_record['wrench'][64:]
        for group,r in rec.items():
            metrics+=metric_rows(case,group,t,r['wrench'],truth)
            if case=='step':
                tt=t[t>=1];z=r['wrench'][t>=1,2];error=z-3
                def cross(level):
                    k=np.flatnonzero(z>=level);return float(tt[k[0]]-1) if len(k) else None
                stable=np.logical_and.accumulate((abs(error)<=.15)[::-1])[::-1];hit=np.flatnonzero(stable)
                settling=float(tt[hit[0]]-1) if len(hit) else None
                responses.append(dict(group=group,t10_s=cross(.3),t90_s=cross(2.7),overshoot_N=float(max(0,z.max()-3)),overshoot_percent=float(max(0,z.max()-3)/3*100),settle_5pct_s=settling,remaining_in_band_s=float(tt[-1]-tt[hit[0]]) if len(hit) else None,steady_bias_N=float((r['wrench'][t>=2,2]-3).mean()),steady_sd_N=float(r['wrench'][t>=2,2].std()),peak_Fx_N=float(abs(r['wrench'][t>=1,0]).max())))
            check[case+'_'+group]=dict(min_P_eigenvalue=float(min(np.linalg.eigvalsh(P)[0] for P in r['covariance'])),max_P_asymmetry=float(np.max(abs(r['covariance']-r['covariance'].transpose(0,2,1)))))
        results[case]=(s,truth,rec)
        for j in range(6):
            summaries.append(dict(case=case,joint=j+1,min_selected_variance_Nm2=float(sigma[:,j].min()),max_selected_variance_Nm2=float(sigma[:,j].max()),baseline_equivalent_variance_Nm2=float(base['observer']['Qp_continuous_diag'][j]/.008)))
    csvwrite(out/'six_axis_metrics.csv',metrics);csvwrite(out/'response.csv',responses);csvwrite(out/'selected_covariance_ranges.csv',summaries)
    diffs=[]
    for row in metrics:
        if row['group']!='A':
            for against in (['A'] if row['group']=='B' else ['A','B']):
                prior=next(x for x in metrics if all(x[k]==row[k] for k in ['case','window','axis']) and x['group']==against)
                diffs.append(dict(case=row['case'],comparison=row['group']+'-'+against,window=row['window'],axis=row['axis'],unit=row['unit'],delta_rmse=row['rmse']-prior['rmse'],delta_bias=row['bias']-prior['bias'],delta_peak_abs=row['peak_abs']-prior['peak_abs']))
    csvwrite(out/'paired_differences.csv',diffs)
    fig,axes=plt.subplots(6,2,figsize=(13,13),sharex=True)
    for ci,case in enumerate(cfg['cases']):
        s,truth,rec=results[case]
        for j,axis in enumerate(['Fx','Fy','Fz','Mx','My','Mz']):
            ax=axes[j,ci]
            for group,r in rec.items():ax.plot(s['t'],r['wrench'][:,j],label=group,lw=.9)
            ax.plot(s['t'],truth[:,j],'k--',lw=1,label='truth');ax.set_ylabel(axis+(' / N' if j<3 else ' / N m'));ax.grid(alpha=.25)
            if j==0:ax.set_title(case);ax.legend(ncol=4,fontsize=8)
            if j==5:ax.set_xlabel('Time / s')
    fig.tight_layout();fig.savefig(out/'six_axis_wrench.png',dpi=140);plt.close(fig)
    s,truth,rec=results['step'];fig,axes=plt.subplots(2,1,figsize=(10,7),sharex=True);mask=(s['t']>=.8)&(s['t']<=2)
    for group,r in rec.items():axes[0].plot(s['t'][mask],r['wrench'][mask,2],label=group);axes[1].plot(s['t'][mask],r['wrench'][mask,0],label=group)
    axes[0].plot(s['t'][mask],truth[mask,2],'k--',label='truth');axes[1].axhline(0,c='k',ls='--');axes[0].set_ylabel('Fz / N');axes[1].set_ylabel('Fx / N');axes[1].set_xlabel('Time / s')
    for ax in axes:ax.grid(alpha=.25);ax.legend();ax.axvline(1,c='gray',ls=':')
    fig.tight_layout();fig.savefig(out/'step_response.png',dpi=150);plt.close(fig)
    write(out/'checks.json',dict(passed=True,groups=check,BC_input_same=True,AB_PK_same=True,calls=ledger['calls'],lookup_uses_previous_frame=True))
    write(out/'summary.json',dict(responses=responses,metrics=metrics,covariance_ranges=summaries))
    return dict(new_KF_calls=ledger['calls'],metrics=len(metrics))


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--config',type=Path,required=True);ap.add_argument('--output-dir',type=Path,required=True);ap.add_argument('--mode',choices=['calibrate','precheck','compare'],required=True);a=ap.parse_args()
    cfg=json.loads(a.config.read_text());out=a.output_dir;out.mkdir(exist_ok=True,parents=True);start=time.perf_counter();ledger=dict(calls=0)
    try:
        if a.mode=='calibrate':result=calibrate(cfg,out)
        elif a.mode=='precheck':result=precheck(cfg,out,ledger)
        else:
            prior=json.loads((Path(cfg['precheck_dir'])/'metrics.json').read_text());require(prior['passed'],'Precheck not passed');require(prior['new_KF_calls']+4122<=cfg['new_KF_cap'],'Remaining KF budget insufficient')
            result=evaluate(cfg,out,ledger)
        write(out/'metrics.json',dict(passed=True,new_physics_steps=0,wall_seconds=time.perf_counter()-start,**result))
    except Exception as exc:
        write(out/'failure.json',dict(error=repr(exc),traceback=traceback.format_exc()));write(out/'metrics.json',dict(passed=False,new_KF_calls=ledger['calls'],new_physics_steps=0,wall_seconds=time.perf_counter()-start));raise
    finally:write(out/'call_ledger.json',ledger)
if __name__=='__main__':main()
