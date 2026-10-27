"""One additional raw-input, residual-calibrated-Q comparison; reuse prior groups."""
import argparse,csv,json,time,traceback
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from akf_comparison import CaptureObserver,read_table,dynamic,metric_rows,close
from input_statistics import write,csvwrite,load,require


def replay(cfg,base,s,sigma,ledger,length=None):
    obs=CaptureObserver(base,True);records={};N=len(s['t']) if length is None else length
    for k in range(N):
        require(ledger['calls']<cfg['new_KF_cap'],'Call budget exceeded');ledger['calls']+=1
        r=obs.update(s['t'][k],s['q'][k],s['v'][k],s['raw_current'][k]*cfg['D'],sigma)
        require(all(np.isfinite(x).all() for x in r.values()),'Nonfinite KF result')
        require(np.linalg.eigvalsh(r['covariance'])[0]>0,'P not positive definite')
        if k:require(np.linalg.eigvalsh(r['Q'])[0]>0,'Q not positive definite')
        for key,value in r.items():records.setdefault(key,[]).append(np.array(value,copy=True))
    records={k:np.asarray(v) for k,v in records.items()}
    close(records['used_sigma'][1:],records['selected_sigma'][:-1],cfg,'Wrong variance time association')
    close(records['input_source_t'][1:],records['t'][:-1],cfg,'Wrong input timestamp')
    return records


def precheck(cfg,out,ledger):
    # Confirm old controlled inputs still match their frozen snapshots.
    root=Path(cfg['prior_run']);old=json.loads((root/'workflow-state.json').read_text())
    for item in old['inputSnapshots']:
        original=Path('/workspace')/item['path'];require(original.read_bytes()==(root/item['snapshot']).read_bytes(),'Prior comparison input changed: '+item['path'])
    base=json.loads(Path(cfg['base_config']).read_text());table=read_table(cfg)
    raw=np.array([table['raw',1,j] for j in range(1,7)])
    s,_=dynamic(cfg,'step',table)
    baseline=np.asarray(base['observer']['Qp_continuous_diag'])/.008
    r=replay(cfg,base,s,baseline,ledger,151);oldA=load(root/'compare/step_A.npz');err=0.
    for key in ['wrench','covariance','gain','nominal_input','innovation','Q','R']:
        err=max(err,close(r[key],oldA[key][:151],cfg,'Raw baseline recovery failed '+key))
    write(out/'checks.json',dict(passed=True,old_input_snapshots=len(old['inputSnapshots']),baseline_recovery_max=err,raw_variance_Nm2=raw,source='existing raw residual table, no refit',calls=ledger['calls']))
    return dict(new_KF_calls=ledger['calls'])


def response(t,y):
    tt=t[t>=1];z=y[t>=1,2];error=z-3
    def cross(level):
        ix=np.flatnonzero(z>=level);return float(tt[ix[0]]-1) if len(ix) else None
    good=np.logical_and.accumulate((abs(error)<=.15)[::-1])[::-1];hit=np.flatnonzero(good)
    return dict(group='D',t10_s=cross(.3),t90_s=cross(2.7),overshoot_N=float(max(0,z.max()-3)),overshoot_percent=float(max(0,z.max()-3)/3*100),settle_5pct_s=float(tt[hit[0]]-1) if len(hit) else None,remaining_in_band_s=float(tt[-1]-tt[hit[0]]) if len(hit) else None,steady_bias_N=float((y[t>=2,2]-3).mean()),steady_sd_N=float(y[t>=2,2].std()),peak_Fx_N=float(abs(y[t>=1,0]).max()))


def compare(cfg,out,ledger):
    base=json.loads(Path(cfg['base_config']).read_text());table=read_table(cfg);raw=np.array([table['raw',1,j] for j in range(1,7)])
    root=Path(cfg['prior_run'])/'compare';metrics=list(csv.DictReader((root/'six_axis_metrics.csv').open()));responses=list(csv.DictReader((root/'response.csv').open()));checks={};ratios=[];results={}
    for case in cfg['cases']:
        s,filtered_variance=dynamic(cfg,case,table);t=s['t'];r=replay(cfg,base,s,raw,ledger)
        np.savez_compressed(out/(case+'_D.npz'),**r)
        oldA=load(root/(case+'_A.npz'));oldC=load(root/(case+'_C.npz'))
        require(np.array_equal(t,oldC['t']) and np.array_equal(t,oldA['t']),'Prior output timestamp changed')
        close(r['nominal_input'],oldA['nominal_input'],cfg,'D/A input differs');close(r['R'],oldC['R'],cfg,'D/C R differs');close(r['covariance'][0],oldC['covariance'][0],cfg,'D/C P0 differs')
        dt=r['dt'][1:,None,None]
        expected=np.diag(raw-np.asarray(base['observer']['Qp_continuous_diag'])/.008)
        close((r['Q'][1:,:6,:6]-oldA['Q'][1:,:6,:6])/dt**2,expected,cfg,'D differs from A beyond designated input Q block')
        close(r['Q'][:,6:,:],oldC['Q'][:,6:,:],cfg,'External/cross covariance block changed')
        close(r['Q'][:,:6,6:],oldC['Q'][:,:6,6:],cfg,'Cross covariance block changed')
        truth_record=load(Path(cfg['current_run'])/'probe'/case/'diagnostics.npz');require(np.array_equal(t,truth_record['t'][64:]),'Truth time mismatch');truth=truth_record['wrench'][64:]
        metrics+=metric_rows(case,'D',t,r['wrench'],truth)
        if case=='step':responses.append(response(t,r['wrench']))
        results[case]=(t,truth,r,oldA,oldC)
        checks[case]=dict(min_P_eigenvalue=float(min(np.linalg.eigvalsh(P)[0] for P in r['covariance'])),max_P_asymmetry=float(np.max(abs(r['covariance']-r['covariance'].transpose(0,2,1)))),calls=len(t))
        for j in range(6):ratios.append(dict(case=case,joint=j+1,raw_variance_Nm2=float(raw[j]),C_min_over_D=float((filtered_variance[:,j]/raw[j]).min()),C_max_over_D=float((filtered_variance[:,j]/raw[j]).max()),C_mean_over_D=float((filtered_variance[:,j]/raw[j]).mean())))
    csvwrite(out/'all_groups_metrics.csv',metrics);csvwrite(out/'all_groups_response.csv',responses);csvwrite(out/'input_variance_ratio.csv',ratios)
    differences=[]
    for r in metrics:
        if r['group']!='D':continue
        for g in ['A','C']:
            old=next(x for x in metrics if x['group']==g and all(x[k]==r[k] for k in ['case','window','axis']))
            differences.append(dict(case=r['case'],window=r['window'],axis=r['axis'],unit=r['unit'],comparison='D-'+g,delta_rmse=float(r['rmse'])-float(old['rmse']),delta_bias=float(r['bias'])-float(old['bias']),delta_peak_abs=float(r['peak_abs'])-float(old['peak_abs'])))
    csvwrite(out/'paired_differences.csv',differences)
    fig,axes=plt.subplots(6,2,figsize=(13,13),sharex=True)
    for col,(case,(t,truth,r,a,c)) in enumerate(results.items()):
        for j,axis in enumerate(['Fx','Fy','Fz','Mx','My','Mz']):
            ax=axes[j,col]
            for label,item in [('A: raw/manual Q',a),('D: raw/calibrated Q',r),('C: MSMA/calibrated Q',c)]:ax.plot(t,item['wrench'][:,j],label=label,lw=.9)
            ax.plot(t,truth[:,j],'k--',lw=1);ax.grid(alpha=.2);ax.set_ylabel(axis+(' / N' if j<3 else ' / N m'))
            if j==0:ax.set_title(case);ax.legend(fontsize=7)
            if j==5:ax.set_xlabel('Time / s')
    fig.tight_layout();fig.savefig(out/'six_axis_comparison.png',dpi=140);plt.close(fig)
    t,truth,r,a,c=results['step'];mask=(t>=.8)&(t<=2);fig,axes=plt.subplots(2,1,figsize=(10,7),sharex=True)
    for ax,j in zip(axes,[2,0]):
        for label,item in [('A',a),('D',r),('C',c)]:ax.plot(t[mask],item['wrench'][mask,j],label=label)
        ax.plot(t[mask],truth[mask,j],'k--',label='truth');ax.axvline(1,c='gray',ls=':');ax.grid(alpha=.2);ax.legend();ax.set_ylabel(['Fx','Fy','Fz'][j]+' / N')
    axes[-1].set_xlabel('Time / s');fig.tight_layout();fig.savefig(out/'step_response.png',dpi=150);plt.close(fig)
    write(out/'checks.json',dict(passed=True,cases=checks,calls=ledger['calls'],DA_inputs_equal=True,DC_R_initialization_Qw_cross_blocks_equal=True,raw_table_reused=True))
    return dict(new_KF_calls=ledger['calls'],metric_rows=len(metrics))


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--config',type=Path,required=True);ap.add_argument('--output-dir',type=Path,required=True);ap.add_argument('--mode',choices=['precheck','compare'],required=True);a=ap.parse_args();cfg=json.loads(a.config.read_text());out=a.output_dir;out.mkdir(parents=True,exist_ok=True);start=time.perf_counter();ledger=dict(calls=0)
    try:
        if a.mode=='precheck':result=precheck(cfg,out,ledger)
        else:
            prior=json.loads((Path(cfg['this_precheck_dir'])/'metrics.json').read_text());require(prior['passed'] and prior['new_KF_calls']+1374<=cfg['new_KF_cap'],'Precheck/budget failure');result=compare(cfg,out,ledger)
        write(out/'metrics.json',dict(passed=True,new_physics_steps=0,wall_seconds=time.perf_counter()-start,**result))
    except Exception as exc:
        write(out/'failure.json',dict(error=repr(exc),traceback=traceback.format_exc()));write(out/'metrics.json',dict(passed=False,new_KF_calls=ledger['calls'],wall_seconds=time.perf_counter()-start));raise
    finally:write(out/'call_ledger.json',ledger)
if __name__=='__main__':main()
