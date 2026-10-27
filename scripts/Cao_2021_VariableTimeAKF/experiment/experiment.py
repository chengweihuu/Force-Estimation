"""Explicit binding for a bounded new-data current-filter comparison."""
import argparse,json,sys,time,traceback
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from input_statistics import write,csvwrite,load,require
from msma import wma_weights,hma_weights,linear_series
import constant_speed_v2 as cs
import paper_motion
import controlled_collection
import engineering_checks


def calibrate(cfg,base,out,ledger):
    cc=cfg['calibration'];_,oracle,helpers=cs.environment(cc)
    kernels=[('raw',1,np.ones(1))]+[('wma',n,wma_weights(n)) for n in range(1,51)]+[('hma',50,hma_weights(50))]
    rows=[];table=[];approx=[];physics=dict(steps=0);D=np.asarray(cfg['D'])
    try:
        for case in cc['cases']:
            dest=out/case['name'];cs.simulate(cc,case,dest,base,oracle,helpers,physics)
            s=load(dest/'measurements.npz');mask=(s['t']>=3)&(s['t']<7);j=case['joint']-1
            require(mask.sum()==500 and np.flatnonzero(mask)[0]>=55,'Calibration full-history core')
            noise=np.random.default_rng(case['seed']).normal(size=s['current'].shape)*np.asarray(cc['sigma_Nm'])/D
            np.savez_compressed(dest/'noise.npz',t=s['t'],noise=noise)
            x=s['current']+noise
            for method,n,w in kernels:
                residual=s['tau_ref_no_acc'][mask,j]-linear_series(x,w)[mask,j]*D[j]
                rows.append(dict(case=case['name'],joint=j+1,method=method,length=n,samples=int(mask.sum()),mean_Nm=float(residual.mean()),sample_variance_Nm2=float(residual.var(ddof=1)),rmse_Nm=float(np.sqrt(np.mean(residual**2)))))
            for axis in range(6):
                approx.append(dict(case=case['name'],moving_joint=j+1,joint=axis+1,speed_error_rms_rad_s=float(np.sqrt(np.mean((s['v'][mask,axis]-s['vref'][mask,axis])**2))),qacc_rms_rad_s2=float(np.sqrt(np.mean(s['qacc'][mask,axis]**2))),omitted_inertia_rms_Nm=float(np.sqrt(np.mean(s['inertial'][mask,axis]**2))),reference_actuator_rmse_Nm=float(np.sqrt(np.mean((s['tau_ref_no_acc'][mask,axis]-s['tau_act'][mask,axis])**2)))))
            print('calibration complete',case['name'],flush=True)
    finally: ledger['physics_steps']=physics['steps']
    for j in range(1,7):
        for method,n,_ in kernels:
            pair=[r for r in rows if r['joint']==j and r['method']==method and r['length']==n]
            require(len(pair)==2,'Signed calibration pair')
            value=float(np.mean([r['sample_variance_Nm2'] for r in pair]));require(np.isfinite(value) and value>0,'Variance invalid')
            table.append(dict(joint=j,method=method,length=n,variance_Nm2=value,positive_mean_Nm=pair[0]['mean_Nm'],negative_mean_Nm=pair[1]['mean_Nm'],pooled_MSE_Nm2=float(np.mean([r['rmse_Nm']**2 for r in pair]))))
    require(len(table)==312 and ledger['physics_steps']==1536000,'Calibration completeness')
    csvwrite(out/'direction_statistics.csv',rows);csvwrite(out/'residual_variance_table.csv',table);csvwrite(out/'constant_speed_diagnostics.csv',approx)
    write(out/'checks.json',dict(passed=True,entries=len(table),direction_rows=len(rows),core_samples_per_direction=500,variance_definition='mean of positive and negative ddof=1 variances; bias is retained',seeds=[c['seed'] for c in cc['cases']],max_omitted_inertia_rms_Nm=max(r['omitted_inertia_rms_Nm'] for r in approx),max_speed_error_rms_rad_s=max(r['speed_error_rms_rad_s'] for r in approx)))
    return dict(entries=len(table),calibration_cases=12)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--config',type=Path,required=True);ap.add_argument('--output-dir',type=Path,required=True);ap.add_argument('--mode',choices=['units','path','precheck','calibrate','collect','compare'],required=True);args=ap.parse_args()
    cfg=json.loads(args.config.read_text());base=json.loads(Path(cfg['base_config']).read_text());out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
    start=time.perf_counter();ledger=dict(physics_steps=0,KF_calls=0,static_states=0,kinematic_evaluations=0);result={};passed=False
    try:
        if args.mode=='units':result=engineering_checks.units(cfg,base,out,ledger)
        elif args.mode=='path':result=paper_motion.path(cfg,base,out,ledger)
        elif args.mode=='precheck':result=engineering_checks.controller(cfg,base,out,ledger)
        elif args.mode=='calibrate':result=calibrate(cfg,base,out,ledger)
        elif args.mode=='collect':result=controlled_collection.collect(cfg,base,out,ledger)
        else:
            from evaluation import compare
            result=compare(cfg,base,out,ledger)
        passed=True
    except Exception as exc:
        write(out/'failure.json',dict(error=str(exc),traceback=traceback.format_exc()));raise
    finally:
        ledger['elapsed_s']=time.perf_counter()-start;write(out/'ledger.json',ledger);write(out/'metrics.json',dict(passed=passed,**ledger,**result))

if __name__=='__main__':main()
