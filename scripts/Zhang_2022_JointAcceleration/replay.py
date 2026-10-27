import time
import numpy as np
from common import Model,MomentumResidual,packet,write
from acceleration import AccelerationBank,METHODS

def run(c,root,out):
    calls=queries=0;records=[]
    for settings in c['replay_modes']:
        mode=settings['name'];factor=settings['damping_factor'];sigma=settings['noise_std_rad_s'];seed=settings['seed']
        stride=round(1./settings['rate_Hz']/c['record_dt_s'])
        for caseid,(case,scene) in enumerate(c['cases'].items()):
            if case not in settings.get('cases',c['cases']):continue
            dest=out/mode/case;dest.mkdir(parents=True)
            data=packet(root/'collect'/case/'measurements.npz');data={key:val[::stride] for key,val in data.items()}
            rng=np.random.default_rng(np.random.SeedSequence([seed,caseid])) if seed>=0 else None
            noise=sigma*rng.standard_normal(data['v'].shape) if rng is not None else np.zeros_like(data['v'])
            measured=data['v']+noise
            nominal=Model(c['model'],c['derivative_epsilon_s']);bank=AccelerationBank(nominal,MomentumResidual(c['gain_s_inv']),c['hodo_gains'],factor)
            history={};timing=[];finished=0;st=time.monotonic()
            try:
                for k,t in enumerate(data['t']):
                    tick=time.perf_counter();result=bank.update(float(t),data['q'][k],measured[k],data['tau'][k]);timing.append(time.perf_counter()-tick)
                    for key,val in result.items():history.setdefault(key,[]).append(val.copy())
                    finished+=1;calls+=6;queries+=1
                    assert calls<c['execution_limits']['observer_calls'] and queries<c['execution_limits']['model_evaluations'],'Replay budget exceeded'
            except Exception as exc:
                write(dest/'failure.json',dict(error=repr(exc),frames=finished,case=case,mode=mode,observer_calls=calls,model_evaluations=queries));raise
            finally:
                np.savez_compressed(dest/'history.npz',t=data['t'][:finished],v_measured=measured[:finished],**history)
            values=np.asarray(timing)
            if mode=='clean' and case not in ['static','motion']:
                baseline=packet(out/'clean/motion/history.npz')
                for key,value in history.items():
                    assert np.array_equal(np.asarray(value)[:1000],baseline[key][:1000]), 'Preload outputs differ: '+key
            record=dict(case=case,mode=mode,seed=seed,damping_factor=factor,velocity_noise_std_rad_s=sigma,rate_hz=settings['rate_Hz'],frames=finished,methods=METHODS,wall_seconds=time.monotonic()-st,update_mean_s=float(values.mean()),update_p95_s=float(np.quantile(values,.95)),update_max_s=float(values.max()))
            write(dest/'info.json',record);records.append(record)
        print(mode,'completed',flush=True)
    write(out/'replay_index.json',records)
    return dict(passed=True,simulation_steps=0,observer_calls=calls,model_evaluations=queries,sequences=len(records))
