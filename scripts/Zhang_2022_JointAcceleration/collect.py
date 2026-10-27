import numpy as np
import mujoco,time
from common import Model,helper,write

def disturbance(t,scene):
    a=np.zeros(6)
    if scene.get('frequency_Hz') is None and not scene.get('chirp'):return a
    if not 1<=t<9:return a
    s=t-1.;envelope=1.
    if s<.25:envelope=np.sin(.5*np.pi*s/.25)**2
    elif s>7.75:envelope=np.sin(.5*np.pi*(8-s)/.25)**2
    phase=2*np.pi*(6*s+.125*s*s) if scene.get('chirp') else 2*np.pi*scene['frequency_Hz']*s
    a[1]=scene['amplitude_rad_s2']*envelope*np.sin(phase)
    return a

def run(c,out):
    total=queries=frames=0;records=[]
    expected_steps=expected_frames=0
    assert 'motion' in c['cases'], 'A fresh no-force motion baseline is required'
    for case,scene in c['cases'].items():
        dest=out/case;dest.mkdir();m=mujoco.MjModel.from_binary_path(c['model']);d=mujoco.MjData(m)
        control=Model(c['model'],c['derivative_epsilon_s']);mask=helper.moving_mask(m);d.qpos[:]=c['initial_q_rad']
        dt=c['physics_dt_s'];steps=round(scene['duration_s']/dt);control_stride=round(c['control_dt_s']/dt);stride=round(c['record_dt_s']/dt)
        expected_steps+=steps;expected_frames+=steps//stride+1
        kp=np.array(c['controller']['kp_Nm_rad']);kd=np.array(c['controller']['kd_Nm_s_rad'])
        command=np.zeros(6);data={k:[] for k in ['t','q','v','tau']};truth={k:[] for k in ['t','qacc','tau_ext','d_a','q_ref','command']}
        M=np.zeros((6,6));completed=0;balance=tracking=0.;start=time.monotonic()
        try:
            for n in range(steps+1):
                t=n*dt;d.qfrc_applied[:]=0;d.xfrc_applied[:]=0;mujoco.mj_forward(m,d)
                qr,vr,ar=helper.trajectory(t,c,scene['motion'])
                if n%control_stride==0:
                    Mc,h,_,_=control.evaluate(d.qpos.copy(),d.qvel.copy());queries+=1
                    command=Mc@ar+h+control.B*d.qvel+kp*(qr-d.qpos)+kd*(vr-d.qvel);d.ctrl[:]=command
                mujoco.mj_fullM(m,M,d.qM);da=disturbance(t,scene);ext=M@da;d.qfrc_applied[:]=ext
                mujoco.mj_forward(m,d);helper.valid(m,d,mask,command)
                err=M@d.qacc+d.qfrc_bias-d.qfrc_actuator-d.qfrc_passive-ext
                balance=max(balance,float(abs(err).max()));helper.require(np.allclose(err,0,rtol=0,atol=1e-9),'Plant balance')
                tracking=max(tracking,float(abs(d.qpos-qr).max()))
                if n%stride==0:
                    for key,value in zip(data,[t,d.qpos.copy(),d.qvel.copy(),d.qfrc_actuator.copy()]):data[key].append(value)
                    for key,value in zip(truth,[t,d.qacc.copy(),ext.copy(),da,qr,command.copy()]):truth[key].append(value)
                if n<steps:mujoco.mj_step(m,d);completed+=1
        except Exception as exc:
            write(dest/'failure.json',dict(error=repr(exc),time_s=float(d.time),steps=completed));raise
        finally:
            np.savez_compressed(dest/'measurements.npz',**data);np.savez_compressed(dest/'reference.npz',**truth)
        if case not in ['static','motion']:
            with np.load(out/'motion/measurements.npz') as base:
                for key in data:assert np.array_equal(np.asarray(data[key])[:2000],base[key][:2000]),'Preload histories differ'
        record=dict(case=case,passed=True,steps=completed,frames=len(data['t']),max_balance_Nm=balance,max_tracking_rad=tracking,wall_seconds=time.monotonic()-start)
        write(dest/'checks.json',record);records.append(record);total+=completed;frames+=len(data['t']);print(case,'collected',flush=True)
    assert total==expected_steps and frames==expected_frames
    write(out/'scene_checks.json',records)
    return dict(passed=True,simulation_steps=total,observer_calls=0,model_evaluations=queries,frames=frames)
