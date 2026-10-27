"""Fixed Cartesian-path preparation and one bounded physical collection."""
import argparse,json,sys,time,traceback
from pathlib import Path
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from input_statistics import write,load,require
from runner import load as module, sample


def target(t):
    if t<=1:return 0.,0.,0.
    if t<2:
        u=t-1;return .03*(u-np.sin(np.pi*u)/np.pi),.03*(1-np.cos(np.pi*u)),.03*np.pi*np.sin(np.pi*u)
    if t<=5:return .03+.06*(t-2),.06,0.
    u=min(t-5,1.);return .21+.03*(u+np.sin(np.pi*u)/np.pi),.03*(1+np.cos(np.pi*u)),-.03*np.pi*np.sin(np.pi*u)


def kinematics(m,d,q):
    d.qpos[:]=q;d.qvel[:]=0;mujoco.mj_forward(m,d);site=m.site('tool0').id
    jp,jr=np.zeros((3,6)),np.zeros((3,6));mujoco.mj_jacSite(m,d,jp,jr,site)
    return d.site_xpos[site].copy(),d.site_xmat[site].reshape(3,3).copy(),np.vstack([jp,jr])


def path(cfg,base,out,ledger):
    m=mujoco.MjModel.from_binary_path(base['model']);d=mujoco.MjData(m);q=np.array(base['initial_q_rad']);p0,R0,_=kinematics(m,d,q)
    helper=module(base['simulation_helpers_source'],'path_guards');mask=helper.moving_mask(m)
    records={};max_iterations=0;eps=base['derivative_epsilon_s'];errors=[];failed_state={}
    try:
        for n in range(6001):
            t=n*.001;z,v,a=target(t);pt=p0+[0,0,z]
            for it in range(cfg['ik_max_iterations']):
                p,R,J=kinematics(m,d,q);ledger['kinematic_evaluations']+=1
                err=np.r_[pt-p,Rotation.from_matrix(R0@R.T).as_rotvec()]
                failed_state=dict(t=t,q=q.copy(),position_error_m=err[:3],orientation_error_rad=err[3:])
                if np.linalg.norm(err[:3])<=cfg['ik_position_tolerance_m'] and np.linalg.norm(err[3:])<=cfg['ik_rotation_tolerance_rad']:break
                scaled=J.copy();scaled[:3]/=.5;sv=np.linalg.svd(scaled,compute_uv=False)
                require(sv[-1]>np.finfo(float).eps*6*sv[0],'Jacobian rank loss in IK')
                dq=np.linalg.solve(J,err)
                require(np.max(abs(dq))<=cfg['ik_max_increment_rad'],'IK increment exceeds fixed local branch bound')
                q=q+dq
            else:raise RuntimeError('IK failed at fixed iteration budget')
            max_iterations=max(max_iterations,it+1)
            helper.valid(m,d,mask,np.zeros(6))
            scaled=J.copy();scaled[:3]/=.5;sv=np.linalg.svd(scaled,compute_uv=False)
            require(sv[-1]>np.finfo(float).eps*6*sv[0],'Jacobian rank loss')
            vr=np.linalg.solve(J,np.array([0,0,v,0,0,0]))
            Jp=kinematics(m,d,q+eps*vr)[2];Jm=kinematics(m,d,q-eps*vr)[2];ledger['kinematic_evaluations']+=2
            Jd=(Jp-Jm)/(2*eps);ar=np.linalg.solve(J,np.array([0,0,a,0,0,0])-Jd@vr)
            sample(records,t=t,q=q,v=vr,a=ar,tcp_ref=pt,R_ref=R0,J=J,min_scaled_sv=sv[-1],position_error_m=err[:3],orientation_error_rad=err[3:])
            if n and n%1000==0:print('path',n,'/6000',flush=True)
        r={k:np.asarray(v) for k,v in records.items()};np.savez_compressed(out/'trajectory.npz',**r)
        ve=(r['q'][2:]-r['q'][:-2])/.002-r['v'][1:-1]
        ae=(r['v'][2:]-r['v'][:-2])/.002-r['a'][1:-1]
        check=dict(position_peak_m=float(np.linalg.norm(r['position_error_m'],axis=1).max()),orientation_peak_rad=float(np.linalg.norm(r['orientation_error_rad'],axis=1).max()),velocity_derivative_peak_rad_s=float(abs(ve).max()),acceleration_derivative_peak_rad_s2=float(abs(ae).max()),min_scaled_sv=float(r['min_scaled_sv'].min()),max_iterations=max_iterations,initial_tcp_m=p0,final_tcp_m=r['tcp_ref'][-1])
        write(out/'checks.json',check)
        require(abs(ve).max()<=cfg['reference_velocity_tolerance_rad_s'],'Reference position/velocity derivative mismatch')
        require(abs(ae).max()<=cfg['reference_acceleration_tolerance_rad_s2'],'Reference velocity/acceleration derivative mismatch')
        return dict(path_samples=len(r['t']))
    except Exception:
        write(out/'failed_state.json',failed_state)
        if records:np.savez_compressed(out/'partial_trajectory.npz',**{k:np.asarray(v) for k,v in records.items()})
        raise


def collect(cfg,base,out,ledger):
    root=Path(cfg['run_root']);require(json.loads((root/'path/metrics.json').read_text())['passed'],'Path not accepted')
    tr=load(root/'path/trajectory.npz');m=mujoco.MjModel.from_binary_path(base['model']);d=mujoco.MjData(m);d.qpos[:]=base['initial_q_rad']
    oracle=module(base['oracle_source'],'plant_reference').Model(base['model'],base['derivative_epsilon_s']);helper=module(base['simulation_helpers_source'],'plant_guards');mask=helper.moving_mask(m)
    site=m.site('tool0').id;body=int(m.site_bodyid[site]);kp=np.array(base['controller']['kp_Nm_rad']);kd=np.array(base['controller']['kd_Nm_s_rad']);B=np.array(base['damping']);feedback={};diagnostic={};control={};peaks=dict(balance=0.,mapping=0.,offset_equivalence=0.);qref=tr['q'][0];vref=tr['v'][0];aref=tr['a'][0];cmd=np.zeros(6)
    try:
        for n in range(96001):
            t=n*base['physics_dt_s'];d.qfrc_applied[:]=0;d.xfrc_applied[:]=0;mujoco.mj_forward(m,d)
            if n%16==0:
                j=n//16;qref=tr['q'][j];vref=tr['v'][j];aref=tr['a'][j]
                M,h,_,_=oracle.evaluate(d.qpos.copy(),d.qvel.copy());cmd=M@aref+h+B*d.qvel+kp*(qref-d.qpos)+kd*(vref-d.qvel);d.ctrl[:]=cmd
                sample(control,t=t,q_ref=qref,v_ref=vref,a_ref=aref,command=cmd)
            wrench=np.array(cfg['wrench']) if n>=cfg['load_physics_index'] else np.zeros(6)
            mujoco.mj_applyFT(m,d,wrench[:3],np.zeros(3),d.site_xpos[site]+np.array(cfg['offset_world_m']),body,d.qfrc_applied)
            mujoco.mj_forward(m,d);helper.valid(m,d,mask,cmd)
            M=np.zeros((6,6));mujoco.mj_fullM(m,M,d.qM)
            bal=M@d.qacc+d.qfrc_bias-d.qfrc_actuator-d.qfrc_passive-d.qfrc_applied;peaks['balance']=max(peaks['balance'],float(abs(bal).max()))
            require(np.allclose(bal,0,rtol=0,atol=cfg['atol']),'Physical balance mismatch')
            if n%128==0:
                jp,jr=np.zeros((3,6)),np.zeros((3,6));mujoco.mj_jacSite(m,d,jp,jr,site);J=np.vstack([jp,jr]);mapped=J.T@wrench;eq=np.zeros(6)
                mujoco.mj_applyFT(m,d,wrench[:3],wrench[3:],d.site_xpos[site],body,eq)
                peaks['mapping']=max(peaks['mapping'],float(abs(mapped-d.qfrc_applied).max()));peaks['offset_equivalence']=max(peaks['offset_equivalence'],float(abs(eq-d.qfrc_applied).max()))
                require(np.allclose(mapped,d.qfrc_applied,rtol=cfg['rtol'],atol=cfg['atol']) and np.allclose(eq,d.qfrc_applied,rtol=cfg['rtol'],atol=cfg['atol']),'Offset wrench map mismatch')
                scaled=J.copy();scaled[:3]/=.5;sv=np.linalg.svd(scaled,compute_uv=False);require(sv[-1]>np.finfo(float).eps*6*sv[0],'Feedback Jacobian rank loss')
                z,v,a=target(t);pr=tr['tcp_ref'][0]+[0,0,z];orient=Rotation.from_matrix(tr['R_ref'][0]@d.site_xmat[site].reshape(3,3).T).as_rotvec()
                sample(feedback,t=t,q=d.qpos,v=d.qvel,current=d.qfrc_actuator/np.array(cfg['D']))
                sample(diagnostic,t=t,wrench=wrench,tau_act=d.qfrc_actuator,qacc=d.qacc,J=J,q_ref=qref,v_ref=vref,v_true=d.qvel,tcp=d.site_xpos[site],tcp_ref=pr,tcp_velocity=jp@d.qvel,tcp_velocity_ref=[0,0,v],orientation_error=orient,tau_ext=d.qfrc_applied,command=cmd,physical_time=d.time)
            if n<96000:
                require(ledger['physics_steps']<cfg['physics_cap'],'Physical budget exhausted');mujoco.mj_step(m,d);ledger['physics_steps']+=1
            if n and n%16000==0:print('physics',n,'/96000',flush=True)
    finally:
        for name,rows in [('measurements',feedback),('diagnostics',diagnostic),('control',control)]:
            if rows:np.savez_compressed(out/(name+'.npz'),**{k:np.asarray(v) for k,v in rows.items()})
    s={k:np.asarray(v) for k,v in diagnostic.items()};core=(s['t']>=2)&(s['t']<5)
    pe=float(np.linalg.norm(s['tcp'][core]-s['tcp_ref'][core],axis=1).max());oe=float(np.linalg.norm(s['orientation_error'][core],axis=1).max());ve=float(np.sqrt(np.mean(np.sum((s['tcp_velocity'][core]-s['tcp_velocity_ref'][core])**2,axis=1))))
    check=dict(position_peak_m=pe,orientation_peak_rad=oe,velocity_vector_rmse_m_s=ve,feedback_samples=len(s['t']),guard_peaks=peaks,velocity_measurement_error_max=0.,joint_velocity_tracking_rmse_rad_s=np.sqrt(np.mean((s['v_true'][core]-s['v_ref'][core])**2,axis=0)))
    write(out/'checks.json',check)
    require(pe<=cfg['tracking_position_max_m'],'Cartesian position tracking gate failed')
    require(oe<=cfg['tracking_rotation_max_rad'],'Cartesian orientation tracking gate failed')
    require(ve<=cfg['tracking_velocity_rmse_max_m_s'],'Cartesian velocity tracking gate failed')
    return dict(feedback_samples=len(s['t']))


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--config',type=Path,required=True);ap.add_argument('--output-dir',type=Path,required=True);ap.add_argument('--mode',choices=['path','collect'],required=True);a=ap.parse_args();cfg=json.loads(a.config.read_text());base=json.loads(Path(cfg['base_config']).read_text());out=a.output_dir;out.mkdir(parents=True,exist_ok=True);start=time.perf_counter();ledger=dict(physics_steps=0,KF_calls=0,kinematic_evaluations=0)
    try:
        r=(path if a.mode=='path' else collect)(cfg,base,out,ledger);write(out/'metrics.json',dict(passed=True,wall_seconds=time.perf_counter()-start,**ledger,**r))
    except Exception as exc:
        write(out/'failure.json',dict(error=repr(exc),traceback=traceback.format_exc()));write(out/'metrics.json',dict(passed=False,wall_seconds=time.perf_counter()-start,**ledger));raise
    finally:write(out/'ledger.json',ledger)
if __name__=='__main__':main()
