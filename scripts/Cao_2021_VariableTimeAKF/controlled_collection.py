"""Approved controller precheck, single collection, and unchanged estimator comparison."""
import argparse,json,sys,time,traceback
from pathlib import Path
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from input_statistics import write,load,require,csvwrite
from runner import load as module,sample
from paper_motion import target
from integral_controller import IntegralController,frozen_closed_loop


def make_controller(cfg,base,scale=None,rate=None):
    return IntegralController(base['controller']['kp_Nm_rad'],base['controller']['kd_Nm_s_rad'],
        cfg['control_scale'] if scale is None else scale,cfg['integral_rate_s_inv'] if rate is None else rate,
        cfg['torque_limits_Nm'],cfg['integral_limit_fraction'],base['control_dt_s'])

def collect(cfg,base,out,ledger):
    root=Path(cfg['source_run']);require(json.loads((Path(cfg['run_root'])/'precheck/metrics.json').read_text())['passed'],'Controller precheck not passed');require(json.loads((root/'path/metrics.json').read_text())['passed'],'Old path not accepted')
    tr=load(root/'path/trajectory.npz');m=mujoco.MjModel.from_binary_path(base['model']);d=mujoco.MjData(m);d.qpos[:]=base['initial_q_rad']
    oracle=module(base['oracle_source'],'plant_reference').Model(base['model'],base['derivative_epsilon_s']);helper=module(base['simulation_helpers_source'],'plant_guards');mask=helper.moving_mask(m)
    site=m.site('tool0').id;body=int(m.site_bodyid[site]);controller=make_controller(cfg,base);B=np.array(base['damping']);feedback={};diagnostic={};control={};peaks=dict(balance=0.,mapping=0.,offset_equivalence=0.);qref=tr['q'][0];vref=tr['v'][0];aref=tr['a'][0];cmd=np.zeros(6)
    try:
        for n in range(96001):
            t=n*base['physics_dt_s'];d.qfrc_applied[:]=0;d.xfrc_applied[:]=0;mujoco.mj_forward(m,d)
            if n%16==0:
                j=n//16;qref=tr['q'][j];vref=tr['v'][j];aref=tr['a'][j]
                M,h,_,_=oracle.evaluate(d.qpos.copy(),d.qvel.copy());components=controller.update(d.qpos.copy(),d.qvel.copy(),qref,vref,aref,M,h,B);cmd=components['command']
                sample(control,t=t,q_ref=qref,v_ref=vref,a_ref=aref,**components)
                require(not np.any(components['command_exceeds_limit']),'Unclipped command exceeds original actuator limit');d.ctrl[:]=cmd
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
    check['integral_upper_hits']=np.sum(np.asarray(control['integral_upper_hit']),axis=0);check['integral_lower_hits']=np.sum(np.asarray(control['integral_lower_hit']),axis=0);check['antiwindup_freezes']=np.sum(np.asarray(control['antiwindup_frozen']),axis=0);check['integral_peak_abs_Nm']=np.max(abs(np.asarray(control['I_used'])),axis=0);write(out/'checks.json',check)
    require(pe<=cfg['tracking_position_max_m'],'Cartesian position tracking gate failed')
    require(oe<=cfg['tracking_rotation_max_rad'],'Cartesian orientation tracking gate failed')
    require(ve<=cfg['tracking_velocity_rmse_max_m_s'],'Cartesian velocity tracking gate failed')
    return dict(feedback_samples=len(s['t']))



def precheck(cfg,base,out,ledger):
    def close(a,b,label):
        require(np.allclose(a,b,rtol=cfg['rtol'],atol=cfg['atol']),label)
        return float(np.max(abs(np.asarray(a)-np.asarray(b))))
    old=Path(cfg['source_run']);state=json.loads((old/'workflow-state.json').read_text())
    for x in state['inputSnapshots']:require((Path('/workspace')/x['path']).read_bytes()==(old/x['snapshot']).read_bytes(),'Original input changed: '+x['path'])
    M=np.diag([3.,2.,1.,.1,.01,.0002]);B=np.asarray(base['damping']);z=np.zeros(6);h=np.arange(6)*.1;q=np.arange(6)*.002;v=np.arange(6)*.003;qr=q+.001;vr=v-.002;ar=np.arange(6)*.004
    original=make_controller(cfg,base,scale=1.,rate=0.)
    expected=M@ar+h+B*v+np.asarray(base['controller']['kp_Nm_rad'])*(qr-q)+np.asarray(base['controller']['kd_Nm_s_rad'])*(vr-v)
    r=original.update(q,v,qr,vr,ar,M,h,B);restore=close(r['command'],expected,'Zero integral/scale1 does not restore original controller');close(original.integral,z,'Disabled integral accumulated')
    c=make_controller(cfg,base);e=np.full(6,.0001);r=c.update(e,z,z,z,z,M,z,B)
    close(r['I_used'],z,'Integral used before first update');close(r['command'],-c.kp*e,'Current command uses new integral');close(r['I_next'],-.001*c.ki*e,'Integral sign/units wrong')
    r2=c.update(e,z,z,z,z,M,z,B);close(r2['I_used'],r['I_next'],'Integral update not causal')
    freeze_checks=[]
    for sign in [-1.,1.]:
        c=make_controller(cfg,base);e=np.full(6,-sign*.0001);bias=sign*(c.limits+1)
        r=c.update(e,z,z,z,z,M,bias,B);require(np.all(r['antiwindup_frozen']),'Outward saturation integral not frozen');close(c.integral,z,'Frozen integral changed')
        c=make_controller(cfg,base);e=np.full(6,sign*.0001)
        r=c.update(e,z,z,z,z,M,bias+sign*c.kp*.0001,B);require(not np.any(r['antiwindup_frozen']),'Recovery integral frozen');require(np.all(sign*c.integral<0),'Recovery direction incorrect');freeze_checks.append(sign)
        c=make_controller(cfg,base);e=np.full(6,-sign*1000);r=c.update(e,z,z,z,z,M,c.kp*e,B)
        close(c.integral,sign*c.imax,'Integral torque cap failed');require(np.all(r['integral_upper_hit'] if sign>0 else r['integral_lower_hit']),'Integral cap flag missing')
    # Independent explicit 16-step recurrence verifies the assembled linear map.
    c=make_controller(cfg,base);initial=np.linspace(-1e-5,1e-5,18);c.integral=initial[12:].copy()
    r=c.update(initial[:6],initial[6:12],z,z,z,M,z,B)
    e=initial[:6].copy();vel=initial[6:12].copy();hp=base['physics_dt_s']
    for _ in range(16):
        vel=np.linalg.solve(M+hp*np.diag(B),M@vel+hp*r['command']);e=e+hp*vel
    A=frozen_closed_loop(M,B,c.kp,c.kd,c.ki,hp,base['control_dt_s']);map_error=close(A@initial,np.r_[e,vel,c.integral],'Discrete map differs from explicit recurrence')
    write(out/'unit_checks.json',dict(passed=True,restore_max=restore,map_error_max=map_error,integral_sign_order_cap_freeze=True,source_snapshots=len(state['inputSnapshots'])))
    # Model evaluations are static; no mj_step and no KF replay.
    m=mujoco.MjModel.from_binary_path(base['model']);d=mujoco.MjData(m)
    require(m.opt.integrator==mujoco.mjtIntegrator.mjINT_IMPLICITFAST,'Unexpected integrator');require(m.opt.timestep==hp,'Unexpected physics step');close(m.dof_damping,B,'Damping changed');require(not np.any(m.dof_armature) and not np.any(m.dof_frictionloss),'Unexpected passive inertia/friction')
    tr=load(old/'path/trajectory.npz');obs=load(old/'collect/measurements.npz');require(len(obs['t'])==751,'Wrong feedback population');close(tr['t'][::8],obs['t'],'Reference feedback times differ')
    rows=[];eigs=[];worst=None;c=make_controller(cfg,base)
    for name,qq in [('reference',tr['q'][::8]),('actual',obs['q'])]:
        for k,q in enumerate(qq):
            require(ledger['static_states']<cfg['static_state_cap'],'Static evaluation cap exceeded');ledger['static_states']+=1
            d.qpos[:]=q;d.qvel[:]=0;mujoco.mj_forward(m,d);M=np.zeros((6,6));mujoco.mj_fullM(m,M,d.qM)
            require(np.linalg.eigvalsh(M)[0]>0,'Nonpositive mass matrix')
            A=frozen_closed_loop(M,B,c.kp,c.kd,c.ki,hp,base['control_dt_s']);ev=np.linalg.eigvals(A);require(np.isfinite(ev).all(),'Nonfinite local poles');rho=float(abs(ev).max());i=int(np.argmax(abs(ev)))
            row=dict(population=name,time_s=float(obs['t'][k]),spectral_radius=rho,dominant_real=float(ev[i].real),dominant_imag=float(ev[i].imag),strictly_inside_unit_circle=bool(rho<1))
            rows.append(row);eigs.append(ev)
            if worst is None or rho>worst['spectral_radius']:worst=row.copy()
    csvwrite(out/'local_stability.csv',rows);np.savez_compressed(out/'local_eigenvalues.npz',eigenvalues=np.asarray(eigs))
    result=dict(passed=all(r['strictly_inside_unit_circle'] for r in rows),samples=len(rows),worst=worst,failed_states=sum(not r['strictly_inside_unit_circle'] for r in rows),kp=c.kp,kd=c.kd,ki=c.ki,integral_limits_Nm=c.imax,scope='Frozen M/B sampled-control screen only; not nonlinear/time-varying stability proof')
    write(out/'stability_checks.json',result)
    require(result['passed'],'Fixed controller failed frozen discrete stability screening')
    return dict(precheck='units and all1502 static states passed')


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--config',type=Path,required=True);ap.add_argument('--output-dir',type=Path,required=True);ap.add_argument('--mode',choices=['precheck','collect','compare'],required=True);a=ap.parse_args();cfg=json.loads(a.config.read_text());base=json.loads(Path(cfg['base_config']).read_text());out=a.output_dir;out.mkdir(parents=True,exist_ok=True);start=time.perf_counter();ledger=dict(physics_steps=0,KF_calls=0,static_states=0)
    try:
        if a.mode=='precheck':result=precheck(cfg,base,out,ledger)
        elif a.mode=='collect':result=collect(cfg,base,out,ledger)
        else:
            from paper_compare import compare
            result=compare(cfg,base,out,ledger)
        write(out/'metrics.json',dict(passed=True,wall_seconds=time.perf_counter()-start,**ledger,**result))
    except Exception as exc:
        write(out/'failure.json',dict(error=repr(exc),traceback=traceback.format_exc()));write(out/'metrics.json',dict(passed=False,wall_seconds=time.perf_counter()-start,**ledger));raise
    finally:write(out/'ledger.json',ledger)
if __name__=='__main__':main()
