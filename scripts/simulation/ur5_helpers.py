"""Shared UR5 trajectory and simulation guards; no estimator or experiment runner."""
import numpy as np


def require(ok,msg):
    if not ok:raise RuntimeError(msg)


def trajectory(t,cfg,moving):
    q0=np.array(cfg['initial_q_rad']);A=np.array(cfg['trajectory']['amplitude_rad']);w=2*np.pi*cfg['trajectory']['frequency_Hz']
    if not moving:return q0,np.zeros(6),np.zeros(6)
    return q0+A*(1-np.cos(w*t)),A*w*np.sin(w*t),A*w*w*np.cos(w*t)


def moving_mask(m):
    mask=np.zeros(m.nbody,dtype=bool)
    for b in range(1,m.nbody):mask[b]=mask[m.body_parentid[b]] or m.body_dofnum[b]>0
    return mask


def valid(m,d,mask,command):
    require(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all() and np.isfinite(d.qacc).all(),'Nonfinite state')
    require(not np.any(d.warning.number),'MuJoCo warning')
    limited=m.jnt_limited.astype(bool)
    require(np.all(d.qpos[limited]>m.jnt_range[limited,0]) and np.all(d.qpos[limited]<m.jnt_range[limited,1]),'Joint limit')
    lim=m.actuator_ctrllimited.astype(bool)
    require(np.all(command[lim]>=m.actuator_ctrlrange[lim,0]) and np.all(command[lim]<=m.actuator_ctrlrange[lim,1]),'Torque saturation')
    for i in range(d.ncon):
        c=d.contact[i]
        if c.geom1>=0 and c.geom2>=0 and (mask[m.geom_bodyid[c.geom1]] or mask[m.geom_bodyid[c.geom2]]):
            require(not (c.dist<=0 or c.efc_address>=0),'Moving link contact')
    require(np.max(np.abs(d.qfrc_constraint))<=1e-9,'Unexpected constraint torque')
    require(np.allclose(d.qfrc_actuator,command,rtol=1e-8,atol=1e-9),'Actual motor torque differs')
