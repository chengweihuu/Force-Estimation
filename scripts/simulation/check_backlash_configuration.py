"""Static configuration checks for named drive/gap pairs; never advances physics.

This is not a closed-loop or reversal-response acceptance test. Artificial states
just inside/outside each limit check constraint wiring, not dynamic penetration.
"""
import argparse
import csv
import json
from pathlib import Path
from types import SimpleNamespace
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import yaml

from equivalent_current import measurement_packet


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--config-dir', type=Path, default=Path(__file__).resolve().parents[1] / 'models/ur5/control_backlash')
    args = parser.parse_args()
    start = time.perf_counter()
    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    cfg = args.config_dir
    design = json.loads((cfg / 'design.json').read_text())
    feedback = json.loads((cfg / 'equivalent_current.json').read_text())
    names = feedback['joint_names']
    gaps = [feedback['gap_joint_by_drive'][n] for n in names]
    model = mujoco.MjModel.from_xml_path(str((cfg / design['model']).resolve()))
    rigid = mujoco.MjModel.from_xml_path(str(cfg.parent / 'scene_equivalent_current.xml'))
    data, ref = mujoco.MjData(model), mujoco.MjData(rigid)
    drive_ids = np.array([model.joint(n).id for n in names])
    gap_ids = np.array([model.joint(n).id for n in gaps])
    dq, gq = model.jnt_qposadr[drive_ids], model.jnt_qposadr[gap_ids]
    dv, gv = model.jnt_dofadr[drive_ids], model.jnt_dofadr[gap_ids]
    ai = np.array([model.actuator(n + '_motor').id for n in names])
    half = model.jnt_range[gap_ids, 1]
    checks = {
        'twelve_coordinates_six_motors': model.nq == model.nv == 12 and model.nu == 6,
        'six_gap_pairs': len(set(gaps)) == 6,
        'one_arcmin_full_width': bool(np.max(abs(2 * half - np.pi / 10800)) < 1e-15),
        'symmetric_gap_limits': bool(np.array_equal(model.jnt_range[gap_ids, 0], -half)),
        'motor_targets': bool(np.array_equal(model.actuator_trnid[ai, 0], drive_ids)),
        'unit_gearing': bool(np.all(model.actuator_gear[ai, 0] == 1) and np.all(model.actuator_gear[ai, 1:] == 0)),
        'coincident_hinges': bool(np.array_equal(model.jnt_bodyid[drive_ids], model.jnt_bodyid[gap_ids]) and np.array_equal(model.jnt_pos[drive_ids], model.jnt_pos[gap_ids]) and np.array_equal(model.jnt_axis[drive_ids], model.jnt_axis[gap_ids])),
        'drive_armature_positive': bool(np.all(model.dof_armature[dv] > 0)),
        'rigid_body_parameters_unchanged': all(np.array_equal(getattr(model, x), getattr(rigid, x)) for x in ('body_mass', 'body_inertia', 'body_ipos', 'body_iquat', 'body_pos', 'body_quat')),
    }
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    mass = np.empty((model.nv, model.nv))
    mujoco.mj_fullM(model, data, mass)
    min_eigenvalue = float(np.linalg.eigvalsh(mass).min())
    checks['positive_mass_at_hold'] = min_eigenvalue > 0
    checks['zero_gap_at_hold'] = bool(np.all(data.qpos[gq] == 0))
    checks['no_limit_at_gap_center'] = data.nefc == 0
    records = []
    # Deliberately assign static states; these are not samples of a trajectory.
    for index, name in enumerate(names):
        for side in (-1, 1):
            mujoco.mj_resetDataKeyframe(model, data, 0)
            data.qpos[gq[index]] = side * half[index] * 1.1
            mujoco.mj_forward(model, data)
            reaction = float(data.qfrc_constraint[gv[index]])
            records.append([name, side, float(data.qpos[gq[index]]), reaction])
    checks['both_static_stop_signs_all_six'] = all(side * reaction < 0 for _, side, _, reaction in records)

    # Named addressing matters: qpos is drive, gap, drive, gap, ... .
    mujoco.mj_resetDataKeyframe(model, data, 0)
    data.qpos[gq] = half * np.array([.5, -.4, .3, -.2, .1, -.5])
    data.qvel[dv] = [.01, -.02, .03, -.04, .05, -.06]
    data.qvel[gv] = [.002, -.001, .003, -.002, .001, -.003]
    data.ctrl[ai] = [1, -2, 3, -.4, .5, -.6]
    mujoco.mj_forward(model, data)
    for index, name in enumerate(names):
        rj = rigid.joint(name).id
        ref.qpos[rigid.jnt_qposadr[rj]] = data.qpos[dq[index]] + data.qpos[gq[index]]
        ref.qvel[rigid.jnt_dofadr[rj]] = data.qvel[dv[index]] + data.qvel[gv[index]]
    mujoco.mj_forward(rigid, ref)
    kin_error = max(float(np.max(abs(getattr(data, field) - getattr(ref, field)))) for field in ('site_xpos', 'site_xmat'))
    checks['summed_link_kinematics'] = kin_error < 1e-12
    msg = SimpleNamespace(name=[model.joint(j).name for j in range(model.njnt)], position=data.qpos.tolist(), velocity=data.qvel.tolist(), effort=data.qfrc_actuator.tolist(), header=SimpleNamespace(stamp=SimpleNamespace(sec=12, nanosec=345)))
    for j in gv:
        msg.effort[j] = 100.0  # A passive reaction must never become motor current.
    packet = measurement_packet(msg, feedback)
    mapping_error = max(float(np.max(abs(np.array(packet['position_rad']) - data.qpos[dq] - data.qpos[gq]))), float(np.max(abs(np.array(packet['velocity_rad_s']) - data.qvel[dv] - data.qvel[gv]))), float(np.max(abs(np.array(packet['drive_torque_estimate_Nm']) - data.actuator_force[ai]))))
    checks['current_and_load_mapping'] = mapping_error < 1e-12
    checks['source_timestamp_preserved'] = (packet['stamp_sec'], packet['stamp_nanosec']) == (12, 345)
    missing_rejected = []
    for gap in gaps:
        ix = [j for j, n in enumerate(msg.name) if n != gap]
        incomplete = SimpleNamespace(**{f: [getattr(msg, f)[j] for j in ix] for f in ('name', 'position', 'velocity', 'effort')}, header=msg.header)
        try:
            measurement_packet(incomplete, feedback)
            missing_rejected.append(False)
        except ValueError:
            missing_rejected.append(True)
    checks['every_missing_gap_rejected'] = all(missing_rejected)
    robot = ET.parse(cfg / 'robot.urdf').getroot()
    interface = robot.find('ros2_control')
    checks['ros_six_commands_twelve_states'] = len(interface.findall('joint/command_interface')) == 6 and len(interface.findall('joint')) == 12
    checks['ros_gap_state_only'] = all(len(interface.find(f"joint[@name='{g}']").findall('command_interface')) == 0 for g in gaps)
    checks['urdf_gap_limits'] = all(np.max(abs(np.array([float(robot.find(f"joint[@name='{g}']/limit").get(a)) for a in ('lower', 'upper')]) - model.jnt_range[j])) < 1e-15 for g, j in zip(gaps, gap_ids))
    checks['unchanged_controller_gains'] = (cfg / 'pids.yaml').read_bytes() == (cfg.parent / 'control/pids.yaml').read_bytes()
    commands = yaml.safe_load((cfg / 'controllers.yaml').read_text())['joint_trajectory_controller']['ros__parameters']['joints']
    checks['trajectory_commands_only_drives'] = commands == names
    checks['finite_no_warning_no_collision'] = bool(np.isfinite(data.qacc).all() and not np.any(data.warning.number) and data.ncon == 0)
    metrics = dict(passed=all(checks.values()), checks=checks, nq=model.nq, nu=model.nu, physics_steps=0, ROS_end_to_end_tested=False, min_mass_eigenvalue_kgm2=min_eigenvalue, kinematic_max_error=kin_error, mapping_max_error=mapping_error, elapsed_s=time.perf_counter()-start, scope='Static configuration and mapping only; no dynamic penetration, stability or closed-loop acceptance', stop_reactions=records)
    (out / 'metrics.json').write_text(json.dumps(metrics, indent=2) + '\n')
    with (out / 'static_stops.csv').open('w') as f:
        writer = csv.writer(f)
        writer.writerow(['joint', 'side', 'assigned_static_gap_rad', 'gap_constraint_Nm'])
        writer.writerows(records)
    print(json.dumps(metrics, indent=2))
    return 0 if metrics['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
