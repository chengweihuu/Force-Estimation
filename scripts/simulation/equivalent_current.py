"""Read-only simulation current adapter. JSON topic, amperes explicit; no control writes."""
import argparse,json,math
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

def measurement_packet(msg, cfg):
    """Convert one atomic source message; never infer current from gap reaction."""
    names = cfg['joint_names']
    k = cfg['K_equivalent_Nm_A']
    gaps = cfg.get('gap_joint_by_drive', {})
    required = list(names) + list(gaps.values())
    if not all(n in msg.name for n in required):
        raise ValueError('Missing drive/gap feedback')
    if not (len(msg.position) == len(msg.velocity) == len(msg.effort) == len(msg.name)):
        raise ValueError('Incomplete joint feedback')
    ix = [msg.name.index(n) for n in names]
    motor_q = [msg.position[j] for j in ix]
    motor_v = [msg.velocity[j] for j in ix]
    tau = [msg.effort[j] for j in ix]
    gap_q = [msg.position[msg.name.index(gaps[n])] if n in gaps else 0.0 for n in names]
    gap_v = [msg.velocity[msg.name.index(gaps[n])] if n in gaps else 0.0 for n in names]
    if not all(math.isfinite(x) for x in motor_q + motor_v + tau + gap_q + gap_v):
        raise ValueError('Nonfinite feedback')
    current = [t/c for t,c in zip(tau,k)]
    packet = {'schema': 'ur5_equivalent_current_v2' if gaps else 'ur5_equivalent_current_v1',
              'stamp_sec': msg.header.stamp.sec, 'stamp_nanosec': msg.header.stamp.nanosec,
              'joint_names': names, 'position_rad': [x+y for x,y in zip(motor_q,gap_q)],
              'velocity_rad_s': [x+y for x,y in zip(motor_v,gap_v)], 'current_A': current,
              'drive_torque_estimate_Nm': [c*i for c,i in zip(k,current)],
              'current_kind': 'joint_side_equivalent', 'noise_enabled': False}
    if gaps:
        packet.update(position_coordinate='load_side', control_coordinate='drive_side',
                      motor_position_rad=motor_q, motor_velocity_rad_s=motor_v,
                      backlash_angle_rad=gap_q, backlash_velocity_rad_s=gap_v)
    return packet

def main():
    p=argparse.ArgumentParser();p.add_argument('--config',required=True);a=p.parse_args()
    cfg=json.loads(Path(a.config).read_text());names=cfg['joint_names'];k=cfg['K_equivalent_Nm_A']
    if len(names)!=6 or len(k)!=6 or not all(math.isfinite(x) and x>0 for x in k):raise ValueError('Invalid current conversion configuration')
    rclpy.init();node=rclpy.create_node('equivalent_current_reader')
    pub=node.create_publisher(String,'/ur5/measurement',10)
    def callback(msg):
        try:
            packet=measurement_packet(msg,cfg)
        except ValueError as exc:
            node.get_logger().error(str(exc));return
        out=String();out.data=json.dumps(packet,allow_nan=False);pub.publish(out)
    node.create_subscription(JointState,'/joint_states',callback,qos_profile_sensor_data)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
