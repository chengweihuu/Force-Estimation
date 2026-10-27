import argparse,csv,json,math,os,signal,subprocess,time,traceback
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from controller_manager_msgs.srv import ListControllers
p=argparse.ArgumentParser();p.add_argument('--output-dir',required=True);p.add_argument('--config-dir',default=str(Path(__file__).resolve().parents[1]/'models/ur5/control'));a=p.parse_args();out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=True);cfg=Path(a.config_dir);design=json.loads((cfg/'design.json').read_text());names=design['joint_names'];target=design['q_hold_rad'];limits=[150]*3+[28]*3
if json.loads((cfg/'equivalent_current.json').read_text()).get('gap_joint_by_drive'):
 raise ValueError('This probe scores rigid-joint feedback; use check_backlash_configuration.py for static configuration checks only. Backlash ROS closed-loop scoring is not validated.')
packets=[]
start=time.monotonic();rows=[];clocks=[];latest=None;phase='startup';violation=None;proc=None;node=None
metrics={'motion_passed':False,'robot_connections':0,'kf_updates':0,'optimizer_updates':0,'native_current_published':False};log=(out/'launch.log').open('w')
cmd=['ros2','launch',str(Path(__file__).with_name('position_control.launch.py')),'config_dir:='+str(cfg)]
print('EXECUTE:', ' '.join(cmd),flush=True)
try:
 rclpy.init();node=rclpy.create_node('ur5_small_motion_probe')
 def state_cb(msg):
  global latest,violation
  if not all(n in msg.name for n in names):return
  idx=[msg.name.index(n) for n in names];q=[msg.position[i] for i in idx];v=[msg.velocity[i] for i in idx];tau=[msg.effort[i] for i in idx] if len(msg.effort)==len(msg.name) else [float('nan')]*6
  row={'wall_s':time.monotonic()-start,'time_s':msg.header.stamp.sec+1e-9*msg.header.stamp.nanosec,'phase':phase,'q':q,'v':v,'tau':tau};rows.append(row);latest=row
  if not all(math.isfinite(x) for x in q+v+tau):violation='nonfinite_feedback'
  elif max(abs(x-y) for x,y in zip(q,target))>=.35:violation='emergency_tracking_0.35_rad'
  elif any(abs(x)>=lim-1e-6 for x,lim in zip(tau,limits)):violation='sampled_actuator_saturation'
 current_sub=node.create_subscription(String,'/ur5/measurement',lambda msg:packets.append(json.loads(msg.data)),10)
 sub=node.create_subscription(JointState,'/joint_states',state_cb,qos_profile_sensor_data)
 clocksub=node.create_subscription(Clock,'/clock',lambda msg:clocks.append(msg.clock.sec+1e-9*msg.clock.nanosec),qos_profile_sensor_data)
 pub=node.create_publisher(JointTrajectory,'/joint_trajectory_controller/joint_trajectory',10);client=node.create_client(ListControllers,'/controller_manager/list_controllers');future=None;active=False
 proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 while time.monotonic()-start<12:
  if proc.poll() is not None:raise RuntimeError('launch terminated during startup')
  rclpy.spin_once(node,timeout_sec=.03)
  if violation:raise RuntimeError(violation)
  if future is None and client.service_is_ready():future=client.call_async(ListControllers.Request())
  if future is not None and future.done():
   cs=future.result().controller;metrics['controllers']=[{'name':c.name,'state':c.state,'claimed_interfaces':list(c.claimed_interfaces)} for c in cs];active=all(any(c.name==name and c.state=='active' for c in cs) for name in ('joint_state_broadcaster','joint_trajectory_controller'));future=None
  if active and latest and clocks and pub.get_subscription_count()>0:break
 else:raise RuntimeError('startup 12s limit')
 metrics['startup_s']=time.monotonic()-start
 phase='motion';command=JointTrajectory();command.joint_names=names
 trajectory_start=clocks[-1]+0.25
 ns=round(trajectory_start*1e9);command.header.stamp.sec=ns//1000000000;command.header.stamp.nanosec=ns%1000000000
 knots=[(0.,list(target)),(2.,list(target))]
 for i in range(6):
  peak=list(target);peak[i]+=.02
  knots.extend([(2.+i+.5,peak),(3.+i,list(target))])
 knots.append((9.,list(target)))
 for t,qref in knots:
  point=JointTrajectoryPoint();point.positions=qref;point.velocities=[0.]*6;point.accelerations=[0.]*6
  point.time_from_start.sec=int(t);point.time_from_start.nanosec=round((t-int(t))*1e9);command.points.append(point)
 def reference(time_s):
  t=time_s-trajectory_start
  if t<=0:return list(target)
  for (ta,qa),(tb,qb) in zip(knots,knots[1:]):
   if t<=tb:
    u=max(0.,min(1.,(t-ta)/(tb-ta)));blend=10*u**3-15*u**4+6*u**5
    return [x+blend*(y-x) for x,y in zip(qa,qb)]
  return list(target)
 (out/'trajectory.json').write_text(json.dumps({'start_s':trajectory_start,'knots_s_rad':knots,'interpolation':'quintic, zero endpoint velocity and acceleration'},indent=2))
 pub.publish(command);metrics['command_wall_s']=time.monotonic()-start
 stop=time.monotonic()+14
 while not latest or latest['time_s']<trajectory_start+9:
  if time.monotonic()>stop:raise RuntimeError('motion simulation-time completion timeout')
  rclpy.spin_once(node,timeout_sec=.02)
  if violation:raise RuntimeError(violation)
  if proc.poll() is not None:raise RuntimeError('launch terminated during motion')
 rr=[r for r in rows if trajectory_start+2<=r['time_s']<=trajectory_start+9]
 tail=[r for r in rr if r['time_s']>=trajectory_start+8];stats=[]
 for i,name in enumerate(names):
  e=[r['q'][i]-reference(r['time_s'])[i] for r in rr];te=[r['q'][i]-target[i] for r in tail]
  own=[r for r in rr if trajectory_start+2+i<=r['time_s']<=trajectory_start+3+i]
  span=max(r['q'][i] for r in own)-min(r['q'][i] for r in own)
  stats.append({'joint':name,'motion_rmse_rad':math.sqrt(sum(x*x for x in e)/len(e)),'motion_peak_rad':max(map(abs,e)),'tail_bias_rad':sum(te)/len(te),'tail_peak_rad':max(map(abs,te)),'tail_velocity_peak_rad_s':max(abs(r['v'][i]) for r in tail),'effort_peak_Nm':max(abs(r['tau'][i]) for r in rr),'own_segment_range_rad':span})
 checks={'active':active,'samples_ge_200':len(rr)>=200,'time_increasing':all(b['time_s']>a['time_s'] for a,b in zip(rows,rows[1:])),'motion_error_lt_0p02':all(s['motion_peak_rad']<.02 for s in stats),'motion_observed_each_joint':all(s['own_segment_range_rad']>.005 for s in stats),'tail_error_lt_0p02':all(s['tail_peak_rad']<.02 for s in stats),'tail_velocity_lt_0p02':all(s['tail_velocity_peak_rad_s']<.02 for s in stats),'no_sampled_emergency':violation is None,'clock_nondecreasing':all(b>=a for a,b in zip(clocks,clocks[1:]))}
 source={round(r['time_s']*1e9):r for r in rows}
 paired=[(packet,source.get(packet['stamp_sec']*1000000000+packet['stamp_nanosec'])) for packet in packets]
 paired=[(packet,row) for packet,row in paired if row is not None]
 conversion=json.loads((cfg/'equivalent_current.json').read_text())['K_equivalent_Nm_A']
 closure=max((abs(packet['current_A'][i]*conversion[i]-row['tau'][i]) for packet,row in paired for i in range(6)),default=float('inf'))
 state_error=max((abs(packet[field][i]-row[key][i]) for packet,row in paired for field,key in [('position_rad','q'),('velocity_rad_s','v')] for i in range(6)),default=float('inf'))
 checks.update(current_paired_samples_ge_200=len(paired)>=200,current_closure=closure<1e-12,current_same_frame_state=state_error==0)
 metrics.update(equivalent_current_packets=len(packets),paired_current_packets=len(paired),current_closure_Nm=closure,current_state_error=state_error,current_read_only=True)
 metrics.update(joints=stats,checks=checks,motion_samples=len(rr),motion_sim_span_s=rr[-1]['time_s']-rr[0]['time_s'],motion_passed=all(checks.values()))
except Exception as exc:metrics.update(error=str(exc),traceback=traceback.format_exc())
finally:
 if proc and proc.poll() is None:
  os.killpg(proc.pid,signal.SIGINT)
  try:proc.wait(timeout=4)
  except subprocess.TimeoutExpired:
   os.killpg(proc.pid,signal.SIGTERM)
   try:proc.wait(timeout=2)
   except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=1)
 if node:node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
 log.close();metrics.update(elapsed_s=time.monotonic()-start,feedback_samples=len(rows),launch_returncode=proc.returncode if proc else None,command_target_rad=target)
 (out/'measurement.json').write_text(json.dumps(packets,indent=2))
 (out/'metrics.json').write_text(json.dumps(metrics,indent=2));(out/'clock.json').write_text(json.dumps(clocks));(out/'feedback.json').write_text(json.dumps(rows,indent=2));print(json.dumps(metrics,indent=2),flush=True)
raise SystemExit(0 if metrics['motion_passed'] else 1)
