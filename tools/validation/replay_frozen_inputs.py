#!/usr/bin/env python3
"""Offline, frame-acknowledged ROS replay of frozen XYZ/attitude/EKF inputs."""
import argparse,bisect,json,os,signal,subprocess,time
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.parameter import Parameter
from rclpy.serialization import deserialize_message
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from sensor_msgs.msg import PointCloud2,Imu
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--executable',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(a.input/'output_bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
wanted={'/points_aligned_xyz','/deskew_reference_attitude','/tf','/tf_static'};types={x.name:get_message(x.type) for x in reader.get_all_topics_and_types() if x.name in wanted};clouds={};refs={};odom={};static=[]
while reader.has_next():
 topic,data,_=reader.read_next()
 if topic not in wanted:continue
 m=deserialize_message(data,types[topic])
 if topic=='/points_aligned_xyz':clouds[stamp(m)]=m
 elif topic=='/deskew_reference_attitude':refs[stamp(m)]=m
 elif topic=='/tf_static':static=m.transforms
 else:
  for tr in m.transforms:
   if (tr.header.frame_id,tr.child_frame_id)==('odom','base_link'):odom[stamp(tr)]=tr
assert set(clouds)==set(refs)
os.environ['ROS_LOG_DIR']=str(a.output/'ros_logs');rclpy.init();node=rclpy.create_node('frozen_hector_migration_check',parameter_overrides=[Parameter('use_sim_time',value=True)])
qos=QoSProfile(depth=2000,reliability=ReliabilityPolicy.RELIABLE);transient=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
pubs={t:node.create_publisher(typ,t,transient if t=='/tf_static' else qos) for t,typ in [('/points_aligned_xyz',PointCloud2),('/deskew_reference_attitude',Imu),('/tf',TFMessage),('/tf_static',TFMessage),('/initialpose',PoseWithCovarianceStamped),('/clock',Clock)]}
poses={};statuses={}
def pose_cb(m):poses[stamp(m)]=m
subs=[node.create_subscription(PoseStamped,'/slam_out_pose',pose_cb,qos),node.create_subscription(String,'/localization_status',lambda m:statuses.update({int(round(json.loads(m.data)['stamp']*1e9)):json.loads(m.data)}),qos)]
config=yaml.safe_load((a.input/'hector.yaml').read_text());(a.output/'hector.yaml').write_text(yaml.safe_dump(config))
log=open(a.output/'node.log','w');proc=subprocess.Popen([str(a.executable),'--ros-args','--params-file',str(a.output/'hector.yaml')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
def spin_until(predicate,limit=10):
 end=time.monotonic()+limit
 while not predicate() and time.monotonic()<end:
  if proc.poll() is not None:raise RuntimeError('Hector exited; inspect node.log')
  rclpy.spin_once(node,timeout_sec=.005)
 if not predicate():raise RuntimeError('Timed out waiting for deterministic frame acknowledgement')
try:
 spin_until(lambda:pubs['/initialpose'].get_subscription_count()>0 and pubs['/points_aligned_xyz'].get_subscription_count()>0)
 ts=sorted(clouds);clock=Clock();clock.clock.sec,clock.clock.nanosec=divmod(ts[0]-100000000,10**9);pubs['/clock'].publish(clock);pubs['/tf_static'].publish(TFMessage(transforms=static))
 seed=json.loads((a.input/'seed.json').read_text())['pose'];m=PoseWithCovarianceStamped();m.header.frame_id='map';m.header.stamp=clock.clock;m.pose.pose.position.x=seed[0];m.pose.pose.position.y=seed[1];m.pose.pose.orientation.z=np.sin(seed[2]/2);m.pose.pose.orientation.w=np.cos(seed[2]/2);m.pose.covariance[0]=m.pose.covariance[7]=.25;m.pose.covariance[35]=.068538919;pubs['/initialpose'].publish(m)
 end=time.monotonic()+.4
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
 ot=sorted(odom);j=0
 for i,t in enumerate(ts):
  clock.clock.sec,clock.clock.nanosec=divmod(t,10**9);pubs['/clock'].publish(clock)
  end=bisect.bisect_right(ot,t+50000000)
  if end>j:pubs['/tf'].publish(TFMessage(transforms=[odom[k] for k in ot[j:end]]));j=end
  pubs['/deskew_reference_attitude'].publish(refs[t]);pubs['/points_aligned_xyz'].publish(clouds[t]);spin_until(lambda:t in poses)
  if (i+1)%200==0:print('Acknowledged',i+1,flush=True)
 rows=[]
 for t,m in sorted(poses.items()):
  v=m.pose.position;q=m.pose.orientation;rows.append([t,v.x,v.y,v.z,q.x,q.y,q.z,q.w])
 np.save(a.output/'poses.npy',np.array(rows));(a.output/'validation.json').write_text(json.dumps({'frames':len(rows),'input_frames':len(ts),'timestamp_exact':set(poses)==set(clouds),'executable':str(a.executable),'replay':'offline frozen per-frame inputs with acknowledgement, not accelerated bag playback'},indent=2)+'\n')
 print('Completed',len(rows),flush=True)
finally:
 if proc.poll() is None:
  os.killpg(proc.pid,signal.SIGINT)
  try:proc.wait(timeout=5)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
 log.close();node.destroy_node();rclpy.try_shutdown()
