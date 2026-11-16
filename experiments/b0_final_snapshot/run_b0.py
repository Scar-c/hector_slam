#!/usr/bin/env python3
"""Single horizontal MultiEcho lidar + IMU + causal Hector pose deskew/mapping."""
import argparse,hashlib,json,math,os,shutil,signal,subprocess,time
from collections import Counter
from pathlib import Path
import numpy as np
from PIL import Image
import yaml,rclpy,rosbag2_py
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from rclpy.serialization import serialize_message
from sensor_msgs.msg import MultiEchoLaserScan,Imu,LaserScan,PointCloud2
from nav_msgs.msg import OccupancyGrid,Path as NavPath
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from rosbag2_interfaces.srv import Resume
ROOT=Path(__file__).resolve().parent;WS=ROOT.parents[1]
def main():
 p=argparse.ArgumentParser();p.add_argument('--bag',type=Path,default=Path('/home/lc/Downloads/b0-2014-07-11-11-00-49_ros2'));p.add_argument('--output',type=Path,required=True);p.add_argument('--duration',type=float,default=0);p.add_argument('--rviz',action=argparse.BooleanOptionalAction,default=False);p.add_argument('--hold',action=argparse.BooleanOptionalAction,default=False);p.add_argument('--translation',action=argparse.BooleanOptionalAction,default=True);a=p.parse_args();out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
 os.environ['ROS_LOG_DIR']=str(out/'ros_logs');os.environ.setdefault('MPLCONFIGDIR',str(out/'matplotlib_cache'))
 meta=yaml.safe_load((a.bag/'metadata.yaml').read_text())['rosbag2_bagfile_information'];expected={x['topic_metadata']['name'].strip('/'):x['message_count'] for x in meta['topics_with_message_count']}
 snapshot=out/'config_snapshot';snapshot.mkdir();files=[Path(__file__),ROOT/'deskew.cpp',ROOT/'backpack_2d.urdf',ROOT/'hector_cloud_node.cpp'];hashes={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in files}
 for file in files:shutil.copy2(file,snapshot/file.name)
 shutil.copytree(ROOT/'include',snapshot/'include')
 shutil.copy2(ROOT/'CMakeLists.txt',snapshot/'CMakeLists.txt')
 (snapshot/'sha256.json').write_text(json.dumps(hashes,indent=2))
 hp={'hector_mapping':{'ros__parameters':{'use_sim_time':True,'localization_only':False,'scan_topic':'/scan_carto','imu_topic':'/imu','use_imu':True,'map_frame':'map','base_frame':'base_link','odom_frame':'base_link','pub_map_odom_transform':True,'pub_odometry':False,'use_odom_pose_start_estimate':False,'use_tf_pose_start_estimate':False,'map_size':(8192 if not a.duration else 4096),'map_resolution':.05,'map_multi_res_levels':3,'map_pub_period':2.0,'map_update_distance_thresh':.2,'map_update_angle_thresh':.08,'laser_min_dist':.5,'laser_z_min_value':-.8,'laser_z_max_value':2.0,'cloud_voxel_size':.025,'laser_max_dist':30.0,'scan_subscriber_queue_size':1000,'output_timing':True}}}
 (out/'hector.yaml').write_text(yaml.safe_dump(hp));(out/'robot.yaml').write_text(yaml.safe_dump({'robot_state_publisher':{'ros__parameters':{'robot_description':(ROOT/'backpack_2d.urdf').read_text(),'use_sim_time':True}}}))
 rclpy.init();n=rclpy.create_node('b0_hector_deskew_replay',parameter_overrides=[Parameter('use_sim_time',value=True)]);processes=[];handles=[];subs=[];counts=Counter();poses={};statuses=[];cloud_statuses=[];maps=[];writer=None;path=NavPath();path.header.frame_id='map';path_state={'added':0,'published':0}
 def spin(seconds,bootstrap=False):
  end=time.monotonic()+seconds;last=0
  while time.monotonic()<end:
   if bootstrap and time.monotonic()-last>.05:clockpub.publish(clock);last=time.monotonic()
   rclpy.spin_once(n,timeout_sec=.01)
 def start(name,cmd):
  f=(out/(name+'.log')).open('w');handles.append(f);proc=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append((name,proc));(out/'runtime_processes.json').write_text(json.dumps([{'name':k,'pid':v.pid,'command':v.args} for k,v in processes],indent=2));return proc
 spin(1.5)
 occupied={t:n.count_publishers(t) for t in ['/map','/clock','/imu','/horizontal_laser_2d']}
 if any(occupied.values()):raise RuntimeError('ROS domain occupied: '+str(occupied))
 clockpub=n.create_publisher(Clock,'/clock',10);clock=Clock();clock.clock.sec,clock.clock.nanosec=divmod(meta['starting_time']['nanoseconds_since_epoch'],10**9)
 pathpub=n.create_publisher(NavPath,'/hector_path',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
 writer=rosbag2_py.SequentialWriter();writer.open(rosbag2_py.StorageOptions(uri=str(out/'output_bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
 def callback(topic,msg,record=True):
  counts[topic]+=1
  stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec if hasattr(msg,'header') else n.get_clock().now().nanoseconds
  if record and writer is not None:writer.write(topic,serialize_message(msg),max(0,stamp))
  if topic=='/map':maps[:]=[msg]
  if topic=='/slam_out_pose':
   poses[stamp]=msg
   if stamp-path_state['added']>=100000000:path.header=msg.header;path.poses.append(msg);path_state['added']=stamp
   if stamp-path_state['published']>=500000000:pathpub.publish(path);path_state['published']=stamp
  if topic=='/cloud_input_status':cloud_statuses.append(json.loads(msg.data))
  if topic=='/deskew_status':
   s=json.loads(msg.data);statuses.append(s)
   if s['velocity_pose_stamp_ns'] and s['velocity_pose_stamp_ns']>=s['source_first_stamp_ns']:raise RuntimeError('Deskew used nonhistorical pose')
 for topic,typ,record in [('/horizontal_laser_2d',MultiEchoLaserScan,False),('/imu',Imu,True),('/scan',LaserScan,True),('/scan_carto',LaserScan,True),('/points_carto_raw',PointCloud2,True),('/points_carto',PointCloud2,True),('/hector_matching_points',PointCloud2,True),('/slam_out_pose',PoseStamped,True),('/poseupdate',PoseWithCovarianceStamped,True),('/deskew_status',String,True),('/deskew_reference_attitude',Imu,True),('/cloud_input_status',String,True),('/tf',TFMessage,True),('/tf_static',TFMessage,True),('/map',OccupancyGrid,False)]:
  if record:writer.create_topic(rosbag2_py.TopicMetadata(id=len(subs),name=topic,type=typ.__module__.split('.')[0]+'/msg/'+typ.__name__,serialization_format='cdr'))
  qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL) if topic in ['/map','/tf_static'] else (QoSProfile(depth=4000) if topic in ['/deskew_status','/deskew_reference_attitude','/cloud_input_status','/slam_out_pose','/poseupdate'] else QoSProfile(depth=4000,reliability=qos_profile_sensor_data.reliability))
  subs.append(n.create_subscription(typ,topic,lambda msg,t=topic,r=record:callback(t,msg,r),qos))
 try:
  start('robot',['/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher','--ros-args','--params-file',str(out/'robot.yaml')])
  start('deskew',[str(ROOT/'build/carto_rotation_deskew'),'--ros-args','-p','use_sim_time:=true','-p','single_scan:=true','-p','publish_scan:=true','-p','compensate_translation:='+str(a.translation).lower(),'-p','historical_pose_topic:=/poseupdate','-p','min_range:=0.5','-p','crop_stamp_compensation:=false'])
  start('hector',[str(ROOT/'build/hector_cloud_node'),'--ros-args','--params-file',str(out/'hector.yaml')])
  spin(3,True)
  if a.rviz:
   start('rviz',['rviz2','-d',str(ROOT/'b0_hector.rviz'),'--ros-args','-p','use_sim_time:=true']);spin(5,True)
   try:
    import sys;sys.path.insert(0,str(WS/'test/hector_prior_ws/scripts'));from raise_rviz_window import raise_window;print('RViz foreground:',raise_window(str(ROOT/'b0_hector.rviz')),flush=True)
   except Exception as e:print('RViz focus:',e,flush=True)
  cmd=['ros2','bag','play',str(a.bag),'--clock','200','--rate','1.0','--start-paused','--disable-keyboard-controls']
  if a.duration:cmd+=['--playback-duration',str(a.duration)]
  player=start('player',cmd);client=n.create_client(Resume,'/rosbag2_player/resume');end=time.monotonic()+15
  while not client.service_is_ready() and time.monotonic()<end:spin(.1,True)
  if not client.service_is_ready():raise RuntimeError('No bag resume service')
  spin(3,True)
  assert n.count_publishers('/clock')==2
  future=client.call_async(Resume.Request());end=time.monotonic()+5
  while not future.done() and time.monotonic()<end:spin(.05)
  assert future.done();print('Playing at 1x:',a.bag,flush=True)
  end=time.monotonic()+(a.duration or meta['duration']['nanoseconds']/1e9)+45;last=time.monotonic()
  while player.poll() is None and time.monotonic()<end:
   spin(.1)
   for name,proc in processes:
    if name not in ['player','rviz'] and proc.poll() is not None:raise RuntimeError(name+' exited')
   if time.monotonic()-last>30:print('Progress:',dict(counts),'poses',len(poses),flush=True);last=time.monotonic()
  assert player.poll()==0,'Playback incomplete';spin(2)
  assert poses and maps and statuses,'Missing deskew/mapping output'
  assert len(poses)==counts['/points_carto']==counts['/cloud_input_status']==counts['/deskew_reference_attitude']==counts['/deskew_status'],dict(counts)
  if not a.duration:
   assert counts['/horizontal_laser_2d']==expected['horizontal_laser_2d'],dict(counts)
   assert counts['/imu']==expected['imu'],dict(counts)
  saver=start('map_saver',['/opt/ros/jazzy/lib/nav2_map_server/map_saver_cli','-f',str(out/'map'),'-t','/map','--ros-args','-p','map_subscribe_transient_local:=true','-p','save_map_timeout:=15.0']);end=time.monotonic()+25
  while saver.poll() is None and time.monotonic()<end:spin(.1)
  assert saver.poll()==0,'Map saving failed'
  image=Image.open(out/'map.pgm');image.save(out/'map.png')
  with (out/'trajectory_hector.tum').open('w') as f:
   for t,m in sorted(poses.items()):
    q=m.pose.orientation;v=m.pose.position;f.write(f'{t//10**9}.{t%10**9:09d} {v.x} {v.y} {v.z} {q.x} {q.y} {q.z} {q.w}\n')
  (out/'cloud_input_status.jsonl').write_text(''.join(json.dumps(s)+'\n' for s in cloud_statuses))
  (out/'deskew_status.jsonl').write_text(''.join(json.dumps(s)+'\n' for s in statuses))
  summary={'bag':str(a.bag),'algorithm':'Hector mapping','deskew':'Existing Cartographer ImuTracker rotation + causal historical Hector pose translation' if a.translation else 'Existing Cartographer ImuTracker rotation only','translation':a.translation,'cartographer_mapping':False,'input_pipeline':'Full XYZ deskew cloud + exact shared reference attitude + one gravity projection + height [-0.8,2.0]m + 0.025m 3D voxel filter; original Hector matching core','ekf':False,'wheel_odom':False,'multi_echo_policy':'First echo; empty beam -> infinity','vertical_laser_used':False,'replay_rate':1.0,'counts':dict(counts),'poses':len(poses),'translation_valid_clouds':sum(s['translation_valid'] for s in statuses),'deskew_clouds':len(statuses),'all_velocity_poses_strictly_historical':all(not s['velocity_pose_stamp_ns'] or s['velocity_pose_stamp_ns']<s['source_first_stamp_ns'] for s in statuses),'map_size':list(image.size),'full_bag':not bool(a.duration)}
  (out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2),flush=True)
  writer.close();writer=None
  subprocess.run(['python3',str(ROOT/'plot_b0.py'),'--output',str(out),'--bag',str(a.bag),'--method','hector'],check=True)
  overview=Path('/home/lc/slam_11_13/overview');overview.mkdir(parents=True,exist_ok=True)
  label='full' if not a.duration else f'{a.duration:g}s'
  shutil.copy2(out/'localization_overview.png',overview/f'b0_hector_gravity_cloud_{label}.png')
  if a.hold:
   print('Map saved; holding RViz. Ctrl+C to stop.',flush=True)
   while rclpy.ok():spin(.5)
 finally:
  for name,proc in reversed(processes):
   if proc.poll() is None:
    os.killpg(proc.pid,signal.SIGINT)
    try:proc.wait(timeout=8)
    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
  if writer is not None:writer.close()
  n.destroy_node();rclpy.try_shutdown()
  for f in handles:f.close()
if __name__=='__main__':
 try:main()
 except KeyboardInterrupt:pass
