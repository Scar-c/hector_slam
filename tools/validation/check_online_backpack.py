#!/usr/bin/env python3
"""Real 1x bag + fresh Hector frontend + online backend end-to-end check."""
import argparse,gzip,json,os,signal,subprocess,time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from std_srvs.srv import Trigger
from hector_backend_msgs.msg import SubmapList
from hector_backend_msgs.srv import SubmapQuery
p=argparse.ArgumentParser();p.add_argument('--bag',required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--duration',type=float,default=30);p.add_argument('--rviz',action='store_true');a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);os.environ['ROS_LOG_DIR']=str(a.output/'ros_logs')
rclpy.init();node=rclpy.create_node('hector_online_validation');counts={};last_status={};submap_count=[];pose_rows=[];processes=[];handles=[];topics=[]
def cb(topic,m):
 counts[topic]=counts.get(topic,0)+1
 if topic=='/backend_status':last_status.update(json.loads(m.data))
 if topic=='/submap_list':submap_count.append(len(m.submap))
 if topic=='/slam_out_pose':
  assert m.header.frame_id=='hector_local_map';s=m.header.stamp.sec+m.header.stamp.nanosec*1e-9;p=m.pose.position;q=m.pose.orientation;pose_rows.append([s,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
for topic,typ in [('/slam_out_pose',PoseStamped),('/hector_matching_points',PointCloud2),('/submap_list',SubmapList),('/backend_status',String),('/map',OccupancyGrid)]:
 qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL) if topic in ['/submap_list','/map'] else QoSProfile(depth=2000,reliability=ReliabilityPolicy.BEST_EFFORT)
 topics.append(node.create_subscription(typ,topic,lambda m,t=topic:cb(t,m),qos))
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
def start(name,cmd):
 f=open(a.output/(name+'.log'),'w');handles.append(f);p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p);return p
def call(name,typ,req):
 client=node.create_client(typ,name);end=time.monotonic()+180
 while not client.service_is_ready() and time.monotonic()<end:spin(.05)
 assert client.service_is_ready(),name
 future=client.call_async(req)
 while not future.done() and time.monotonic()<end:spin(.05)
 assert future.done(),name
 result=future.result();node.destroy_client(client);return result
try:
 launch=start('pipeline',['ros2','launch','hector_backend','online_backpack.launch.py','use_sim_time:=true','rviz:='+str(a.rviz).lower(),'output_directory:='+str(a.output/'backend')]);spin(5);assert launch.poll() is None
 cmd=['ros2','bag','play',a.bag,'--clock','200','--rate','1','--delay','2','--disable-keyboard-controls']
 if a.duration:cmd+=['--playback-duration',str(a.duration)]
 player=start('player',cmd);deadline=time.monotonic()+(a.duration or 514)+45
 while player.poll() is None and time.monotonic()<deadline:
  spin(.1);assert launch.poll() is None,'Pipeline exited'
 assert player.poll()==0,'Player failed or timed out'
 spin(3)
 req=SubmapQuery.Request();req.trajectory_id=0;req.submap_index=0;response=call('/submap_query',SubmapQuery,req);assert response.status.code==0 and len(response.textures)>0
 tex=response.textures[0];decoded=gzip.decompress(bytes(tex.cells));assert len(decoded)==tex.width*tex.height*2
 finish=call('/finish_hector_backend',Trigger,Trigger.Request());assert finish.success,finish.message;spin(1)
 assert last_status.get('finished') and last_status['nodes']>0 and max(submap_count)>=2
 assert last_status['frames']==len(pose_rows), 'Backend input coverage differs from frontend poses'
 assert 'PluginlibFactory' not in (a.output/'pipeline.log').read_text(), 'RViz plugin failed to load'
 import numpy as np
 np.savetxt(a.output/'trajectory_hector_local.tum',np.array(pose_rows),fmt='%.12f')
 summary={'passed':True,'real_playback_rate':1,'duration_s':a.duration or 'full','rviz_requested':a.rviz,'counts':counts,'backend':last_status,'submap_count_progression_unique':sorted(set(submap_count)),'texture_query':{'status':response.status.code,'width':tex.width,'height':tex.height,'decoded_bytes':len(decoded)},'frontend_frame':'hector_local_map','backend_frame':'map','external_cartographer_prefix_required':False}
 (a.output/'validation.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2),flush=True)
finally:
 for proc in reversed(processes):
  if proc.poll() is None:
   os.killpg(proc.pid,signal.SIGINT)
   try:proc.wait(timeout=12)
   except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
 node.destroy_node();rclpy.try_shutdown()
 for f in handles:f.close()
