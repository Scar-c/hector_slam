#!/usr/bin/env python3
"""Isolated scan+IMU replay for the developed fixed-map Hector adapter."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import shutil
import subprocess
import time
import yaml
import numpy as np
from PIL import Image
import rclpy
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.parameter import Parameter
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path as NavPath, Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rosbag2_interfaces.srv import Resume
import rosbag2_py

ROOT=Path(__file__).resolve().parents[1]
WORKSPACE=ROOT.parents[1]

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--bag',type=Path,default=Path('/home/lc/slam_11_13'))
    parser.add_argument('--map',type=Path,default=Path('/home/lc/slam_11_13/results/business_single/lo/map.yaml'))
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--ekf',action=argparse.BooleanOptionalAction,default=False)
    parser.add_argument('--duration',type=float,default=0)
    parser.add_argument('--global-localization',action=argparse.BooleanOptionalAction,default=True)
    parser.add_argument('--hold',action=argparse.BooleanOptionalAction,default=True)
    parser.add_argument('--rviz',action=argparse.BooleanOptionalAction,default=True)
    parser.add_argument('--rviz-config',type=Path,default=Path('/home/lc/slam_11_13/scripts/hector.rviz'))
    parser.add_argument('--seed-offset',type=float,nargs=3,default=[.6,-.4,.2])
    args=parser.parse_args()
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    os.environ['ROS_LOG_DIR']=str(out/'ros_logs')
    os.environ.setdefault('MPLCONFIGDIR',str(out/'matplotlib_cache'))
    shutil.copyfile(args.map,out/'map.yaml')
    source_map=yaml.safe_load(args.map.read_text())
    map_image=args.map.parent/source_map['image']
    shutil.copyfile(map_image,out/'map.pgm')
    source_map['image']='map.pgm'
    (out/'map.yaml').write_text(yaml.safe_dump(source_map))
    metadata=yaml.safe_load((args.bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    map_config=yaml.safe_load(args.map.read_text());pgm=args.map.parent/map_config['image']
    files=[args.map,pgm,args.bag/'metadata.yaml',*[args.bag/x for x in metadata['relative_file_paths']]]
    ekf_source=WORKSPACE/'src/algorithm/odom_fusion/config/ekf.yaml'
    if args.ekf:files+=[ekf_source]
    original_hash={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    params={'hector_mapping':{'ros__parameters':{
        'use_sim_time':True,'localization_only':True,'prior_map_yaml':str(args.map),
        'scan_topic':'/scan_amcl','imu_topic':'/imu','use_imu':True,
        'base_frame':'base_link','odom_frame':'odom' if args.ekf else 'base_link','use_tf_pose_start_estimate':False,
        'use_odom_pose_start_estimate':args.ekf,'transform_tolerance':.3 if args.ekf else 0.0,
        'pub_odometry':True,'global_relocalization':args.global_localization,
        'map_multi_res_levels':3,'laser_max_dist':30.0,'map_pub_period':1.0,
        'output_timing':True,'localization_min_score':.45}}}
    if args.ekf:shutil.copyfile(ekf_source,out/'ekf.yaml')
    (out/'hector.yaml').write_text(yaml.safe_dump(params))
    urdf=WORKSPACE/'src/zx_description/urdf/sw2.urdf'
    (out/'robot.yaml').write_text(yaml.safe_dump({'robot_state_publisher':{'ros__parameters':{
        'robot_description':urdf.read_text(),'use_sim_time':True}}}))
    rclpy.init();node=rclpy.create_node('hector_prior_replay',parameter_overrides=[Parameter('use_sim_time',value=True)])
    writer=rosbag2_py.SequentialWriter();writer.open(rosbag2_py.StorageOptions(uri=str(out/'output_bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
    processes=[];logs=[];subscriptions=[];counts={};poses={};business_poses={};ekf_poses={};transforms={};statuses=[];map_hashes=set();graph={};map_received=[]
    path=NavPath();path.header.frame_id='map';path_pub=node.create_publisher(NavPath,'/hector_path',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def callback(topic,msg):
        counts[topic]=counts.get(topic,0)+1
        stamp=node.get_clock().now().nanoseconds
        if hasattr(msg,'header'):stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if writer is not None:writer.write(topic,serialize_message(msg),max(0,stamp))
        if topic=='/slam_out_pose':
            poses[stamp]=msg
            if not args.ekf:path.header=msg.header;path.poses.append(msg);path_pub.publish(path)
        if topic=='/base_pose':
            if stamp not in business_poses and args.ekf:
                path.header=msg.header;path.poses.append(msg);path_pub.publish(path)
            business_poses[stamp]=msg
        if topic=='/odom':ekf_poses[stamp]=PoseStamped(header=msg.header,pose=msg.pose.pose)
        if topic=='/tf':
            for t in msg.transforms:
                key=(t.header.frame_id,t.child_frame_id)
                if key in [('map','odom'),('odom','base_link')]:
                    transforms.setdefault(key,{})[t.header.stamp.sec*10**9+t.header.stamp.nanosec]=t.transform
        if topic=='/localization_status':statuses.append(json.loads(msg.data))
        if topic=='/map':
            map_received[:]=[msg]
            signature=serialize_message(msg.info.origin)+bytes(np.asarray(msg.data,dtype=np.int8))+str((msg.info.width,msg.info.height,msg.info.resolution)).encode()
            map_hashes.add(hashlib.sha256(signature).hexdigest())
    for topic,typ in [('/scan',LaserScan),('/scan_back',LaserScan),('/scan_amcl',LaserScan),('/imu',Imu),('/clock',Clock),('/tf',TFMessage),('/tf_static',TFMessage),('/map',OccupancyGrid),('/slam_out_pose',PoseStamped),('/poseupdate',PoseWithCovarianceStamped),('/localization_status',String),('/hector_path',NavPath),('/wheel_odom',Odometry),('/odom',Odometry),('/base_pose',PoseStamped)]:
        writer.create_topic(rosbag2_py.TopicMetadata(id=len(subscriptions),name=topic,type=typ.__module__.split('.')[0]+'/msg/'+typ.__name__,serialization_format='cdr'))
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL) if topic in ['/map','/tf_static','/hector_path'] else QoSProfile(depth=4000,reliability=ReliabilityPolicy.BEST_EFFORT)
        subscriptions.append(node.create_subscription(typ,topic,lambda msg,t=topic:callback(t,msg),qos))
    initial_pub=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',1)
    clock_pub=node.create_publisher(Clock,'/clock',10)
    bootstrap=Clock();bootstrap.clock.sec,bootstrap.clock.nanosec=divmod(metadata['starting_time']['nanoseconds_since_epoch'],10**9)
    last_bootstrap=[0.0]
    def spin(seconds,bootstrap_clock=False):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            if bootstrap_clock and time.monotonic()-last_bootstrap[0]>.05:
                clock_pub.publish(bootstrap);last_bootstrap[0]=time.monotonic()
            rclpy.spin_once(node,timeout_sec=.01)
    def start(name,command):
        handle=open(out/(name+'.log'),'w');logs.append(handle)
        process=subprocess.Popen(command,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append((name,process))
        (out/'runtime_processes.json').write_text(json.dumps([{'name':n,'pid':p.pid,'command':p.args} for n,p in processes],indent=2))
        return process
    try:
        start('robot',['/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher','--ros-args','--params-file',str(out/'robot.yaml')])
        start('merger',[str(WORKSPACE/'install/zx_lidar_filter/lib/zx_lidar_filter/dual_scan_merger_node'),'--ros-args','--params-file',str(WORKSPACE/'src/algorithm/zx_lidar_filter/config/dual_scan_merger.yaml'),'-p','use_sim_time:=true'])
        if args.ekf:
            start('ekf',['/opt/ros/jazzy/lib/robot_localization/ekf_node','--ros-args','--params-file',str(out/'ekf.yaml'),'-p','use_sim_time:=true','-r','odometry/filtered:=/odom'])
            start('base_pose',[str(ROOT/'install/hector_mapping/lib/hector_mapping/hector_base_pose_from_tf'),'--ros-args','-p','use_sim_time:=true','-p','publish_rate_hz:=10.0'])
        start('hector',[str(ROOT/'install/hector_mapping/lib/hector_mapping/hector_mapping_node'),'--ros-args','--params-file',str(out/'hector.yaml')])
        deadline=time.monotonic()+15
        while (not map_received or initial_pub.get_subscription_count()<1) and time.monotonic()<deadline:spin(.1,True)
        if not map_received:raise RuntimeError('Hector did not publish its prior map; inspect logs')
        if initial_pub.get_subscription_count()<1:raise RuntimeError('No Hector initialpose subscriber')
        graph={'nodes':node.get_node_names(),'hector_subscriptions':[(x,[*t]) for x,t in node.get_subscriber_names_and_types_by_node('hector_mapping','/')]}
        if not args.global_localization:
            seed=json.loads((args.map.parent/'replay_initial_pose.json').read_text())
            seed_pose=[seed['initial_pose_x']+args.seed_offset[0],seed['initial_pose_y']+args.seed_offset[1],seed['initial_pose_yaw']+args.seed_offset[2]]
            (out/'seed.json').write_text(json.dumps({'pose':seed_pose,'offset':args.seed_offset,'reference':seed},indent=2))
            msg=PoseWithCovarianceStamped();msg.header.frame_id='map';msg.header.stamp=bootstrap.clock
            msg.pose.pose.position.x,msg.pose.pose.position.y=seed_pose[:2]
            msg.pose.pose.orientation.z=math.sin(seed_pose[2]/2);msg.pose.pose.orientation.w=math.cos(seed_pose[2]/2)
            initial_pub.publish(msg);spin(.3,True)
        if args.rviz:
            shutil.copyfile(args.rviz_config,out/'hector.rviz')
            start('rviz',['rviz2','-d',str(args.rviz_config),'--ros-args','-p','use_sim_time:=true'])
            spin(2,True)
        command=['ros2','bag','play',str(args.bag),'--clock','200','--topics','/scan','/scan_back','/imu','--disable-keyboard-controls','--start-paused']
        if args.ekf:command+=['--topics','/scan','/scan_back','/imu','/wheel_odom']
        if args.duration:command+=['--playback-duration',str(args.duration)]
        player=start('player',command)
        resume=node.create_client(Resume,'/rosbag2_player/resume')
        discovery_end=time.monotonic()+15
        while not resume.service_is_ready() and time.monotonic()<discovery_end:spin(.1,True)
        if not resume.service_is_ready():raise RuntimeError('Bag player resume service unavailable')
        spin(3,True)  # Discover the player's publishers before releasing the first sensor samples.
        future=resume.call_async(Resume.Request())
        resume_end=time.monotonic()+5
        while not future.done() and time.monotonic()<resume_end:spin(.05)
        if not future.done() or future.result() is None:raise RuntimeError('Bag resume timed out')
        node.destroy_client(resume)
        deadline=time.monotonic()+(args.duration or metadata['duration']['nanoseconds']/1e9)+40
        while player.poll() is None and time.monotonic()<deadline:
            spin(.1)
            for name,process in processes:
                if name not in ['player','rviz'] and process.poll() is not None:raise RuntimeError(f'{name} exited {process.returncode}')
        if player.poll() is None:raise RuntimeError('Bag playback timed out')
        if player.returncode:raise RuntimeError(f'Bag playback failed {player.returncode}')
        for name,process in processes:
            if name=='merger' and process.poll() is None:
                os.killpg(process.pid,signal.SIGINT);process.wait(timeout=5)
        spin(1)
        expected_map=np.flipud(np.asarray(Image.open(pgm),dtype=float));prob=expected_map/255 if map_config['negate'] else 1-expected_map/255
        expected_map=np.where(prob>map_config['occupied_thresh'],100,np.where(prob<map_config['free_thresh'],0,-1)).astype(np.int8)
        actual_map=np.array(map_received[0].data,dtype=np.int8).reshape(expected_map.shape)
        accepted=[x for x in statuses if x['accepted']]
        stamps=sorted(poses)
        summary={'counts':counts,'poses':len(poses),'statuses':len(statuses),'accepted':len(accepted),'rejected':len(statuses)-len(accepted),
            'score_min':min((x['score'] for x in statuses),default=None),'score_mean':float(np.mean([x['score'] for x in statuses])) if statuses else None,
            'pose_span_seconds':(stamps[-1]-stamps[0])/1e9 if stamps else 0,'max_pose_gap_seconds':max(np.diff(stamps)/1e9) if len(stamps)>1 else None,
            'published_map_versions':len(map_hashes),'published_map_equals_pgm_yaml':bool(np.array_equal(expected_map,actual_map)),
            'source_files_unchanged':all(hashlib.sha256(p.read_bytes()).hexdigest()==original_hash[str(p)] for p in files),
            'graph':graph,'global_localization':args.global_localization,'seed_offset':args.seed_offset,'input_hashes':original_hash,'core_pyramid_invariance':'Checked at every map publication; aborts on mutation',
            'rviz_config_path':str(args.rviz_config),'preview_nodes_held':args.hold,
            'ekf_enabled':args.ekf,'extra_correction_smoothing':False,'transform_tolerance':.3 if args.ekf else 0.0,
            'business_poses':len(business_poses),'ekf_poses':len(ekf_poses),
            'tf_counts':{'/'.join(k):len(v) for k,v in transforms.items()},
            'expected_input_counts':{t['topic_metadata']['name']:t['message_count'] for t in metadata['topics_with_message_count'] if args.ekf or t['topic_metadata']['name']!='/wheel_odom'}}
        (out/'summary.json').write_text(json.dumps(summary,indent=2))
        (out/'status.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in statuses))
        with (out/'trajectory_hector.tum').open('w') as f:
            for stamp,msg in sorted(poses.items()):
                p=msg.pose.position;q=msg.pose.orientation
                f.write(f'{stamp/1e9:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} {q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n')
        for filename,collection in [('trajectory_business.tum',business_poses),('trajectory_ekf.tum',ekf_poses)]:
            if collection:
                with (out/filename).open('w') as f:
                    for stamp,msg in sorted(collection.items()):
                        p=msg.pose.position;q=msg.pose.orientation
                        f.write(f'{stamp/1e9:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} {q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n')
        if args.ekf:
            from validate_ekf_composition import validate_composition
            summary['tf_composition']=validate_composition(poses,transforms,.3)
            if not business_poses or not ekf_poses:raise RuntimeError('Missing business/EKF poses')
            if summary['tf_composition']['maximum_position_error_m']>1e-4:raise RuntimeError('Map correction differs from direct Hector/odom composition')
        if not args.duration:
            summary['all_sensor_messages_received']=all(counts.get(k,0)==v for k,v in summary['expected_input_counts'].items())
        (out/'summary.json').write_text(json.dumps(summary,indent=2))
        writer.close();writer=None
        plot_script=Path('/home/lc/slam_11_13/scripts/plot_localization.py')
        plot_command=['python3',str(plot_script),'--method','hector_ekf' if args.ekf else 'hector','--output',str(out),'--bag',str(args.bag)]
        if args.ekf:plot_command+=['--baseline','/home/lc/slam_11_13/results/hector_localization/trajectory_hector.tum']
        subprocess.run(plot_command,check=True)
        print(json.dumps(summary,indent=2),flush=True)
        if not poses or not summary['published_map_equals_pgm_yaml'] or len(map_hashes)!=1:raise RuntimeError('Localization/map invariance validation failed')
        if not args.duration and not summary['all_sensor_messages_received']:raise RuntimeError('Some input sensor messages were lost; inspect summary')
        if args.hold:
            print(f'Replay complete. Results: {out}. RViz and fixed-map Hector remain open. Ctrl+C stops this session.',flush=True)
            while rclpy.ok():spin(.2)
    finally:
        for name,process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        if writer is not None:writer.close()
        node.destroy_node();rclpy.try_shutdown()
        for handle in logs:handle.close()

if __name__=='__main__':
    try:main()
    except (KeyboardInterrupt, ExternalShutdownException):pass
