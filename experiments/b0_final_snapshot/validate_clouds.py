import argparse,json
from pathlib import Path
import numpy as np,rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2,Imu
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
p=argparse.ArgumentParser();p.add_argument('run',type=Path);args=p.parse_args();r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(args.run/'output_bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
poses={};clouds={};statuses={};refs={};types={'/slam_out_pose':PoseStamped,'/hector_matching_points':PointCloud2,'/cloud_input_status':String,'/deskew_reference_attitude':Imu}
def ns(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
while r.has_next():
 topic,data,_=r.read_next()
 if topic not in types:continue
 m=deserialize_message(data,types[topic])
 if topic=='/cloud_input_status':s=json.loads(m.data);statuses[s['stamp_ns']]=s;continue
 if topic=='/slam_out_pose':q=m.pose.orientation;poses[ns(m)]=[q.x,q.y,q.z,q.w]
 elif topic=='/deskew_reference_attitude':q=m.orientation;refs[ns(m)]=[q.x,q.y,q.z,q.w]
 else:
  offsets={f.name:f.offset for f in m.fields};xyz=np.empty((m.width*m.height,3))
  for i,name in enumerate(['x','y','z']):xyz[:,i]=np.ndarray((m.width*m.height,),dtype='<f4',buffer=m.data,offset=offsets[name],strides=(m.point_step,))
  clouds[ns(m)]=xyz
assert set(poses)==set(refs)==set(statuses)==set(clouds)
max_z=0
for t,xyz in clouds.items():
 assert len(xyz)==statuses[t]['voxel_points'];assert statuses[t]['reference_stamp_ns']==t
 assert abs(np.linalg.norm(refs[t])-1)<1e-6
 xyz_map=Rotation.from_quat(poses[t]).apply(xyz);max_z=max(max_z,float(np.max(abs(xyz_map[:,2]))))
assert max_z<1e-5
print(json.dumps({'paired_poses_references_statuses_clouds':len(poses),'all_stamps_identical':True,'all_point_counts_match':True,'reference_quaternions_unit':True,'matching_points_max_abs_map_z_m':max_z},indent=2))
