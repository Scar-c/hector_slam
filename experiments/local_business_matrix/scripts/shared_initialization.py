"""Compute/cache both initializers once from a common startup scan snapshot."""
import fcntl
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
ROOT=Path(__file__).resolve().parents[1]
WORKSPACE=ROOT.parents[1]
BUSINESS=WORKSPACE/'src/algorithm/map2map_matcher/map2map_matcher'

def shared_initialization(bag,map_yaml,method):
    bag=Path(bag);map_yaml=Path(map_yaml);config=yaml.safe_load(map_yaml.read_text())
    image=map_yaml.parent/config['image'];urdf=WORKSPACE/'src/zx_description/urdf/sw2.urdf'
    business_config=BUSINESS/'config/config.yaml'
    # Freeze versions and raw input, not a previous localization trajectory.
    paths=[map_yaml,image,bag/'metadata.yaml',urdf,business_config,BUSINESS/'src/core.cpp',BUSINESS/'include/map2map_matcher/core.hpp',ROOT/'initializers/shared_initializers.cpp',Path(__file__),ROOT/'src/hector_mapping/include/prior_map.hpp',ROOT/'src/hector_mapping/include/hector_slam_lib/map/OccGridMapUtil.h']
    metadata=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    paths += [bag/name for name in metadata['relative_file_paths']]
    paths += sorted((ROOT/'src/hector_mapping/include/hector_slam_lib').rglob('*.h'))
    hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    key=hashlib.sha256(json.dumps(hashes,sort_keys=True).encode()).hexdigest()[:20]
    directory=ROOT/'initialization_cache'/key;directory.mkdir(parents=True,exist_ok=True)
    with (directory/'cache.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        result_path=directory/(method+'.json')
        if not (directory/'complete.json').exists():
            reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
            types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types()};scans={};imu=None;raw_digest=hashlib.sha256()
            while reader.has_next() and (len(scans)<2 or imu is None):
                topic,data,record_stamp=reader.read_next()
                if topic in ['/scan','/scan_back'] and topic not in scans:
                    scans[topic]=deserialize_message(data,types[topic]);raw_digest.update(topic.encode()+data)
                if topic=='/imu' and imu is None:imu=deserialize_message(data,types[topic])
            if len(scans)!=2:raise RuntimeError('Shared initialization requires first front/back scans')
            joints={}
            for joint in ET.fromstring(urdf.read_text()).findall('joint'):
                if joint.attrib.get('type')!='fixed':continue
                parent=joint.find('parent').attrib['link'];child=joint.find('child').attrib['link'];origin=joint.find('origin')
                xyz=np.fromstring(origin.attrib.get('xyz','0 0 0'),sep=' ') if origin is not None else np.zeros(3)
                rpy=np.fromstring(origin.attrib.get('rpy','0 0 0'),sep=' ') if origin is not None else np.zeros(3)
                joints[child]=(parent,Rotation.from_euler('xyz',rpy).as_matrix(),xyz)
            def transform(frame):
                if frame=='base_link':return np.eye(3),np.zeros(3)
                parent,R,p=joints[frame];Rp,tp=transform(parent);return Rp@R,Rp@p+tp
            clouds=[];stamps={}
            for topic,scan in scans.items():
                rr=np.asarray(scan.ranges,dtype=float);a=scan.angle_min+np.arange(len(rr))*scan.angle_increment
                valid=np.isfinite(rr)&(rr>max(.1,scan.range_min))&(rr<min(30,scan.range_max));R,p=transform(scan.header.frame_id)
                xyz=np.stack([rr[valid]*np.cos(a[valid]),rr[valid]*np.sin(a[valid]),np.zeros(valid.sum())],axis=-1)@R.T+p
                xyz[:,2]=0;clouds.append(xyz);stamps[topic]=scan.header.stamp.sec*10**9+scan.header.stamp.nanosec
            points=np.concatenate(clouds)
            if imu is not None and imu.orientation_covariance[0]>=0:
                q=imu.orientation;rotation=Rotation.from_quat([q.x,q.y,q.z,q.w]);yaw=rotation.as_euler('xyz')[2]
                points=(Rotation.from_euler('z',-yaw)*rotation).apply(points)
            points=points[:,:2];np.savetxt(directory/'input_cloud.xy',points,fmt='%.12f')
            provenance={'input_hashes':hashes,'initial_snapshot':'First front/back raw scans in base_link using original URDF extrinsics and initial IMU tilt; no localized poses or saved seed used','source_scan_stamps_ns':stamps,'reference_time_ns':max(stamps.values()),'source_cdr_sha256':raw_digest.hexdigest(),'cloud_sha256':hashlib.sha256((directory/'input_cloud.xy').read_bytes()).hexdigest(),'source_points':len(points),'covariance_diagonal_xy_yaw':[.25,.25,.068538919],'covariance_note':'Same input covariance; AMCL uses it for particles, Hector uses the shared x/y/yaw seed without a probabilistic covariance prior.'}
            (directory/'provenance.json').write_text(json.dumps(provenance,indent=2))
            executable=ROOT/'initializers/build/shared_initializers'
            if not executable.exists():raise RuntimeError('Build initializers/shared_initializers before launching the matrix')
            with (directory/'initializer.log').open('w') as log:subprocess.run([str(executable),str(map_yaml),str(directory/'input_cloud.xy'),str(directory)],stdout=log,stderr=subprocess.STDOUT,check=True)
            (directory/'complete.json').write_text(json.dumps({'cache_key':key,'methods':['hector_search','business_fft']}))
        result=json.loads(result_path.read_text());result['cache_key']=key;result['provenance']=json.loads((directory/'provenance.json').read_text())
        result['result_sha256']=hashlib.sha256(result_path.read_bytes()).hexdigest()
        return result,directory

if __name__=='__main__':
    import sys
    for method in ['hector_search','business_fft']:
        result,path=shared_initialization(Path(sys.argv[1]),Path(sys.argv[2]),method)
        print(method,json.dumps(result,ensure_ascii=False),path)
