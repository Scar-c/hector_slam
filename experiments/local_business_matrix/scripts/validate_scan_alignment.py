"""Reconstruct aligned scans from recorded raw scans and EKF TF; no ROS nodes."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec

def validate_output_bag(folder):
    folder=Path(folder)
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(folder/'output_bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    wanted={'/scan','/scan_back','/scan_amcl','/tf','/tf_static','/scan_alignment_status'}
    types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in wanted}
    raw={'/scan':{},'/scan_back':{}};merged={};statuses={};static={};odometry={}
    while reader.has_next():
        topic,data,_=reader.read_next()
        if topic not in wanted:continue
        m=deserialize_message(data,types[topic])
        if topic in raw:raw[topic][stamp(m)]=m
        if topic=='/scan_amcl':merged[stamp(m)]=m
        if topic=='/scan_alignment_status':
            s=json.loads(m.data);statuses[s['reference_stamp_ns']]=s
        if topic=='/tf_static':
            for tr in m.transforms:
                p=tr.transform.translation;q=tr.transform.rotation
                static[tr.child_frame_id]=(tr.header.frame_id,Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix(),np.array([p.x,p.y,p.z]))
        if topic=='/tf':
            for tr in m.transforms:
                if (tr.header.frame_id,tr.child_frame_id)==('odom','base_link'):
                    p=tr.transform.translation;q=tr.transform.rotation
                    yaw=np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                    odometry[tr.header.stamp.sec*10**9+tr.header.stamp.nanosec]=[p.x,p.y,yaw]
    times=np.array(sorted(odometry),dtype=np.int64);poses=np.array([odometry[t] for t in times]);origin=int(times[0]) if len(times) else 0;relative=(times-origin)/1e9
    if len(times):poses[:,2]=np.unwrap(poses[:,2])
    def at(t):
        values=np.asarray(t,dtype=np.int64)
        if not len(times) or np.any(values<times[0]) or np.any(values>times[-1]):raise RuntimeError('Missing recorded EKF bracket for scan processing')
        return np.stack([np.interp((values-origin)/1e9,relative,poses[:,j]) for j in [0,1,2]],axis=-1)
    def base_from_sensor(frame):
        if frame=='base_link':return np.eye(3),np.zeros(3)
        parent,R,p=static[frame];Rp,tp=base_from_sensor(parent);return Rp@R,Rp@p+tp
    agreements=[];ages=[];skews=[]
    for t,msg in sorted(merged.items()):
        s=statuses[t];time_align=s.get('time_alignment',True);deskew=s.get('beam_deskew',False);points=[];source_stamps=[]
        for topic,key in [('/scan','front_stamp_ns'),('/scan_back','back_stamp_ns')]:
            source_stamp=s[key]
            if not source_stamp:continue
            scan=raw[topic][source_stamp];source_stamps.append(source_stamp);ages.append((t-source_stamp)/1e6)
            rr=np.array(scan.ranges,dtype=float);indices=np.arange(len(rr));a=scan.angle_min+indices*scan.angle_increment
            valid=np.isfinite(rr)&(rr>=max(.1,scan.range_min))&(rr<=min(40,scan.range_max))
            R,p=base_from_sensor(scan.header.frame_id)
            xyz=np.stack([rr[valid]*np.cos(a[valid]),rr[valid]*np.sin(a[valid]),np.zeros(valid.sum())],axis=-1)@R.T+p
            if time_align or deskew:
                ref=at(t if time_align else source_stamp)
                offset=s.get('beam_time_offset_sec',0.0);direction=s.get('beam_time_direction',1.0)
                beam_stamps=source_stamp+np.rint((offset+direction*indices[valid]*scan.time_increment)*1e9).astype(np.int64) if deskew else source_stamp
                src=at(beam_stamps);d=src[...,:2]-ref[:2];c,sn=np.cos(ref[2]),np.sin(ref[2]);translation=np.stack([c*d[...,0]+sn*d[...,1],-sn*d[...,0]+c*d[...,1]],axis=-1)
                angle=src[...,2]-ref[2];c,sn=np.cos(angle),np.sin(angle)
                points.append(np.stack([c*xyz[:,0]-sn*xyz[:,1],sn*xyz[:,0]+c*xyz[:,1]],axis=-1)+translation)
            else:points.append(xyz[:,:2])
        if len(source_stamps)==2:skews.append(abs(source_stamps[0]-source_stamps[1])/1e6)
        p=np.concatenate(points);rr=np.linalg.norm(p,axis=1);a=np.arctan2(p[:,1],p[:,0]);amin=-3.1415926;inc=.00872665;n=len(msg.ranges);amax=amin+(n-1)*inc
        valid=(rr>=.1)&(rr<=40)&(a>=amin)&(a<=amax);bins=np.floor((a[valid]-amin)/inc+.5).astype(int)
        pred=np.full(n,np.inf,dtype=np.float32);np.minimum.at(pred,bins,rr[valid].astype(np.float32));actual=np.array(msg.ranges,dtype=np.float32)
        finite=np.isfinite(pred)&np.isfinite(actual);same=(~np.isfinite(pred)&~np.isfinite(actual))|(finite & np.isclose(pred,actual,atol=3e-5,rtol=1e-5))
        agreements.append(float(np.mean(same)))
    if not agreements:raise RuntimeError('No aligned output scans to validate')
    result={'verified_frames':len(agreements),'mean_beam_agreement_fraction':float(np.mean(agreements)),'worst_frame_beam_agreement_fraction':min(agreements),'source_header_age_ms_median':float(np.median(ages)),'front_back_header_skew_ms_p95':float(np.quantile(skews,.95)) if skews else None,'method':'Reconstructed source static extrinsics and EKF odom(source_time) -> odom(reference_time), then identical range/angle rebinning','pose_smoothing':False,'per_beam_deskew':any(v.get('beam_deskew',False) for v in statuses.values()),'frame_time_alignment':any(v.get('time_alignment',True) for v in statuses.values())}
    (folder/'scan_alignment_validation.json').write_text(json.dumps(result,indent=2))
    if result['mean_beam_agreement_fraction']<.999:raise RuntimeError('Aligned scan geometry reconstruction failed: '+str(result))
    return result

if __name__=='__main__':
    import sys
    print(json.dumps(validate_output_bag(sys.argv[1]),indent=2))
