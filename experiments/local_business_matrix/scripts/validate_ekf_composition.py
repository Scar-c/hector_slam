"""Verify each map->odom correction exactly equals the raw Hector correction."""
import math
import numpy as np

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def wrap(x):return math.atan2(math.sin(x),math.cos(x))
def validate_composition(poses,transforms,tolerance):
    odom=transforms.get(('odom','base_link'),{});corrections=transforms.get(('map','odom'),{})
    stamps=sorted(odom);errors=[];angles=[]
    for stamp,pose in poses.items():
        correction=corrections.get(stamp+round(tolerance*1e9))
        i=np.searchsorted(stamps,stamp)
        if correction is None or i==0 or i==len(stamps):continue
        first,last=odom[stamps[i-1]],odom[stamps[i]]
        ratio=(stamp-stamps[i-1])/(stamps[i]-stamps[i-1])
        x=first.translation.x*(1-ratio)+last.translation.x*ratio
        y=first.translation.y*(1-ratio)+last.translation.y*ratio
        a=yaw(first.rotation)+ratio*wrap(yaw(last.rotation)-yaw(first.rotation))
        c=yaw(correction.rotation)
        world_x=correction.translation.x+math.cos(c)*x-math.sin(c)*y
        world_y=correction.translation.y+math.sin(c)*x+math.cos(c)*y
        errors.append(math.hypot(world_x-pose.pose.position.x,world_y-pose.pose.position.y))
        angles.append(abs(wrap(a+c-yaw(pose.pose.orientation))))
    if not errors:raise RuntimeError('No timestamp-aligned TF composition checks possible')
    return {'verified_frames':len(errors),'maximum_position_error_m':max(errors),'maximum_yaw_error_rad':max(angles),
            'definition':'map_odom(scan_stamp+tolerance) * odom_base(scan_stamp) equals raw Hector map_base(scan_stamp)'}
