#!/usr/bin/env python3
"""Display and record raw fixed-map Hector poses; no estimation or optimization."""
import hashlib,json
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage,OccupancyGrid
from std_msgs.msg import String

class Visualizer(Node):
    def __init__(self):
        super().__init__('hector_localization_visualizer')
        output=self.declare_parameter('output_directory','').value
        self.output=Path(output) if output else None
        self.files=[];self.count=0;self.status_count=0;self.accepted=0;self.low_scores=0;self.map_hash=None
        self.first_stamp=None;self.last_stamp=None
        self.tum=self.status_file=None
        if self.output:
            self.output.mkdir(parents=True,exist_ok=True)
            self.tum=(self.output/'trajectory_localization.tum').open('x',buffering=1)
            self.status_file=(self.output/'localization_status.jsonl').open('x',buffering=1)
            self.files=[self.tum,self.status_file]
        self.path=PathMessage()
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub=self.create_publisher(PathMessage,'/hector_path',qos)
        self.subs=[self.create_subscription(PoseStamped,'/slam_out_pose',self.pose,1000),
                   self.create_subscription(String,'/localization_status',self.status,1000),
                   self.create_subscription(OccupancyGrid,'/map',self.map,qos)]
        self.create_timer(.2,self.publish,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(2.,self.summary,clock=Clock(clock_type=ClockType.STEADY_TIME))
    def pose(self,m):
        self.path.header=m.header;self.path.poses.append(m);self.count+=1
        t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if self.first_stamp is None:self.first_stamp=t
        self.last_stamp=t
        if self.tum:
            p=m.pose.position;q=m.pose.orientation
            self.tum.write(f'{t:.9f} {p.x:.12f} {p.y:.12f} {p.z:.12f} {q.x:.12f} {q.y:.12f} {q.z:.12f} {q.w:.12f}\n')
    def status(self,m):
        s=json.loads(m.data);self.status_count+=1;self.accepted+=bool(s['accepted']);self.low_scores+=bool(s.get('score_below_threshold',False))
        if self.status_file:self.status_file.write(m.data+'\n')
    def map(self,m):
        h=hashlib.sha256(m.data.tobytes()).hexdigest()
        if self.map_hash is not None and self.map_hash!=h:raise RuntimeError('Fixed map changed')
        self.map_hash=h
    def publish(self):
        if self.path.poses:self.pub.publish(self.path)
    def summary(self):
        if self.output:
            (self.output/'summary.json').write_text(json.dumps({'poses':self.count,'matches':self.status_count,
                'accepted':self.accepted,'low_score_frames':self.low_scores,'rejected':self.status_count-self.accepted,'first_stamp':self.first_stamp,
                'last_stamp':self.last_stamp,'map_data_sha256':self.map_hash,
                'map_changed':False,'backend_optimization':False},indent=2)+'\n')

def main():
    rclpy.init();node=Visualizer()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.summary()
        for f in node.files:f.close()
        node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
