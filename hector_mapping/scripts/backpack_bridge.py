#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Convert first valid echo to LaserScan and retain a path for the RViz demo."""
import math
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan, MultiEchoLaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class BackpackBridge(Node):
    def __init__(self):
        super().__init__('hector_backpack_bridge')
        self.scan_pub=self.create_publisher(LaserScan,'scan',qos_profile_sensor_data)
        self.path_pub=self.create_publisher(Path,'hector_path',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.path=Path()
        self.create_subscription(MultiEchoLaserScan,'horizontal_laser_2d',self.scan,qos_profile_sensor_data)
        self.create_subscription(PoseStamped,'slam_out_pose',self.pose,100)
        self.create_timer(0.5,self.publish_path,clock=Clock(clock_type=ClockType.STEADY_TIME))
    def scan(self,msg):
        out=LaserScan(); out.header=msg.header
        for key in ('angle_min','angle_max','angle_increment','time_increment','scan_time','range_min','range_max'):
            setattr(out,key,getattr(msg,key))
        out.ranges=[next((v for v in ray.echoes if math.isfinite(v) and msg.range_min<v<msg.range_max), math.inf) for ray in msg.ranges]
        self.scan_pub.publish(out)
    def pose(self,msg):
        if self.path.poses and (msg.header.stamp.sec,msg.header.stamp.nanosec)<=(self.path.header.stamp.sec,self.path.header.stamp.nanosec):
            self.path.poses.clear()
        self.path.header=msg.header
        self.path.poses.append(msg)
        if len(self.path.poses)>20000: self.path.poses.pop(0)
    def publish_path(self):
        if self.path.poses: self.path_pub.publish(self.path)

def main():
    rclpy.init(); node=BackpackBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__=='__main__': main()
