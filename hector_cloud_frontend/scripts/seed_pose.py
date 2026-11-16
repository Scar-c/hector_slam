#!/usr/bin/env python3
"""Publish a previously computed shared seed; does not run a new initializer."""
import json
import math
from pathlib import Path
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped


def main():
    rclpy.init()
    node=rclpy.create_node('hector_shared_seed')
    file=node.declare_parameter('pose_file','').value
    data=json.loads(Path(file).read_text())
    if 'initial_pose_x' in data:
        pose=[data['initial_pose_x'],data['initial_pose_y'],data['initial_pose_yaw']]
    else:
        pose=data['pose']
    if len(pose)!=3 or not all(math.isfinite(float(x)) for x in pose):
        raise ValueError('Expected finite x/y/yaw shared seed')
    pub=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',1)
    sent=False
    def publish():
        nonlocal sent
        if sent or pub.get_subscription_count()<1:return
        if node.get_parameter('use_sim_time').value and node.get_clock().now().nanoseconds==0:return
        msg=PoseWithCovarianceStamped();msg.header.frame_id='map';msg.header.stamp=node.get_clock().now().to_msg()
        msg.pose.pose.position.x=float(pose[0]);msg.pose.pose.position.y=float(pose[1])
        msg.pose.pose.orientation.z=math.sin(float(pose[2])/2);msg.pose.pose.orientation.w=math.cos(float(pose[2])/2)
        msg.pose.covariance[0]=msg.pose.covariance[7]=.25;msg.pose.covariance[35]=.068538919
        pub.publish(msg);sent=True;node.get_logger().info('Shared initialization pose published once')
    node.create_timer(.1,publish)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
