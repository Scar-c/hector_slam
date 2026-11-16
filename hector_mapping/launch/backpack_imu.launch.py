"""Hector laser/IMU fusion demo for Cartographer's public 2D backpack bag."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share=FindPackageShare('hector_mapping')
    return LaunchDescription([
        DeclareLaunchArgument('bag', description='Cartographer 2D backpack rosbag2 directory'),
        DeclareLaunchArgument('rate',default_value='1.0'),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('map_size',default_value='8192'),
        Node(package='hector_mapping',executable='hector_mapping_node',name='hector_mapping',output='screen',
             parameters=[PathJoinSubstitution([share,'config','mapping.yaml']),{
                 'use_sim_time':True,'use_imu':True,'imu_topic':'imu','map_size':ParameterValue(LaunchConfiguration('map_size'),value_type=int),
                 'map_resolution':0.05,'map_multi_res_levels':3,'laser_max_dist':30.0,
                 'scan_subscriber_queue_size':200,'map_pub_period':2.0,'map_update_distance_thresh':0.2,'map_update_angle_thresh':0.08,
             }]),
        Node(package='hector_mapping',executable='backpack_bridge.py',output='screen',
             parameters=[{'use_sim_time':True}]),
        # Extrinsics from Cartographer's backpack_2d.urdf (Apache-2.0).
        Node(package='tf2_ros',executable='static_transform_publisher',name='backpack_laser_mount',
             arguments=['--x','0.1007','--z','0.0558','--frame-id','base_link','--child-frame-id','horizontal_laser_link'],
             parameters=[{'use_sim_time':True}]),
        Node(package='tf2_ros',executable='static_transform_publisher',name='backpack_imu_mount',
             arguments=['--frame-id','base_link','--child-frame-id','imu_link'],
             parameters=[{'use_sim_time':True}]),
        Node(package='rviz2',executable='rviz2',name='hector_imu_rviz',
             arguments=['-d',PathJoinSubstitution([share,'config','backpack.rviz'])],
             parameters=[{'use_sim_time':True}],condition=IfCondition(LaunchConfiguration('rviz')),
             additional_env={'LIBGL_ALWAYS_SOFTWARE':'1'},output='screen'),
        ExecuteProcess(cmd=['ros2','bag','play',LaunchConfiguration('bag'),'--storage','mcap',
                            '--clock','100','--delay','5',
                            '--disable-keyboard-controls','--rate',LaunchConfiguration('rate')],output='screen'),
    ])
