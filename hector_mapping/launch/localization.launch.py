"""Fixed-prior Hector localization. Supply /initialpose or enable global search."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute PGM/YAML prior map YAML path'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('odom_frame', default_value='base_link'),
        DeclareLaunchArgument('use_odom_pose_start_estimate', default_value='false'),
        DeclareLaunchArgument('transform_tolerance', default_value='0.0'),
        DeclareLaunchArgument('global_relocalization', default_value='false'),
        Node(package='hector_mapping', executable='hector_mapping_node', name='hector_mapping', output='screen',
             parameters=[{
                 'localization_only': True,
                 'prior_map_yaml': LaunchConfiguration('map'),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'global_relocalization': ParameterValue(LaunchConfiguration('global_relocalization'), value_type=bool),
                 'scan_topic': '/scan_amcl', 'imu_topic': '/imu', 'use_imu': True,
                 'map_frame': 'map', 'base_frame': 'base_link', 'odom_frame': LaunchConfiguration('odom_frame'),
                 'use_odom_pose_start_estimate': ParameterValue(LaunchConfiguration('use_odom_pose_start_estimate'), value_type=bool),
                 'transform_tolerance': ParameterValue(LaunchConfiguration('transform_tolerance'), value_type=float),
                 'use_tf_pose_start_estimate': False, 'map_multi_res_levels': 3,
                 'pub_odometry': True, 'laser_max_dist': 30.0,
                 'relocalization_xy_radius': 1.0, 'relocalization_yaw_radius': 0.35,
                 'localization_min_score': 0.45, 'localization_max_step': 0.8,
             }]),
    ])
