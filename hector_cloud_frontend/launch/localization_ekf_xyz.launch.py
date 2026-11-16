"""Fixed prior, EKF prediction, per-beam deskew and common reference time."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def nodes(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    sim = value('use_sim_time').lower() == 'true'
    common = {'use_sim_time': sim}
    actions = [
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[value('ekf_config'), common], remappings=[('odometry/filtered', '/odom')]),
        Node(package='hector_mapping', executable='time_aligned_scan_merger_node',
             parameters=[value('merger_config'), common,
                         {'apply_frame_alignment': True, 'apply_beam_deskew': True}]),
        Node(package='hector_mapping', executable='matrix_base_pose_from_tf',
             parameters=[common, {'publish_rate_hz': 10.0}]),
        Node(package='hector_cloud_frontend', executable='hector_reference_attitude', parameters=[common]),
        Node(package='hector_cloud_frontend', executable='hector_cloud_localization_node', output='screen', parameters=[common, {
            'localization_only': True, 'prior_map_yaml': value('map'),
            'use_imu': True, 'map_frame': 'map', 'base_frame': 'base_link', 'odom_frame': 'odom',
            'use_odom_pose_start_estimate': True, 'transform_tolerance': 0.3,
            'use_tf_pose_start_estimate': False, 'global_relocalization': False,
            'map_multi_res_levels': 3, 'laser_min_dist': 0.1, 'laser_max_dist': 30.0,
            'laser_z_min_value': -0.8, 'laser_z_max_value': 2.0,
            'cloud_voxel_size': 0.025, 'scan_subscriber_queue_size': 1000,
            'ray_origin_x': 0.0, 'ray_origin_y': 0.0, 'ray_origin_z': 0.0,
            'pub_odometry': True, 'output_timing': True,
        }]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', value('rviz_config')],
             parameters=[common], condition=IfCondition(LaunchConfiguration('rviz'))),
    ]
    if value('robot_description_file'):
        actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                            parameters=[common, {'robot_description': Path(value('robot_description_file')).read_text()}]))
    if value('initial_pose_file'):
        actions.append(Node(package='hector_cloud_frontend', executable='seed_pose.py',
                            parameters=[common, {'pose_file': value('initial_pose_file')}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Prior map YAML'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('ekf_config', default_value=PathJoinSubstitution([FindPackageShare('hector_mapping'), 'config', 'ekf.yaml'])),
        DeclareLaunchArgument('merger_config', default_value=PathJoinSubstitution([FindPackageShare('hector_mapping'), 'config', 'dual_scan_merger.yaml'])),
        DeclareLaunchArgument('robot_description_file', default_value=''),
        DeclareLaunchArgument('initial_pose_file', default_value='', description='Shared FFT result JSON, or publish /initialpose externally'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value=PathJoinSubstitution([FindPackageShare('hector_cloud_frontend'), 'config', 'fixed_prior.rviz'])),
        OpaqueFunction(function=nodes),
    ])
