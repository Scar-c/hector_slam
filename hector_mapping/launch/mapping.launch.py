from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('hector_mapping'), 'config', 'mapping.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('scan_topic', default_value='scan'),
        DeclareLaunchArgument('use_imu', default_value='false'),
        DeclareLaunchArgument('imu_topic', default_value='imu'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('odom_frame', default_value='base_link',
                              description='Use base frame for mapping without odometry'),
        Node(package='hector_mapping', executable='hector_mapping_node',
             name='hector_mapping', output='screen', parameters=[
                 LaunchConfiguration('params_file'), {
                     'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                     'scan_topic': LaunchConfiguration('scan_topic'),
                     'use_imu': ParameterValue(LaunchConfiguration('use_imu'), value_type=bool),
                     'imu_topic': LaunchConfiguration('imu_topic'),
                     'base_frame': LaunchConfiguration('base_frame'),
                     'odom_frame': LaunchConfiguration('odom_frame'),
                 }]),
    ])
