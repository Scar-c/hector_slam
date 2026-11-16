"""Replay one LaserScan topic with an explicitly configured laser mounting TF."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument('bag', description='Absolute rosbag2 directory'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('laser_frame', default_value='laser'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('rate', default_value='1.0'),
    ]
    for name in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
        declarations.append(DeclareLaunchArgument('laser_' + name, default_value='0.0'))
    tf_args = []
    for name in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
        tf_args.extend(['--' + name, LaunchConfiguration('laser_' + name)])
    tf_args.extend(['--frame-id', LaunchConfiguration('base_frame'),
                    '--child-frame-id', LaunchConfiguration('laser_frame')])
    return LaunchDescription(declarations + [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('hector_mapping'), 'launch', 'mapping.launch.py'])),
            launch_arguments={'use_sim_time': 'true',
                              'scan_topic': LaunchConfiguration('scan_topic'),
                              'base_frame': LaunchConfiguration('base_frame'),
                              'odom_frame': LaunchConfiguration('base_frame')}.items()),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='replay_laser_mount', arguments=tf_args,
             parameters=[{'use_sim_time': True}], output='screen'),
        ExecuteProcess(cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag'),
                            '--topics', LaunchConfiguration('scan_topic'),
                            '--clock', '--delay', '3', '--disable-keyboard-controls',
                            '--rate', LaunchConfiguration('rate')], output='screen'),
    ])
