"""Same wheel+IMU EKF and TF chain as business AMCL; no correction smoothing."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    sim=LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('global_relocalization',default_value='true'),
        Node(package='robot_localization',executable='ekf_node',name='ekf_filter_node',output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('hector_mapping'),'config','ekf.yaml']),
                         {'use_sim_time':ParameterValue(sim,value_type=bool)}],
             remappings=[('odometry/filtered','odom')]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('hector_mapping'),'launch','localization.launch.py'])),
            launch_arguments={'map':LaunchConfiguration('map'),'use_sim_time':sim,
                'global_relocalization':LaunchConfiguration('global_relocalization'),
                'odom_frame':'odom','use_odom_pose_start_estimate':'true','transform_tolerance':'0.3'}.items()),
        Node(package='hector_mapping',executable='hector_base_pose_from_tf',name='localization_base_pose_from_tf',output='screen',
             parameters=[{'use_sim_time':ParameterValue(sim,value_type=bool),'publish_rate_hz':10.0}]),
    ])
