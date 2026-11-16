"""Final b0 horizontal laser + IMU mapping pipeline; no EKF or loop backend."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share=Path(get_package_share_directory('hector_cloud_frontend'))
    sim=ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('map_size',default_value='8192'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{
            'use_sim_time':sim,'robot_description':(share/'config/backpack_2d.urdf').read_text()}]),
        Node(package='hector_cloud_frontend',executable='hector_carto_deskew',parameters=[{
            'use_sim_time':sim,'single_scan':True,'compensate_translation':True,
            'historical_pose_topic':'/poseupdate','publish_scan':True,'min_range':0.5,'max_range':30.0}]),
        Node(package='hector_cloud_frontend',executable='hector_cloud_mapping_node',output='screen',parameters=[{
            'use_sim_time':sim,'localization_only':False,'use_imu':True,
            'map_frame':'map','base_frame':'base_link','odom_frame':'base_link',
            'map_size':ParameterValue(LaunchConfiguration('map_size'),value_type=int),
            'map_resolution':0.05,'map_multi_res_levels':3,'map_update_distance_thresh':0.2,
            'map_update_angle_thresh':0.08,'scan_subscriber_queue_size':1000,'cloud_voxel_size':0.025,
            'laser_min_dist':0.5,'laser_max_dist':30.0,'laser_z_min_value':-0.8,'laser_z_max_value':2.0}]),
    ])
