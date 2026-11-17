"""Fixed-map b0 localization using the repaired mapping frontend; no graph backend."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share=Path(get_package_share_directory('hector_cloud_frontend'))
    sim=ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('map',description='Fixed PGM/YAML prior'),
        DeclareLaunchArgument('use_sim_time',default_value='true'),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('output_directory',default_value=''),
        DeclareLaunchArgument('initial_pose_file',description='Required result JSON from a complete global search; no manual start pose'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{
            'use_sim_time':sim,'robot_description':(share/'config/backpack_2d.urdf').read_text()}]),
        Node(package='hector_cloud_frontend',executable='hector_carto_deskew',parameters=[{
            'use_sim_time':sim,'single_scan':True,'compensate_translation':True,
            'historical_pose_topic':'/poseupdate','historical_pose_frame':'map','publish_scan':True,
            'min_range':0.5,'max_range':30.0}]),
        Node(package='hector_cloud_frontend',executable='hector_cloud_mapping_node',name='hector_localization',output='screen',parameters=[{
            'use_sim_time':sim,'localization_only':True,'prior_map_yaml':LaunchConfiguration('map'),
            'use_imu':True,'map_frame':'map','base_frame':'base_link','odom_frame':'base_link',
            'global_relocalization':False,'use_odom_pose_start_estimate':False,'enforce_localization_score':False,
            'map_pub_period':1.0,'map_resolution':0.05,'map_multi_res_levels':3,
            'scan_subscriber_queue_size':1000,'cloud_voxel_size':0.025,
            'laser_min_dist':0.5,'laser_max_dist':30.0,'laser_z_min_value':-0.8,'laser_z_max_value':2.0}]),
        Node(package='hector_cloud_frontend',executable='seed_pose.py',parameters=[{
            'use_sim_time':sim,'pose_file':LaunchConfiguration('initial_pose_file')}]),
        Node(package='hector_cloud_frontend',executable='localization_visualizer.py',parameters=[{
            'use_sim_time':sim,'output_directory':LaunchConfiguration('output_directory')}]),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'config/backpack_localization.rviz')],
             parameters=[{'use_sim_time':sim}],condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
