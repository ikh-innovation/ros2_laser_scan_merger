import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution

def generate_launch_description():
    pkg = get_package_share_directory('ros2_laser_scan_merger')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='True'
    )

    declare_laserscan_merger_params_file = DeclareLaunchArgument(
        'laserscan_merger_params_file',
        default_value=os.path.join(pkg, 'config', 'params.yaml')
    )

    declare_pcl2_to_laserscan_params_file = DeclareLaunchArgument(
        'pcl2_to_laserscan_params_file',
        default_value=os.path.join(pkg, 'config', 'pcl2_to_laserscan.yaml')
    )

    declare_prefix = DeclareLaunchArgument('prefix', default_value='mara_')

    declare_merged_pointcloud_topic = DeclareLaunchArgument(
        'merged_pointcloud_topic', default_value='/mid70/merged_points'
    )
    declare_merged_scan_topic = DeclareLaunchArgument(
        'merged_scan_topic', default_value='/mid70/merged_scan'
    )

    declare_front_pointcloud_topic = DeclareLaunchArgument(
        'front_pointcloud_topic', default_value='/mid70/front_scan/points'
    )
    declare_front_scan_topic = DeclareLaunchArgument(
        'front_scan_topic', default_value='/mid70/front_scan/scan'
    )

    declare_right_pointcloud_topic = DeclareLaunchArgument(
        'right_pointcloud_topic', default_value='/mid70/right_scan/points'
    )
    declare_right_scan_topic = DeclareLaunchArgument(
        'right_scan_topic', default_value='/mid70/right_scan/scan'
    )

    declare_left_pointcloud_topic = DeclareLaunchArgument(
        'left_pointcloud_topic', default_value='/mid70/left_scan/points'
    )
    declare_left_scan_topic = DeclareLaunchArgument(
        'left_scan_topic', default_value='/mid70/left_scan/scan'
    )

    # include your existing launch that actually does the work
    merge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'merge_2_scan_launch.py'])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration('use_sim_time'),
            "laserscan_merger_params_file": LaunchConfiguration('laserscan_merger_params_file'),
            "pcl2_to_laserscan_params_file": LaunchConfiguration('pcl2_to_laserscan_params_file'),
            "prefix": LaunchConfiguration('prefix'),
            "merged_pointcloud_topic": LaunchConfiguration('merged_pointcloud_topic'),
            "merged_scan_topic": LaunchConfiguration('merged_scan_topic'),
            "front_pointcloud_topic": LaunchConfiguration('front_pointcloud_topic'),
            "front_scan_topic": LaunchConfiguration('front_scan_topic'),
            "right_pointcloud_topic": LaunchConfiguration('right_pointcloud_topic'),
            "right_scan_topic": LaunchConfiguration('right_scan_topic'),
            "left_pointcloud_topic": LaunchConfiguration('left_pointcloud_topic'),
            "left_scan_topic": LaunchConfiguration('left_scan_topic'),
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_laserscan_merger_params_file)
    ld.add_action(declare_pcl2_to_laserscan_params_file)
    ld.add_action(declare_prefix)
    ld.add_action(declare_merged_pointcloud_topic)
    ld.add_action(declare_merged_scan_topic)
    ld.add_action(declare_front_pointcloud_topic)
    ld.add_action(declare_front_scan_topic)
    ld.add_action(declare_right_pointcloud_topic)
    ld.add_action(declare_right_scan_topic)
    ld.add_action(declare_left_pointcloud_topic)
    ld.add_action(declare_left_scan_topic)
    ld.add_action(merge_launch)
    return ld
