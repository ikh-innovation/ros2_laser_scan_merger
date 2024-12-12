#
#   created by: Michael Jonathan (mich1342)
#   github.com/mich1342
#   24/2/2022
#
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration

def generate_launch_description():
    # Launch Configuration variables
    simulation = LaunchConfiguration('use_sim_time')
    
    # Get package directories
    ros2_laser_scan_merger_pkg = get_package_share_directory(
        'ros2_laser_scan_merger'
    )
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Whether to use simulation'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(ros2_laser_scan_merger_pkg, 'config', 'params.yaml'),
        description='Full path to the ROS2 Laser Scan Merger parameters file to load'
    )
    
    declare_pcl2_to_laserscan_params_file_cmd = DeclareLaunchArgument(
        'pcl2_to_laserscan_params_file',
        default_value=os.path.join(ros2_laser_scan_merger_pkg, 'config', 'pcl2_to_laserscan.yaml'),
        description='Full path to the Pointcloud to Laserscan parameters file to load'
    )
    
    declare_prefix_cmd = DeclareLaunchArgument(
        'prefix',
        default_value='mara_', 
        description='Robot prefix'
    )
    
    declare_laserscan_frame_cmd = SetLaunchConfiguration(
        'laserscan_frame',
        value=[LaunchConfiguration('prefix'),TextSubstitution(text='top_deck')]
    )
    
    declare_merged_pointcloud_topic_cmd = DeclareLaunchArgument(
        'merged_pointcloud_topic',
        default_value='/mid70/merged_points',
        description='Merged Pointcloud Topic'
    )
    
    declare_merged_scan_topic_cmd = DeclareLaunchArgument(
        'merged_scan_topic',
        default_value='/mid70/merged_scan',
        description='Merged Scan Topic'
    )
    
    declare_front_pointcloud_topic_cmd = DeclareLaunchArgument(
        'front_pointcloud_topic',
        default_value='/mid70/front_scan_PointCloud2',
        description='Front Scan Pointcloud'
    )
    
    declare_front_scan_cmd = DeclareLaunchArgument(
        'front_scan_topic',
        default_value='/mid70/front_scan/scan',
        description='Front Scan Topic'
    )
    
    declare_rear_pointcloud_topic_cmd = DeclareLaunchArgument(
        'rear_pointcloud_topic',
        default_value='/mid70/rear_scan_PointCloud2',
        description='Rear Scan Pointcloud'
    )
    
    declare_rear_scan_topic_cmd = DeclareLaunchArgument(
        'rear_scan_topic',
        default_value='/mid70/rear_scan/scan',
        description='Rear Scan Topic'
    )
    
    
    # Parameters Configuration files
    
    # Front Pointcloud to Lasercan Node
    front_pointcloud_to_laserscan_node = Node(
        name='front_pointcloud_to_laserscan',
        package='pointcloud_to_laserscan', 
        executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', LaunchConfiguration('front_pointcloud_topic')),
                    ('scan', LaunchConfiguration('front_scan_topic'))],
        parameters=[LaunchConfiguration('pcl2_to_laserscan_params_file'), {'use_sim_time': simulation}],
    )
    
    # Rear Pointcloud to Lasercan Node
    rear_pointcloud_to_laserscan_node = Node(
        name='rear_pointcloud_to_laserscan',
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', LaunchConfiguration('rear_pointcloud_topic')),
                    ('scan', LaunchConfiguration('rear_scan_topic'))],
        parameters=[LaunchConfiguration('pcl2_to_laserscan_params_file'), {'use_sim_time': simulation,}],
    )
    
    # ROS2 Laser Scan Merger Node
    ros2_laser_scan_merger_node = Node(
        name='ros2_laser_scan_merger',
        package='ros2_laser_scan_merger',
        executable='ros2_laser_scan_merger',
        parameters=[LaunchConfiguration('params_file'), 
                {'use_sim_time': simulation, 
                'pointCloudTopic': LaunchConfiguration('merged_pointcloud_topic'),
                'scanTopic1': LaunchConfiguration('front_scan_topic'),
                'scanTopic2': LaunchConfiguration('rear_scan_topic'),
                'pointCloudFrameId': LaunchConfiguration('laserscan_frame'),
                'target_frame': LaunchConfiguration('laserscan_frame'),
                }],
        output='screen',
        respawn=True,
        respawn_delay=2,
    )

    # Pointcloud to Lasercan Node
    merged_pointcloud_to_laserscan_node = Node(
        name='merged_pointcloud_to_laserscan',
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', LaunchConfiguration('merged_pointcloud_topic')),
                    ('scan', LaunchConfiguration('merged_scan_topic'))],
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': simulation}]
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_pcl2_to_laserscan_params_file_cmd)
    ld.add_action(declare_prefix_cmd)
    ld.add_action(declare_laserscan_frame_cmd)
    ld.add_action(declare_merged_pointcloud_topic_cmd)
    ld.add_action(declare_merged_scan_topic_cmd)
    ld.add_action(declare_front_pointcloud_topic_cmd)
    ld.add_action(declare_front_scan_cmd)
    ld.add_action(declare_rear_pointcloud_topic_cmd)
    ld.add_action(declare_rear_scan_topic_cmd)
    
    
    # Add the actions to launch all the nodes
    ld.add_action(front_pointcloud_to_laserscan_node)
    ld.add_action(rear_pointcloud_to_laserscan_node)
    ld.add_action(ros2_laser_scan_merger_node)
    ld.add_action(merged_pointcloud_to_laserscan_node)
    
    return ld