"""
SLAM Simulation Launch File

Launches custom SLAM toolbox with RViz visualization for
real-time mapping and localization.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('custom_nav')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_dir, 'config', 'slam_params.yaml'),
        description='Full path to SLAM parameters file'
    )
    
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mapping',
        description='SLAM mode: mapping or localization'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_dir, 'rviz', 'slam.rviz'),
        description='Full path to RViz config file'
    )
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    mode = LaunchConfiguration('mode')
    rviz_config = LaunchConfiguration('rviz_config')
    
    # SLAM Toolbox Node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {
                'use_sim_time': use_sim_time,
                'mode': mode
            }
        ],
        remappings=[
            ('/scan', '/scan'),
        ]
    )
    
    # RViz2 Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Map Saver (optional - for saving maps)
    # Uncomment to auto-save maps periodically
    # map_saver_node = Node(
    #     package='nav2_map_server',
    #     executable='map_saver_server',
    #     name='map_saver',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': use_sim_time},
    #         {'save_map_timeout': 5000},
    #         {'free_thresh_default': 0.25},
    #         {'occupied_thresh_default': 0.65}
    #     ]
    # )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        slam_params_file_arg,
        mode_arg,
        rviz_config_arg,
        
        # Nodes
        slam_toolbox_node,
        rviz_node,
        # map_saver_node  # Uncomment if needed
    ])
