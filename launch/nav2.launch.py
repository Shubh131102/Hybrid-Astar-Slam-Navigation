"""
Nav2 Navigation Stack Launch File

Launches complete Nav2 stack with custom parameters for
Hybrid A* planning and MPC control.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
        description='Full path to Nav2 parameters file'
    )
    
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dir, 'maps', 'map.yaml'),
        description='Full path to map yaml file'
    )
    
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start Nav2 stack'
    )
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    
    # Nav2 bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'map': map_file,
            'autostart': autostart
        }.items()
    )
    
    # Lifecycle manager for Nav2 nodes
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'
            ]}
        ]
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        params_file_arg,
        map_file_arg,
        autostart_arg,
        
        # Nodes
        nav2_bringup_launch,
        lifecycle_manager
    ])
