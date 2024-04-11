# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,Command, LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
import xacro
import yaml
    
# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():

    # Use sim time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Package name
    package_name = 'pmzbbot_description'

        # ***Rviz*** #
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'navigation.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])

    # ***** SLAM TOOLBOX ***** #
    # SLAM map
    slam_map_path = os.path.join(
        get_package_share_directory(package_name),
        'maps',
        'office_pmzbbot.yaml')
    
    slam_map_file = LaunchConfiguration('map', default=slam_map_path)

    # ***** NAVIGATION ***** #
    # Navigation parameters
    nav2_param_path = os.path.join(
        get_package_share_directory('pmzbbot_control'),
        'config',
        'navigation_param.yaml')
    
    nav2_param_file = LaunchConfiguration('params', default=nav2_param_path)

    # launch file directory
    nav2_launch_file_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch')

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([

        rviz_node,
    # SLAM Toolbox and Navigation
        DeclareLaunchArgument(
            'map',
            default_value=slam_map_file,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params',
            default_value=nav2_param_file,
            description='Full path to param file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_file_path, '/bringup_launch.py']),
            launch_arguments={
                'map': slam_map_file,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_file}.items(),
        )

    ])