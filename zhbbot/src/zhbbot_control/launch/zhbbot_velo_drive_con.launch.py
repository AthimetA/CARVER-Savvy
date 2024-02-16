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
    package_name = 'zhbbot_description'
    launch_file_subfolder = 'launch'
    # File names
    gazebo_launch_file_name = 'zhbbot_gazebo_sim.launch.py'

    # file paths
    gazebo_launch_subpath = launch_file_subfolder + '/' + gazebo_launch_file_name

    # ***** GAZEBO ***** #
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare(package_name), gazebo_launch_subpath])),
                )

    # Velocity Controller
    velocity_controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_cont", "-c", "/controller_manager"],
    )

    # Forward Kinematics Controller
    diff_controller = Node(
        package='zhbbot_control',
        executable='diff_controller_fk.py',
        name='diff_controller_fk',
    )

    # Robot Localization
    pkg_name = 'zhbbot_control'
    file_subfolder = 'config'
    file_name = 'zhbbot_ekf.yaml'

    file_path_full = file_subfolder + '/' + file_name

    file_path = get_package_share_directory(pkg_name) + '/' + file_path_full
    
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[file_path],
        # remappings=[("odometry/filtered", "/odom")],
    ) 

    # ***** SLAM TOOLBOX ***** #
    # SLAM map
    slam_map_path = os.path.join(
        get_package_share_directory(package_name),
        'maps',
        'pal_office_map.yaml')
    
    slam_map_file = LaunchConfiguration('map', default=slam_map_path)

    # ***** NAVIGATION ***** #
    # Navigation parameters
    nav2_param_path = os.path.join(
        get_package_share_directory('zhbbot_control'),
        'config',
        'navigation_param.yaml')
    
    nav2_param_file = LaunchConfiguration('params', default=nav2_param_path)

    # launch file directory
    nav2_launch_file_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch')


    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        
        gazebo,
        velocity_controllers,
        diff_controller,
        robot_localization_node,

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