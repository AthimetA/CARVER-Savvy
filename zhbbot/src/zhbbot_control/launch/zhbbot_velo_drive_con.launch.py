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

    # Forward Kinematics Node
    zhbbot_forward_kinemetic = Node(
        package='zhbbot_control',
        executable='zhbbot_forward_kinemetic.py',
        name='zhbbot_forward_kinemetic',
    )

    # Inverse Kinematics Node
    zhbbot_inverse_kinemetic = Node(
        package='zhbbot_control',
        executable='zhbbot_inverse_kinemetic.py',
        name='zhbbot_inverse_kinemetic',
    )

    # Robot Localization
    robot_localization_dir = get_package_share_directory('zhbbot_control')
    parameters_file_dir = os.path.join(robot_localization_dir, 'config')
    parameters_file_path = os.path.join(parameters_file_dir, 'zhbbot_map_ekf.yaml')
    os.environ['FILE_PATH'] = str(parameters_file_dir)
    
    ekf_filter_node_odom = Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_odom',
	        output='screen',
            parameters=[parameters_file_path, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', 'odometry/local')]           
           )
    ekf_filter_node_map = Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_map',
	        output='screen',
            parameters=[parameters_file_path, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', 'odometry/global')]
           ) 
    
    # navsat_transform = Node(
    #         package='robot_localization', 
    #         executable='navsat_transform_node', 
    #         name='navsat_transform',
	#         output='screen',
    #         parameters=[parameters_file_path, {'use_sim_time': use_sim_time}],
    #         remappings=[('imu', 'imu/data'),
    #                     ('gps/fix', 'gps/fix'), 
    #                     ('gps/filtered', 'gps/filtered'),
    #                     ('odometry/gps', 'odometry/gps'),
    #                     ('odometry/filtered', 'odometry/global')]           
    #        )     

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
        
        # Gazebo
        gazebo,
        # Controllers
        velocity_controllers,
        # Kinematics
        zhbbot_forward_kinemetic,
        zhbbot_inverse_kinemetic,

        # EKF
        ekf_filter_node_odom,
        ekf_filter_node_map,
        # navsat_transform,

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