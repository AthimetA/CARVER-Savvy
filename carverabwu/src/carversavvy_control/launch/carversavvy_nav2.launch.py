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
# from nav2_common.launch import RewrittenYaml
    
# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():

    # ========== **GET PACKAGE SHARE DIRECTORY** ========== #
    # carversavvy_sensors_dir = get_package_share_directory('carversavvy_sensors')
    carversavvy_description_dir = get_package_share_directory('carversavvy_description')
    carversavvy_control_dir = get_package_share_directory('carversavvy_control')

    package_name = 'carversavvy_control'

    # Robot Description
    description_file_subpath = 'description/carversavvy.urdf.xacro'
    xacro_file = os.path.join(carversavvy_description_dir, description_file_subpath) # Use xacro to process the file
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    # ROBOT STATE PUBLISHER NODE:
    node_robot_state_publisher = Node(     # Configure the node
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description_raw}, 
                    {'use_sim_time': False}]
    )

    '''
    
    CONTROLLER
    
    '''

    control_package_name = 'carversavvy_control'
    control_package_dir = get_package_share_directory(control_package_name)

    # Forward Kinematics Node
    carversavvy_forward_kinematic = Node(
        package=control_package_name,
        executable='carversavvy_forward_kinematic.py',
        name='carversavvyFKNode',
    )

    # Robot Localization
    ekf_parameters_file_dir = os.path.join(control_package_dir, 'config')
    ekf_filter_node_odom = Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node',
	        output='screen',
            parameters=[
                os.path.join(ekf_parameters_file_dir, 'carversavvy_ekf.yaml'),
            ],
            remappings=[('odometry/filtered', '/carversavvy/odom')]           
           )

    # Rviz Node
    rviz_config_file = os.path.join(carversavvy_control_dir, 'rviz2', 'carversavvy_slam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # RPLidar Node
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB1')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='lidar_link')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Express')

    rplidar_node = Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_port, 
                         'serial_baudrate': serial_baudrate, 
                         'frame_id': frame_id,
                         'inverted': inverted, 
                         'angle_compensate': angle_compensate,
                         'scan_mode': scan_mode}],
            output='screen')

    # Twist Mux
    twist_mux_config = os.path.join(
        get_package_share_directory('carversavvy_control'),
        'config',
        'twist_mux.yaml')
    
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_config],
        remappings=[('/cmd_vel_out','/carversavvy_cmd_vel')],
        )
    
    # ***** SLAM TOOLBOX ***** #
    # SLAM map
    slam_map_path = os.path.join(
        get_package_share_directory(package_name),
        'maps',
        'FIBOFL5.yaml')
    
    slam_map_file = LaunchConfiguration('map', default=slam_map_path)

    # ***** NAVIGATION ***** #
    # Navigation parameters
    nav2_param_path = os.path.join(
        get_package_share_directory('carversavvy_control'),
        'config',
        'navigation_param.yaml')
    
    nav2_param_file = LaunchConfiguration('params', default=nav2_param_path)

    # launch file directory
    nav2_launch_file_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch')
    

    # ***Rviz*** #
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name),
        'rviz2',
        'carversavvy_slam.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', rviz_config_file]
        )

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        
        # Robot State Publisher
        # node_robot_state_publisher,

        # Forward Kinematics Node
        # carversavvy_forward_kinematic,

        # EKF Node
        # ekf_filter_node_odom,

        # RPLidar Node
        rplidar_node,

        # Twist Mux Node
        twist_mux_node,

        # Rviz Node

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
                'use_sim_time': 'false',
                'params_file': nav2_param_file}.items(),
        )

    ])