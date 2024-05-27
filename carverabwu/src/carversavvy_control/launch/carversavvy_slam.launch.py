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

    # Joint State Publisher Node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    package_name = 'zhbbot_description'

    joy_params = os.path.join(get_package_share_directory(package_name),'config','carversavvy_joystick.yaml')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time': False}],
         )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params, {'use_sim_time': False}],
            remappings=[('/cmd_vel','/cmd_vel_joy')]
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

    # SLAM Toolbox and Navigation
    default_params_file = os.path.join(get_package_share_directory("carversavvy_control"),
                                       'config', 'mapper_params_online_async.yaml')

    start_sync_slam_toolbox_node = Node(
        parameters=[
          default_params_file,
          {'use_sim_time': False}
        ],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='sync_slam_toolbox_node',
        output='screen')

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        

        # Robot State Publisher Node
        node_robot_state_publisher,
        
        # Joint State Publisher Node
        joint_state_publisher_node,
        # joint_state_broadcaster_spawner,

        # velo_drive_controllers,

        carversavvy_forward_kinematic,

        ekf_filter_node_odom,

        rplidar_node,

        start_sync_slam_toolbox_node,

        # Rviz Node
        rviz_node,

        joy_node,
        teleop_node,

        # Twist Mux
        twist_mux_node

    ])