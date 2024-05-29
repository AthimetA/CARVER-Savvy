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
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    carversavvy_description_package_name = 'carversavvy_description'
    carversavvy_description_dir = get_package_share_directory(carversavvy_description_package_name)
    carversavvy_control_package_name = 'carversavvy_control'
    carversavvy_control_dir = get_package_share_directory(carversavvy_control_package_name)

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
                    {'use_sim_time': use_sim_time}]
    )


    # Forward Kinematics Node
    carversavvy_forward_kinematic = Node(
        package=carversavvy_control_package_name,
        executable='carversavvy_forward_kinematic.py',
        name='carversavvyFKNode',
    )

    # Robot Localization
    ekf_parameters_file_dir = os.path.join(carversavvy_control_dir, 'config')
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
        get_package_share_directory(carversavvy_control_package_name),
        'config',
        'twist_mux.yaml')
    
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_config],
        remappings=[('/cmd_vel_out','/carversavvy_cmd_vel')],
        )

    control_path = get_package_share_directory('carversavvy_control')
    launch_path = os.path.join(control_path, 'launch')
    launch_file = os.path.join(launch_path, 'carversavvy_rviz.launch.py')
    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file)
    )

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([

        node_robot_state_publisher,

        carversavvy_forward_kinematic,

        ekf_filter_node_odom,

        rplidar_node,

        twist_mux_node,

        rviz_node

    ])