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
from nav2_common.launch import RewrittenYaml
    
# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():

    # ========== **GET PACKAGE SHARE DIRECTORY** ========== #
    pmzbbot_sensors_dir = get_package_share_directory('pmzbbot_sensors')
    pmzbbot_description_dir = get_package_share_directory('pmzbbot_description')
    pmzbbot_control_dir = get_package_share_directory('pmzbbot_control')

    # Robot Description
    description_file_subpath = 'description/pmzbbot.urdf.xacro'
    xacro_file = os.path.join(pmzbbot_description_dir, description_file_subpath) # Use xacro to process the file
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

    # Rviz Node
    rviz_config_file = os.path.join(pmzbbot_control_dir, 'rviz2', 'pmzb_rviz2.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # Add Ros2 Bridge with EKF Launch
  
    ros_bridge_with_ekf_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('pmzbbot_sensors'), 'launch', 'pmzb_ekf_sensors.launch.py'])),
            )
    

    # Twist Mux
    twist_mux_config = os.path.join(
        get_package_share_directory('pmzbbot_control'),
        'config',
        'twist_mux.yaml')
    
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_config],
        remappings=[('/cmd_vel_out','/cmd_vel')],
        )


    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        

        # Robot State Publisher Node
        node_robot_state_publisher,
        
        # Joint State Publisher Node
        joint_state_publisher_node,

        # Rviz Node
        rviz_node,

        # Ros2 Bridge with EKF Launch
        ros_bridge_with_ekf_launch,

        # Twist Mux
        twist_mux_node

    ])