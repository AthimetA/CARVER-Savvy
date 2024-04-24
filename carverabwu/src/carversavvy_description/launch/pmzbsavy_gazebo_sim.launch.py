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
    
    package_name = 'carversavvy_description'
    launch_file_subfolder = 'launch'

    # File names
    gazebo_launch_file_name = 'pmzbsavy_gazebo_base.launch.py'
    joy_launch_file_name = 'pmzbsavy_joystick.launch.py'
    
    # file paths
    gazebo_launch_subpath = launch_file_subfolder + '/' + gazebo_launch_file_name
    joy_launch_subpath = launch_file_subfolder + '/' + joy_launch_file_name

    # ***** GAZEBO ***** #
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare(package_name), gazebo_launch_subpath])),
                )
    
    # ***** JOYSTICK ***** #
    joy = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare(package_name), joy_launch_subpath])),
                )

    # Twist Mux
    twist_mux_config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'twist_mux.yaml')
    
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_config, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        
        # Gazebo
        gazebo,
        # Joystick
        joy,
        # Twist Mux
        twist_mux_node,

    ])