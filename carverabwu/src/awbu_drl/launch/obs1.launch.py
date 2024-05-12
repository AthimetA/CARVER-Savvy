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

    # Add Obstacle description
    drl_package_name = 'awbu_drl'

    obstacle_1_description_file = os.path.join(
        get_package_share_directory(drl_package_name),
        'obstacles',
        'cylinder_obstacle_1.urdf'
    )

    # ROBOT STATE PUBLISHER NODE:
    node_robot_state_publisher = Node(     # Configure the node
        name='obstacle_1_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[obstacle_1_description_file]
    )

    # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        name='spawn_obstacle_1',
                        arguments=['-entity', 'abwubot2',
                                    '-file', obstacle_1_description_file,
                                    '-x', '9.0',
                                    '-y', '9.0',
                                    '-z', '0.0',
                                    '-R', '0.0',
                                    '-P', '0.0',
                                    '-Y', '0.0',],
                        output='screen',
                        )

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        # Robot
        node_robot_state_publisher,
        spawn_entity,
    ])