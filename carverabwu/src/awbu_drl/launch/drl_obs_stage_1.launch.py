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

    obstacle_bot_description_file = os.path.join(
        get_package_share_directory(drl_package_name),
        'obstacles',
        'obstacle_bot.urdf.xacro'
    )
    robot_description_raw = xacro.process_file(obstacle_bot_description_file).toxml()

    # ROBOT STATE PUBLISHER NODE:
    node_robot_state_publisher = Node(
        namespace='obstacle_bot',
        name='obstacle_bot_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': use_sim_time}]
    )

    # SPAWN ROBOT TO GAZEBO:
    spawn_entity_1 = Node(package='gazebo_ros', executable='spawn_entity.py', name='spawn_entity_1',
                        arguments=['-topic', 'obstacle_bot/robot_description',
                                   '-entity', 'obstacle_bot_1',
                                   '-x', '-8.0',
                                    '-y', '-8.0',
                                    '-z', '0.0',
                                    '-R', '0.0',
                                    '-P', '0.0',
                                    '-Y', '0.0',
                                    '-robot_namespace', 'obstacle_bot_1',],
                        output='screen',
                        )
    
    spawn_entity_2 = Node(package='gazebo_ros', executable='spawn_entity.py', name='spawn_entity_2',
                        arguments=['-topic', 'obstacle_bot/robot_description',
                                   '-entity', 'obstacle_bot_2',
                                   '-x', '0.0',
                                    '-y', '-4.0',
                                    '-z', '0.0',
                                    '-R', '0.0',
                                    '-P', '0.0',
                                    '-Y', '0.0',
                                    '-robot_namespace', 'obstacle_bot_2',],
                        output='screen',
                        )
    
    spawn_entity_3 = Node(package='gazebo_ros', executable='spawn_entity.py', name='spawn_entity_3',
                        arguments=['-topic', 'obstacle_bot/robot_description',
                                   '-entity', 'obstacle_bot_3',
                                   '-x', '9.0',
                                    '-y', '9.0',
                                    '-z', '0.0',
                                    '-R', '0.0',
                                    '-P', '0.0',
                                    '-Y', '0.0',
                                    '-robot_namespace', 'obstacle_bot_3',],
                        output='screen',
                        )
    
    spawn_entity_4 = Node(package='gazebo_ros', executable='spawn_entity.py', name='spawn_entity_4',
                        arguments=['-topic', 'obstacle_bot/robot_description',
                                      '-entity', 'obstacle_bot_4',
                                      '-x', '-8.0',
                                        '-y', '8.0',
                                        '-z', '0.0',
                                        '-R', '0.0',
                                        '-P', '0.0',
                                        '-Y', '0.0',
                                        '-robot_namespace', 'obstacle_bot_4',],
                        output='screen',
                        )

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        # Robot
        node_robot_state_publisher,
        spawn_entity_1,
        spawn_entity_2,
        spawn_entity_3,
        spawn_entity_4,
    ])
