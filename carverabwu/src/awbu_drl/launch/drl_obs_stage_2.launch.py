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

    spawn_entity_list = [
        # Left side
        ('obstacle_bot_1', 5.4, 4.0, 0.0, 0.0, 0.0, -1.5708),
        ('obstacle_bot_2', 3.0, 7.4, 0.0, 0.0, 0.0, -1.5708),
        ('obstacle_bot_3', 3.0, 3.7, 0.0, 0.0, 0.0, -1.5708),
        ('obstacle_bot_4', 0.0, 5.7, 0.0, 0.0, 0.0, -1.5708),
        ('obstacle_bot_5', -2.8, 3.0, 0.0, 0.0, 0.0, -1.5708),
        ('obstacle_bot_6', 6.8, 6.2, 0.0, 0.0, 0.0, -1.5708),
        # Right side
        ('obstacle_bot_7', 4.2, -5.4, 0.0, 0.0, 0.0, 1.5708),
        ('obstacle_bot_8', 1.5, -4.7, 0.0, 0.0, 0.0, 1.5708),
        ('obstacle_bot_9', -1.3, -6.2, 0.0, 0.0, 0.0, 1.5708),
        ('obstacle_bot_10', -4.0, -4.0, 0.0, 0.0, 0.0, 1.5708),
    ]

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        # Robot
        node_robot_state_publisher,
        # Obstacles
        *[spawn_entity(entity_name, x, y, z, R, P, Y) for entity_name, x, y, z, R, P, Y in spawn_entity_list],
    ])

def spawn_entity(entity_name, x, y, z, R, P, Y):
    return Node(package='gazebo_ros', executable='spawn_entity.py', name=f'spawn_entity_{entity_name}',
                arguments=['-topic', f'obstacle_bot/robot_description',
                           '-entity', f'{entity_name}',
                           '-x', f'{x}',
                            '-y', f'{y}',
                            '-z', f'{z}',
                            '-R', f'{R}',
                            '-P', f'{P}',
                            '-Y', f'{Y}',
                            '-robot_namespace', f'{entity_name}',],
                output='screen',
                )