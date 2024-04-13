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
    # Pause simulation
    pause = LaunchConfiguration('pause', default='true')

    world_package_name = 'abwu_gazebo'

    # ***** GAZEBO ***** #   
    # DECLARE Gazebo WORLD file:
    world_file_name = "testworld.world"
    world_file_path = os.path.join(
        get_package_share_directory(world_package_name),
        'worlds',
        world_file_name)
    
    # gzserver launch with world 
    gazebo_server_with_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={'world': world_file_path}.items()
        # launch_arguments={'world': world_file_path, 'pause': pause}.items()
    )

    # gzclient
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )


    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        # Gazebo
        gazebo_server_with_world,
        gazebo_client,

    ])