# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import yaml
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None
# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None
    
# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():
    
    description_package_name = 'abwu_gazebo'

    # ***** GAZEBO ***** #   
    # DECLARE Gazebo WORLD file:
    world_file = os.path.join(
        get_package_share_directory(description_package_name),
        'worlds',
        'office.world')
    
    # Gazebo parameters file
    gazebo_params_file = os.path.join(
        get_package_share_directory(description_package_name),
        'config',
        'gazebo_params.yaml')

    # DECLARE Gazebo LAUNCH file:
    # gazebo = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    #             launch_arguments={'world': world_file, 'extra_gazebo_args': '--ros-args --params-file '+ gazebo_params_file}.items(),
    #          )
    
    # gzserver launch with world 
    gazebo_server_with_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        # launch_arguments={'world': world_file}.items()
        launch_arguments={'world': world_file, 'extra_gazebo_args': '--ros-args --params-file '+ gazebo_params_file}.items(),
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
        # gazebo, 
        gazebo_server_with_world,
        gazebo_client,
    ])