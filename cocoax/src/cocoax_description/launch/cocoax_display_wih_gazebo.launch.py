import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler

from launch_ros.actions import Node
import xacro
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

import sys

def generate_launch_description():
    
    # Create the launch description 
    launch_description = LaunchDescription()
    
    # Add Robot description
    description_package_name = 'cocoax_description'
    description_file_subpath = 'robot/cocoax_robot.urdf.xacro'
    xacro_file = os.path.join(get_package_share_directory(description_package_name),description_file_subpath) # Use xacro to process the file
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    node_robot_state_publisher = Node(     # Configure the node
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}]
    )
    launch_description.add_action(node_robot_state_publisher)

    # Add Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )
    launch_description.add_action(gazebo)

    # Add spawn entity
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'robot'],
                    output='screen')
    launch_description.add_action(spawn_entity)

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    launch_description.add_action(joint_state_broadcaster_spawner)

    # Velocity Controller
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["cocoax_joint_group_VelocityController", "--controller-manager", "/controller_manager"],
    )
    launch_description.add_action(robot_controller_spawner)
    
    package_name = 'cocoax_description'
    path_to_package = get_package_share_directory(package_name)
    sub_folder = 'config'
    file_name = 'cocoax.rviz'
    rviz_file_path = os.path.join(path_to_package,sub_folder,file_name) 
    print(rviz_file_path)
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       name='rviz',
       arguments=['-d', rviz_file_path],
       output='screen')
    launch_description.add_action(rviz)
    
    return launch_description

def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()

if __name__ == '__main__':
    main()