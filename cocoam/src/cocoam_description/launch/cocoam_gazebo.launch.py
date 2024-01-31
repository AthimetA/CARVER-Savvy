import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro


def generate_launch_description():

    # Create the launch description 
    launch_description = LaunchDescription()
    
    # Add Robot description
    description_package_name = 'cocoam_description'
    description_file_subpath = 'description/cocoam_robot.urdf.xacro'
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
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    #     )
    # launch_description.add_action(gazebo)

    # # Add spawn entity
    # # SPAWN ROBOT TO GAZEBO:
    # spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
    #                     arguments=['-topic', 'robot_description',
    #                                '-entity', 'robot',
    #                                '-x', '0.0',
    #                                 '-y', '0.0',
    #                                 '-z', '0.0',
    #                                 '-R', '0.0',
    #                                 '-P', '0.0',
    #                                 '-Y', '0.0',],
    #                     output='screen')
    # launch_description.add_action(spawn_entity)

    # # ***** CONTROLLERS ***** #
    # # Joint state broadcaster:
    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    # )
    # launch_description.add_action(joint_state_broadcaster_spawner)


    return launch_description

def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()

if __name__ == '__main__':
    main()