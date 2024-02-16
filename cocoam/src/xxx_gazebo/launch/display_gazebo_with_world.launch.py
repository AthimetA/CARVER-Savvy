#!usr/bin/python3

import sys
import yaml
import xacro
import os
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # Package Path
    xxx_gazebo_path = get_package_share_directory("xxx_gazebo")
    xxx_description_path = get_package_share_directory("xxx_description")
    april_tag_spawner_path = get_package_share_directory("april_tag_spawner")

    # Select World Type
    world_name = 'office_with_marker.world'
    # world_name = 'office.world'

    # Set the path to the world file
    world_path = os.path.join(xxx_gazebo_path, 
                                'worlds', 
                                world_name)

    #Set the path to the SDF model files.
    gazebo_models_path_1 = os.path.join(xxx_gazebo_path, 'models')
    gazebo_models_path_2 = os.path.join(april_tag_spawner_path, 'models')
    os.environ["GAZEBO_MODEL_PATH"] = ":".join([gazebo_models_path_1, gazebo_models_path_2])
    
    # gzserver without world 
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ])
    )

    # gzserver launch with world 
    gazebo_server_with_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={'world': world_path}.items()
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
    
    robot_description_path = os.path.join(xxx_gazebo_path, 
                                    "robot",
                                    "xxx.xacro")
    
    robot_description = xacro.process_file(robot_description_path).toxml()
    

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-topic','robot_description',
            '-entity', 'xxx',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-R', '0',
            '-P', '0',
            '-Y', '0',],
        output='screen',
    )

    gazebo_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description}
        ]
    )
    

    controller_config_path = os.path.join(xxx_gazebo_path, 
                                    "config",
                                    "ros2_controllers_config.yaml")
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': robot_description},
            controller_config_path
        ],
        output="both"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]
    )
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controllers", "-c", "/controller_manager"],
    )
    
    property_urdf = os.path.join(xxx_description_path, 'config', 'properties.yaml')
    
    ik = Node(
        package='xxx_control',
        executable='inverse_kinematics.py',
        remappings=[('cmd_vel','turtle1/cmd_vel')],
        arguments=[property_urdf]
    )
    
    launch_description = LaunchDescription()
    # launch_description.add_action(gazebo_server)
    launch_description.add_action(gazebo_server_with_world)
    launch_description.add_action(gazebo_client)
    launch_description.add_action(gazebo_state_publisher)
    launch_description.add_action(control_node)
    launch_description.add_action(joint_state_broadcaster_spawner)
    launch_description.add_action(robot_controller_spawner)
    launch_description.add_action(spawn_entity)
    launch_description.add_action(ik)
    
    return launch_description

def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        sys.exit()

if __name__ == '__main__':
    main()
    
