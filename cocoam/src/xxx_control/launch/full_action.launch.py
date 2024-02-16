#!usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    interface = Node(
        package='turtlesim_control',
        executable='follower_interface.py',
    )

    controller = Node(
        package='turtlesim_control',
        executable='via_point_follower.py',
        remappings=[
            ('/pose','/turtle1/pose'),
            ('/cmd_vel','/turtle1/cmd_vel'),
        ]
    )

    simulator = Node(
        package='turtlesim',
        executable='turtlesim_node',
    )

    entity_to_run = [interface,controller,simulator]
    return LaunchDescription(entity_to_run)