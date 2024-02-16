#!usr/bin/python3
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    scheduler = Node(
        package='turtlesim_control',
        executable='scheduler.py',
    )


    leader = Node(
        package='turtlesim_control',
        executable='via_point_follower.py',
        remappings=[
            ('/pose','/turtle1/pose'),
            ('/cmd_vel','/turtle1/cmd_vel'),
        ]
    )

    turtle_following_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlesim_control'),
                'launch',
                'turtle_following.launch.py'
            ])
        ]),
        launch_arguments={
            'new_background_r': '0',
        }.items()
    )

    entity_to_run = [scheduler,leader,turtle_following_launch]
    return LaunchDescription(entity_to_run)

    