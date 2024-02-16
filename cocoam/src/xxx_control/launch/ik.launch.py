#!usr/bin/python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    
    property_urdf = os.path.join(get_package_share_directory('xxx_description'),'config','properties.yaml')
    
    ik = Node(
        package='xxx_control',
        executable='inverse_kinematics.py',
        remappings=[('cmd_vel','turtle1/cmd_vel')],
        arguments=[property_urdf]
    )

    entity_to_run = [ik]
    return LaunchDescription(entity_to_run)

    