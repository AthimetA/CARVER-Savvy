#!usr/bin/python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

    
def generate_launch_description():
    entity = LaunchConfiguration('entity') 
    file = LaunchConfiguration('file') 
    
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    R = LaunchConfiguration('R')
    P = LaunchConfiguration('P')
    Y = LaunchConfiguration('Y')
    
    entity_launch_arg = DeclareLaunchArgument('entity')
    file_launch_arg = DeclareLaunchArgument('file')
    x_launch_arg = DeclareLaunchArgument('x',default_value='0.0')
    y_launch_arg = DeclareLaunchArgument('y',default_value='0.0')
    z_launch_arg = DeclareLaunchArgument('z',default_value='0.0')
    R_launch_arg = DeclareLaunchArgument('R',default_value='0.0')
    P_launch_arg = DeclareLaunchArgument('P',default_value='0.0')
    Y_launch_arg = DeclareLaunchArgument('Y',default_value='0.0')
    package_name = 'april_tag_spawner'
    model_name = entity
    sdf_model_path = PathJoinSubstitution(
        [get_package_share_directory(package_name),'models',file,'model.sdf']
    )
    spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', model_name,
            '-file', sdf_model_path,
            '-x',x,
            '-y',y,
            '-z',z,
            '-R',R,
            '-P',P,
            '-Y',Y
        ],
        output='screen',
    )
    
    launch_description = LaunchDescription()
    launch_description.add_action(x_launch_arg)
    launch_description.add_action(y_launch_arg)
    launch_description.add_action(z_launch_arg)
    launch_description.add_action(R_launch_arg)
    launch_description.add_action(P_launch_arg)
    launch_description.add_action(Y_launch_arg)
    launch_description.add_action(entity_launch_arg)
    launch_description.add_action(file_launch_arg)
    launch_description.add_action(spawner)
    
    return launch_description