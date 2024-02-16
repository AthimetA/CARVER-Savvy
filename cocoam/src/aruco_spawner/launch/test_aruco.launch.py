#!usr/bin/python3
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
def id_argument(ID):
    return {'entity': 'marker_'+ID,'file': 'aruco_marker_'+ID}
def generate_launch_description():
    
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ])
    )
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    """
    launch_arg_dict = {'z': '0.5'}
    launch_arg_dict.update(id_argument('00'))
    single_spawner_00 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('aruco_spawner'),
                'launch',
                'aruco_spawner.launch.py'
            ])
        ]),
        launch_arguments=launch_arg_dict.items()
    )
    """  

    launch_description = LaunchDescription()
    launch_description.add_action(gazebo_server)
    launch_description.add_action(gazebo_client)


    for i in range(3):
        launch_arg_dict = {'z': '0.5','y': str(i*0.5)}
        launch_arg_dict.update(id_argument('0'+str(i)))
        spawner = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('aruco_spawner'),
                    'launch',
                    'aruco_spawner.launch.py'
                ])
            ]),
            launch_arguments=launch_arg_dict.items()
        )
        launch_description.add_action(spawner)

    return launch_description

    