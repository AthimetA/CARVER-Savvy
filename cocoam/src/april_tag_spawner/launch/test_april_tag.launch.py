#!usr/bin/python3
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os
def id_argument(ID):
    return {'entity': 'Apriltag36_11_000'+ID,'file': 'Apriltag36_11_000'+ID}
def generate_launch_description():

    ############################################################################
    
    model_path1 = os.path.join(get_package_share_directory(
                                    "enviroment"), 
                                    'models')

    model_path2 = os.path.join(get_package_share_directory(
                                    "april_tag_spawner"), 
                                    'models')

    os.environ["GAZEBO_MODEL_PATH"] = ":".join([model_path1,model_path2])
    
    ############################################################################

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
    launch_arg_dict.update(id_argument('15'))
    single_spawner_15 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('april_tag_spawner'),
                'launch',
                'april_tag_spawner.launch.py'
            ])
        ]),
        launch_arguments=launch_arg_dict.items()
    )
    """  

    launch_description = LaunchDescription()
    launch_description.add_action(gazebo_server)
    launch_description.add_action(gazebo_client)

    
    for i in range(2):
        launch_arg_dict = {'z': '0.5','y': str(i*0.5)}
        launch_arg_dict.update(id_argument('0'+str(i)))
        spawner = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('april_tag_spawner'),
                    'launch',
                    'april_tag_spawner.launch.py'
                ])
            ]),
            launch_arguments=launch_arg_dict.items()
        )
        launch_description.add_action(spawner)

    
    #launch_description.add_action(single_spawner_00)
    return launch_description

    