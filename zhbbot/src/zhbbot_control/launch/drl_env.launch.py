# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,Command, LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
import xacro
import yaml
    
# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():

    # Use sim time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    control_package_name = 'zhbbot_control'
    control_package_dir = get_package_share_directory(control_package_name)
    # Read config file
    controller_config_file = os.path.join(control_package_dir, 'config', 'zhbbot_control_params.yaml')

    with open(controller_config_file, 'r') as file:
        controller_config = yaml.safe_load(file)
        local_planner_name = controller_config['zhbbot_local_planer_name'][controller_config['zhbbot_selceted_local_planer']]
        local_planner_python_file = controller_config['zhbbot_local_planner_python_file'][controller_config['zhbbot_selceted_local_planer']]

    zhbbot_local_planer = Node(
        package='zhbbot_control',
        executable=local_planner_python_file,
        name=local_planner_name,
    )

    zhbbot_handler = Node(
        package='zhbbot_control',
        executable='zhbbot_handler.py',
        name='zhbbotHandlerNode',
        arguments=[local_planner_name],
    )

    drl_obstacle_control = Node(
        package='zhbbot_control',
        executable='drl_obstacle_control.py',
        name='drl_obstacle_control',
    )

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        
        zhbbot_local_planer,
        zhbbot_handler,
        drl_obstacle_control,

    ])