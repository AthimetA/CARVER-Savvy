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
    # Pause simulation
    pause = LaunchConfiguration('pause', default='true')
    
    test_node = Node(
            package='awbu_drl',
            executable='testnode.py',
            name='testnode',
            parameters=[{'use_sim_time': use_sim_time}],
         )
    
    drl_gazebo = Node(
            package='awbu_drl',
            executable='drl_gazebo.py',
            name='drl_gazebo',
            parameters=[{'use_sim_time': use_sim_time}],
         )
    
    drl_env = Node(
            package='awbu_drl',
            executable='drl_environment.py',
            name='drl_env',
            parameters=[{'use_sim_time': use_sim_time}],
         )

#     package_name = 'abwu_simulation'

#     joy_params = os.path.join(get_package_share_directory(package_name),'config','joystick.yaml')

#     joy_node = Node(
#             package='joy',
#             executable='joy_node',
#             parameters=[joy_params, {'use_sim_time': use_sim_time}],
#          )

#     teleop_node = Node(
#             package='teleop_twist_joy',
#             executable='teleop_node',
#             name='teleop_node',
#             parameters=[joy_params, {'use_sim_time': use_sim_time}],
#             # remappings=[('/cmd_vel','/diff_cont/cmd_vel_unstamped')]
#             remappings=[('/cmd_vel','/cmd_vel_joy')]
#          )

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([

        test_node,

        drl_gazebo,

        drl_env,

        # joy_node,

        # teleop_node,

    ])