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

import sys
import os
sys.path.append('/home/athimeta/CARVER-Savvy/')
    
# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():

    # Use sim time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    test_node = Node(
            package='awbu_drl',
            executable='testnode.py',
            name='testnode',
            # parameters=[{'use_sim_time': use_sim_time}],
         )

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        
        test_node,

    ])