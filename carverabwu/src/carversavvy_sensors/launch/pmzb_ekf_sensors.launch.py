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
from nav2_common.launch import RewrittenYaml
    
# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():

    # ========== **GET PACKAGE SHARE DIRECTORY** ========== #
    carversavvybot_sensors_dir = get_package_share_directory('carversavvybot_sensors')

    # Ros2 Bridge Node
    carversavvy_ros_bridge_node = Node(
        package='carversavvybot_sensors',
        executable='carversavvy_ros_bridge.py',
        name='carversavvy_ros_bridge',
    )
    
    # EKF Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[
            os.path.join(carversavvybot_sensors_dir, 'config', 'carversavvy_ekf.yaml'),
        ],
        remappings=[('odometry/filtered', '/carversavvy_ros/odometry')]
    )

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        
        # Ros2 Bridge Node
        carversavvy_ros_bridge_node,

        # EKF Node
        ekf_node,

    ])