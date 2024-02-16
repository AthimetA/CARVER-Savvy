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

    # Create our own temporary YAML files that include substitutions
    lifecycle_nodes = ['filter_mask_server', 'costmap_filter_info_server']

    # Parameters
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    mask_yaml_file = LaunchConfiguration('mask')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_mask_yaml_file_cmd = DeclareLaunchArgument(
            'mask',
            description='Full path to filter mask yaml file to load')

    # Make re-written yaml
    param_substitutions = {
        'use_sim_time': use_sim_time,}
        # 'yaml_filename': mask_yaml_file}

    # configured_params = RewrittenYaml(
    #     source_file=params_file,
    #     root_key=namespace,
    #     param_rewrites=param_substitutions,
    #     convert_types=True)

    # Nodes launching commands
    start_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_filters',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    start_map_server_cmd = Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[])

    start_costmap_filter_info_server_cmd = Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[])

    # Robot Localization

    nav2_params = '/home/athimet/FRA532_Mobile_Robot/zhbbot/src/zhbbot_description/config/navigation_param.yaml'

    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )
    

    bringup_dir = get_package_share_directory('nav2_bringup')
    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        # declare_params_file_cmd,
        # declare_mask_yaml_file_cmd,
        # configured_params,
        start_lifecycle_manager_cmd,
        start_map_server_cmd,
        start_costmap_filter_info_server_cmd,

        navigation2_cmd,

    ])