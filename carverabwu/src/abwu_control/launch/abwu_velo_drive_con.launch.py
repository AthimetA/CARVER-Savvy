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

    '''
    
    GAZEBO
    
    '''
    world_package_name = 'awbu_drl'

    launch_file_subfolder = 'launch'

    # File names
    gazebo_launch_file_name = 'abwu_drl_gazebo_world.launch.py'
    
    # file paths
    gazebo_launch_subpath = launch_file_subfolder + '/' + gazebo_launch_file_name

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare(world_package_name), gazebo_launch_subpath])),
                ) 
    
    '''
    
    ROBOT DESCRIPTION
    
    '''
    robot_name = 'carversavvy'

    description_package_name = f'{robot_name}_description'

    # Add Robot description
    description_file_subpath = f'description/{robot_name}.urdf.xacro'
    xacro_file = os.path.join(get_package_share_directory(description_package_name),description_file_subpath) # Use xacro to process the file
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    # ROBOT STATE PUBLISHER NODE:
    node_robot_state_publisher = Node(     # Configure the node
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': use_sim_time}]
    )

    # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'abwubot',
                                   '-x', '0.0',
                                    '-y', '0.0',
                                    '-z', '0.0',
                                    '-R', '0.0',
                                    '-P', '0.0',
                                    '-Y', '0.0',],
                        output='screen',
                        )
    

    # Joint state broadcaster:
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Diff Drive Controller
    diff_drive_controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_cont", "-c", "/controller_manager"],
    )

    '''
    
    CONTROLLER
    
    '''

    control_package_name = 'abwu_control'
    control_package_dir = get_package_share_directory(control_package_name)

    # Inverse Kinematics Node
    abwu_inverse_kinemetic = Node(
        package=control_package_name,
        executable='abwu_inverse_kinematic.py',
        name='AbwuIKNode',
    )

    # Forward Kinematics Node
    abwu_forward_kinematic = Node(
        package=control_package_name,
        executable='abwu_forward_kinematic.py',
        name='AbwuFKNode',
    )

    # Robot Localization
    parameters_file_dir = os.path.join(control_package_dir, 'config')
    parameters_file_path = os.path.join(parameters_file_dir, 'abwu_ekf.yaml')
    os.environ['FILE_PATH'] = str(parameters_file_dir)
    
    ekf_filter_node_odom = Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_odom',
	        output='screen',
            parameters=[parameters_file_path, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', '/abwubot/odom')]           
           )


    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        
        # Gazebo
        gazebo,
        # Robot
        node_robot_state_publisher,
        spawn_entity,
        # Controllers
        RegisterEventHandler(
            OnProcessExit(
                target_action = spawn_entity,
                on_exit = [
                    joint_state_broadcaster_spawner,
                    diff_drive_controllers,
                    ekf_filter_node_odom,
                    abwu_inverse_kinemetic,
                    abwu_forward_kinematic
                ]
            )
        ),
    ])