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
    pause = LaunchConfiguration('pause', default='false')

    world_package_name = 'abwu_gazebo'

    gazebo_params_file = os.path.join(
        get_package_share_directory(world_package_name),
        'config',
        'gazebo_params.yaml')

    # ***** GAZEBO ***** #   
    # DECLARE Gazebo WORLD file:
    world_file_name = "abwu_drl_stage_1.world"
    world_file_path = os.path.join(
        get_package_share_directory(world_package_name),
        'worlds',
        world_file_name)
    
    # gzserver launch with world 
    gazebo_server_with_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        # launch_arguments={'world': world_file_path}.items()
        launch_arguments={'world': world_file_path,
                           'pause': pause,
                            'params_file': gazebo_params_file,
                        }.items()
    )

    # gzclient
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
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

    # Inverse Kinematics Node
    abwu_inverse_kinemetic = Node(
        package=control_package_name,
        executable='abwu_inverse_kinematic.py',
        name='AbwuIKNode',
    )
   
    # Add Obstacle launch file
    drl_package_name = 'awbu_drl'

    obstacle_stage_1_launch_file = os.path.join(
        get_package_share_directory(drl_package_name),
        'launch',
        'drl_obs_stage_1.launch.py'
    )

    # Launch the obstacle stage 1
    obstacle_stage_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(obstacle_stage_1_launch_file)
    )

    stage_temp_file_init(number=1)

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        # Gazebo
        gazebo_server_with_world,
        gazebo_client,
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
                    abwu_inverse_kinemetic,
                    obstacle_stage_1,
                ]
            )
        ),
    ])


def stage_temp_file_init(number):
    # Create a temporary file to store the stage number
    file_path = os.getenv('ABWUDRL_BASE_PATH') +'/tmp/abwu_current_stage.txt'

    if not os.path.exists(file_path):
        # Create the file
        os.makedirs(os.path.dirname(file_path), exist_ok=True)
        with open(file_path, 'w') as file:
            file.write(str(number))
            file.close()
    else:
        with open(file_path, 'r') as file:
            current_stage = int(file.read())
            file.close()
            if current_stage != number:
                with open(file_path, 'w') as file:
                    file.write(str(number))
                    file.close()
    
    print(f"Stage {number} initialized")