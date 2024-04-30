#!/usr/bin/python3
import time
import numpy as np
import rclpy
from rclpy.node import Node
import yaml

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty

from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import GetModelList, SetEntityState, GetEntityState

from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock

from settings.constparams import EPISODE_TIMEOUT_SECONDS

from awbu_interfaces.srv import ObstacleStart

SIMUALTION_TIME_SCALE = 4.0 # 4x faster than real time
PATH_INTERVAL_PER_EPISODE = 4

from ament_index_python import get_package_share_directory
class ObstacleHandler(Node):
    def __init__(self):
        super().__init__('ObstacleHandler')
        # --------------- ROS Parameters --------------- #
        qos = QoSProfile(depth=10)
        qos_clock = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        # Clock subscriber
        self.time_sec = 0
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, qos_clock)
        
        # Initialise services servers
        self.obstacle_start_srv = self.create_service(ObstacleStart, '/obstacle_start', self.obstacle_start_callback)

        # Gazebo service client
        self.set_entity_state_client    = self.create_client(SetEntityState, '/gazebo_drl/set_entity_state')
        self.get_entity_state_client    = self.create_client(GetEntityState, '/gazebo_drl/get_entity_state')
        self.get_model_list_client      = self.create_client(GetModelList, '/get_model_list')
        self.reset_simulation_client    = self.create_client(Empty, '/reset_world')

        self.reset_simulation()

        # Gazebo model list subscriber
        self.model_list = self.get_model_list()
        self.obstacle_list =  self.init_obstacles()

        # Load the obstacle information yaml file
        self.__PKG_NAME = 'awbu_drl'
        self.__PKG_PATH = get_package_share_directory(self.__PKG_NAME)

        with open(f'{self.__PKG_PATH}/config/obstacle_params.yaml', 'r') as file:
            __CFG = yaml.safe_load(file)
            
            __OBSTACLE_PARAMS = __CFG['ObstacleParams']

            for obstacle in self.obstacle_list:
                if obstacle.name in __OBSTACLE_PARAMS['obstacleNames']:
                    __TGP = __CFG[obstacle.name]['TargetPose']
                    # Set the target pose of the obstacle
                    target_pose = Pose()
                    target_pose.position.x = float(__TGP[0])
                    target_pose.position.y = float(__TGP[1])
                    target_pose.position.z = float(__TGP[2])
                    obstacle.set_target_pose(target_pose)

        del __CFG
        del __OBSTACLE_PARAMS
        del __TGP

        print(f'Obstacle list: \n{self.obstacle_list}')

        # Control loop
        self.control_loop_hz =  10.0  # Based on 10Hz
        self.EPISODE_TIMEOUT_SECONDS = EPISODE_TIMEOUT_SECONDS
        self.interval_per_episode = PATH_INTERVAL_PER_EPISODE
        self.epsode_interval_step = (EPISODE_TIMEOUT_SECONDS / PATH_INTERVAL_PER_EPISODE)

        # Control timer
        self.control_timer = self.create_timer(1.0/(self.control_loop_hz*SIMUALTION_TIME_SCALE), self.obstacle_control_loop_callback)
        self.time_loop_on = False
        self.current_interval = 0

        # Episode time
        self.start_episode_time = self.time_sec
        self.time_episode_sec = -1.0
        self.time_episode_sec_last = -1.0


    def obstacle_control_loop_callback(self):
        # Get the current time
        self.time_episode_sec = (self.time_sec - self.start_episode_time)

        if self.time_episode_sec != self.time_episode_sec_last and self.time_loop_on:

            if self.time_episode_sec > self.EPISODE_TIMEOUT_SECONDS:
                # Disable the time loop
                self.time_loop_on = False

            elif self.time_episode_sec % self.epsode_interval_step == 0:
                self.get_logger().info(f'Time: {self.time_episode_sec}, current interval: {self.current_interval}')
                
                for obstacle in self.obstacle_list:
                    # # Update the obstacle state
                    out_pose, out_twist = obstacle.get_state_at_time(self.current_interval)
                    self.set_entity_state(obstacle.name, out_pose, out_twist)

                # Check if it's past the episode duration
                if self.current_interval > self.interval_per_episode:
                    # Reset the interval and print a message
                    self.current_interval = 0

                self.current_interval += 1

            self.time_episode_sec_last = self.time_episode_sec

        # if not self.time_loop_on:
        #     self.get_logger().info('IDLE : Please start the episode')

    def clock_callback(self, msg: Clock):
        # Get the current time in seconds
        self.time_sec = msg.clock.sec

    def obstacle_start_callback(self, request: ObstacleStart.Request, response: ObstacleStart.Response):
        self.get_logger().info('===============Obstacle start callback===============')
        self.time_loop_on = True
        self.current_interval = 0
        self.start_episode_time = self.time_sec
        self.time_episode_sec_last = self.time_episode_sec
        # Response
        response.obstacle_status = True
        return response

    def timer_callback(self):
        self.get_logger().info('Timer callback')

    def init_obstacles(self):
        # Initialize the obstacles
        obstacle_list = []
        for obstacle in self.model_list:
            if 'obstacle' in obstacle:
                # Get the initial pose and twist of the obstacle
                pose, twist = self.get_entity_state(obstacle)
                obstacle_list.append(DynamicObstacle(name=obstacle, initial_pose=pose))
        return obstacle_list

    def get_model_list(self):
        request = GetModelList.Request()
        while not self.get_model_list_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        try:
            future = self.get_model_list_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            return response.model_names

        except Exception as e:
            self.get_logger().info(f'Error: {e}')

    def get_entity_state(self, entity_name):
        request = GetEntityState.Request()
        request.name = entity_name
        request.reference_frame = 'world'
        while not self.get_entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        try:
            future = self.get_entity_state_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            return response.state.pose, response.state.twist
        except Exception as e:
            self.get_logger().info(f'Error: {e}')

    def set_entity_state(self, entity_name: str, pose: Pose, twist: Twist):
        request = SetEntityState.Request()
        request.state.name = entity_name
        request.state.pose = pose
        request.state.twist = twist
        request.state.reference_frame = 'world'
        while not self.set_entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        try:
            self.get_logger().info(f'Setting entity state for {entity_name}...')
            future = self.set_entity_state_client.call_async(request)
            # rclpy.spin_until_future_complete(self, future)
            response = future.result()
            # self.get_logger().info(f'Response: {response}')
        except Exception as e:
            self.get_logger().info(f'Error: {e}')

    def reset_simulation(self):
        while not self.reset_simulation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('reset service not available, waiting again...')
        self.reset_simulation_client.call_async(Empty.Request())

class DynamicObstacle:
    def __init__(self,
    name: str,
    initial_pose: Pose,
    initial_twist: Twist = Twist(),
    interval_per_episode: int = PATH_INTERVAL_PER_EPISODE, # Number of loops that the obstacle will move between the initial and target pose per episode
    random_velocity: bool = False, # Randomize the velocity of the obstacle within a range of calculated valocity if not the calculated velocity will be used
    ):
        # Obstacle information
        self.name = name
        self.initial_pose = initial_pose
        self.initial_twist = initial_twist
        self.interval_per_episode = interval_per_episode
        self.random_velocity = random_velocity

        self.intervals = np.linspace(0.0, EPISODE_TIMEOUT_SECONDS, self.interval_per_episode +1)

        self.target_pose = initial_pose
        self.target_twist = Twist()
        self.time_to_target = 0.0 # seconds

    def set_initial_pose(self, pose: Pose):
        self.initial_pose = pose

    def set_target_pose(self, pose: Pose):
        self.target_pose = pose
        self.linear_path_velocity_calculation()

    def set_interval_per_episode(self, interval: float):
        self.interval_per_episode = interval
        self.intervals = np.linspace(0.0, EPISODE_TIMEOUT_SECONDS, self.interval_per_episode +1)

    def linear_path_velocity_calculation(self):
        # Calculate the target twist based on the target pose
        distance_x = np.abs(self.target_pose.position.x - self.initial_pose.position.x)
        distance_y = np.abs(self.target_pose.position.y - self.initial_pose.position.y)

        # Calculate the target twist based on the difference in position and the episode timeout
        self.time_to_target = EPISODE_TIMEOUT_SECONDS / self.interval_per_episode

        # Calculate the target twist
        linear_x = distance_x / self.time_to_target
        linear_y = distance_y / self.time_to_target
        angular_z = 0.0

        # Randomize the velocity if enabled
        if self.random_velocity:
            linear_x = np.random.uniform(0.0, linear_x)
            linear_y = np.random.uniform(0.0, linear_y)

        self.target_twist.linear.x = linear_x * np.sign(self.target_pose.position.x - self.initial_pose.position.x)
        self.target_twist.linear.y = linear_y * np.sign(self.target_pose.position.y - self.initial_pose.position.y)
        self.target_twist.angular.z = angular_z

    def get_state_at_time(self, time_loc:int):

        if time_loc == PATH_INTERVAL_PER_EPISODE:
            return self.initial_pose, Twist()

        # Create a Twist message to store the output
        out_twist = Twist()

        # Set the linear velocity components from the target twist
        out_twist.linear.x = self.target_twist.linear.x
        out_twist.linear.y = self.target_twist.linear.y
        out_twist.linear.z = self.target_twist.linear.z

        if time_loc % 2 == 0:
            out_pose = self.initial_pose
        elif time_loc % 2 == 1:
            out_pose = self.target_pose
            out_twist.linear.x *= -1
            out_twist.linear.y *= -1
        return out_pose, out_twist
        
    def __repr__(self) -> str:
        return f'Obstacle: {self.name}, Initial Pose: ({self.initial_pose.position.x:.2f}, {self.initial_pose.position.y:.2f}), Target Pose: ({self.target_pose.position.x:.2f}, {self.target_pose.position.y:.2f})\n'
        


def main(args=None):
    rclpy.init(args=args)
    test_node = ObstacleHandler()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()