#!/usr/bin/python3
import time
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty

from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import GetModelList, SetEntityState, GetEntityState

from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock

from settings.constparams import EPISODE_TIMEOUT_SECONDS

SIMUALTION_TIME_SCALE = 4.0 # 4x faster than real time
PATH_INTERVAL_PER_EPISODE = 4

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

        self.current_interval = 1
        self.intervals = np.linspace(0.0, EPISODE_TIMEOUT_SECONDS, self.interval_per_episode +1)

        self.target_pose = initial_pose
        self.target_twist = Twist()
        self.time_to_target = 0.0 # seconds

    def reset(self):
        self.current_interval = 1

    def set_initial_pose(self, pose: Pose):
        self.initial_pose = pose

    def set_target_pose(self, pose: Pose):
        self.target_pose = pose
        self.linear_path_velocity_calculation()

    def set_interval_per_episode(self, interval: float):
        self.interval_per_episode = interval
        self.current_interval = 1
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

    def get_twist_at_time(self, time : float):
        # Create a Twist message to store the output
        out_twist = Twist()

        # Set the linear velocity components from the target twist
        out_twist.linear.x = self.target_twist.linear.x
        out_twist.linear.y = self.target_twist.linear.y
        out_twist.linear.z = self.target_twist.linear.z

        # Check if the time is within the current interval
        if time <= self.intervals[self.current_interval]:
            # If it's an odd interval, return the original twist
            if self.current_interval % 2 == 1:
                return out_twist
            # If it's an even interval, reverse the direction
            else:
                out_twist.linear.x *= -1
                out_twist.linear.y *= -1
                return out_twist

        # Handle the case where time reaches the end of the current interval
        elif time > self.intervals[self.current_interval]:
            # Move to the next interval
            self.current_interval += 1

            # Check if it's past the episode duration
            if self.current_interval > self.interval_per_episode:
                # Reset the interval and print a message
                self.reset()

            # Since the object reached the target pose, return zero twist
            return Twist()

        # If time is beyond the expected interval, print an error message
        else:
            print(f'Error: Time: {time}, Current Interval: {self.current_interval}, Intervals: {self.intervals}')
            return Twist()
        
    def __repr__(self) -> str:
        return f'Obstacle: {self.name}, Initial Pose: ({self.initial_pose.position.x:.2f}, {self.initial_pose.position.y:.2f}), Target Pose: ({self.target_pose.position.x:.2f}, {self.target_pose.position.y:.2f})\n'
        

class ObstacleHandler(Node):
    def __init__(self):
        super().__init__('ObstacleHandler')

        self.test_timer = self.create_timer(2.0, self.timer_callback)

        # --------------- ROS Parameters --------------- #
        qos = QoSProfile(depth=10)
        qos_clock = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Gazebo service client
        self.set_entity_state_client    = self.create_client(SetEntityState, '/gazebo_drl/set_entity_state')
        self.get_entity_state_client    = self.create_client(GetEntityState, '/gazebo_drl/get_entity_state')
        self.get_model_list_client      = self.create_client(GetModelList, '/get_model_list')
        self.reset_simulation_client    = self.create_client(Empty, '/reset_world')

        self.reset_simulation()

        # Gazebo model list subscriber
        self.model_list = self.get_model_list()
        self.obstacle_list =  self.init_obstacles()

        print(f'Obstacle list: \n{self.obstacle_list}')


        # Control loop
        self.control_loop_hz =  10.0  # Based on 10Hz
        self.control_loop_period = 1e9/self.control_loop_hz # Convert to nanoseconds
        self.start_loop_time = time.perf_counter_ns()
        
        self.start_episode_time = time.perf_counter_ns()
        self.EPISODE_TIMEOUT_SECONDS = EPISODE_TIMEOUT_SECONDS / SIMUALTION_TIME_SCALE
        self.current_interval = 1
        self.interval_per_episode = PATH_INTERVAL_PER_EPISODE
        self.intervals = np.linspace(0.0, EPISODE_TIMEOUT_SECONDS, self.interval_per_episode +1) / SIMUALTION_TIME_SCALE

        self.get_logger().info(f'Interval per episode: {self.interval_per_episode}, Intervals: {self.intervals}')

        # tgp = Pose()
        # tgp.position.x = -5.5
        # tgp.position.y = 6.0
        # tgp.position.z = 0.5

        # self.obstacle_list[0].set_target_pose(tgp)

        # # for t in range(1, (EPISODE_TIMEOUT_SECONDS + 1)):
        # #     self.get_logger().info(f'Time: {t}, Obstacle Twist: {self.obstacle_list[0].get_twist_at_time(t)}')

        # self.set_entity_state(self.obstacle_list[0].name, self.obstacle_list[0].initial_pose, self.obstacle_list[0].get_twist_at_time(2.0))

        self.obstacle_control_loop()

    def obstacle_control_loop(self):
        tgp = Pose()
        tgp.position.x = -5.5
        tgp.position.y = 6.0
        tgp.position.z = 0.5

        self.obstacle_list[0].set_target_pose(tgp)

        self.start_episode_time = time.perf_counter_ns()
        time_episode_sec = (time.perf_counter_ns() - self.start_episode_time) / 1e9

        self.set_entity_state(self.obstacle_list[0].name, self.obstacle_list[0].initial_pose, self.obstacle_list[0].get_twist_at_time(time_episode_sec))

        self.start_loop_time = time.perf_counter_ns()

        while (True):
            
            # Get the current time
            time_loop_sec = (time.perf_counter_ns() - self.start_loop_time)

            # Control loop at 10Hz * SIMUALTION_TIME_SCALE
            if time_loop_sec >= self.control_loop_period:
                self.start_loop_time = time.perf_counter_ns()

                # Obstacle control loop
                time_episode_sec = (time.perf_counter_ns() - self.start_episode_time) / 1e9
                time_episode_sec = np.round(time_episode_sec, 2)

                print(f'Epsiode Time: {time_episode_sec}, Current Interval: {self.current_interval}')

                if time_episode_sec > self.EPISODE_TIMEOUT_SECONDS:
                            
                    self.start_episode_time = time.perf_counter_ns()

                    # Reset the obstacles
                    for obstacle in self.obstacle_list:
                        obstacle.reset()

                    # Set the target pose for the obstacle
                    self.set_entity_state(self.obstacle_list[0].name, self.obstacle_list[0].initial_pose, self.obstacle_list[0].initial_twist)

                    break

                elif time_episode_sec == self.intervals[self.current_interval]:
                    self.current_interval += 1
                    self.get_logger().info(f'Current Interval: {self.current_interval}, Time: {time_episode_sec}')
                    self.get_logger().info(f'Obstacle Pose: {self.obstacle_list[0].target_pose}')
                    self.get_logger().info(f'Obstacle Twist: {self.obstacle_list[0].get_twist_at_time(time_episode_sec*SIMUALTION_TIME_SCALE)}')
                    if self.current_interval % 2 == 1:
                        self.set_entity_state(self.obstacle_list[0].name, self.obstacle_list[0].target_pose, self.obstacle_list[0].get_twist_at_time(time_episode_sec*SIMUALTION_TIME_SCALE))
                    else:
                        self.set_entity_state(self.obstacle_list[0].name, self.obstacle_list[0].initial_pose, self.obstacle_list[0].get_twist_at_time(time_episode_sec*SIMUALTION_TIME_SCALE))
                
        self.get_logger().info('Obstacle control loop finished')

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

    def clock_callback(self, msg: Clock):
        # Get the current time in nanoseconds
        self.time_sec = msg.clock.sec * 1e9 + msg.clock.nanosec

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

    def set_entity_state(self, entity_name, pose, twist):
        request = SetEntityState.Request()
        request.state.name = entity_name
        request.state.pose = pose
        request.state.twist = twist
        request.state.reference_frame = 'world'
        while not self.set_entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        try:
            self.get_logger().info(f'Setting entity state...')
            future = self.set_entity_state_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            self.get_logger().info(f'Set entity state: {response}')
        except Exception as e:
            self.get_logger().info(f'Error: {e}')

    def reset_simulation(self):
        while not self.reset_simulation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('reset service not available, waiting again...')
        self.reset_simulation_client.call_async(Empty.Request())


def main(args=None):
    rclpy.init(args=args)
    test_node = ObstacleHandler()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()