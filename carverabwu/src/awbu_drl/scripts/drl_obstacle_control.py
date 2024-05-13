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

# Topic name imports
from settings.constparams import TOPIC_CLOCK

from awbu_interfaces.srv import ObstacleStart

from env_utils import get_simulation_speed, read_stage

OBSTACLE_VELOCITY_SCALING = 2.0

from ament_index_python import get_package_share_directory
class ObstacleHandler(Node):
    def __init__(self):
        super().__init__('ObstacleHandler')
        self.stage = read_stage()
        self.sim_speed = get_simulation_speed(stage=self.stage)
        
        # Initialise services servers
        self.obstacle_start_srv = self.create_service(ObstacleStart, '/obstacle_start', self.obstacle_start_callback)

        # Gazebo service client
        self.get_model_list_client      = self.create_client(GetModelList, '/get_model_list')
        self.reset_simulation_client    = self.create_client(Empty, '/reset_world')

        self.reset_simulation()

        # Gazebo model list subscriber
        self.model_list = self.get_model_list()
        self.obstacle_list =  [model for model in self.model_list if 'obstacle' in model]
        self.get_logger().info(f'Obstacle list: {self.obstacle_list}')

        # Creace a publisher for the obstacle control
        self.obstacle_control_pub_list = [self.create_publisher(Twist, f'/{obstacle}/cmd_vel', 10) for obstacle in self.obstacle_list]

        # Control loop
        self.new_velo_time = 5.0 # seconds # Time to update the velocity
        self.control_loop_period = self.new_velo_time / self.sim_speed
        self.twist_list = [Twist() for _ in range(len(self.obstacle_list))]

        self.timer = self.create_timer(self.control_loop_period, self.timer_callback)
        self.timer_velo = self.create_timer(1.0 / (30.0 * self.sim_speed), self.velo_pub_timer)

    def velo_pub_timer(self):
        # Publish the velocity
        for i, pub in enumerate(self.obstacle_control_pub_list):
            pub.publish(self.twist_list[i])

        self.get_logger().info(f'Published: {self.twist_list}')

    def timer_callback(self):
        # Generate new velocity
        for twist in self.twist_list:
            twist.linear.x = np.random.uniform(-1.0, 1.0) * OBSTACLE_VELOCITY_SCALING
            twist.angular.z = np.random.uniform(-1.0, 1.0) * OBSTACLE_VELOCITY_SCALING


    def obstacle_start_callback(self, request: ObstacleStart.Request, response: ObstacleStart.Response):
        response.obstacle_status = True
        return response

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