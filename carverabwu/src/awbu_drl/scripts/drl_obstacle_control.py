#!/usr/bin/python3

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import GetModelList, SetEntityState, GetEntityState

class TestNode(Node):
    def __init__(self):
        # Initialize the node with the name 'TestNodeA'
        super().__init__('TestNodeA')
        self.timer_period = 1.0
        self.scale = -1.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # Odom subscriber
        self.odom_topic = '/abwubot/odom'
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        # Gazebo service client
        self.set_entity_state_client = self.create_client(SetEntityState, '/gazebo_drl/set_entity_state')
        self.get_entity_state_client = self.create_client(GetEntityState, '/gazebo_drl/get_entity_state')
        self.get_model_list_client = self.create_client(GetModelList, '/get_model_list')

        # Gazebo model list subscriber
        self.model_list = self.get_model_list()
        self.obstacle_list = [obstacle for obstacle in self.model_list if 'obstacle' in obstacle]
        self.get_entity_state('cylinder_obstacle_1')

        twist = Twist()
        twist.linear.x = -0.1
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 1.0
        pose = Pose()
        pose.position.x = 4.0
        pose.position.y = 4.0
        self.set_entity_state('cylinder_obstacle_1', pose, twist)

    def get_model_list(self):
        request = GetModelList.Request()
        while not self.get_model_list_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        try:
            future = self.get_model_list_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            self.get_logger().info(f'Model list: {response}')

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
            self.get_logger().info(f'Entity state: {response}')
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

    def timer_callback(self):
        self.scale *= -1.0
        self.get_logger().info('='*20)
        self.get_logger().info(f'Odom pose: x: {self.x}, y: {self.y}, theta: {self.theta}')
        self.get_logger().info(f'Model list: {self.model_list}, Obstacle list: {self.obstacle_list}')

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = msg.pose.pose.orientation.z


def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()