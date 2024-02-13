#!/usr/bin/python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf2_ros
import tf_transformations


class TestNode(Node):
    def __init__(self):
        # Initialize the node with the name 'TestNodeA'
        super().__init__('TestNodeA')
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Hello World!')

def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()