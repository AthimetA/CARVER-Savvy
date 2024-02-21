#!/usr/bin/python3

# Import necessary ROS 2 and other Python libraries
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped, Twist, Point
import tf_transformations
import math
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import Pose, TransformStamped
import numpy as np
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Float64MultiArray

from nav_msgs.msg import Odometry
from zhbbot_interfaces.srv import ZhbbotSetNodeStaus

class ZhbbotIKNode(Node):
    # Constructor of the class
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__('ZhbbotIKNode')

        # Create a subscriber for the Diff Drive Publisher
        self.create_subscription(Twist, '/diff_drive_zhbbot', self.diff_drive_cont_sub_callback, 10)
        self.diff_drive_velocity = Twist()

        # Create a publisher for robot velocity commands
        self.velocity_cont_pub = self.create_publisher(Float64MultiArray, '/velocity_cont/commands', 10) # publish to /cmd_vel_zhbbot topic
        self.cont_timer = 10
        self.create_timer(1 / self.cont_timer, self.velocity_cont_timer_callback)


        '''
        
        Set Node Status
        
        '''
        self.node_status = "DISABLED" # SLEEP, ACTIVE, DISABLED

        self._node_name = "ZhbbotIKNode"

        self.set_node_status_service = self.create_service(ZhbbotSetNodeStaus,
                                                            f'/zhbbot_service/{self._node_name}/set_node_status',
                                                              self.set_node_status_callback)

        self.get_logger().info(f'zhbbot_inverse_kinemetic.py started with node name: {self._node_name}')

    def set_node_status_callback(self, request: ZhbbotSetNodeStaus.Request, response: ZhbbotSetNodeStaus.Response):
        # Request to set the node status
        self.node_status = request.node_status
        # Response to the request
        response.node_name = self._node_name
        response.call_back_status = self.node_status
        return response

    def velocity_cont_timer_callback(self):
        if self.node_status == 'ENABLED':
            # Create a Float64MultiArray message to publish the velocity commands
            velocity_cont_msg = Float64MultiArray()
            velocity_cont_msg.data = self.velocity_controller_inverse_kinematics(self.diff_drive_velocity)
            # Publish the velocity commands
            self.velocity_cont_pub.publish(velocity_cont_msg)
        else:
            # Create a Float64MultiArray message to publish the velocity commands
            velocity_cont_msg = Float64MultiArray()
            velocity_cont_msg.data = [0.0, 0.0]
            # Publish the velocity commands
            self.velocity_cont_pub.publish(velocity_cont_msg)

    def diff_drive_cont_sub_callback(self, msg):
        # Create a Twist message to subscribe to the diff drive controller
        self.diff_drive_velocity.linear.x = msg.linear.x
        self.diff_drive_velocity.angular.z = msg.angular.z

    def velocity_controller_inverse_kinematics(self, diff_drive_velocity):
        # Create a Float64MultiArray message to publish the velocity commands
        vx = diff_drive_velocity.linear.x   # linear velocity
        w = diff_drive_velocity.angular.z   # angular velocity
        r = 0.075                           # radius of the wheel
        d = 0.4                             # distance between the wheels

        wl = (vx / r) - ((w * d) / (2 * r))
        wr = (vx / r) + ((w * d) / (2 * r))

        return wl, wr

# Main function to initialize and run the ROS 2 node
def main(args=None):
    rclpy.init(args=args)
    node = ZhbbotIKNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
