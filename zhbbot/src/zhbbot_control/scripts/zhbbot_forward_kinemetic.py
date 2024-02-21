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
from geometry_msgs.msg import Pose, TransformStamped, Quaternion
import numpy as np
from sensor_msgs.msg import LaserScan, JointState
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry

from zhbbot_interfaces.srv import RobotSentgoal, Goalreach

# import all other neccesary libraries
import sys

class ZhbbotFKNode(Node):
    # Constructor of the class
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__('ZhbbotFKNode')

        self.node_name = 'ZhbbotFKNode'
        self.node_enabled = True

        self.joint_states_buffer = JointState()
        # Create a subscriber for the Joint State Publisher
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)

        # Create a publisher for robot velocity commands
        self.odom_topic_name = 'zhbbot_wheel/odom'
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic_name, 10) # publish to /odom topic

        # create timer_callback
        self.timer_hz = 10
        self.create_timer(1.0 / self.timer_hz, self.timer_callback)

        # init parameters
        self.x = 0.0
        self.y = 0.0
        self.wz = 0.0

    def timer_callback(self):
        if self.node_enabled:
            try:
                self.forward_kinematic_cal(self.joint_states_buffer)
            except:
                self.get_logger().error('Did not receive joint_states yet')

    def joint_states_callback(self, msg:JointState):
        # Create a Twist message to subscribe to the diff drive controller
        self.joint_states_buffer = msg

    def forward_kinematic_cal(self, joint_states:JointState):
        now = self.get_clock().now()

        # Create a Float64MultiArray message to publish the velocity commands
        # Get the wheel velocities from the joint_states
        velocities = np.array(joint_states.velocity).astype(np.float64)

        wl = velocities[0]
        wr = velocities[1]
        r = 0.075                           # radius of the wheel
        base_width = 0.4                             # distance between the wheels

        vx = (r/2) * (wl + wr)
        wz = (r/base_width) * (wr - wl)

        # d_left = joint_states.position[0]
        # d_right = joint_states.position[1]

        # # distance traveled is the average of the two wheels 
        # d = (d_left + d_right) / 2
        # # this approximation works (in radians) for small angles
        # th = (d_right - d_left) / base_width

        if vx != 0:
            # calculate distance traveled in x and y
            x = np.cos(wz) * vx
            y = -np.sin(wz) * vx
            # calculate the final position of the robot
            self.x = self.x + (np.cos(self.wz) * x - np.sin(self.wz) * y)
            self.y = self.y + (np.sin(self.wz) * x + np.cos(self.wz) * y)
        if wz != 0:
            self.wz = self.wz + wz

        # publish the odom information
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = np.sin(self.wz / 2)
        quaternion.w = np.cos(self.wz / 2)

        # Create an Odometry message to publish the odometry information
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = "base_footprint"
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = wz
        odom.pose.covariance = [ 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
        odom.twist.covariance = [ 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
        self.odom_pub.publish(odom)

# Main function to initialize and run the ROS 2 node
def main(args=None):
    rclpy.init(args=args)
    node = ZhbbotFKNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()