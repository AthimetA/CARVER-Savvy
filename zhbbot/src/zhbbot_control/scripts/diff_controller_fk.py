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

class DifferentialDriveTransform(Node):
    # Constructor of the class
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__('diffential_drive_transform')

        # Create a subscriber for the Joint State Publisher
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.joint_states_buffer = JointState()

        # Create a publisher for robot velocity commands
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10) # publish to /odom topic

        # create timer_callback
        self.create_timer(1/20, self.differential_drive_inverse_kinematic)

        # Create a TransformBroadcaster
        self.odom_broadcaster = tf2_ros.TransformBroadcaster(self)

        # init parameters
        self.x = 0.0
        self.y = 0.0
        self.wz = 0.0

    def timer_callback(self):
        self.differential_drive_inverse_kinematic(self.joint_states_buffer)

    def joint_states_callback(self, msg:JointState):
        # Create a Twist message to subscribe to the diff drive controller
        self.joint_states_buffer = msg

    def differential_drive_inverse_kinematic(self, joint_states:JointState):
        now = self.get_clock().now()

        # Create a Float64MultiArray message to publish the velocity commands
        wl = joint_states.velocity[0]
        wr = joint_states.velocity[1]
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

        transform_stamped_msg = TransformStamped()
        transform_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        transform_stamped_msg.header.frame_id = 'base_link'
        transform_stamped_msg.child_frame_id = 'odom'
        transform_stamped_msg.transform.translation.x = self.x
        transform_stamped_msg.transform.translation.y = self.y
        transform_stamped_msg.transform.translation.z = 0.0
        transform_stamped_msg.transform.rotation.x = quaternion.x
        transform_stamped_msg.transform.rotation.y = quaternion.y
        transform_stamped_msg.transform.rotation.z = quaternion.z
        transform_stamped_msg.transform.rotation.w = quaternion.w

        self.odom_broadcaster.sendTransform(transform_stamped_msg)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

# Main function to initialize and run the ROS 2 node
def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDriveTransform()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()