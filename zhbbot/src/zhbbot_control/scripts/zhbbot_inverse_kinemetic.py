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
from zhbbot_interfaces.srv import RobotSentgoal, Goalreach

class DifferentialDrivetoVelocityController(Node):
    # Constructor of the class
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__('differential_drive_to_velocity_controller')
        self.get_logger().info('Differential Drive to Velocity Controller Node Initialized')
        self.node_enabled = False

        # Create a subscriber for the Diff Drive Publisher
        self.create_subscription(Twist, '/diff_drive_zhbbot', self.diff_drive_cont_sub_callback, 10)
        self.diff_drive_velocity = Twist()

        # Create a publisher for robot velocity commands
        self.velocity_cont_pub = self.create_publisher(Float64MultiArray, '/velocity_cont/commands', 10) # publish to /cmd_vel_zhbbot topic
        self.create_timer(0.05, self.velocity_cont_timer_callback)


        self.odom_ekf_read = self.create_subscription(Odometry, '/odometry/local', self.odom_ekf_callback, 10)
        self.x = 0.0
        self.y = 0.0
        self.error_range = 1.0

        # Action server to get the goal pose
        self.robot_sent_goal_service_server = self.create_service(RobotSentgoal, 'zhbbot/robot_sent_goal', self.robot_sent_goal_callback)
        self.goal_x = 0.0
        self.goal_y = 0.0

        # Action client to send the goal pose
        self.goal_reach_client = self.create_client(Goalreach, 'zhbbot/goal_reach')

    def odom_ekf_callback(self, msg:Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = tf_transformations.euler_from_quaternion(quaternion)
        self.wz = euler[2]
        print(f'x: {self.x}, y: {self.y}, wz: {self.wz}')

    # Callback function for the robot_sent_goal service
    # Receives the goal pose from the action client and stores it in the class variables
    def robot_sent_goal_callback(self, request, response):
        self.goal_x = request.goal_x
        self.goal_y = request.goal_y
        response.status = 'Foward Kinematic: Goal received'
        self.get_logger().info(f'Goal received: {self.goal_x}, {self.goal_y}')
        self.get_logger().info(f'Current position: {self.x}, {self.y}')
        self.get_logger().info(f'IK node: Enabled')
        self.node_enabled = True
        return response

    def velocity_cont_timer_callback(self):
        if self.node_enabled:
            self.get_logger().info(f'Current position: {self.x}, {self.y}')
            # Create a Float64MultiArray message to publish the velocity commands
            velocity_cont_msg = Float64MultiArray()
            velocity_cont_msg.data = self.velocity_controller_inverse_kinematics(self.diff_drive_velocity)
            # Publish the velocity commands
            self.velocity_cont_pub.publish(velocity_cont_msg)

            if (abs(self.x - self.goal_x) < self.error_range) and (abs(self.y - self.goal_y) < self.error_range):
                self.node_enabled = False
                self.get_logger().info(f'Goal reached: {self.x}, {self.y}')
                self.get_logger().info(f'IK node: Disabled')
                self.get_logger().info(f'Goal sent to the action server')
        else:
            self.velocity_cont_pub.publish(Float64MultiArray(data=[0.0, 0.0]))

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
    node = DifferentialDrivetoVelocityController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
