#!/usr/bin/python3

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
from nav_msgs.msg import Odometry
import numpy as np
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from zhbbot_interfaces.srv import RobotSentgoal, Goalreach

class DiffDriveDynamicWindowApproach(Node):
    # Constructor of the class
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__('DiffDriveDynamicWindowApproach')

        # Create a subscription to the laser scan topic
        self.create_subscription(LaserScan, "/scan", self.laser_scan_callback, 10)
        # Variable to store the latest laser scan data
        self.laser_scan = None

        # Create a subscription for the current pose of the robot
        self.odom_buffer = Odometry()
        self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 10)

        # Create a timer to control the control loop
        self.timer_callback_loop = self.create_timer(1/10, self.timer_callback)

        # Create a publisher for robot velocity commands
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel_zhbbot', 10) # publish to /cmd_vel_zhbbot topic

        
        self.goal_radius = 0.5

        self.min_speed = -0.00
        self.max_speed = 0.40
        self.min_rot_speed = -np.pi
        self.max_rot_speed = np.pi
        # self.goal = [5.76, 2.84]
        self.goal = [7.2,-1.29]


        '''
        
        Edit
        
        '''

        # Create an action client for ComputePathToPose to get the path for the robot
        self.action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        self.path = None

    # Timer callback for the control loop
    def timer_callback(self):
        # Get the best velocity command
        best_velocity = self.select_best_trajectory(self.goal)

        msg = Twist()
        msg.linear.x = best_velocity[0]
        msg.angular.z = best_velocity[1]
        self.velocity_publisher.publish(msg)
        
    # Callback for processing laser scan messages
    def laser_scan_callback(self, msg):
        # Store the laser scan data for use in the controller
        self.laser_scan = msg

    # Callback for processing odometry messages
    def odom_callback(self, msg):
        # Store the odometry data for use in the controller
        self.odom_buffer = msg

    def score_trajectory(self, new_x, new_y, goal, goal_distance):
        # Calculate distance to goal and Normalize the distance to goal
        distance_diff = (np.sqrt((goal[0]-new_x)**2 + (goal[1]-new_y)**2))/goal_distance
        # Calculate obstacle difference
        obstacle_diff = self.obstacle_diff(new_x, new_y)
        # Calculate total score
        total_score = - distance_diff + obstacle_diff
        return total_score
    
    def obstacle_diff(self, new_x, new_y):
        # Initialize obstacle difference to 1 (range 0 to 1)
        obstacle_diff = 1.0
        obstacle_max_distance = 1.0
        
        # Get the nearest obstacle distance and angle
        nearest_obstacle_angle = np.argmin(self.laser_scan.ranges)

        dist_nearest = self.laser_scan.ranges[nearest_obstacle_angle]

        if dist_nearest < obstacle_max_distance:
            obstacle_angle = self.laser_scan.angle_min + (self.laser_scan.angle_increment * nearest_obstacle_angle)
            obstacle_x = self.laser_scan.ranges[nearest_obstacle_angle] * math.cos(obstacle_angle)
            obstacle_y = self.laser_scan.ranges[nearest_obstacle_angle] * math.sin(obstacle_angle)
            obstacle_diff = math.sqrt((obstacle_x-new_x)**2 + (obstacle_y-new_y)**2)

            return obstacle_diff
        else:
            return obstacle_diff

    def select_best_trajectory(self, goal):

        if self.laser_scan is None:
            return [0.0, 0.0]
        else:
            # Get the current position and orientation of the robot
            x = self.odom_buffer.pose.pose.position.x
            y = self.odom_buffer.pose.pose.position.y
            quaternion = (self.odom_buffer.pose.pose.orientation.x, self.odom_buffer.pose.pose.orientation.y, self.odom_buffer.pose.pose.orientation.z, self.odom_buffer.pose.pose.orientation.w)
            _, _, theta = tf_transformations.euler_from_quaternion(quaternion)
            # --------------------------------------------
            # Initialize best score to negative infinity
            best_score = float('-inf')

            # Calculate distance to goal
            goal_distance = math.sqrt((goal[0]-x)**2 + (goal[1]-y)**2)
            
            # Calculate best velocity command
            while goal_distance > self.goal_radius:
                for linear_speed in np.arange(self.min_speed, self.max_speed, 0.02):
                    for rot_speed in np.arange(self.min_rot_speed, self.max_rot_speed, 0.05):
                        # Simulate trajectory
                        new_x = x + linear_speed * math.cos(rot_speed + theta)
                        new_y = y + linear_speed * math.sin(rot_speed + theta)

                        # Score trajectory
                        score = self.score_trajectory(new_x, new_y, goal, goal_distance)

                        if score > best_score:
                            best_score = score
                            best_velocity = [linear_speed, rot_speed]

                return best_velocity
            
            # If the goal is reached, stop the robot
            return [0.0, 0.0]
        
    # Method to send a navigation goal to the ComputePathToPose action server
    def send_goal(self, pose):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = pose
        self.action_client.wait_for_server()
        self.future = self.action_client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)

    # Callback for handling the response from the ComputePathToPose action server
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    # Callback for handling the path result from the ComputePathToPose action server
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Path received')

        self.follow_path(result.path.poses)

    # Method to store the received path for following
    def follow_path(self, path):
        self.path = path

# Main function to initialize and run the ROS 2 node
def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveDynamicWindowApproach()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()