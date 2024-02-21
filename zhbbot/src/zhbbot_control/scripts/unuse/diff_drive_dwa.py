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
        self.timer_callback_loop = self.create_timer(1/40, self.timer_callback)

        # Create a publisher for robot velocity commands
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel_zhbbot', 10) # publish to /cmd_vel_zhbbot topic

        
        self.goal_radius = 0.5

        self.min_speed = -0.00
        self.max_speed = 0.40
        self.min_rot_speed = -3.0
        self.max_rot_speed = 3.0
        # self.xxxgoal = [5.76, 2.84]
        # self.xxxgoal = [7.2,-1.29]
        self.xxxgoal = [5.43,-4.84]


        '''
        
        Edit
        
        '''

        # Create an action client for ComputePathToPose to get the path for the robot
        self.action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        self.path = None
        self.current_pose_index = 0
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    # Timer callback for the control loop
    def timer_callback(self):
        # # Get the best velocity command
        # best_velocity = self.select_best_trajectory(self.goal)

        # msg = Twist()
        # msg.linear.x = best_velocity[0]
        # msg.angular.z = best_velocity[1]
        # self.velocity_publisher.publish(msg)

        # Method implementing the Pure Pursuit control logic
        # self.get_logger().info('Timer callback')
        if self.path is not None and self.current_pose_index < len(self.path):
            self.get_logger().info(f'Current pose index: {self.current_pose_index}/{len(self.path)}')
            # Get the current target pose from the path
            goal_pose = self.path[self.current_pose_index]
            # Retrieve the robot's current pose
            best_velocity = self.select_best_trajectory([goal_pose.pose.position.x, goal_pose.pose.position.y])
            # Calculate the distance to the current target pose
            self.publish_velocity(best_velocity[0], best_velocity[1])

    def publish_velocity(self, linear_vel, angular_vel):
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
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
            # x = self.odom_buffer.pose.pose.position.x
            # y = self.odom_buffer.pose.pose.position.y
            # quaternion = (self.odom_buffer.pose.pose.orientation.x, self.odom_buffer.pose.pose.orientation.y, self.odom_buffer.pose.pose.orientation.z, self.odom_buffer.pose.pose.orientation.w)
            # _, _, theta = tf_transformations.euler_from_quaternion(quaternion)
            x, y, theta = self.get_robot_pose()

            # --------------------------------------------
            # Initialize best score to negative infinity
            best_score = float('-inf')

            # Calculate distance to goal
            goal_distance = math.sqrt((goal[0]-x)**2 + (goal[1]-y)**2)
            
            # Calculate best velocity command
            while goal_distance > self.goal_radius:
                for linear_speed in np.arange(self.min_speed, self.max_speed, 0.05):
                    for rot_speed in np.arange(self.min_rot_speed, self.max_rot_speed, 0.10):
                        # Simulate trajectory
                        new_x = x + linear_speed * math.cos(rot_speed + theta)
                        new_y = y + linear_speed * math.sin(rot_speed + theta)

                        # Score trajectory
                        score = self.score_trajectory(new_x, new_y, goal, goal_distance)

                        if score > best_score:
                            best_score = score
                            best_velocity = [linear_speed, rot_speed]
                self.get_logger().info(f'Best score: {best_score}, Linear speed: {best_velocity[0]}, Rotational speed: {best_velocity[1]}')
                return best_velocity
            
            # If the goal is reached, stop the robot
            self.current_pose_index += 1
            return [0.0, 0.0]
        
    # Method to send a navigation goal to the ComputePathToPose action server
    def send_goal(self, pose):
        gp= PoseStamped()
        gp.header.frame_id = "map"
        gp.pose.position.x = pose[0]
        gp.pose.position.y = pose[1]

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = gp
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
        self.current_pose_index = 0

    # Method to get the current pose of the robot using TF2 transformations
    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose = Pose()
            pose.position.x = trans.transform.translation.x
            pose.position.y = trans.transform.translation.y
            pose.position.z = trans.transform.translation.z
            pose.orientation = trans.transform.rotation

            x = pose.position.x
            y = pose.position.y
            quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            _, _, theta = tf_transformations.euler_from_quaternion(quaternion)

            return x, y, theta
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error('Could not transform from base_link to map: %s' % str(e))
            x = self.odom_buffer.pose.pose.position.x
            y = self.odom_buffer.pose.pose.position.y
            quaternion = (self.odom_buffer.pose.pose.orientation.x, self.odom_buffer.pose.pose.orientation.y, self.odom_buffer.pose.pose.orientation.z, self.odom_buffer.pose.pose.orientation.w)
            _, _, theta = tf_transformations.euler_from_quaternion(quaternion)
            return x , y, theta

# Main function to initialize and run the ROS 2 node
def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveDynamicWindowApproach()
    node.send_goal(node.xxxgoal)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()