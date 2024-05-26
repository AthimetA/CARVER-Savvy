#!/usr/bin/python3

# Import necessary ROS 2 and other Python libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Point
import tf_transformations
import math
from geometry_msgs.msg import Pose, TransformStamped
import numpy as np
from sensor_msgs.msg import LaserScan

from zhbbot_interfaces.srv import ZhbbotSendPath, ZhbbotSetNodeStaus

from nav_msgs.msg import Odometry

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

class ZhbbotDWANode(Node):
    # Constructor of the class
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__(node_name='ZhbbotDWANode')

        '''
        
        Staic Parameters
        
        '''
        self.__verbose = False

        # Qos profile for the publisher and subscriber
        self.__qos_profile = 10

        self._GOAL_RADIUS = 0.25

        self._MIN_LINEAR_VELOCITY = 0.00
        self._MAX_LINEAR_VELOCITY = 0.60
        self._LINEAR_VELOCITY_INCREMENT = 0.02

        self._MIN_ANGULAR_VELOCITY = -3.0
        self._MAX_ANGULAR_VELOCITY = 3.0
        self._ANGULAR_VELOCITY_INCREMENT = 0.10


        self._OBTACLE_DIFF = 1.0
        self._OBSTACLE_MAX_DISTANCE = 1.0

        '''
        
        Subscribers, Publishers

        '''
        # Create a subscription to the laser scan topic
        self.create_subscription(LaserScan, "/scan", self.laser_scan_callback, self.__qos_profile)
        # Variable to store the latest laser scan data
        self.laser_scan = None

        # Publisher for ik controller
        self.velocity_publisher_ik = self.create_publisher(Twist, '/diff_drive_zhbbot', self.__qos_profile)
        self.best_velocity = [0.0, 0.0]

        # Create a subscription to the robot's odometry topic
        self.odom_ekf_read = self.create_subscription(Odometry, '/odometry/local', self.odom_ekf_callback, self.__qos_profile)
        self.robot_pose = Pose()

        '''
        
        Service Clients and Servers

        '''

        # Create a service to set the node status
        self.node_status = "DISABLED" # SLEEP, ACTIVE, DISABLED

        self._node_name = "ZhbbotDWANode"

        self.set_node_status_service = self.create_service(ZhbbotSetNodeStaus,
                                                            f'/zhbbot_service/{self._node_name}/set_node_status',
                                                              self.set_node_status_callback)

        self.get_logger().info(f'zhbbot_local_planer_dwa.py started with node name: {self._node_name}')


        # Create an action server for ZhbbotSendPath service
        self.send_path_service_server = self.create_service(ZhbbotSendPath, '/zhbbot_service/send_path', self.send_path_callback)
        # Initialize path and current pose index
        self.path = None
        self.current_pose_index = 0


        '''
        
        Timer
        
        '''
        # Create a timer to periodically run the pure pursuit controller method
        self.__timer_hz = 30
        self.create_timer(1.0 / self.__timer_hz, self.timer_callback)

    '''
    
    Calulation Functions

    '''
    # Controller method to calculate the robot's velocity commands
    def timer_callback(self):
        if self.node_status == 'ENABLED':
            if self.path is not None and self.current_pose_index < len(self.path):
                # self.get_logger().info(f'Current pose index: {self.current_pose_index}/{len(self.path)}')
                # Get the current target pose from the path
                goal_pose = self.path[self.current_pose_index]
                # Retrieve the robot's current pose
                best_velocity = self.select_best_trajectory([goal_pose.pose.position.x, goal_pose.pose.position.y])
                # Calculate the distance to the current target pose
                self.publish_velocity(best_velocity[0], best_velocity[1])
        elif self.node_status == 'DISABLED':
            self.reset_node()
    
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
        obstacle_diff = self._OBTACLE_DIFF
        obstacle_max_distance = self._OBSTACLE_MAX_DISTANCE
        
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
            self.get_logger().info('Laser scan data not available')
            return [0.0, 0.0]
        else:
            # Get the current position and orientation of the robot
            x = self.robot_pose.position.x
            y = self.robot_pose.position.y
            quaternion = (self.robot_pose.orientation.x, self.robot_pose.orientation.y, self.robot_pose.orientation.z, self.robot_pose.orientation.w)
            _, _, theta = tf_transformations.euler_from_quaternion(quaternion)

            # --------------------------------------------
            # Initialize best score to negative infinity
            best_score = float('-inf')

            # Calculate distance to goal
            goal_distance = math.sqrt((goal[0]-x)**2 + (goal[1]-y)**2)
            
            # Calculate best velocity command
            while goal_distance > self._GOAL_RADIUS:
                for linear_speed in np.arange(self._MIN_LINEAR_VELOCITY, self._MAX_LINEAR_VELOCITY, self._LINEAR_VELOCITY_INCREMENT):
                    for rot_speed in np.arange(self._MIN_ANGULAR_VELOCITY, self._MAX_ANGULAR_VELOCITY, self._ANGULAR_VELOCITY_INCREMENT):
                        # Simulate trajectory
                        new_x = x + linear_speed * math.cos(rot_speed + theta)
                        new_y = y + linear_speed * math.sin(rot_speed + theta)

                        # Score trajectory
                        score = self.score_trajectory(new_x, new_y, goal, goal_distance)

                        if score > best_score:
                            best_score = score
                            best_velocity = [linear_speed, rot_speed]
                # self.get_logger().info(f'Best score: {best_score}, Linear speed: {best_velocity[0]}, Rotational speed: {best_velocity[1]}')
                self.best_velocity = best_velocity
                return best_velocity
            
            # If the goal is reached, stop the robot
            self.current_pose_index += 3
            return self.best_velocity

    ''''
    
    Base Functions
    
    '''
    
    def reset_node(self):
        # Reset the path and current pose index
        self.path = None
        self.current_pose_index = 0
        # Reset the laser scan data
        self.laser_scan = None

    '''
    
    Service callback functions and Client functions

    '''

    # Service callback function to receive the path from the ZhbbotHandlerNode
    def send_path_callback(self, request: ZhbbotSendPath.Request, response: ZhbbotSendPath.Response):
        self.get_logger().info('Path received from ZhbbotHandlerNode')
        # Process the path and send the goal to the robot
        response.status = "ZhbbotVFFNode: Path received from ZhbbotHandlerNode"
        self.path = request.path.poses
        self.current_pose_index = 0
        return response
    
    # Service callback function to set the node status
    def set_node_status_callback(self, request: ZhbbotSetNodeStaus.Request, response: ZhbbotSetNodeStaus.Response):
        # Request to set the node status
        self.node_status = request.node_status
        # Response to the request
        response.node_name = self._node_name
        response.call_back_status = self.node_status
        return response

    '''
    
    Msg Functions
    
    '''

    # Callback for processing laser scan messages
    def laser_scan_callback(self, msg):
        # Store the laser scan data for use in the controller
        self.laser_scan = msg

    def publish_velocity(self, linear_vel, angular_vel):
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.velocity_publisher_ik.publish(twist)

    # Callback for processing odometry messages
    def odom_ekf_callback(self, msg:Odometry):
        pos = Pose()
        pos.position.x = msg.pose.pose.position.x
        pos.position.y = msg.pose.pose.position.y
        pos.position.z = msg.pose.pose.position.z
        pos.orientation.x = msg.pose.pose.orientation.x
        pos.orientation.y = msg.pose.pose.orientation.y
        pos.orientation.z = msg.pose.pose.orientation.z
        pos.orientation.w = msg.pose.pose.orientation.w
        self.robot_pose = pos

    '''
    
    Utility Functions
    
    '''

    # Utility method to calculate the Euclidean distance between two points
    def distance_between_points(self, point1, point2):
        return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

    # Utility method to normalize an angle to the range [-pi, pi]
    def normalize_angle(self, angle):
        if angle > math.pi:
            angle -= 2.0 * math.pi
        if angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    # Utility method to print verbose messages
    def verbose_print(self, *args, **kwargs):
        if self.__verbose:
            print(*args, **kwargs)

# Main function to initialize and run the ROS 2 node
def main(args=None):
    rclpy.init(args=args)
    node = ZhbbotDWANode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
