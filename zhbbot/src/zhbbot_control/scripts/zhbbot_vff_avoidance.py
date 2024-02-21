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

from zhbbot_interfaces.srv import ZhbbotSendPath, ZhbbotSetNodeStaus

from nav_msgs.msg import Odometry

class ZhbbotVFFNode(Node):
    # Constructor of the class
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__(node_name='ZhbbotVFFNode')

        '''
        
        Staic Parameters
        
        '''
        self.__verbose = False

        # Qos profile for the publisher and subscriber
        self.__qos_profile = 10

        # Set lookahead distance for the Pure Pursuit algorithm
        self.__LOOKAHEAD_DISTANCE = 1.0
        # Set the threshold to determine if the goal is reached
        self.__GOAL_THRESHOLD = 0.80
        # VFF controller parameters
        self.__OBSTACLE_DISTANCE = 0.80 # Threshold distance for obstacle influence
        self.__GAIN = 2.5  # Gain for the attractive vector

        # Constants for visualization colors
        self.__RED = 0
        self.__GREEN = 1
        self.__BLUE = 2

        '''
        
        General Variables
        
        '''

        # Create an action server for ZhbbotSendPath service
        self.send_path_service_server = self.create_service(ZhbbotSendPath, '/zhbbot_service/send_path', self.send_path_callback)
        # Initialize path and current pose index
        self.path = None
        self.current_pose_index = 0


        '''
        
        Subscribers, Publishers

        '''
        # Create a subscription to the laser scan topic
        self.create_subscription(LaserScan, "/scan", self.laser_scan_callback, self.__qos_profile)
        # Variable to store the latest laser scan data
        self.laser_scan = None

        # Publisher for ik controller
        self.velocity_publisher_ik = self.create_publisher(Twist, '/diff_drive_zhbbot', self.__qos_profile)

        # Create a subscription to the robot's odometry topic
        self.odom_ekf_read = self.create_subscription(Odometry, '/odometry/local', self.odom_ekf_callback, 10)  
        self.robot_pose = Pose()

        '''
        
        Service Clients and Servers

        '''

        self.node_status = "DISABLED" # SLEEP, ACTIVE, DISABLED

        self._node_name = "ZhbbotVFFNode"

        self.set_node_status_service = self.create_service(ZhbbotSetNodeStaus,
                                                            f'/zhbbot_service/{self._node_name}/set_node_status',
                                                              self.set_node_status_callback)

        self.get_logger().info(f'ZhbbotVFFNode.py started with node name: {self._node_name}')


        '''
        
        Timer
        
        '''
        # Create a timer to periodically run the pure pursuit controller method
        self.__timer_hz = 20
        self.create_timer(1.0 / self.__timer_hz, self.timer_callback)

        '''
        
        Marker Visualization
        
        '''
        # Publisher for lookahead marker
        self.lookahead_marker_publisher = self.create_publisher(Marker, 'zhbbot/lookahead_marker', self.__qos_profile)
        # Publisher for visualization markers
        self.vff_marker_pub = self.create_publisher(MarkerArray, "zhbbot/vff_marker", self.__qos_profile)



    def reset_node(self):
        # Reset the path and current pose index
        self.path = None
        self.current_pose_index = 0
        # Reset the laser scan data
        self.laser_scan = None

    def set_node_status_callback(self, request: ZhbbotSetNodeStaus.Request, response: ZhbbotSetNodeStaus.Response):
        # Request to set the node status
        self.node_status = request.node_status
        # Response to the request
        response.node_name = self._node_name
        response.call_back_status = self.node_status
        return response

    '''
    
    Calulation Functions

    '''

    def timer_callback(self):
        if self.node_status == 'ENABLED':
            if self.path is not None and self.current_pose_index < len(self.path):
                self.get_logger().info(f'current_pose_index: {self.current_pose_index}/{len(self.path)}')
                # Get the current target pose from the path
                current_pose = self.path[self.current_pose_index]
                # Retrieve the robot's current pose
                robot_pose = self.get_robot_pose()
                if robot_pose is not None:
                    # Calculate the goal point based on the robot's pose and the path
                    goal_point = self.calculate_goal_point(self.path, robot_pose, self.current_pose_index)
                    if goal_point is not None:
                        # Vff controller
                        linear_vel, angular_vel = self.vff_controller(goal_point, robot_pose)
                        # Publish the velocity commands
                        self.publish_velocity(linear_vel, angular_vel)
                        # Check if the current target pose is reached
                        if self.is_goal_reached(robot_pose, current_pose):
                            self.current_pose_index += 1
        elif self.node_status == 'DISABLED':
            self.vff_marker_pub.publish(MarkerArray())
            self.reset_node()

    # Method to check if the goal is reached
    def is_goal_reached(self, robot_pose, goal_pose):
        return self.distance_between_points(robot_pose.position, goal_pose.pose.position) <= self.__GOAL_THRESHOLD

    # Method to get the current pose of the robot using TF2 transformations
    def get_robot_pose(self):
        return self.robot_pose

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
        
    # Method to calculate the goal point based on the lookahead distance
    def calculate_goal_point(self, path, robot_pose, start_index):
        for i in range(start_index, len(path)):
            if self.distance_between_points(robot_pose.position, path[i].pose.position) >= self.__LOOKAHEAD_DISTANCE:
                self.lookahead_marker_publisher.publish(self._get_marker_lookahead(robot_pose))
                return path[i]
        return None

    # The main controller logic for VFF-based obstacle avoidance
    def vff_controller(self, goal_point, robot_pose):
        # Only proceed if laser scan data is available
        if self.laser_scan != None:
            
            # Calculate the VFF based on the current laser scan
            # Extract the resultant vector for calculating velocity commands
            vff_vectors = self.get_vff(self.laser_scan, goal_point, robot_pose)

            v = vff_vectors['result']

            # Publish the VFF vectors as visualization markers
            self.vff_marker_pub.publish(self._get_vff_marker(vff_vectors, robot_pose, goal_point))
            
            # Calculate the linear and angular velocities based on the VFF vectors
            max_linear_velocity = 0.50  # Maximum linear velocity
            max_angular_velocity = 0.60  # Maximum angular velocity
            # Calculate the angle to the goal point
            angle = np.arctan2(v[1], v[0])

            # Convert the robot's orientation from quaternion to Euler angles
            _, _, theta = tf_transformations.euler_from_quaternion([robot_pose.orientation.x,
                                                                robot_pose.orientation.y,
                                                                robot_pose.orientation.z,
                                                                robot_pose.orientation.w])
            
            # Calculate the heading error
            heading_error = self.normalize_angle(angle - theta)
            
            # Calculate the linear and angular velocities
            linear_velocity = max_linear_velocity * (1 - abs(heading_error))
            angular_velocity = max_angular_velocity * heading_error

            return linear_velocity, angular_velocity
        
        else:
            self.get_logger().info('Laser scan data not available')
            return 0.0, 0.0
        
    # Calculate the Virtual Force Field based on laser scan data
    def get_vff(self, scan, goal_point, robot_pose):
        
        # Constants for the VFF algorithm
        # Calculate the attractive vector based on the goal point and the robot's pose and robot's orientation

        _, _, theta = tf_transformations.euler_from_quaternion([robot_pose.orientation.x,
                                                                robot_pose.orientation.y,
                                                                robot_pose.orientation.z,
                                                                robot_pose.orientation.w])

        # Initialize the VFF vectors
        vff_vector = {'attractive': [goal_point.pose.position.x - robot_pose.position.x, goal_point.pose.position.y - robot_pose.position.y],  # Goal-directed vector
                      'repulsive': [0.0, 0.0],  # Obstacle-repulsive vector
                      'result': [0.0, 0.0]}
        # Find the nearest obstacle
        dist_nearest_angle = np.argmin(scan.ranges)
        dist_nearest = scan.ranges[dist_nearest_angle]

        # If the nearest obstacle is within the influence threshold, calculate the repulsive vector
        # if distance_min < OBSTACLE_DISTANCE:
        if dist_nearest <= self.__OBSTACLE_DISTANCE:
            print(f'Obstacle detected at {dist_nearest}m')
            # Opposite direction to the obstacle
            # Convert to Cartesian coordinates
            e = (scan.angle_min + theta) + (scan.angle_increment * dist_nearest_angle)
            obstacle_angle =  np.arctan2(np.sin(e), np.cos(e))
            opposite_angle = obstacle_angle - math.pi
            obstacle_dist = self.__OBSTACLE_DISTANCE - dist_nearest
            vff_vector['repulsive'][0] = math.cos(opposite_angle) * obstacle_dist * self.__GAIN
            vff_vector['repulsive'][1] = math.sin(opposite_angle) * obstacle_dist * self.__GAIN

        # Calculate the resultant vector by combining attractive and repulsive vectors
        vff_vector['result'][0] = (vff_vector['repulsive'][0] + vff_vector['attractive'][0])
        vff_vector['result'][1] = (vff_vector['repulsive'][1] + vff_vector['attractive'][1])

        return vff_vector

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

    '''
    
    Start msg callback functions
    
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

    '''
    
    Marker Functions
    
    '''

    def _get_marker_lookahead(self, robot_pose):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = robot_pose.position.x
        marker.pose.position.y = robot_pose.position.y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.__LOOKAHEAD_DISTANCE * 2
        marker.scale.y = self.__LOOKAHEAD_DISTANCE * 2
        marker.scale.z = 0.1
        marker.color.a = 0.25
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        return marker
    
    # Utility function to create a marker for visualization
    def _make_marker(self, vector_star, vector_stop, vff_color):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        # Define the start and end points of the marker
        start = Point(x=vector_star[0], y=vector_star[1], z=0.0)
        end = Point(x=vector_stop[0], y=vector_stop[1], z=0.0)
        marker.points = [start, end]
        # Set the scale of the marker
        marker.scale.x = 0.05
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        # Set the color of the marker based on the type of vector
        color = ColorRGBA(a=1.0)  # Fully opaque
        if vff_color == self.__RED:
            marker.id = 0
            color.r = 1.0
        elif vff_color == self.__GREEN:
            marker.id = 1
            color.g = 1.0
        elif vff_color == self.__BLUE:
            marker.id = 2
            color.b = 1.0
        marker.color = color
        return marker
    
    # Generate visualization markers for the VFF vectors
    def _get_vff_marker(self, vff_vectors, robot_pose, goal_point):
        marker_array = MarkerArray()
        # Create and add markers for attractive, repulsive, and resultant vectors

        # Attractive vector
        attractive_vector_start = [robot_pose.position.x, robot_pose.position.y]
        attractive_vector_end = [goal_point.pose.position.x , goal_point.pose.position.y]

        marker_array.markers.append(self._make_marker(attractive_vector_start, attractive_vector_end, self.__BLUE))

        # Repulsive vector
        repulsive_vector_start = [robot_pose.position.x, robot_pose.position.y]
        repulsive_vector_end = [robot_pose.position.x + vff_vectors['repulsive'][0], robot_pose.position.y + vff_vectors['repulsive'][1]]
        marker_array.markers.append(self._make_marker(repulsive_vector_start, repulsive_vector_end, self.__RED))

        # Resultant vector
        resultant_vector_start = [robot_pose.position.x, robot_pose.position.y]
        resultant_vector_end = [robot_pose.position.x + vff_vectors['result'][0], robot_pose.position.y + vff_vectors['result'][1]]
        marker_array.markers.append(self._make_marker(resultant_vector_start, resultant_vector_end, self.__GREEN))

        return marker_array

    '''
    
    Start Utility Functions
    
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
    node = ZhbbotVFFNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
