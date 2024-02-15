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

class DifferentialDrivePurePursuitVFFAvoidance(Node):
    # Constructor of the class
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__('differential_drive_pure_pursuit_vff_avoidance')

        # Create an action client for ComputePathToPose to get the path for the robot
        self.action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        # Create a subscription to the laser scan topic
        self.create_subscription(LaserScan, "/scan", self.laser_scan_callback, 10)
        # Variable to store the latest laser scan data
        self.laser_scan = None

        # Create a publisher for robot velocity commands
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel_zhbbot', 10) # publish to /cmd_vel_zhbbot topic

        # Publisher for ik controller
        self.velocity_publisher_ik = self.create_publisher(Twist, '/diff_drive_zhbbot', 10) 

        # ----- Visualization ----- #
        # Publisher for lookahead marker
        self.lookahead_marker_publisher = self.create_publisher(Marker, '/lookahead_marker', 10)
        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(MarkerArray, "/vff_marker", 10)
        # Constants for visualization colors
        self.RED = 0
        self.GREEN = 1
        self.BLUE = 2

        # Set lookahead distance for the Pure Pursuit algorithm
        self.lookahead_distance = 1.0
        # Set the threshold to determine if the goal is reached
        self.goal_threshold = 0.5
        # Initialize TF2 buffer and listener for pose transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Create a timer to periodically run the pure pursuit controller method
        self.create_timer(0.05, self.pure_pursuit_controller)
        # Initialize path and current pose index
        self.path = None
        self.current_pose_index = 0

    # Method implementing the Pure Pursuit control logic
    def pure_pursuit_controller(self):
        if self.path is not None and self.current_pose_index < len(self.path):
            # Get the current target pose from the path
            current_pose = self.path[self.current_pose_index]
            # Retrieve the robot's current pose
            robot_pose = self.get_robot_pose()
            if robot_pose is not None:
                # Calculate the goal point based on the robot's pose and the path
                goal_point = self.calculate_goal_point(self.path, robot_pose, self.current_pose_index)
                if goal_point is not None:
                    print('-'*50)
                    print(f'Current index: {self.current_pose_index}/{len(self.path)}')
                    print(f'Goal point: {goal_point.pose.position.x}, {goal_point.pose.position.y}')
                    print(f'Robot pose: {robot_pose.position.x}, {robot_pose.position.y}')
                    # Vff controller
                    linear_vel, angular_vel = self.vff_controller(goal_point, robot_pose)
                    print(f'VFF linear velocity: {linear_vel}, angular velocity: {angular_vel}')
                    # Publish the velocity commands
                    self.publish_velocity(linear_vel, angular_vel)
                    # Check if the current target pose is reached
                    if self.is_goal_reached(robot_pose, current_pose):
                        self.current_pose_index += 1

    # Method to calculate the goal point based on the lookahead distance
    def calculate_goal_point(self, path, robot_pose, start_index):
        for i in range(start_index, len(path)):
            if self.distance_between_points(robot_pose.position, path[i].pose.position) >= self.lookahead_distance:
                self.lookahead_marker_publisher.publish(self.get_debug_lookahead(robot_pose))
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

            self.marker_pub.publish(self.get_debug_vff(vff_vectors, robot_pose, goal_point))
            
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
        
    # Calculate the Virtual Force Field based on laser scan data
    def get_vff(self, scan, goal_point, robot_pose):
        
        # Constants for the VFF algorithm
        gain = 1.5 # Gain for the attractive vector
        OBSTACLE_DISTANCE = 0.75  # Threshold distance for obstacle influence
        # Initialize the VFF vectors
        vff_vector = {'attractive': [goal_point.pose.position.x - robot_pose.position.x, goal_point.pose.position.y - robot_pose.position.y],  # Goal-directed vector
                      'repulsive': [0.0, 0.0],  # Obstacle-repulsive vector
                      'result': [0.0, 0.0]}
        # Find the nearest obstacle
        dist_nearest_angle = np.argmin(scan.ranges)
        dist_nearest = scan.ranges[dist_nearest_angle]

        # print("min_idx = " + str(dist_nearest_angle) + "min_dist = " + str(dist_nearest))

        # If the nearest obstacle is within the influence threshold, calculate the repulsive vector
        # if distance_min < OBSTACLE_DISTANCE:
        if dist_nearest <= OBSTACLE_DISTANCE:
            print(f'Obstacle detected at {dist_nearest}m')
            # Opposite direction to the obstacle
            # Convert to Cartesian coordinates
            obstacle_angle = scan.angle_min + scan.angle_increment * dist_nearest_angle
            opposite_angle = obstacle_angle - math.pi
            obstacle_dist = OBSTACLE_DISTANCE - dist_nearest
            vff_vector['repulsive'][0] = math.cos(opposite_angle) * obstacle_dist * gain
            vff_vector['repulsive'][1] = math.sin(opposite_angle) * obstacle_dist * gain

        # Calculate the resultant vector by combining attractive and repulsive vectors
        vff_vector['result'][0] = (vff_vector['repulsive'][0] + vff_vector['attractive'][0])
        vff_vector['result'][1] = (vff_vector['repulsive'][1] + vff_vector['attractive'][1])

        return vff_vector
    
    # Generate visualization markers for the VFF vectors
    def get_debug_vff(self, vff_vectors, robot_pose, goal_point):
        marker_array = MarkerArray()
        # Create and add markers for attractive, repulsive, and resultant vectors

        # Attractive vector
        attractive_vector_start = [robot_pose.position.x, robot_pose.position.y]
        attractive_vector_end = [goal_point.pose.position.x , goal_point.pose.position.y]

        marker_array.markers.append(self.make_marker(attractive_vector_start, attractive_vector_end, self.BLUE))

        # Repulsive vector
        repulsive_vector_start = [robot_pose.position.x, robot_pose.position.y]
        repulsive_vector_end = [robot_pose.position.x + vff_vectors['repulsive'][0], robot_pose.position.y + vff_vectors['repulsive'][1]]
        marker_array.markers.append(self.make_marker(repulsive_vector_start, repulsive_vector_end, self.RED))

        # Resultant vector
        resultant_vector_start = [robot_pose.position.x, robot_pose.position.y]
        resultant_vector_end = [robot_pose.position.x + vff_vectors['result'][0], robot_pose.position.y + vff_vectors['result'][1]]
        marker_array.markers.append(self.make_marker(resultant_vector_start, resultant_vector_end, self.GREEN))

        return marker_array

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

    
    # Callback for processing laser scan messages
    def laser_scan_callback(self, msg):
        # Store the laser scan data for use in the controller
        self.laser_scan = msg


    # Method to publish the calculated velocity commands
    def publish_velocity(self, linear_vel, angular_vel):
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.velocity_publisher.publish(twist)
        self.velocity_publisher_ik.publish(twist)

    # Method to check if the goal is reached
    def is_goal_reached(self, robot_pose, goal_pose):
        return self.distance_between_points(robot_pose.position, goal_pose.pose.position) <= self.goal_threshold

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

    # Method to get the current pose of the robot using TF2 transformations
    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose = Pose()
            pose.position.x = trans.transform.translation.x
            pose.position.y = trans.transform.translation.y
            pose.position.z = trans.transform.translation.z
            pose.orientation = trans.transform.rotation
            return pose
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error('Could not transform from base_link to map: %s' % str(e))
            return None
        
    def get_debug_lookahead(self, robot_pose):
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
        marker.scale.x = self.lookahead_distance * 2
        marker.scale.y = self.lookahead_distance * 2
        marker.scale.z = 0.1
        marker.color.a = 0.25
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        return marker
    
    # Utility function to create a marker for visualization
    def make_marker(self, vector_star, vector_stop, vff_color):
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
        if vff_color == self.RED:
            marker.id = 0
            color.r = 1.0
        elif vff_color == self.GREEN:
            marker.id = 1
            color.g = 1.0
        elif vff_color == self.BLUE:
            marker.id = 2
            color.b = 1.0
        marker.color = color
        return marker


# Main function to initialize and run the ROS 2 node
def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDrivePurePursuitVFFAvoidance()
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.pose.position.x = 4.0
    node.send_goal(goal_pose)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
