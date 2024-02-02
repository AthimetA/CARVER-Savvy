#!/usr/bin/python3
# Import necessary ROS 2 and other Python libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import math
import numpy as np

# Define a class for implementing VFF Avoidance using ROS 2
class VFF_Avoidance(Node):
    # Constructor of the class
    def __init__(self):
        # Initialize the ROS 2 node with a name
        super().__init__('vff_avoidance')
        # Create a subscription to the laser scan topic
        self.create_subscription(LaserScan, "/scan", self.laser_scan_callback, 10)
        # Create a timer to regularly invoke the VFF controller logic
        self.create_timer(0.05, self.vff_controller)
        # Publisher for robot velocity commands
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(MarkerArray, "marker_debug", 10)
        # Variable to store the latest laser scan data
        self.laser_scan = None
        # Constants for visualization colors
        self.RED = 0
        self.GREEN = 1
        self.BLUE = 2

    # Callback for processing laser scan messages
    def laser_scan_callback(self, msg):
        # Store the laser scan data for use in the controller
        self.laser_scan = msg
        self.get_vff(self.laser_scan)
    
    # Generate visualization markers for the VFF vectors
    def get_debug_vff(self, vff_vectors):
        marker_array = MarkerArray()
        # Create and add markers for attractive, repulsive, and resultant vectors
        marker_array.markers.append(self.make_marker(vff_vectors['attractive'], self.BLUE))
        marker_array.markers.append(self.make_marker(vff_vectors['repulsive'], self.RED))
        marker_array.markers.append(self.make_marker(vff_vectors['result'], self.GREEN))
        return marker_array

    # Utility function to create a marker for visualization
    def make_marker(self, vector, vff_color):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        # Define the start and end points of the marker
        start = Point(x=0.0, y=0.0, z=0.0)
        end = Point(x=vector[0], y=vector[1], z=0.0)
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

    # The main controller logic for VFF-based obstacle avoidance
    def vff_controller(self):
        # Only proceed if laser scan data is available
        if self.laser_scan != None:
            
            # Calculate the VFF based on the current laser scan
            # Extract the resultant vector for calculating velocity commands
            # Create the velocity command message
            # Publish the velocity command
            # Publish visualization markers

            pass
    
    # Calculate the Virtual Force Field based on laser scan data
    def get_vff(self, scan):
        OBSTACLE_DISTANCE = 2.00  # Threshold distance for obstacle influence
        # Initialize the VFF vectors
        vff_vector = {'attractive': [OBSTACLE_DISTANCE, 0.0],  # Goal-directed vector
                      'repulsive': [0.0, 0.0],  # Obstacle avoidance vector
                      'result': [0.0, 0.0]}  # Combined vector
        # Find the nearest obstacle
        dist_nearest_angle = np.argmin(scan.ranges)
        dist_nearest = scan.ranges[dist_nearest_angle]

        print("min_idx = " + str(dist_nearest_angle) + "min_dist = " + str(dist_nearest))

        # If the nearest obstacle is within the influence threshold, calculate the repulsive vector
        # if distance_min < OBSTACLE_DISTANCE:
        if dist_nearest <= OBSTACLE_DISTANCE:

            # Opposite direction to the obstacle
            # Convert to Cartesian coordinates
            obstacle_angle = scan.angle_min + scan.angle_increment * dist_nearest_angle
            opposite_angle = obstacle_angle - math.pi
            obstacle_dist = OBSTACLE_DISTANCE - dist_nearest
            vff_vector.repulsive[0] = math.cos(opposite_angle) + obstacle_dist
            vff_vector.repulsive[1] = math.sin(opposite_angle) + obstacle_dist

        # Calculate the resultant vector by combining attractive and repulsive vectors
        vff_vector.result[0] = (vff_vector.repulsive[0] + vff_vector.attractive[0])
        vff_vector.result[1] = (vff_vector.repulsive[1] + vff_vector.attractive[1])
        
        return vff_vector

# Main function to execute the node
def main(args=None):
    rclpy.init(args=args)
    node = VFF_Avoidance()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
