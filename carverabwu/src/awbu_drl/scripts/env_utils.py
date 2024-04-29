import os
import numpy as np
import xml.etree.ElementTree as ET

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

COLLITION_MARGIN = 0.5 # Margin to be added to the obstacles to calculate the collision [m]

class ObstacleManager:
    def __init__(self, obstacle_name: list = ['wall_outler', 'pillar_1', 'pillar_2', 'wall_inner_1']):
        self.obstacle_name = obstacle_name
        self.obstacle_coordinates = self.get_obstacle_coordinates(self.obstacle_name)

    '''
    
    Obstacle functions

    '''

    def get_obstacle_coordinates(self, obstacle_name: list )->list:
        obstacle_coordinates = []
        for name in obstacle_name:
            obstacle_coordinates += self._sdf_obstacle_reader(name)
        return obstacle_coordinates
    
    def _sdf_obstacle_reader(self, name: str)->list:
        path = os.environ['SIM_MODEL_PATH'] + name + '/model.sdf'
        tree = ET.parse(path)
        root = tree.getroot()
        obstacle_coordinates = []
        # Get the coordinates of the walls
        for wall in root.find('model').findall('link'):
            pose = wall.find('pose').text.split(" ")
            size = wall.find('collision').find('geometry').find('box').find('size').text.split()
            pose_x = float(pose[0])
            pose_y = float(pose[1])
            # Check if the wall is rotated
            # If the wall is rotated the size is swapped for x and y
            rotation = float(pose[-1])
            if rotation == 0 or rotation == 3.14159:
                size_x = float(size[0]) + COLLITION_MARGIN * 2
                size_y = float(size[1]) + COLLITION_MARGIN * 2
            else:
                size_x = float(size[1]) + COLLITION_MARGIN * 2
                size_y = float(size[0]) + COLLITION_MARGIN * 2
            # Calculate the corners of the obstacle
            step_x = size_x / 2
            step_y = size_y / 2
            top_left = [pose_x - step_x, pose_y + step_y]
            top_right = [pose_x + step_x, pose_y + step_y]
            bottom_right = [pose_x + step_x, pose_y - step_y]
            bottom_left = [pose_x - step_x, pose_y - step_y]
            # Create a list of the corners
            wall_points = [top_right, bottom_right, bottom_left, top_left]
            obstacle_coordinates.append(wall_points)
        return obstacle_coordinates
    
class Robot:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # --------------- Goal --------------- #
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.distance_to_goal = 0.0
        self.heading_to_goal = 0.0
        self.goal_angle = 0.0

        # --------------- Previous Values --------------- #
        self.prev_x = 0.0
        self.prev_y = 0.0

        # --------------- Computational --------------- #
        self.distance_traveled = 0.0

    def update_goal(self, goal_x: float, goal_y: float):
        self.goal_x = goal_x
        self.goal_y = goal_y

    def reset(self):
        # Reset the robot position
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.distance_traveled = 0.0
        self.prev_x = 0.0
        self.prev_y = 0.0
        
        self.distance_to_goal = self.calculate_distance(target_x=self.goal_x, target_y=self.goal_y, x=self.x, y=self.y)
        self.heading_to_goal = self.calculate_heading(target_x=self.goal_x, target_y=self.goal_y, x=self.x, y=self.y)
        self.goal_angle = self.calculate_angle(target_angle=self.heading_to_goal, angle=self.theta)

    def update_position(self, msg: Odometry):
        # Update the robot position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        _, _, self.theta = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.linear_velocity = msg.twist.twist.linear.x
        self.angular_velocity = msg.twist.twist.angular.z

        # Update the distance traveled
        self.distance_traveled += np.sqrt((self.x - self.prev_x)**2 + (self.y - self.prev_y)**2)
        self.prev_x = self.x
        self.prev_y = self.y

        # Update the goal information
        self.distance_to_goal = self.calculate_distance(target_x=self.goal_x, target_y=self.goal_y, x=self.x, y=self.y)
        self.heading_to_goal = self.calculate_heading(target_x=self.goal_x, target_y=self.goal_y, x=self.x, y=self.y)
        self.goal_angle = self.calculate_angle(target_angle=self.heading_to_goal, angle=self.theta)

    def calculate_distance(self, target_x: float, target_y: float, x: float, y: float):
        return np.sqrt((target_x - x)**2 + (target_y - y)**2)
    
    def calculate_heading(self, target_x: float, target_y: float, x: float, y: float):
        return np.arctan2(target_y - y, target_x - x)
    
    def calculate_angle(self, target_angle: float, angle: float):
        angle = target_angle - angle
        # Normalize the angle
        if angle > np.pi:
            # If the angle is greater than pi, subtract 2pi
            angle -= 2 * np.pi
        if angle < -np.pi:
            # If the angle is less than -pi, add 2pi
            angle += 2 * np.pi
        return angle

    def euler_from_quaternion(self, quat: Quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quat = [x, y, z, w]
        """
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w*x + y*z)
        cosr_cosp = 1 - 2*(x*x + y*y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w*y - z*x)
        if sinp < -1:
            sinp = -1
        if sinp > 1:
            sinp = 1
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w*z + x*y)
        cosy_cosp = 1 - 2 * (y*y + z*z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw