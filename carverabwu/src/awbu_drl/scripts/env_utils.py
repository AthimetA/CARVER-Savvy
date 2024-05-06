import os
import numpy as np
import random
import math
import xml.etree.ElementTree as ET

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

from settings.constparams import ARENA_LENGTH, ARENA_WIDTH, ENABLE_DYNAMIC_GOALS, ENABLE_TRUE_RANDOM_GOALS

from settings.constparams import SUCCESS, COLLISION, TIMEOUT, TUMBLE

COLLITION_MARGIN = 0.5 # Margin to be added to the obstacles to calculate the collision [m]

def get_simulation_speed(stage):
    tree = ET.parse(os.getenv('ABWUDRL_BASE_PATH') + '/src/abwu_gazebo/worlds/abwu_drl_stage_' + str(stage) + '.world')
    root = tree.getroot()
    return int(root.find('world').find('physics').find('real_time_factor').text)

def read_stage(stage=None):

    file_path = os.getenv('ABWUDRL_BASE_PATH') +'/tmp/abwu_current_stage.txt'

    if not os.path.exists(file_path):
        print("\033[1m" + "\033[93m" + "Stage file does not exist, creating a new one with default stage 1" + "\033[0m")
        # Create directory if it does not exist
        os.makedirs(os.path.dirname(file_path), exist_ok=True)
        # Create the file
        with open(file_path, 'w') as f:
            f.write('1')

    if stage is None:
        with open(os.getenv('ABWUDRL_BASE_PATH') +'/tmp/abwu_current_stage.txt', 'r') as f:
            stage = f.read()
        return int(stage)
    else:
        with open(os.getenv('ABWUDRL_BASE_PATH') +'/tmp/abwu_current_stage.txt', 'w') as f:
            f.write(str(stage))
        return stage
    
def translate_outcome(outcome):
    if outcome == SUCCESS:
        return "SUCCESS"
    elif outcome == COLLISION:
        return "COLLISION"
    elif outcome == TIMEOUT:
        return "TIMEOUT"
    elif outcome == TUMBLE:
        return "TUMBLE"
    else:
        return f"UNKNOWN: {outcome}"
    
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class GoalManager:
    def __init__(self, obstacle_name: list = ['wall_outler', 'pillar_1', 'pillar_2', 'wall_inner_1']):
        self.obstacle_name = obstacle_name
        self.obstacle_coordinates = self.get_obstacle_coordinates(self.obstacle_name)

        self.goal_x, self.goal_y = 0.0, 0.0

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
    
    '''
    
    Goal generation functions
    
    '''

    def goal_is_valid(self, goal_x: float, goal_y: float)->bool:
        if goal_x > ARENA_LENGTH/2 or goal_x < -ARENA_LENGTH/2 or goal_y > ARENA_WIDTH/2 or goal_y < -ARENA_WIDTH/2:
            return False
        for obstacle in self.obstacle_coordinates:
            # Obstacle is defined by 4 points [top_right, bottom_right, bottom_left, top_left] with [x, y] coordinates
            if goal_x < obstacle[0][0] and goal_x > obstacle[2][0] and goal_y < obstacle[0][1] and goal_y > obstacle[2][1]: # check if goal is inside the obstacle
                    return False
        return True

    def generate_goal_pose(self, robot_x: float, robot_y: float, radius: float)->None:
        MAX_ITERATIONS = 100
        GOAL_SEPARATION_DISTANCE = 5.0
        DYNAMIC_GOAL_RADIUS = float(radius) if radius > GOAL_SEPARATION_DISTANCE else GOAL_SEPARATION_DISTANCE
        PREDEFINED_GOAL_LOCATIONS = [[-(ARENA_LENGTH/2 - 1), -(ARENA_WIDTH/2 - 1)], [ARENA_LENGTH/2 - 1, ARENA_WIDTH/2 - 1],\
                                     [ARENA_LENGTH/2 - 1, -(ARENA_WIDTH/2 - 1)], [-(ARENA_LENGTH/2 - 1), ARENA_WIDTH/2 - 1],\
                                    #  [3.0,0.0]
                                     ]
        self.prev_goal_x = self.goal_x
        self.prev_goal_y = self.goal_y
        iterations = 0
        while iterations < MAX_ITERATIONS:
            # self.get_logger().info(f"Goal generation iteration: {iterations}")
            iterations += 1 # Prevent infinite loop
            if ENABLE_TRUE_RANDOM_GOALS:
                # Random goal generation within the arena
                goal_x = random.uniform(-ARENA_LENGTH/2, ARENA_LENGTH/2)
                goal_y = random.uniform(-ARENA_WIDTH/2, ARENA_WIDTH/2)
            elif ENABLE_DYNAMIC_GOALS:
                # Dynamic goal generation within a radius of the robot position
                goal_x = random.uniform(robot_x - DYNAMIC_GOAL_RADIUS, robot_x + DYNAMIC_GOAL_RADIUS)
                goal_y = random.uniform(robot_y - DYNAMIC_GOAL_RADIUS, robot_y + DYNAMIC_GOAL_RADIUS)
            else:
                # Get the goal from the predefined list
                index = random.randint(0, len(PREDEFINED_GOAL_LOCATIONS) - 1)
                goal_x = PREDEFINED_GOAL_LOCATIONS[index][0]
                goal_y = PREDEFINED_GOAL_LOCATIONS[index][1]

            # Check if the goal is valid and far enough from the previous goal
            if self.goal_is_valid(goal_x, goal_y) and math.sqrt((goal_x - self.prev_goal_x)**2 + (goal_y - self.prev_goal_y)**2) > GOAL_SEPARATION_DISTANCE:
                    break
            else:
                continue 
        if iterations >= MAX_ITERATIONS:
            goal_x = 0.0 # Default goal
            goal_y = 0.0 # Default goal
        # Set the goal pose
        self.goal_x = goal_x
        self.goal_y = goal_y

        return self.goal_x, self.goal_y
    
class Robot:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.row = 0.0
        self.pitch = 0.0
        self.theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # --------------- Goal --------------- #
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.distance_to_goal = 0.0
        self.angle_to_goal = 0.0
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
        self.angle_to_goal = self.calculate_heading(target_x=self.goal_x, target_y=self.goal_y, x=self.x, y=self.y)
        self.goal_angle = self.calculate_angle(target_angle=self.angle_to_goal, angle=self.theta)

    def update_position(self, msg: Odometry):
        # Update the robot position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.row, self.pitch, self.theta = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.linear_velocity = msg.twist.twist.linear.x
        self.angular_velocity = msg.twist.twist.angular.z

        # Update the distance traveled
        self.distance_traveled += np.sqrt((self.x - self.prev_x)**2 + (self.y - self.prev_y)**2)
        self.prev_x = self.x
        self.prev_y = self.y

        # Update the goal information
        self.distance_to_goal = self.calculate_distance(target_x=self.goal_x, target_y=self.goal_y, x=self.x, y=self.y)
        self.angle_to_goal = self.calculate_heading(target_x=self.goal_x, target_y=self.goal_y, x=self.x, y=self.y)
        self.goal_angle = self.calculate_angle(target_angle=self.angle_to_goal, angle=self.theta)

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