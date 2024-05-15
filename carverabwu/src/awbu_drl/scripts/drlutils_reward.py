from settings.constparams import REWARD_FUNCTION, COLLISION, TUMBLE, SUCCESS, TIMEOUT,\
      SPEED_LINEAR_MAX, THRESHOLD_COLLISION, SPEED_ANGULAR_MAX, NUM_SCAN_SAMPLES
import numpy as np
from env_utils import get_simulation_speed, read_stage, bcolors

class Reward():
    def __init__(self):
        self.distance_to_goal = 0
        self.angle_to_goal = 0

        self.initial_distance_to_goal = 0
        self.initial_angle_to_goal = 0
        self.waypoint_list = []
        self.waypoint_idx = 0
        self.waypoint_reached = False

        self.action_linear_prev = 0
        self.action_angular_prev = 0

    def reward_initalize(self, init_distance_to_goal, init_angle_to_goal):
        # Update the initial distance and angle to goal
        self.distance_to_goal = init_distance_to_goal
        self.angle_to_goal = init_angle_to_goal

        self.initial_distance_to_goal = init_distance_to_goal
        self.initial_angle_to_goal = init_angle_to_goal

        self.action_linear_prev = 0
        self.action_angular_prev = 0

        # Calculate the waypoints (initial distance to goal / 5) in reverse order
        self.waypoint_list = [init_distance_to_goal - i * (init_distance_to_goal / 5) for i in range(1, 5)]
        self.waypoint_idx = 0
        self.waypoint_reached = False


    # def get_reward(self,
    # status,  # Status of the robot (SUCCESS, COLLISION, TUMBLE, TIMEOUT)
    # distance_to_goal,  # Distance to the goal
    # angle_to_goal,  # Angle to the goal
    # ):
    #     # Step reward for each action
    #     R_STEP = -4

    #     # Reward for the angle to the goal
    #     if angle_to_goal < self.angle_to_goal:
    #         R_ANGLE = 1
    #     else:
    #         R_ANGLE = 0

    #     # Update the angle to the goal
    #     self.angle_to_goal = angle_to_goal

    #     # Reward for the distance to the goal
    #     if distance_to_goal < self.distance_to_goal:
    #         R_DISTANCE = 1
    #     else:
    #         R_DISTANCE = 0

    #     # Update the distance to the goal
    #     self.distance_to_goal = distance_to_goal

    #     # Reward for status
    #     if status == SUCCESS:
    #         R_STATUS = 2500
    #     elif status == COLLISION:
    #         R_STATUS = -2500
    #     elif status == TIMEOUT:
    #         R_STATUS = -1000
    #     else:
    #         R_STATUS = 0

    #     # Total reward
    #     reward = R_STEP + R_ANGLE + 2*R_DISTANCE + R_STATUS

    #     print(f"R_STEP: {R_STEP}, R_ANGLE: {R_ANGLE}, R_DISTANCE: {R_DISTANCE}, R_STATUS: {R_STATUS}, reward: {reward}")

    #     return float(reward)

    # def get_reward(self,
    # status,  # Status of the robot (SUCCESS, COLLISION, TUMBLE, TIMEOUT)
    # action_linear,  # Linear velocity
    # action_angular,  # Angular velocity
    # distance_to_goal,  # Distance to the goal
    # angle_to_goal,  # Angle to the goal
    # omega,  # Angular velocity
    # ):
    #     # Step reward for each action
    #     R_STEP = - 2

    #     # # Reward for Change in linear velocity
    #     # R_LINEAR = - 1 * abs(action_linear - self.action_linear_prev)

    #     # self.action_linear_prev = action_linear

    #     # # Reward for Change in angular velocity
    #     # R_ANGULAR = - 1 * abs(action_angular - self.action_angular_prev)
    #     # self.action_angular_prev = action_angular

    #     # Reward for the angle to the goal
    #     # #[-3.14, 0]
    #     R_ANGLE = -1 * abs(angle_to_goal)

    #     # Reward for the distance to the goal
    #     R_DISTANCE = -1 * abs(distance_to_goal)

    #     # Reward for the distance to the goal
    #     # if distance_to_goal < self.distance_to_goal:
    #     #     R_DISTANCE = np.abs(self.distance_to_goal - distance_to_goal) * 200

    #     #     self.distance_to_goal = distance_to_goal

    #     # else:
    #     #     # R_DISTANCE = -1 * np.abs(self.distance_to_goal - distance_to_goal) * 200
    #     #     R_DISTANCE = 0


    #     # Reward for the angular velocity
    #     # Penalty for angular velocity to prevent spinning in place
    #     # [-SPEED_ANGULAR_MAX, 0]
    #     # R_OMEGA = -1 * np.abs(omega)

    #     # Waypoint reward
    #     if not self.waypoint_reached and distance_to_goal < self.initial_distance_to_goal * 0.5:
    #         self.waypoint_reached = True
    #         R_WAYPOINT = 50
    #     else:
    #         R_WAYPOINT = 0

    #     # Reward for status
    #     if status == SUCCESS:
    #         R_STATUS = 250
    #     elif status == COLLISION:
    #         R_STATUS = -500
    #     elif status == TIMEOUT:
    #         R_STATUS = -250
    #     else:
    #         R_STATUS = 0

    #     # Total reward
    #     reward = R_STEP + R_ANGLE + R_DISTANCE + R_STATUS + R_WAYPOINT 


    #     return float(reward) , [R_DISTANCE, R_ANGLE, R_WAYPOINT]

    def get_reward(self,
    status,  # Status of the robot (SUCCESS, COLLISION, TUMBLE, TIMEOUT)
    action_linear,  # Linear velocity
    action_angular,  # Angular velocity
    distance_to_goal,  # Distance to the goal
    angle_to_goal,  # Angle to the goal
    omega,  # Angular velocity
    scan_ranges,  # lidar scan
    ):
        # Step reward for each action
        R_STEP = - 2
        # Reward for the angle to the goal
        # [-1, 0] # Angle to the goal (Normalized by 1/3.14)
        R_ANGLE = -1 * abs(angle_to_goal)

        # Reward for the distance to the goal
        # [-1, 0] # Distance to the goal(Normalized by MAX_DISTANCE)
        R_DISTANCE = -1 * abs(distance_to_goal)

        # Waypoint reward
        R_WAYPOINT = 0
        if not self.waypoint_reached:
            
            # If the distance to the goal is less than the waypoint distance
            if distance_to_goal < self.waypoint_list[self.waypoint_idx]:
                self.waypoint_idx += 1 # Move to the next waypoint
                R_WAYPOINT = 15

            # If the last waypoint is reached
            if self.waypoint_idx == 4:
                self.waypoint_reached = True

        # Reward for the scan
        R_FONT_SCAN, R_OTHER_SCAN = self.get_reward_from_scan(scan_ranges)

        # Reward for status
        if status == SUCCESS:
            R_STATUS = 250
        elif status == COLLISION:
            R_STATUS = -500
        elif status == TIMEOUT:
            R_STATUS = -150
        else:
            R_STATUS = 0

        # Total reward
        reward = R_STEP + R_ANGLE + R_DISTANCE + R_STATUS + R_WAYPOINT + R_FONT_SCAN + R_OTHER_SCAN


        return float(reward) , [R_DISTANCE, R_ANGLE, R_WAYPOINT, R_FONT_SCAN, R_OTHER_SCAN]
    
    def get_reward_from_scan(self, scan_ranges):

        # R_FONT_SCAN + R_OTHER_SCAN should be in the range of [-4, 0] 
        # The Threshold is R_FONT_SCAN = -3, R_OTHER_SCAN = -1

        DEG_PER_SCAN = 360 / NUM_SCAN_SAMPLES

        font_angle_fov = 45 # degrees

        front_scan_start_index = int((180 - font_angle_fov) / DEG_PER_SCAN)
        front_scan_end_index = int((180 + font_angle_fov) / DEG_PER_SCAN)

        front_scan = np.asarray(scan_ranges[front_scan_start_index:front_scan_end_index])

        R_FONT_SCAN_MAX = len(front_scan) # Maximum value of R_FONT_SCAN --> all the values are -1

        R_FONT_SCAN = np.sum((front_scan - 1.0)) / R_FONT_SCAN_MAX

        other_scan = np.asarray(scan_ranges[:front_scan_start_index] + scan_ranges[front_scan_end_index:])

        R_OTHER_SCAN_MAX = len(other_scan) # Maximum value of R_OTHER_SCAN --> all the values are -1

        R_OTHER_SCAN = np.sum((other_scan - 1.0)) / R_OTHER_SCAN_MAX

        # Scaling the reward

        R_FONT_SCAN = R_FONT_SCAN * 3

        R_OTHER_SCAN = R_OTHER_SCAN * 1

        return R_FONT_SCAN, R_OTHER_SCAN