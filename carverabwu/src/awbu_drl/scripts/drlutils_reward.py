from settings.constparams import REWARD_FUNCTION, COLLISION, TUMBLE, SUCCESS, TIMEOUT, SPEED_LINEAR_MAX, THRESHOLD_COLLISION, SPEED_ANGULAR_MAX
import numpy as np

class Reward():
    def __init__(self):
        self.distance_to_goal = 0
        self.angle_to_goal = 0

        self.initial_distance_to_goal = 0
        self.waypoint_reached = False

    def reward_initalize(self, init_distance_to_goal, init_angle_to_goal):
        # Update the initial distance and angle to goal
        self.distance_to_goal = init_distance_to_goal
        self.angle_to_goal = init_angle_to_goal

        self.initial_distance_to_goal = init_distance_to_goal
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
    
    def get_reward(self,
    status,  # Status of the robot (SUCCESS, COLLISION, TUMBLE, TIMEOUT)
    distance_to_goal,  # Distance to the goal
    angle_to_goal,  # Angle to the goal
    omega,  # Angular velocity
    ):
        # Step reward for each action
        R_STEP = -5

        # Reward for the angle to the goal
        #[-3.14, 0]
        R_ANGLE = -1 * abs(angle_to_goal)

        # Reward for the distance to the goal
        if distance_to_goal < self.distance_to_goal:
            R_DISTANCE = np.abs(self.distance_to_goal - distance_to_goal) * 100
        else:
            R_DISTANCE = -1 * np.abs(self.distance_to_goal - distance_to_goal) * 100

        self.distance_to_goal = distance_to_goal

        # Reward for the angular velocity
        # Penalty for angular velocity to prevent spinning in place
        # [-SPEED_ANGULAR_MAX, 0]
        R_OMEGA = -1 * np.abs(omega) * 2

        # Waypoint reward
        if not self.waypoint_reached and distance_to_goal < self.initial_distance_to_goal * 0.5:
            self.waypoint_reached = True
            R_WAYPOINT = 1000
        else:
            R_WAYPOINT = 0

        # Reward for status
        if status == SUCCESS:
            R_STATUS = 2500
        elif status == COLLISION:
            R_STATUS = -2500
        elif status == TIMEOUT:
            R_STATUS = -1000
        else:
            R_STATUS = 0

        # Total reward
        reward = R_STEP + R_ANGLE + R_DISTANCE + R_OMEGA + R_STATUS + R_WAYPOINT


        return float(reward) , [R_STEP, R_ANGLE, R_DISTANCE, R_OMEGA, R_STATUS, R_WAYPOINT]

    
