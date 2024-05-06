from settings.constparams import REWARD_FUNCTION, COLLISION, TUMBLE, SUCCESS, TIMEOUT, SPEED_LINEAR_MAX, THRESHOLD_COLLISION

class Reward():
    def __init__(self):
        self.distance_to_goal = 0
        self.angle_to_goal = 0


    def reward_initalize(self, init_distance_to_goal, init_angle_to_goal):
        # Update the initial distance and angle to goal
        self.distance_to_goal = init_distance_to_goal
        self.angle_to_goal = init_angle_to_goal

    def get_reward(self,
    status,  # Status of the robot (SUCCESS, COLLISION, TUMBLE, TIMEOUT)
    distance_to_goal,  # Distance to the goal
    angle_to_goal,  # Angle to the goal
    ):
        # Step reward for each action
        R_STEP = -10

        # Reward for the angle to the goal
        if angle_to_goal < self.angle_to_goal:
            R_ANGLE = 5
            # Update the angle to the goal
            self.angle_to_goal = angle_to_goal
        else:
            R_ANGLE = 0

        # Reward for the distance to the goal
        if distance_to_goal < self.distance_to_goal:
            R_DISTANCE = 5
            # Update the distance to the goal
            self.distance_to_goal = distance_to_goal
        else:
            R_DISTANCE = 0

        # Reward for status
        if status == SUCCESS:
            R_STATUS = 2500
        elif status == COLLISION:
            R_STATUS = -5000
        elif status == TUMBLE:
            R_STATUS = -5000
        elif status == TIMEOUT:
            R_STATUS = -1000
        else:
            R_STATUS = 0

        # Total reward
        reward = R_STEP + R_ANGLE + 2*R_DISTANCE + R_STATUS

        return float(reward)

    
