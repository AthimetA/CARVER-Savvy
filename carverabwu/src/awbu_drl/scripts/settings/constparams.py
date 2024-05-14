# ===================================================================== #
#                           GENERAL SETTINGS                            #
# ===================================================================== #

ENABLE_BACKWARD          = False    # Enable backward movement of the robot
ENABLE_STACKING          = False    # Enable processing multiple consecutive scan frames at every observation step
ENABLE_VISUAL            = True    # Meant to be used only during evaluation/testing phase
ENABLE_TRUE_RANDOM_GOALS = False    # If false, goals are selected semi-randomly from a list of known valid goal positions
ENABLE_DYNAMIC_GOALS     = True    # If true, goal difficulty (distance) is adapted according to current success rate
MODEL_STORE_INTERVAL     = 100      # Store the model weights every N episodes
GRAPH_DRAW_INTERVAL      = 10       # Draw the graph every N episodes (drawing too often will slow down training)
GRAPH_AVERAGE_REWARD     = 10       # Average the reward graph over every N episodes

# ===================================================================== #
#                             ROBOTS PARAMS                             #
# ===================================================================== #
NUM_SCAN_SAMPLES = 90


# ===================================================================== #
#                         ENVIRONMENT SETTINGS                          #
# ===================================================================== #

# --- SIMULATION ENVIRONMENT SETTINGS ---
REWARD_FUNCTION = "A"           # Defined in reward.py

# Sensor and Topic
TOPIC_SCAN = 'scan'
TOPIC_VELO = '/abwubot/cmd_vel'
TOPIC_ODOM = '/abwubot/odom'
TOPIC_CLOCK = '/clock'
TOPIC_OBSTACLES_ODOM = '/abwubot/obstacleCP'

LIDAR_DISTANCE_CAP          = 12.0   # meters
THRESHOLD_COLLISION         = 0.50  # meters
THREHSOLD_GOAL              = 1.0  # meters

ENABLE_MOTOR_NOISE          = False # Add normally distributed noise to motor output to simulate hardware imperfections
LINEAR_VELOCITY_LOC       = 0     # 0 = no noise, 1 = noise enabled
ANGULAR_VELOCITY_LOC      = 1     # 0 = no noise, 1 = noise enabled

# Arena
ARENA_LENGTH                = 20   # meters
ARENA_WIDTH                 = 20   # meters

# General
EPISODE_TIMEOUT_SECONDS     = 40    # Number of seconds after which episode timeout occurs
SPEED_LINEAR_MAX            = 2.0  # m/s
SPEED_ANGULAR_MAX           = 2.0   # rad/s

# Obstacles
OBSTACLE_RADIUS             = 0.16  # meters
MAX_NUMBER_OBSTACLES        = 6

DYNAMIC_GOAL_SEPARATION_DISTANCE_INIT         = 10.00  # meters
DYNAMIC_GOAL_SEPARATION_DISTANCE_MIN          = 5.00  # meters

# ===================================================================== #
#                       DRL ALGORITHM SETTINGS                          #
# ===================================================================== #

# DRL parameters
ACTION_SIZE     = 2         # Not used for DQN, see DQN_ACTION_SIZE
HIDDEN_SIZE     = 128       # Number of neurons in hidden layers

BATCH_SIZE      = 128       # Number of samples per training batch
BUFFER_SIZE     = BATCH_SIZE * 200 # Number of samples stored in replay buffer before FIFO
DISCOUNT_FACTOR = 0.99
LEARNING_RATE   = 0.001
TAU             = 0.010

OBSERVE_STEPS   = BATCH_SIZE * 10 # At training start random actions are taken for N steps for better exploration
STEP_TIME       = 0.01      # Delay between steps, can be set to 0
# EPSILON_DECAY   = 0.9995    # Epsilon decay per step
EPSILON_DECAY   = 0.995    # Epsilon decay per step
EPSILON_MINIMUM = 0.10

# DQN parameters
DQN_ACTION_SIZE = 5
TARGET_UPDATE_FREQUENCY = 1000

# DDPG parameters

# TD3 parameters
# POLICY_NOISE            = 0.4
# POLICY_NOISE_CLIP       = 0.6
POLICY_NOISE            = 0.2
POLICY_NOISE_CLIP       = 0.4
POLICY_UPDATE_FREQUENCY = 2

# Stacking
STACK_DEPTH = 3             # Number of subsequent frames processed per step
FRAME_SKIP  = 4             # Number of frames skipped in between subsequent frames

# Episode outcome enumeration
UNKNOWN = 0
SUCCESS = 1
COLLISION = 2
TIMEOUT = 3
TUMBLE = 4
RESULTS_NUM = 5


# --- REAL ROBOT ENVIRONMENT SETTINGS ---
REAL_TOPIC_SCAN  = 'scan'
REAL_TOPIC_VELO  = 'cmd_vel'
REAL_TOPIC_ODOM  = 'odom'

# LiDAR density count your robot is providing
# NOTE: If you change this value you also have to modify
# NUM_SCAN_SAMPLES for the model in drl_environment.py
# e.g. if you increase this by 320 samples also increase
# NUM_SCAN_SAMPLES by 320 samples.
REAL_N_SCAN_SAMPLES         = 40

REAL_ARENA_LENGTH           = 4.2   # meters
REAL_ARENA_WIDTH            = 4.2   # meters
REAL_SPEED_LINEAR_MAX       = 2.0  # in m/s
REAL_SPEED_ANGULAR_MAX      = 2.0   # in rad/s

REAL_LIDAR_CORRECTION       = 0.40  # meters, subtracted from the real LiDAR values
REAL_LIDAR_DISTANCE_CAP     = 3.5   # meters, scan distances are capped this value
REAL_THRESHOLD_COLLISION    = 0.11  # meters, minimum distance to an object that counts as a collision
REAL_THRESHOLD_GOAL         = 0.35  # meters, minimum distance to goal that counts as reaching the goal