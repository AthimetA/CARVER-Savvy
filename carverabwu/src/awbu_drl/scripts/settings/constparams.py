# ===================================================================== #
#                           GENERAL SETTINGS                            #
# ===================================================================== #

ENABLE_BACKWARD          = False    # Enable backward movement of the robot
ENABLE_STACKING          = False    # Enable processing multiple consecutive scan frames at every observation step
ENABLE_VISUAL            = True    # Meant to be used only during evaluation/testing phase
ENABLE_TRUE_RANDOM_GOALS = False    # If false, goals are selected semi-randomly from a list of known valid goal positions
ENABLE_DYNAMIC_GOALS     = False    # If true, goal difficulty (distance) is adapted according to current success rate
MODEL_STORE_INTERVAL     = 100      # Store the model weights every N episodes
GRAPH_DRAW_INTERVAL      = 10       # Draw the graph every N episodes (drawing too often will slow down training)
GRAPH_AVERAGE_REWARD     = 10       # Average the reward graph over every N episodes

# ===================================================================== #
#                             ROBOTS PARAMS                             #
# ===================================================================== #
NUM_SCAN_SAMPLES = 180

# ===================================================================== #
#                         ENVIRONMENT SETTINGS                          #
# ===================================================================== #
# Srv name
SRV_RESET_OBSTACLES_CP = '/reset_obstacle_cp'
SRV_ENV_COMM = '/env_comm'
SRV_STEP_COMM = '/step_comm'
SRV_SCORE_STEP_COMM = '/score_step_comm'
SRV_OBSTACLE_START = '/obstacle_start'

# Sensor and Topic
TOPIC_SCAN = '/scan'
TOPIC_VELO = '/abwubot/cmd_vel'
# TOPIC_ODOM = '/abwubot/odom'
TOPIC_ODOM = '/carversavvy/odom'
TOPIC_CLOCK = '/clock'
TOPIC_OBSTACLES_ODOM = '/abwubot/obstacleCP'
TOPIC_OBSTACLE_VISUAL_RAW = 'ObstacleVis_raw'
TOPIC_OBSTACLE_VISUAL_CP = 'ObstacleVis_cp'

LIDAR_DISTANCE_CAP          = 8.0   # meters
THRESHOLD_COLLISION         = 0.5  # meters
THREHSOLD_GOAL              = 0.5  # meters

ENABLE_MOTOR_NOISE          = False # Add normally distributed noise to motor output to simulate hardware imperfections
LINEAR_VELOCITY_LOC       = 0     # 0 = no noise, 1 = noise enabled
ANGULAR_VELOCITY_LOC      = 1     # 0 = no noise, 1 = noise enabled

# Arena
ARENA_LENGTH                = 20   # meters
ARENA_WIDTH                 = 20   # meters

# General
EPISODE_TIMEOUT_SECONDS     = 20    # Number of seconds after which episode timeout occurs
SPEED_LINEAR_MAX            = 2.0  # m/s
SPEED_ANGULAR_MAX           = 2.0   # rad/s

# Obstacles
OBSTACLE_RADIUS             = 0.16  # meters
MAX_NUMBER_OBSTACLES        = 6

DYNAMIC_GOAL_SEPARATION_DISTANCE_INIT         = 7.00  # meters
DYNAMIC_GOAL_SEPARATION_DISTANCE_MIN          = 5.00  # meters
DYNAMIC_GOAL_SEPARATION_DISTANCE_MAX          = 10.00  # meters

# ===================================================================== #
#                       DRL ALGORITHM SETTINGS                          #
# ===================================================================== #

# DRL parameters
ACTION_SIZE     = 2         # Not used for DQN, see DQN_ACTION_SIZE
HIDDEN_SIZE     = 256       # Number of neurons in hidden layers

BATCH_SIZE      = 128       # Number of samples per training batch
BUFFER_SIZE     = BATCH_SIZE * 1000 # Number of samples stored in replay buffer before FIFO
DISCOUNT_FACTOR = 0.99
LEARNING_RATE   = 1e-4
LEARNING_RATE_ACTOR = 1e-4
LEARNING_RATE_CRITIC = 1e-4
TAU             = 0.05

OBSERVE_STEPS   = BATCH_SIZE * 10 # At training start random actions are taken for N steps for better exploration
EPSILON_INITIAL = 0.99       # Initial epsilon value for epsilon-greedy policy
EPSILON_DECAY   = 0.9995    # Epsilon decay per step
EPSILON_MINIMUM = 0.20

# TD3 parameters
POLICY_NOISE            = 0.2
POLICY_NOISE_CLIP       = 0.4
POLICY_UPDATE_FREQUENCY = 2

# Episode outcome enumeration
UNKNOWN = 0
SUCCESS = 1
COLLISION = 2
TIMEOUT = 3
TUMBLE = 4
RESULTS_NUM = 5


# --- REAL ROBOT ENVIRONMENT SETTINGS ---
SRV_USER_SET_GOAL = '/abwu_drl_set_goal'

REAL_SPEED_LINEAR_MAX       = 0.1   # in m/s
REAL_SPEED_ANGULAR_MAX      = 0.1   # in rad/s

REAL_LIDAR_DISTANCE_CAP     = 8.0   # meters
REAL_THRESHOLD_COLLISION    = 0.5  # meters
REAL_THRESHOLD_GOAL         = 0.5   # meters

REAL_EPISODE_TIMEOUT_SECONDS = 5*60  # Number of seconds after which episode timeout occurs