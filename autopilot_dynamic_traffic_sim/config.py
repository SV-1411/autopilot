SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
FPS = 120

GRID_CELL_SIZE = 20

AGENT_RADIUS = 10
AGENT_MAX_SPEED = 200.0

BACKGROUND_COLOR = (20, 20, 20)
GRID_LINE_COLOR = (35, 35, 35)
AGENT_COLOR = (0, 200, 0)
AGENT_HEADING_COLOR = (0, 255, 255)
OBSTACLE_COLOR = (200, 50, 50)
PATH_COLOR = (50, 150, 250)
DWA_TRAJ_COLOR = (255, 200, 0)
START_COLOR = (250, 220, 70)
GOAL_COLOR = (180, 80, 250)

WAYPOINT_TOLERANCE_PX = 15.0

REPLAN_INTERVAL = 0.1
REPLAN_PREDICT_TIME = 0.35

MOVING_OBSTACLE_COUNT = 8
MOVING_OBSTACLE_RADIUS = 10.0
MOVING_OBSTACLE_MIN_SPEED = 40.0
MOVING_OBSTACLE_MAX_SPEED = 120.0

DENSE_STATIC_MODE = False
DENSE_STATIC_FILL_PROB = 0.45
DENSE_MOVING_OBSTACLE_COUNT = 40

DYNAMIC_BLOCK_PADDING_CELLS = 1

USE_DWA = True
STATIC_OBSTACLE_SAMPLE_RADIUS_CELLS = 8

DWA_CONFIG = {
    "max_speed": 180.0,
    "min_speed": 0.0,
    "max_yaw_rate": 2.5,
    "max_accel": 260.0,
    "max_delta_yaw_rate": 6.0,
    "v_resolution": 15.0,
    "yaw_rate_resolution": 0.3,
    "dt": 0.1,
    "predict_time": 1.5,
    "to_goal_cost_gain": 1.0,
    "speed_cost_gain": 0.2,
    "obstacle_cost_gain": 1.5,
}

PATH_SMOOTHING = True
PATH_SMOOTHING_MAX_SKIP = 40

CSV_EXPORT = True
CSV_FILENAME = "trajectory_log.csv"

# Controller for the live sim: "dwa" (A* + dynamic window) or "rl" (trained policy).
CONTROLLER = "dwa"
RL_MODEL_PATH = "runs/models/best.zip"
# These must match how the policy was trained (see rl/train.py).
RL_OBSTACLE_COUNT = 8
RL_OBSTACLE_SPEED_MIN = 20.0
RL_OBSTACLE_SPEED_MAX = 60.0
RL_GOAL_RADIUS = 28.0
