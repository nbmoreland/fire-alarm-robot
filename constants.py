from pybricks.parameters import Color

# General robot movement
WHEEL_DIAMETER = 56            # in mm, adjust to your robot's wheel diameter
AXLE_TRACK = 125               # in mm, distance between wheels
GRID_SIZE = 305                # in mm, if relevant for specific grid-based navigation

# Wandering behavior
WANDER_DISTANCE = 150          # distance to move during wandering, in mm
WANDER_TURN_ANGLE = 90         # angle to turn randomly during wandering, in degrees

# Wall-following behavior
WALL_FOLLOW_MIN_DISTANCE = 100  # minimum distance from wall, in mm
WALL_FOLLOW_MAX_DISTANCE = 200  # maximum distance from wall, in mm
WALL_FOLLOW_ADJUST_ANGLE = 10  # angle adjustment when following wall, in degrees
WALL_FOLLOW_STEP = 50          # step size to move forward while following wall, in mm

# Fire detection
FIRE_AREA_COLOR = Color.RED    # color of the paper below the candle
ALARM_FREQUENCY = 2000         # alarm beep frequency in Hz
ALARM_DURATION = 500           # alarm duration in ms

# Extinguish behavior
FAN_SPEED = 1000               # speed of fan motor to extinguish the fire
FAN_RUN_TIME = 3000            # time to run the fan, in ms

# Cooldown or delay after extinguishing fire
# in ms, optional cooldown time after extinguishing
COOLDOWN_AFTER_EXTINGUISH = 5000
