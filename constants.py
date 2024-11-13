from pybricks.parameters import Color

# General robot movement
WHEEL_DIAMETER = 56            # in mm, adjust to your robot's wheel diameter
AXLE_TRACK = 114               # in mm, distance between wheels
BACKUP_DISTANCE = 30           # distance to back up when obstacle is detected, in mm

# Wandering behavior
WANDER_TURN_ANGLE = 90         # angle to turn randomly during wandering, in degrees
WANDER_DRIVE_SPEED = 120       # forward drive speed while wandering, in mm/s

# Wall-following behavior
WALL_DETECT_THRESHOLD = 100    # distance threshold for detecting a wall, in mm
WALL_FOLLOW_TURN_ANGLE = 90    # angle to turn when wall-following, in degrees
WALL_FOLLOW_SPEED = 120        # forward drive speed while wall-following, in mm/s

# Fire detection alarm
ALARM_FREQUENCY = 2000         # alarm beep frequency in Hz
ALARM_DURATION = 500           # alarm duration in ms

# Extinguish behavior
FAN_SPEED = 2000               # speed of fan motor to extinguish the fire
FAN_RUN_TIME = 5000            # time to run the fan, in ms

# Goal Detection (Red Color Threshold)
GOAL_COLOR = Color.RED
