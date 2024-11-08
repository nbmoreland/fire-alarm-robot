from pybricks.robotics import DriveBase
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Direction
from pybricks.hubs import PrimeHub
import constants

# State Constants
WANDER = 1
WALL_FOLLOWING = 2
FIRE_DETECTION = 3
EXTINGUISH = 4
COMPLETE = 5


class Robot:
    def __init__(self):
        # PrimeHub
        self.hub = PrimeHub()

        # Motors
        self.left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
        self.right_motor = Motor(Port.B, Direction.CLOCKWISE)
        self.fan_motor = Motor(Port.F)

        # Sensors
        self.color_sensor = ColorSensor(Port.C)
        self.front_sensor = UltrasonicSensor(Port.D)
        self.side_sensor = UltrasonicSensor(Port.E)
        # Assuming this is a compatible light sensor
        self.light_sensor = UltrasonicSensor(Port.S)

        # Drive Base
        self.drive_base = DriveBase(
            self.left_motor,
            self.right_motor,
            wheel_diameter=constants.WHEEL_DIAMETER,
            axle_track=constants.AXLE_TRACK
        )

        # Initial State
        self.current_state = WANDER

    def detect_obstacle(self):
        """Detect if an obstacle is close using the front sonar sensor."""
        return self.front_sensor.distance() < constants.FRONT_OBSTACLE_DISTANCE_THRESHOLD

    def detect_wall_on_right(self):
        """Detect wall or opening on the right side for wall following."""
        distance = self.side_sensor.distance()
        return distance < constants.WALL_FOLLOW_DISTANCE, distance

    def detect_goal(self):
        """Detect if the robot is on the goal tile (Red)."""
        return self.color_sensor.color() == constants.GOAL_COLOR

    def detect_flame(self):
        """Check for increased light intensity indicating the flame is in that direction."""
        return self.light_sensor.intensity() > constants.FLAME_DETECTION_THRESHOLD

    def initial_scan_for_flame(self):
        """Rotate 360 degrees to detect flame direction."""
        for _ in range(4):  # Divide 360° into four 90° turns
            if self.detect_flame():
                print("Flame detected during initial scan")
                # Begin moving toward the flame direction
                return True
            self.drive_base.turn(90)
        return False

    def approach_flame(self):
        """Move toward the flame's location until the robot detects the red goal area."""
        while not self.detect_goal():
            if self.detect_obstacle():
                # Avoid obstacles while approaching
                self.drive_base.turn(-90)  # Turn left and retry approach
            else:
                self.drive_base.straight(
                    constants.APPROACH_FLAME_STEP_DISTANCE)

        print("Reached flame area. Extinguishing now...")
        self.extinguish()

    def wander(self):
        """Modified wander behavior for searching the wall if no flame is detected."""
        if self.detect_obstacle():
            # Rotate 90° and switch to wall-following behavior
            self.drive_base.turn(90)
            self.transition_to(WALL_FOLLOWING)
        else:
            # Move forward until a wall or obstacle is detected
            self.drive_base.straight(constants.WANDER_DISTANCE)

    def wall_following(self):
        """Move along a wall by maintaining a set distance from it."""
        wall_detected, distance = self.detect_wall_on_right()

        if not wall_detected:
            # If no wall is detected, turn right
            self.drive_base.turn(90)
        elif distance < constants.WALL_FOLLOW_DISTANCE:
            self.drive_base.turn(-constants.WALL_FOLLOW_TURN_ANGLE)
        else:
            self.drive_base.straight(constants.WALL_FOLLOW_STEP_DISTANCE)

    def fire_detection(self):
        """Rotate to find flame direction and approach."""
        self.initial_scan_for_flame()

    def extinguish(self):
        """Activate the fan motor to extinguish the fire."""
        print("Extinguishing fire...")
        self.fan_motor.run_time(constants.FAN_SPEED, constants.FAN_RUN_TIME)
        self.current_state = COMPLETE

    def transition_to(self, new_state):
        """Transition to a new state"""
        self.current_state = new_state
        print(f"Transitioning to {new_state}")

    def raise_alarm(self):
        """Sound an alarm to indicate fire detection."""
        print("Alarm raised!")
        self.hub.speaker.beep(
            frequency=constants.ALARM_FREQUENCY, duration=constants.ALARM_DURATION)

    def update(self):
        """Main update method to handle state transitions and execute current behavior."""
        print(f"Current State: {self.current_state}")

        if self.current_state == WANDER:
            # Initial 360° scan for flame detection
            if not self.initial_scan_for_flame():
                print("No flame detected during initial scan. Wandering...")
                self.wander()
            else:
                self.transition_to(FIRE_DETECTION)

        elif self.current_state == WALL_FOLLOWING:
            self.wall_following()
            if self.detect_goal():
                self.transition_to(FIRE_DETECTION)

        elif self.current_state == FIRE_DETECTION:
            self.approach_flame()
            self.transition_to(EXTINGUISH)

        elif self.current_state == EXTINGUISH:
            self.extinguish()
            print("Cooldown period before returning to wander.")
            self.transition_to(WANDER)
