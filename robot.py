from pybricks.robotics import DriveBase
from pybricks.pupdevices import Motor, ColorSensor, ForceSensor, UltrasonicSensor
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
        self.touch_sensor = ForceSensor(Port.D)
        # Dual-purpose sensor for wall following and flame detection
        self.side_sensor = UltrasonicSensor(Port.E)

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
        """Detect if an obstacle is close by checking the touch sensor."""
        return self.touch_sensor.touched()

    def detect_wall_on_right(self):
        """Detect wall or opening on the right side for wall following."""
        return self.side_sensor.distance() < constants.WALL_FOLLOW_MIN_DISTANCE

    def detect_flame(self):
        """Check for increased light intensity with the ultrasonic sensor for flame detection."""
        return self.side_sensor.distance() < constants.FLAME_DETECTION_THRESHOLD

    def initial_scan_for_flame(self):
        """Rotate 360 degrees to detect flame direction."""
        for _ in range(8):  # Divide 360° into four 90° turns
            if self.detect_flame():
                print("Flame detected during initial scan")
                return True
            self.drive_base.turn(45)
        return False

    def approach_flame(self):
        """Move toward the flame's location until the robot detects the red goal area."""
        while not self.detect_goal():
            if self.detect_obstacle():
                # Avoid obstacles while approaching
                self.drive_base.straight(-constants.WALL_FOLLOW_STEP)
                self.drive_base.turn(-90)  # Turn left and retry approach
            else:
                self.drive_base.straight(
                    constants.APPROACH_FLAME_STEP_DISTANCE)

        print("Reached flame area. Extinguishing now...")
        self.extinguish()

    def wander(self):
        """Drive forward until an obstacle is hit, then switch to wall-following."""
        if self.detect_obstacle():
            self.drive_base.straight(-constants.WALL_FOLLOW_STEP)
            self.drive_base.turn(-90)
            self.transition_to(WALL_FOLLOWING)
        else:
            self.drive_base.straight(constants.WANDER_DISTANCE)

    def wall_following(self):
        """Move along a wall by maintaining a set distance from it."""
        if self.detect_obstacle():
            # If touch sensor activates, back up and turn left
            self.drive_base.straight(-constants.WALL_FOLLOW_STEP)
            self.drive_base.turn(-90)
        elif not self.detect_wall_on_right():
            # If no wall is detected, turn left
            self.drive_base.turn(-90)
        else:
            self.drive_base.straight(constants.WALL_FOLLOW_STEP)

    def fire_detection(self):
        """Rotate to find flame direction and approach."""
        self.initial_scan_for_flame()

    def extinguish(self):
        """Activate the fan motor to extinguish the fire."""
        print("Extinguishing fire...")
        self.fan_motor.run_time(constants.FAN_SPEED, constants.FAN_RUN_TIME)
        self.current_state = COMPLETE

    def transition_to(self, new_state):
        """Transition to a new state."""
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
