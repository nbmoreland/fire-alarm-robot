from state import State
from pybricks.robotics import DriveBase
import constants


class Robot:
    def __init__(self, left_motor, right_motor, fan_motor, color_sensor, distance_sensor, hub):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.fan_motor = fan_motor
        self.color_sensor = color_sensor
        self.distance_sensor = distance_sensor
        self.hub = hub
        self.drive_base = DriveBase(
            left_motor, right_motor, wheel_diameter=constants.WHEEL_DIAMETER, axle_track=constants.AXLE_TRACK)
        self.current_state = State.WANDER
        self.state_timer = 0

    def update(self):
        """Main update method to handle state transitions and execute current behavior."""
        print(f"Current State: {self.current_state}")

        if self.current_state == State.WANDER:
            self.wander()
            if self.detect_wall():
                self.transition_to(State.WALL_FOLLOWING)
            elif self.detect_fire_area():
                self.transition_to(State.FIRE_DETECTION)

        elif self.current_state == State.WALL_FOLLOWING:
            self.wall_following()
            if self.detect_fire_area():
                self.transition_to(State.FIRE_DETECTION)

        elif self.current_state == State.FIRE_DETECTION:
            self.fire_detection()
            self.transition_to(State.EXTINGUISH)

        elif self.current_state == State.EXTINGUISH:
            self.extinguish()
            self.transition_to(
                State.WANDER, delay=constants.COOLDOWN_AFTER_EXTINGUISH)

    def transition_to(self, new_state, delay=0):
        """Transition to a new state, optionally with a delay."""
        self.current_state = new_state
        self.state_timer = delay
        print(f"Transitioning to {new_state} with delay {delay}ms")

    def wander(self):
        """Move the robot around randomly to explore the area."""
        print("Wandering...")
        self.drive_base.straight(constants.WANDER_DISTANCE)
        self.drive_base.turn(constants.WANDER_TURN_ANGLE)

    def wall_following(self):
        """Move along a wall by maintaining a set distance from it."""
        print("Following wall...")
        distance = self.distance_sensor.distance()
        if distance < constants.WALL_FOLLOW_MIN_DISTANCE:
            self.drive_base.turn(-constants.WALL_FOLLOW_ADJUST_ANGLE)
        elif distance > constants.WALL_FOLLOW_MAX_DISTANCE:
            self.drive_base.turn(constants.WALL_FOLLOW_ADJUST_ANGLE)
        self.drive_base.straight(constants.WALL_FOLLOW_STEP)

    def fire_detection(self):
        """Detect the fire area using color sensor and raise an alarm."""
        print("Detecting fire area...")
        if self.detect_fire_area():
            self.raise_alarm()

    def extinguish(self):
        """Activate the fan motor to extinguish the fire."""
        print("Extinguishing fire...")
        self.fan_motor.run_time(constants.FAN_SPEED, constants.FAN_RUN_TIME)

    def detect_wall(self):
        """Detect if a wall is close using distance sensor."""
        wall_detected = self.distance_sensor.distance() < constants.WALL_DETECT_THRESHOLD
        print(f"Wall detected: {wall_detected}")
        return wall_detected

    def detect_fire_area(self):
        """Detect if the robot is over the fire area using color sensor."""
        fire_area_detected = self.color_sensor.color() == constants.FIRE_AREA_COLOR
        print(f"Fire area detected: {fire_area_detected}")
        return fire_area_detected

    def raise_alarm(self):
        """Sound an alarm to indicate fire detection."""
        print("Alarm raised!")
        self.hub.speaker.beep(
            frequency=constants.ALARM_FREQUENCY, duration=constants.ALARM_DURATION)
