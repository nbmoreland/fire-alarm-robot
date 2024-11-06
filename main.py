from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Direction
from pybricks.hubs import PrimeHub
from robot import Robot
import time

# Initialize the hub, motors, and sensors
hub = PrimeHub()
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B, Direction.CLOCKWISE)
fan_motor = Motor(Port.C)  # Motor for extinguishing fan
color_sensor = ColorSensor(Port.D)  # Downward-facing color sensor
distance_sensor = UltrasonicSensor(Port.E)  # Sensor for wall detection

# Initialize DriveBase and Robot
robot = Robot(left_motor, right_motor, fan_motor,
              color_sensor, distance_sensor, hub)


def main():
    """Main loop to update the robot and manage the state machine."""
    print("Starting main loop...")
    while True:
        # Update the robot's state and behaviors
        robot.update()

        # Optional: Add a delay to control loop speed if needed
        time.sleep(0.1)  # 100ms delay


# Run the main loop
if __name__ == "__main__":
    main()
