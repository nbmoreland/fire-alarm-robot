from robot import Robot

# Initialize Robot
robot = Robot()


def main():
    """Main loop to update the robot and manage the state machine."""
    while robot.current_state != 4:
        # Update the robot's state and behaviors
        robot.update()


# Run the main loop
if __name__ == "__main__":
    main()
