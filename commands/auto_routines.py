
import time
#import commands2
from subsystems.drivetrain import DriveSubsystem
from commands2 import Command

class AutoRoutines(Command):
    """A simple autonomous routine for testing robot movement in AdvantageScope."""

    def __init__(self, robot_drive: DriveSubsystem):
        super().__init__()
        self.robot_drive = robot_drive
        self.start_time = 0

        # Requires the drive subsystem
        self.addRequirements(self.robot_drive)

    def initialize(self):
        """Called once when autonomous starts."""
        self.start_time = time.time()
        print("Simple Autonomous Started")

    def execute(self):
        """Runs periodically during autonomous mode and logs swerve states."""
        elapsed_time = time.time() - self.start_time

        if elapsed_time < 2.0:
            # Move forward with more speed to verify movement
            self.robot_drive.drive(1.0, 0.0, 0.0, field_relative=False)
        elif elapsed_time < 4.0:
            # Rotate to see if robot turns in AS
            self.robot_drive.drive(0.0, 0.0, 1.0, field_relative=False)
        else:
            # Stop after 4 seconds
            self.robot_drive.drive(0, 0, 0, False, False)

        # TODO: Uncomment for debugging
        # states = self.robotDrive.getModuleStates()
        # print(f"Auto Running - Time: {elapsed_time:.2f} sec, States: {states}")

        # TODO: Uncomment to log values to NetworkTables
        # self.robotDrive.publishSwerveStates()


    def is_finished(self):
        """End the command after 4 seconds."""
        return time.time() - self.start_time >= 4.0

    def end(self, interrupted):
        """Stop the robot when the command ends."""
        self.robot_drive.drive(0, 0, 0, False, False)
        print("Simple Autonomous Ended")