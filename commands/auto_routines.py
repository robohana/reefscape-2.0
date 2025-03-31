
import time
from subsystems.drivetrain import DriveSubsystem
from commands2 import Command

class SimpleAuto(Command):
    """A simple autonomous routine for testing robot movement in AdvantageScope."""

    def __init__(self, drivetrain: DriveSubsystem):
        super().__init__()
        self.drivetrain = drivetrain
        self.start_time = 0

        #  Requires the drive subsystem
        self.addRequirements(self.drivetrain)

    def initialize(self):
        """Called once when autonomous starts."""
        self.start_time = time.time()
        
        print(" Simple Autonomous Started")

    def execute(self):
        """Runs periodically during autonomous mode and logs swerve states."""
        elapsed_time = time.time() - self.start_time

        if elapsed_time < 2.0:
            # Move forward with more speed to verify movement
            self.drivetrain.drive(0.2, 0.0, 0.0, True)
        # elif elapsed_time < 4.0:
        #     # Rotate to see if robot turns in AS
        #     self.drivetrain.drive(0.0, 0.0, 1.0, True)
        else:
            # Stop after 4 seconds
            self.drivetrain.drive(0, 0, 0, True)

        #  Print swerve module states to see if they are updating
        # states = self.robotDrive.getModuleStates()
        # print(f"Auto Running - Time: {elapsed_time:.2f} sec, States: {states}")

        #  Log the values to NetworkTables
        # self.robotDrive.publishSwerveStates()

    def isFinished(self):
        self.current_pose = self.drivetrain.get_pose()
        self.drivetrain.reset_odometry(self.current_pose)
        """End the command after 4 seconds."""
        return time.time() - self.start_time >= 4.0

    def end(self, interrupted):
        """Stop the robot when the command ends."""
        if interrupted is True:
            self.drivetrain.drive(0, 0, 0, True)
            self.drivetrain.reset_odometry(self.current_pose)
            return True
