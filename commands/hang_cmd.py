from constants import OIConstants, DriveConstants, RobotConstants
from wpimath.filter import SlewRateLimiter
from commands2 import Command 
from wpilib import XboxController
from subsystems.hang_subsystem import HangSubsystem
from wpilib import SmartDashboard as sd
from rev import SparkLowLevel

class HangCmd(Command):

    def __init__(self, hang: HangSubsystem, driver_controller: XboxController, target_position: float):
        super().__init__()
        self.hang = hang
        self.driver_controller = driver_controller
        self.target_position = target_position
        self.addRequirements(self.hang)

    def initialize(self):
        """Move to target position using relative offset-based encoder readings."""
        sd.putString("HangCmd", f"Moving to {self.target_position}")
        self.hang.move_to_position(0.0)
    
    def execute(self):
        """Update dashboard with encoder position."""
        current_position = self.hang.hang_encoder.getPosition()
        sd.putNumber("Hang Position", current_position)

    def end(self, interrupted: bool):
        """Stop the motor when the command ends."""
        sd.putString("HangCmd", "Ended")
        self.hang.hang_motor.set(0)  # Stop motor

    def isFinished(self):
        """Stop when the arm is close to the target position."""
        current_position = self.hang.hang_encoder.getPosition()
        return abs(current_position - self.target_position) < 0.5  # Small tolerance    
