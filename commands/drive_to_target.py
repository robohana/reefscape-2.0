# drive_to_limelight_target.py
import wpilib
from commands2 import Command
from ntcore import NetworkTableInstance
from subsystems.drivetrain import DriveSubsystem

class DriveToLimelightTarget(Command):
    def __init__(self, drivetrain: DriveSubsystem, target_area_threshold=5.0, drive_speed=0.5, target_tag_id=None):
        """
        Drives forward until the Limelight target area exceeds a threshold.
        
        :param drivetrain: The DriveSubsystem.
        :param target_area_threshold: The area value at which to stop (indicating closeness).
        :param drive_speed: The forward speed (as a fraction of max speed).
        """
        super().__init__()
        self.drivetrain = drivetrain
        self.target_area_threshold = target_area_threshold
        self.drive_speed = drive_speed
        self.target_tag_id = target_tag_id
        self.table = NetworkTableInstance.getDefault().getTable("scoreCoral")
        self.addRequirements(self.drivetrain)
        
        # Simple proportional control for turning based on tx (horizontal offset)
        self.kP_turn = 0.03

    def initialize(self):
        print("DriveToLimelightTarget: Initialized")
        
    def execute(self):
        # Get target valid (tv), horizontal offset (tx), and target area (ta)
        tv = self.table.getNumber("tv", 0)
        tx = self.table.getNumber("tx", 0)
        ta = self.table.getNumber("ta", 0)
        
        # If a target is seen:
        if tv >= 1.0:
            print("I see my target...")
            # Compute a turning correction (if tx is non-zero, rotate to center the target)
            turn_correction = -tx * self.kP_turn  # negative because positive tx means target is right, so we need to turn left.
            # Drive forward at drive_speed with correction
            self.drivetrain.drive(self.drive_speed, 0, turn_correction, True)
            print(f"Driving: tx={tx}, ta={ta}, turn_correction={turn_correction}")
        else:
            # No target seen
            self.drivetrain.drive(0, 0, 0, True)
            print("DriveToLimelightTarget: No target detected")
        
    def isFinished(self):
        # End the command when the target area reaches the threshold (indicating you’re close)
        ta = self.table.getNumber("ta", 0)
        if ta >= self.target_area_threshold:
            print("DriveToLimelightTarget: Target reached")
            return True
        return False

    def end(self, interrupted):
        if interrupted is True:
            self.drivetrain.drive(0, 0, 0, True)
            print("DriveToLimelightTarget: Command ended")
