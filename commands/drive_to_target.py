# drive_to_limelight_target.py
from commands2 import Command

from ntcore import NetworkTableInstance

# import limelightresults

from subsystems.drivetrain import DriveSubsystem

from constants.constants import AutoConstants

class DriveToLimelightTarget(Command):
    def __init__(self, drivetrain: DriveSubsystem, target_area_threshold=2.12, drive_speed=0.15, target_tag_id=None):
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
        self.table = NetworkTableInstance.getDefault().getTable("limelight")
        self.addRequirements(self.drivetrain)
        
        # Simple proportional control for turning based on tx (horizontal offset)
        self.kP_turn = 0.017
        self.kP_forward = 0.009

    def initialize(self):
        print("DriveToLimelightTarget: Initialized")
        
    def execute(self):
        # Get target valid (tv), horizontal offset (tx), target area (ta), vertical offset (ty)
        tv = self.table.getNumber("tv", 0)
        tx = self.table.getNumber("tx", 0)
        ta = self.table.getNumber("ta", 0)
        ty = self.table.getNumber("ty", 0)

        # limelightresults.FiducialResult(tx)

        # If a target is seen:
        if tv >= 1.0:
            print("I see my target...")
            # Compute a turning correction (if tx is non-zero, rotate to center the target)
            turn_correction = -tx * self.kP_turn  # negative because positive tx means target is right, so we need to turn left.
            turn_correction *= AutoConstants.K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND
            forward_corractions = ty * self.kP_forward
            forward_corractions *= AutoConstants.K_MAX_SPEED_METERS_PER_SECOND
            # Drive forward at drive_speed with correction
            # TODO: need x value to be ty, say like forward position = ty * kp forward, apply this value to the drive speed
            self.drivetrain.drive(forward_corractions, 0, turn_correction, False) # Field Relative False for Auto
            print(f"Driving: tx={tx}, ty={ty}, ta={ta}, turn_correction={turn_correction}")
        else:
            # No target seen
            self.drivetrain.drive(0, 0, 0, False)
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
            self.drivetrain.drive(0, 0, 0, False)
            print("DriveToLimelightTarget: Command ended")
