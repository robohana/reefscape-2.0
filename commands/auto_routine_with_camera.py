# auto_routines.py
from commands2 import SequentialCommandGroup, InstantCommand
from subsystems.drivetrain import DriveSubsystem
from subsystems.coral_subsystem import CoralSubsystem
from commands.drive_to_target import DriveToLimelightTarget
from commands.coral_cmds import ScoreCoralL2

class SimpleScoreAuto(SequentialCommandGroup):
    def __init__(self, drivetrain: DriveSubsystem, coral: CoralSubsystem, target_tag_id: int):
        """
        Autonomous routine that:
         1. Drives toward the scoring AprilTag (using vision with fallback),
         2. Scores at level 2,
         
        :param drivetrain: DriveSubsystem instance.
        :param coral: CoralSubsystem instance.
        :param vision: VisionSystem instance.
        :param score_tag_id: AprilTag ID for scoring.
        """
        super().__init__()
        
        self.addCommands(
            # Drive to target using Limelight data
            DriveToLimelightTarget(drivetrain, target_area_threshold=5.0, drive_speed=0.5, target_tag_id=target_tag_id),
            InstantCommand(lambda: drivetrain.drive(0, 0, 0, True), drivetrain),
            # Score a coral at L2
            ScoreCoralL2(coral)
            
        )
