# auto_routines.py
from commands2 import SequentialCommandGroup, InstantCommand, WaitCommand
from subsystems.drivetrain import DriveSubsystem
from subsystems.coral_subsystem import CoralSubsystem
from commands.coral_cmds import ScoreCoralL2, IntakeToZero
from commands.auto_align_scoring_cmd import AutoAlignScoringCommand
from commands.auto_reverse_intake_cmd import AutoReverseIntakeCmd

class SimpleScoreAutoLEFT(SequentialCommandGroup):
    def __init__(self, drivetrain: DriveSubsystem, coral: CoralSubsystem, targetTag):
        """
        Autonomous routine that:
         1. Drives toward the scoring AprilTag (using vision with fallback),
         2. Scores at level 2,
        
        :param drivetrain: DriveSubsystem instance.
        :param coral: CoralSubsystem instance.
        :param targetTag: AprilTag ID for scoring.
        """
        super().__init__()
        
        self.addCommands(
            # Drive to target using Limelight data
            AutoAlignScoringCommand(drivetrain, 'left', 2.5, 1.25, targetTag),
            InstantCommand(lambda: drivetrain.drive(0, 0, 0, False), drivetrain),
            # Score a coral at L2
            ScoreCoralL2(coral),
            WaitCommand(1.5),
            AutoReverseIntakeCmd(coral),
            IntakeToZero(coral)
        )

class SimpleScoreAutoRIGHT(SequentialCommandGroup):
    def __init__(self, drivetrain: DriveSubsystem, coral: CoralSubsystem, targetTag):
        """
        Autonomous routine that:
         1. Drives toward the scoring AprilTag (using vision with fallback),
         2. Scores at level 2,
        
        :param drivetrain: DriveSubsystem instance.
        :param coral: CoralSubsystem instance.
        :param targetTag: AprilTag ID for scoring.
        """
        super().__init__()
        
        self.addCommands(
            # Drive to target using Limelight data
            AutoAlignScoringCommand(drivetrain, 'right', 2.5, 1.25, targetTag),
            InstantCommand(lambda: drivetrain.drive(0, 0, 0, False), drivetrain),
            # Score a coral at L2
            ScoreCoralL2(coral),
            WaitCommand(1.5),
            AutoReverseIntakeCmd(coral),
            IntakeToZero(coral)
        )
