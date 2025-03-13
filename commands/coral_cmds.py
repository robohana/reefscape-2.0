from commands2 import SequentialCommandGroup, InstantCommand, WaitCommand, WaitUntilCommand
from subsystems.coral_subsystem import CoralSubsystem
from constants import Setpoint
from commands.intake_cmd import ReleaseIntakeCommand, RunIntakeCommand, MoveToSetpointCommand

class ScoreCoralL1(SequentialCommandGroup):
    def __init__(self, coral: CoralSubsystem):
        super().__init__()
        self.addCommands(
            #Step 1: Move Elevator to L1 setpoint
            #Step 2: Move Arm to L1 Setpoint
            MoveToSetpointCommand(coral, Setpoint.Arm.K_LEVEL_1, Setpoint.Elevator.K_LEVEL_1),
            #Step 3: Release Intake Command. with wait
            ReleaseIntakeCommand(coral),
        )

class ScoreCoralL2(SequentialCommandGroup):
    def __init__(self, coral: CoralSubsystem):
        super().__init__()
        self.addCommands(
            #Step 1: Move Elevator to L2 setpoint
            #Step 2: Move Arm to L2 Setpoint
            MoveToSetpointCommand(coral, Setpoint.Arm.K_LEVEL_2, Setpoint.Elevator.K_LEVEL_2),
            #Step 3: Release Intake Command. with wait
            ReleaseIntakeCommand(coral),
        )

class ScoreCoralL3(SequentialCommandGroup):
    def __init__(self, coral: CoralSubsystem):
        super().__init__()
        self.addCommands(
            #Step 1: Move Elevator to L3 setpoint
            #Step 2: Move Arm to L3 Setpoint
            MoveToSetpointCommand(coral, Setpoint.Arm.K_LEVEL_3, Setpoint.Elevator.K_LEVEL_3),
            #Step 3: Release Intake Command. with wait
            ReleaseIntakeCommand(coral),
        )   

class IntakeCoralStation(SequentialCommandGroup):
    def __init__(self, coral: CoralSubsystem):
        super().__init__()
        self.addCommands(
            #Step 1: Move Elevator to Coral Station setpoint
            #Step 2: Move Arm to Coral Station Setpoint
            MoveToSetpointCommand(coral, Setpoint.Arm.K_CORAL_STATION, Setpoint.Elevator.K_CORAL_STATION),
            #Step 3: Run Intake Command. with wait
            RunIntakeCommand(coral),
        )             
