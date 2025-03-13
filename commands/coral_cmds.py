from commands2 import SequentialCommandGroup, InstantCommand, WaitCommand, WaitUntilCommand
from subsystems.coral_subsystem import CoralSubsystem
from constants import Setpoint
from commands.intake_cmd import ReleaseIntakeCommand, RunIntakeCommand


class ScoreCoralL1(SequentialCommandGroup):
    def __init__(self, coral: CoralSubsystem):
        super().__init__()
        self.addCommands(
            #Step 1: Move Elevator to L1 setpoint
            InstantCommand(lambda: coral.move_to_elevator_setpoint(Setpoint.Elevator.K_LEVEL_1), coral),
            #Step 2: Move Arm to L1 Setpoint
            InstantCommand(lambda: coral.move_to_arm_setpoint(Setpoint.Arm.K_LEVEL_1),coral),
            #Step 3: Release Intake Command. with wait
            ReleaseIntakeCommand(coral),
        )

class ScoreCoralL2(SequentialCommandGroup):
    def __init__(self, coral: CoralSubsystem):
        super().__init__()
        self.addCommands(
            #Step 1: Move Elevator to L2 setpoint
            InstantCommand(lambda: coral.move_to_elevator_setpoint(Setpoint.Elevator.K_LEVEL_2), coral),
            #Step 2: Move Arm to L2 Setpoint
            InstantCommand(lambda: coral.move_to_arm_setpoint(Setpoint.Arm.K_LEVEL_2),coral),
            #Step 3: Release Intake Command. with wait
            ReleaseIntakeCommand(coral),
        )

class ScoreCoralL3(SequentialCommandGroup):
    def __init__(self, coral: CoralSubsystem):
        super().__init__()
        self.addCommands(
            #Step 1: Move Elevator to L3 setpoint
            InstantCommand(lambda: coral.move_to_elevator_setpoint(Setpoint.Elevator.K_LEVEL_3), coral),
            #Step 2: Move Arm to L3 Setpoint
            InstantCommand(lambda: coral.move_to_arm_setpoint(Setpoint.Arm.K_LEVEL_3),coral),
            #Step 3: Release Intake Command. with wait
            ReleaseIntakeCommand(coral),
        )   

class IntakeCoralStation(SequentialCommandGroup):
    def __init__(self, coral: CoralSubsystem):
        super().__init__()
        self.addCommands(
            #Step 1: Move Elevator to Coral Station setpoint
            InstantCommand(lambda: coral.move_to_elevator_setpoint(Setpoint.Elevator.K_CORAL_STATION), coral),
            #Step 2: Move Arm to Coral Station Setpoint
            InstantCommand(lambda: coral.move_to_arm_setpoint(Setpoint.Arm.K_CORAL_STATION),coral),
            #Step 3: Run Intake Command. with wait
            RunIntakeCommand(coral),
        )             
