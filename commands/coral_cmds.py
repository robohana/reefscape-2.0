from commands2 import SequentialCommandGroup

from subsystems.coral_subsystem import CoralSubsystem

from commands.intake_cmd import RunIntakeCommand
from commands.move_to_coral_station_cmd import MoveToCoralStation
from commands.move_to_zero_cmd import MoveToZeroCommand
from commands.move_to_l2_cmd import MoveToL2Command
from commands.move_to_l3_cmd import MoveToL3Command

class IntakeToZero(SequentialCommandGroup):
    def __init__(self, coral: CoralSubsystem):
        super().__init__()
        self.addCommands(
            #Step 1: Move Elevator to L1 setpoint
            #Step 2: Move Arm to L1 Setpoint
            MoveToZeroCommand(coral)
            #Step 3: Release Intake Command. with wait
            # ReleaseIntakeCommand(coral)
        )

class ScoreCoralL2(SequentialCommandGroup):
    def __init__(self, coral: CoralSubsystem):
        super().__init__()
        self.addCommands(
            #Step 1: Move Elevator to L2 setpoint
            #Step 2: Move Arm to L2 Setpoint
            MoveToL2Command(coral)
            #Step 3: Release Intake Command. with wait
            # ReleaseIntakeCommand(coral)
        )

class ScoreCoralL3(SequentialCommandGroup):
    def __init__(self, coral: CoralSubsystem):
        super().__init__()
        self.addCommands(
            #Step 1: Move Elevator to L3 setpoint
            #Step 2: Move Arm to L3 Setpoint
            MoveToL3Command(coral)
            #Step 3: Release Intake Command. with wait
            # ReleaseIntakeCommand(coral)
        )   

class IntakeCoralStation(SequentialCommandGroup):
    def __init__(self, coral: CoralSubsystem):
        super().__init__()
        self.addCommands(
            #Step 1: Move Elevator to Coral Station setpoint
            #Step 2: Move Arm to Coral Station Setpoint
            MoveToCoralStation(coral),
            #Step 3: Run Intake Command. with wait
            RunIntakeCommand(coral)
        )             
