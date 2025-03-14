from commands2 import Command
from subsystems.coral_subsystem import CoralSubsystem
from constants.constants import Setpoint
from commands.intake_cmd import ReleaseIntakeCommand, RunIntakeCommand, MoveToSetpointCommand
from constants.constants import Setpoint
from rev import SparkLowLevel

class MoveToCoralStation(Command):
    def __init__(self, coral: CoralSubsystem):
        super().__init__()
        self.coral = coral
        self.addRequirements(self.coral)

    def initialize(self):
        pass

    def execute(self):
        self.coral.arm_closed_loop_controller.setReference(Setpoint.Arm.K_CORAL_STATION, SparkLowLevel.ControlType.kMAXMotionPositionControl)
        self.coral.elevator_closed_loop_controller.setReference(Setpoint.Elevator.K_CORAL_STATION, SparkLowLevel.ControlType.kMAXMotionPositionControl)

    def end(self, interrupted):
        self.coral.stop_intake()
