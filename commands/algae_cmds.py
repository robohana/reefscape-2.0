from commands2 import Command
from subsystems.algae_subsystem import AlgaeSubsystem
from constants.constants import Setpoint
from rev import SparkBase

class AlgaeLoadCommand(Command):
    def __init__(self, algae: AlgaeSubsystem):
        super().__init__()
        self.algae = algae
        self.addRequirements(self.algae)

    def initialize(self):
        pass

    def execute(self):
        self.algae.move_to_setpoint(Setpoint.Algae.K_LOAD_POSITION)
        self.algae.set_roller_power(Setpoint.Algae.K_LOAD_POWER)

    def isFinished(self):
        return True

    def end(self, interrupted):
        if interrupted is True:
            return True

class AlgaeScoreCommand(Command):
    def __init__(self, algae: AlgaeSubsystem):
        super().__init__()
        self.algae = algae
        self.addRequirements(self.algae)

    def initialize(self):
        pass

    def execute(self):
        self.algae.move_to_setpoint(Setpoint.Algae.K_LOAD_POSITION)
        self.algae.set_roller_power(Setpoint.Algae.K_SCORE_POWER)

    def isFinished(self):
        return True

    def end(self, interrupted):
        if interrupted is True:
            return True
        
class AlgaeZeroCommand(Command):
    def __init__(self, algae: AlgaeSubsystem):
        super().__init__()
        self.algae = algae
        self.addRequirements(self.algae)

    def initialize(self):
        pass

    def execute(self):
        self.algae.move_to_setpoint(Setpoint.Algae.K_LOCKED_POSITION)
        self.algae.set_roller_power(Setpoint.Algae.K_LOCKED_POWER)

    def isFinished(self):
        return True

    def end(self, interrupted):
        if interrupted is True:
            return True
        
class AlgaeOnCommand(Command):
    def __init__(self, algae: AlgaeSubsystem):
        super().__init__()
        self.algae = algae
        self.addRequirements(self.algae)

    def initialize(self):
        pass

    def execute(self):
        self.algae.set_roller_power(Setpoint.Algae.K_LOAD_POWER)

    def isFinished(self):
        return True

    def end(self, interrupted):
        if interrupted is True:
            return True

        
