
from subsystems.coral_subsystem import CoralSubsystem
import time
from commands2 import Command

class AutoReverseIntakeCmd(Command):
        def __init__(self, coral: CoralSubsystem):
            super().__init__()
            self.coral = coral
            self.addRequirements(CoralSubsystem)
            self.start_time = 0

        def initialize(self):
            self.start_time = time.time()

        def execute(self):
            self.elapsed_time = time.time() - self.start_time
            self.coral.reverse_intake_power() 

        def end(self, interuppted: bool):
            self.coral.stop_intake()
            if interuppted is True:
                self.coral.stop_intake()

        def isFinished(self):
            if self.elapsed_time >= 1.0:
                return True
            
    
