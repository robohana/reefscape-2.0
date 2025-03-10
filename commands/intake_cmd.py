import time
import wpilib
from commands2 import Command
from subsystems.coral_subsystem import CoralSubsystem

class RunIntakeCommand(Command):
    def __init__(self, coral: CoralSubsystem):
        super().__init__()
        self.coral = coral
        self.timer = wpilib.Timer()
        self.has_detected = False  # Flag to track if we detected an object

        self.addRequirements([CoralSubsystem])  # Ensures command has exclusive control

    def initialize(self):
        """Called when the command starts."""
        self.timer.reset()
        self.timer.stop()
        self.has_detected = False
        self.coral.run_intake_power()  # Start the intake

    def execute(self):
        """Called repeatedly while the command is running."""
        if self.coral.is_object_detected():  # Sensor detected object
            if not self.has_detected:
                self.has_detected = True
                self.timer.reset()
                self.timer.start()

        # If we have detected an object and 1 second has passed, stop intake and pop it up
        if self.has_detected and self.timer.hasElapsed(1.0):
            self.coral.pop_intake()

    def isFinished(self):
        """End the command after popping intake."""
        return self.has_detected and self.timer.hasElapsed(1.2)  # Add slight buffer

    def end(self, interrupted):
        """Called when the command ends or is interrupted."""
        self.coral.stop_intake()

class ReleaseIntakeCommand(Command):        
    def __init__(self, coral: CoralSubsystem):
        super().__init__(CoralSubsystem.reverse_intake_power)
        self.addRequirements([CoralSubsystem])
