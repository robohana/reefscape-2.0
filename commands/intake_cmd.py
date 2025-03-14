import time
import wpilib
from commands2 import Command, InstantCommand
from wpilib import Timer
from subsystems.coral_subsystem import CoralSubsystem

class RunIntakeCommand(Command):
    def __init__(self, coral: CoralSubsystem):
        super().__init__()
        self.coral = coral
        self.timer = Timer()
        self.has_detected = False  # Flag to track if we detected an object

        self.addRequirements(self.coral)  # Ensures command has exclusive control

    def initialize(self):
        print("in init")
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

        # # If we have detected an object and 1 second has passed, stop intake and pop it up
        # if self.has_detected and self.timer.hasElapsed(1.0):
        #     self.coral.pop_intake()

    def isFinished(self):
        """End the command after popping intake."""
        return self.has_detected and self.timer.hasElapsed(1.2)  # Add slight buffer

    def end(self, interrupted):
        """Called when the command ends or is interrupted."""
        self.coral.stop_intake()

class ReleaseIntakeCommand(InstantCommand):        
    def __init__(self, coral: CoralSubsystem):
        super().__init__()
        self.coral = coral
        self.addRequirements(self.coral)

    def initialize(self):
        pass

    def execute(self):
        self.coral.reverse_intake_power()

    def end(self, interrupted):
        self.coral.stop_intake()    

class MoveToSetpointCommand(Command):
    def __init__(self, coral_subsystem: CoralSubsystem, arm_setpoint: float, elevator_setpoint: float) -> None:
        super().__init__()
        self.coral = coral_subsystem
        self.arm_setpoint = arm_setpoint
        self.elevator_setpoint = elevator_setpoint

    def initialize(self):
        print("Initializing move to setpoint")
        self.start_time = wpilib.Timer.getFPGATimestamp()

    def execute(self):
        # Update setpoint on every cycle
        #self.coral.move_to_setpoint(self.arm_setpoint, self.elevator_setpoint)
        self.coral.move_to_setpoint(self.arm_setpoint, self.elevator_setpoint)
        

    def isFinished(self) -> bool:
        # Check if within tolerances or if timeout is reached.
        print("at setpoint")

    def end(self, interrupted: bool):
        if interrupted:
            print("Move command interrupted.")
        else:
            print("Move command completed successfully.")
