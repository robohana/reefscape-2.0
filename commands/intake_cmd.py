import time
import wpilib
from commands2 import Command, InstantCommand
from subsystems.coral_subsystem import CoralSubsystem

class RunIntakeCommand(Command):
    def __init__(self, coral: CoralSubsystem):
        super().__init__()
        self.coral = coral
        self.timer = wpilib.Timer()
        self.has_detected = False  # Flag to track if we detected an object

        self.addRequirements(self.coral)  # Ensures command has exclusive control

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
    def __init__(self, coral_subsystem: CoralSubsystem, arm_setpoint: float, elevator_setpoint: float, timeout: float = 5.0) -> None:
        super().__init__()
        self.coral = coral_subsystem
        self.arm_setpoint = arm_setpoint
        self.elevator_setpoint = elevator_setpoint
        self.timeout = timeout
        self.start_time = None

    def initialize(self):
        print("Initializing move to setpoint")
        self.start_time = wpilib.Timer.getFPGATimestamp()

    def execute(self):
        # Update setpoint on every cycle
        self.coral.move_to_setpoint(self.arm_setpoint, self.elevator_setpoint)
        current_arm = self.coral.get_arm_position()
        current_elevator = self.coral.get_elevator_position()
        self.arm_error = abs(self.arm_setpoint - current_arm)
        self.elevator_error = abs(self.elevator_setpoint - current_elevator)
        print(f"Arm Error: {self.arm_error}, Current Position: {current_arm}")
        print(f"Elevator Error: {self.elevator_error}, Current Position: {current_elevator}")

    def isFinished(self) -> bool:
        # Check if within tolerances or if timeout is reached.
        current_time = wpilib.Timer.getFPGATimestamp()
        if current_time - self.start_time >= self.timeout:
            print("Timeout reached in MoveToSetpointCommand.")
            return True
        arm_tolerance = 2.0
        elevator_tolerance = 2.0
        if self.arm_error < arm_tolerance and self.elevator_error < elevator_tolerance:
            print("Setpoints reached or within tolerances.")
            return True
        return False

    def end(self, interrupted: bool):
        if interrupted:
            print("Move command interrupted.")
        else:
            print("Move command completed successfully.")

