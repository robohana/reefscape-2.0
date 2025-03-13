import time
import wpilib
from commands2 import Command, InstantCommand
from subsystems.coral_subsystem import CoralSubsystem
from rev import SparkLowLevel

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
    def __init__(self, coral_subsystem: CoralSubsystem, arm_setpoint: float, elevator_setpoint: float) -> None:
        super().__init__()
        self.coral = coral_subsystem
        self.arm_setpoint = arm_setpoint
        self.elevator_setpoint = elevator_setpoint

        # arm_current_position: float
        # elevator_current_position: float

    def initialize(self):
        # Optionally, print initial values or do setup
        print("Initializing arm move to setpoint")
        # self.arm_current_position = self.coral.get_arm_position()
        # self.elevator_current_position = self.coral.get_elevator_position()    

    def execute(self):
        # Set motor to move towards target setpoint
        # Example logic: Set motor output based on PID, or some other method.

        self.coral.move_to_arm_setpoint(self.arm_setpoint)   
        self.coral.move_to_elevator_setpoint(self.elevator_setpoint) 

        # self.coral.arm_closed_loop_controller.setReference(self.arm_setpoint, SparkLowLevel.ControlType.kMAXMotionPositionControl)
        # self.coral.elevator_closed_loop_controller.setReference(self.elevator_setpoint, SparkLowLevel.ControlType.kMAXMotionPositionControl)

        # if self.arm_current_position is None:
        #     self.arm_current_position = 1
        # else:
        #     self.arm_current_position = self.arm_current_position

        # if self.elevator_current_position is None:
        #     self.elevator_current_position = 1
        # else: 
        #     self.elevator_current_position = self.elevator_current_position
        
        # arm_current_position = self.arm_current_position
        
        # elevator_current_position = self.elevator_current_position
        # self.arm_error = abs(self.arm_setpoint - arm_current_position)
        # self.elevator_error = abs(self.elevator_setpoint - elevator_current_position)
        # print(f"Arm Error: {self.arm_error}, Current Position: {self.arm_current_position}")
        # print(f"Elevator Error: {self.elevator_error}, Current Position: {self.elevator_current_position}")

    def isFinished(self) -> bool:
        # Check if the error is small enough to stop
        # arm_tolerance = 2.0  # You may want to adjust this value
        # elevator_tolerance = 2.0
        # if self.arm_error < arm_tolerance and self.elevator_error < elevator_tolerance:
        #     print("Setpoints reached or within tolerances.")
        #     return True
        # return False
        pass

    def end(self, interrupted: bool):
        # Any cleanup if necessary
        if interrupted:
            print("Move command interrupted.")
        else:
            print("Move command completed successfully.")
