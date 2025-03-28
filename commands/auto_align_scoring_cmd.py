from commands2 import Command
from wpimath.controller import PIDController
from ntcore import NetworkTableInstance
from subsystems.drivetrain import DriveSubsystem
from wpilib import SmartDashboard as sd

class AutoAlignScoringCommand(Command):
    def __init__(self, drivetrain:DriveSubsystem , scoring_mode='left', tolerance_tx=1.0, tolerance_ty=1.25):
        """
        Aligns the robot so that the Limelight's tx and ty values reach the ideal values for scoring.
        
        :param drivetrain: Your drive subsystem.
        :param scoring_mode: 'left' or 'right', determining which setpoints to use.
        :param tolerance_tx: Acceptable error in tx (degrees).
        :param tolerance_ty: Acceptable error in ty (degrees).
        """
        super().__init__()
        self.drivetrain = drivetrain
        self.table = NetworkTableInstance.getDefault().getTable("limelight")
        self.tolerance_tx = tolerance_tx
        self.tolerance_ty = tolerance_ty
        
        # Set desired setpoints based on scoring mode.
        if scoring_mode == 'left':
            self.desired_tx = 23.35
            self.desired_ty = -19.65
        elif scoring_mode == 'right':
            self.desired_tx = -20.27
            self.desired_ty = -19.65
        else:
            self.desired_tx = 0
            self.desired_ty = 0
        
        # PID controller for turning (using tx error).
        # self.kP_turn = 0.2
        self.turnPID = PIDController(0.009, 0.0, 0.0)
        self.turnPID.setSetpoint(self.desired_tx)
        
        # Proportional constant for forward drive based on ty error.
        self.kP_fwd = 0.07
        
        self.addRequirements(self.drivetrain)

    def initialize(self):
        print(f"AutoAlignScoringCommand: Initializing with desired tx = {self.desired_tx}, desired ty = {self.desired_ty}")

    def execute(self):
        # Get current tx and ty from Limelight
        current_tx = self.table.getNumber("tx", 0)
        current_ty = self.table.getNumber("ty", 0)
        current_tv = self.table.getNumber("tv", 0)
        
        deadband_forward = 1
        # Compute errors.
       
        error_tx = self.desired_tx - current_tx
        error_ty = self.desired_ty - current_ty
        sd.putNumber("error_tx", error_tx)

        sd.putNumber("error_ty", error_ty)

        # Compute turn correction using the PID controller (for tx)
        # turn_correction = self.turnPID.calculate(current_tx) 
        turn_correction = self.turnPID.calculate(error_tx)  # Limits turn speed to ±0.3

        
        # Compute forward correction using a simple proportional controller on ty error.
        # The negative sign may be needed depending on the camera's coordinate convention.
        # forward_correction = error_ty * self.kP_fwd
        raw_forward = -error_ty * self.kP_fwd  # Limits forward speed to ±0.3
        forward = max(min(raw_forward, 0.2), -0.2)
        
        if abs(forward) > deadband_forward:
            forward = 0

        # Command the drivetrain.
        # Here we assume the first parameter is forward speed.
        if current_tv >= 1.0:
            print("I see my target...")
            self.drivetrain.drive(forward, 0, turn_correction * 0.3, False)
            print(f"AutoAlignScoringCommand: current_tx = {current_tx}, error_tx = {error_tx}, turn_correction = {turn_correction}")
            print(f"                     current_ty = {current_ty}, error_ty = {error_ty}, forward_correction = {forward}")
        else:
            # No target seen
            self.drivetrain.drive(0, 0, 0, False)
            print("DriveToLimelightTarget: No target detected")

    def isFinished(self):
        # Check if both horizontal and vertical errors are within tolerance.
        current_tx = self.table.getNumber("tx", 0)
        current_ty = self.table.getNumber("ty", 0)
        tv = self.table.getNumber("tv", 0)
        error_tx = abs(current_tx - self.desired_tx)
        error_ty = abs(current_ty - self.desired_ty)
        if error_tx < self.tolerance_tx and error_ty < self.tolerance_ty:
            print("AutoAlignScoringCommand: Alignment within tolerances.")
            return True
        # elif tv == 0:
        #     print("AutoAlignScoringCommand: No target detected, ending command.")
        #     return True  # Stop if the Limelight loses its target
        else:
            return False

    def end(self, interrupted):
        self.drivetrain.drive(0, 0, 0, False)
        print("AutoAlignScoringCommand: Command ended, drivetrain stopped.")
        
