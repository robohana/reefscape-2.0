from commands2 import Command
from wpimath.controller import PIDController
from ntcore import NetworkTableInstance
from subsystems.drivetrain import DriveSubsystem
from wpilib import SmartDashboard as sd, LiveWindow
import time

class AutoAlignScoringCommand(Command):
    def __init__(self, drivetrain:DriveSubsystem, scoring_mode, tolerance_tx, tolerance_ty, targetTag):
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
        self.scoring_mode = scoring_mode
        self.targetTag = targetTag

        
        # Set desired setpoints based on scoring mode.
        if self.scoring_mode == 'left':
            self.desired_tx = 23.35
            self.desired_ty = -19.65
        elif self.scoring_mode == 'right':
            self.desired_tx = -20.27
            self.desired_ty = -19.65
        else:
            self.desired_tx = 0
            self.desired_ty = 0
        
        # kp = sd.putNumber("turn p", 0.024)
        # ki = sd.putNumber("turn i", 0.0)
        # kd = sd.putNumber("turn d", 0.0023)

        # PID controller for turning (using tx error).
        # self.kP_turn = 0.2
        self.turnPID = PIDController(0.024, 0, 0.0023)
        self.turnPID.setSetpoint(self.desired_tx)
        # sd.putData("turn controller", self.turnPID)
        
        # Proportional constant for forward drive based on ty error.
        # dp = sd.putNumber("drive p", 0.0054)
        # di = sd.putNumber("drive i", 0.0)
        # dd = sd.putNumber("drive d", 0.0061)
        self.drivePID = PIDController(0.054,0,0.0061)
        self.drivePID.setSetpoint(self.desired_ty)
        # sd.putData("drive controller", self.drivePID)

        self.start_time = 0

        self.tid = self.table.getNumber("tid", 0)
        sd.putNumber("tid", self.tid)

        self.addRequirements(self.drivetrain)


    def initialize(self):
        print(f"AutoAlignScoringCommand: Initializing with desired tx = {self.desired_tx}, desired ty = {self.desired_ty}")
        # self.turnPID.reset()
        # self.drivePID.reset()
        self.start_time = time.time()

        

    def execute(self):
        # # Get current tx and ty from Limelight
        current_tx = self.table.getNumber("tx", 0)
        current_ty = self.table.getNumber("ty", 0)
        current_tv = self.table.getNumber("tv", 0)
        

        elapsed_time = time.time() - self.start_time
        
        # p = sd.getNumber("turn p", 0.0)
        # i = sd.getNumber("turn i", 0.0)
        # d = sd.getNumber("turn d", 0.0)

        # self.turnPID.setP(p)
        # self.turnPID.setI(i)
        # self.turnPID.setD(d)

        # dp = sd.getNumber("drive p", 0.0)
        # di = sd.getNumber("drive i", 0.0)
        # dd = sd.getNumber("drive d", 0.0)

        # self.drivePID.setP(dp)
        # self.drivePID.setI(di)
        # self.drivePID.setD(dd)

        # print ("dp", dp)


        deadband_forward = 1
        # Compute errors.
       
        error_tx = self.desired_tx - current_tx
        error_ty = self.desired_ty - current_ty
        sd.putNumber("error_tx", error_tx)
        sd.putNumber("error_ty", error_ty)

        # Compute turn correction using the PID controller (for tx)
        # turn_correction = self.turnPID.calculate(current_tx) 
        turn_correction = self.turnPID.calculate(current_tx)  # Limits turn speed to ±0.3

        
        # Compute forward correction using a simple proportional controller on ty error.
        # The negative sign may be needed depending on the camera's coordinate convention.
        # forward_correction = error_ty * self.kP_fwd

        forward_correction = self.drivePID.calculate(current_ty)  # Limits forward speed to ±0.3
        forward = max(min(forward_correction, 0.15), -0.15)
        
        # if abs(forward) > deadband_forward:
        #     forward = 0

        # Command the drivetrain.
        # Here we assume the first parameter is forward speed.
        if current_tv >= 1.0 and self.tid == self.targetTag:
            print("I see my target...")
            self.drivetrain.drive(-forward, turn_correction * 0.25, turn_correction * 0.01, False)
            print(f"AutoAlignScoringCommand: current_tx = {current_tx} error_tx = {error_tx}, turn_correction = {turn_correction}")
            print(f"                     current_ty = {current_ty}, error_ty = {error_ty}, forward_correction = {forward}")
        elif current_tv == 0:
            self.drivetrain.drive(0, 0, -0.07, False)
            print("DriveToLimelightTarget: No target detected")
        elif current_tv == 0 and elapsed_time > 5:
            self.drivetrain.drive(0, 0, 0, False)
            print("DriveToLimelightTarget: No target detected")

    
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
        if interrupted is True:
            return True
        
