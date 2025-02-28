import commands2, constants, wpilib, navx, threading, time, math
from wpimath.filter import SlewRateLimiter
from commands2 import Command 
from wpilib import XboxController
from commands2.button import CommandXboxController
import wpimath
from subsystems.drivetrain import DriveSubsystem
from wpimath.kinematics import ChassisSpeeds
from constants.oi_constants import OIConstants
from constants.drive_constants import DriveConstants
from wpilib import SmartDashboard as sd

class SwerveJoystickCmd(Command):

    def __init__(self, robot_drive: DriveSubsystem, driver_controller:XboxController):
        super().__init__()
        self.robot_drive = robot_drive
        self.driver_controller = driver_controller
        self.addRequirements(self.robot_drive)
        # create Slew limiter
        self.xLimiter = SlewRateLimiter(3)
        self.yLimiter = SlewRateLimiter(3)
        self.zRotLimiter = SlewRateLimiter(3)

    def initialize(self):
        pass
    
    def execute(self):
        # these are multiplied by the drivingSpeedLimiter which limit the speed of the robot so it doesn't go too fast
        self.xSpeed = -self.driver_controller.getLeftY() * 1 # self.drivingLimiter#* RobotConstants.kTeleopDriveMaxSpeedMetersPerSecond
        self.ySpeed = self.driver_controller.getLeftX() * 1 # self.drivingLimiter #* RobotConstants.kTeleopDriveMaxSpeedMetersPerSecond
        self.zRotation = self.driver_controller.getRightX() * -1  # self.drivingLimiter
        
        # 1. Get the joystick values and apply deadzone
        self.xSpeed = wpimath.applyDeadband(self.xSpeed, OIConstants.DEADZONE)
        self.ySpeed = wpimath.applyDeadband(self.ySpeed, OIConstants.DEADZONE)
        self.zRotation = wpimath.applyDeadband(self.zRotation, OIConstants.DEADZONE)
        sd.putNumber("xSpeed", self.xSpeed)
        

        # # 2. Add rateLimiter to smooth the joystick values
        self.xSpeed = self.xLimiter.calculate(self.xSpeed) * DriveConstants.K_PHYSICAL_MAX_SPEED_METERS_PER_SECOND
        self.ySpeed = self.yLimiter.calculate(self.ySpeed) * DriveConstants.K_PHYSICAL_MAX_SPEED_METERS_PER_SECOND
        self.zRotation = self.zRotLimiter.calculate(self.zRotation) * DriveConstants.K_TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND #DrivingConstants.kMaxSpeedMetersPerSecond * DrivingConstants.kMaxSpeedMetersPerSecond / 0.418480
        #! Not sure why this is needed, for some reason without it, the robot rotates slower 

        # if self.fieldOriented:
        chasisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(self.xSpeed, self.ySpeed, self.zRotation, self.robot_drive.get_rotation2d())
        sd.putNumber("chasisSpeeds", chasisSpeeds.vx)
        # else:
        # If robotOrinted is desired (But what's the fun in that?)
        #     chasisSpeeds = ChassisSpeeds(self.xSpeed, self.ySpeed, self.zRotation)

        # 3. convert chasis speeds to module states
        moduleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(chasisSpeeds)




        self.robot_drive.set_module_states(moduleStates)
        

    # def end(self, interrupted: bool):
    #     self.robot_drive.stop()

    def isFinished(self):
        return False
