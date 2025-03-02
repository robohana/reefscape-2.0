from constants import OIConstants, DriveConstants, RobotConstants
from wpimath.filter import SlewRateLimiter
from commands2 import Command 
from wpilib import XboxController
import wpimath
from subsystems.drivesubsystem import DriveSubsystem
from wpimath.kinematics import ChassisSpeeds
from wpilib import SmartDashboard as sd

class SwerveJoystickCmd(Command):

    def __init__(self, robotDrive: DriveSubsystem, driverController:XboxController):
        super().__init__()
        self.robotDrive = robotDrive
        self.driverController = driverController
        self.addRequirements(self.robotDrive)
        # create Slew limiter
        self.xLimiter = SlewRateLimiter(1)
        self.yLimiter = SlewRateLimiter(1)
        self.zRotLimiter = SlewRateLimiter(1)

        

    def initialize(self):
        pass
    
    def execute(self):
        # these are multiplied by the drivingSpeedLimiter which limit the speed of the robot so it doesn't go too fast
        self.xSpeed = -self.driverController.getLeftX() * DriveConstants.K_PHYSICAL_MAX_SPEED_METERS_PER_SECOND # self.drivingLimiter#* RobotConstants.kTeleopDriveMaxSpeedMetersPerSecond
        self.ySpeed = -self.driverController.getLeftY() * DriveConstants.K_PHYSICAL_MAX_SPEED_METERS_PER_SECOND # self.drivingLimiter #* RobotConstants.kTeleopDriveMaxSpeedMetersPerSecond
        self.zRotation = -self.driverController.getRightX() * DriveConstants.K_TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND # self.drivingLimiter
        
        # 1. Get the joystick values and apply deadzone
        self.xSpeed = wpimath.applyDeadband(self.xSpeed, OIConstants.deadzone)
        self.ySpeed = wpimath.applyDeadband(self.ySpeed, OIConstants.deadzone)
        self.zRotation = wpimath.applyDeadband(self.zRotation, OIConstants.deadzone)
        
        # if self.fieldOriented:
        chasisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(self.xSpeed, self.ySpeed, self.zRotation, self.robotDrive.getRotation2d())
        sd.putNumber("chasisSpeeds", chasisSpeeds.vx)
        # else:
        # If robotOrinted is desired (But what's the fun in that?)
        #     chasisSpeeds = ChassisSpeeds(self.xSpeed, self.ySpeed, self.zRotation)

        # 3. convert chasis speeds to module states
        moduleStates = RobotConstants.KINEMATICS.toSwerveModuleStates(chasisSpeeds)

        self.robotDrive.setModuleStates(moduleStates)
        

    def end(self, interrupted: bool):
        self.robotDrive.stop()

    def isFinished(self):
        return False
