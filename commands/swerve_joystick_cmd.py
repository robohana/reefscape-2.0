from constants.constants import OIConstants, RobotConstants
from wpimath.filter import SlewRateLimiter
from commands2 import Command 
from wpilib import XboxController
from wpimath import applyDeadband
from subsystems.drivetrain import DriveSubsystem
from wpimath.kinematics import ChassisSpeeds
from wpilib import SmartDashboard as sd

class SwerveJoystickCommand(Command):
    def __init__(self, drivetrain: DriveSubsystem, driver_controller:XboxController):
        super().__init__()
        self.drivetrain = drivetrain
        self.driver_controller = driver_controller
        self.addRequirements(self.drivetrain)
        # create Slew limiter
        self.x_limiter = SlewRateLimiter(1)
        self.y_limiter = SlewRateLimiter(1)
        self.z_rot_limiter = SlewRateLimiter(1)

    def initialize(self):
        pass
    
    def execute(self):
        # these are multiplied by the drivingSpeedLimiter which limit the speed of the robot so it doesn't go too fast
        self.x_speed = self.driver_controller.getLeftY() 
        self.y_speed = self.driver_controller.getLeftX() 
        self.z_rotation = self.driver_controller.getRightX() 
        
        # 1. Get the joystick values and apply deadzone
        self.x_speed = applyDeadband(self.x_speed, OIConstants.DEADZONE)
        self.y_speed = applyDeadband(self.y_speed, OIConstants.DEADZONE)
        self.z_rotation = applyDeadband(self.z_rotation, OIConstants.DEADZONE)
        
        # if self.fieldOriented:
        chasis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(self.x_speed, self.y_speed, self.z_rotation, self.drivetrain.get_rotation_2d())
        sd.putNumber("chasisSpeeds", chasis_speeds.vx)

        # 3. convert chasis speeds to module states
        module_states = RobotConstants.KINEMATICS.toSwerveModuleStates(chasis_speeds)

        self.drivetrain.set_module_states(module_states)

    def end(self, interrupted: bool):
        self.drivetrain.stop()

    def isFinished(self):
        return False
