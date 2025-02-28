import math
import typing
import time
import struct
import threading
from wpilib import DriverStation

import wpilib

from commands2 import Subsystem
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModuleState,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition
)

from constants.robot_constants import RobotConstants
from constants.module_constants import ModuleConstants
from constants.drive_constants import DriveConstants
# import swerveutils
from subsystems.MAX_swerve_module import MAXSwerveModule
from navx import AHRS
import navx
from ntcore import NetworkTableInstance, StructPublisher, StructArrayPublisher
import time
import commands2

from wpilib import SmartDashboard as sd

class DriveSubsystem(Subsystem):
    """Represents the drive subsystem of the robot. """

    def __init__(self) -> None:
        super().__init__()

        self.front_left = MAXSwerveModule(
            ModuleConstants.K_FRONT_LEFT_DRIVE_ID, 
            ModuleConstants.K_FRONT_LEFT_TURN_ID, 
            ModuleConstants.K_FRONT_LEFT_CHASSIS_ANGULAR_OFFSET
        )
        
        self.front_right = MAXSwerveModule(
            ModuleConstants.K_FRONT_RIGHT_DRIVE_ID, 
            ModuleConstants.K_FRONT_RIGHT_TURN_ID, 
            ModuleConstants.K_FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET
        )
        
        self.back_left = MAXSwerveModule(
            ModuleConstants.K_BACK_LEFT_DRIVE_ID, 
            ModuleConstants.K_BACK_LEFT_TURN_ID, 
            ModuleConstants.K_BACK_LEFT_CHASSIS_ANGULAR_OFFSET 
        )
        
        self.back_right = MAXSwerveModule(
            ModuleConstants.K_BACK_RIGHT_DRIVE_ID, 
            ModuleConstants.K_BACK_RIGHT_TURN_ID, 
            ModuleConstants.K_BACK_RIGHT_CHASSIS_ANGULAR_OFFSET
        )
        
        self.gyro = AHRS(comType=navx._navx.AHRS.NavXComType.kMXP_SPI)

        
        self.odometry = SwerveDrive4Odometry(
            DriveConstants.KINEMATICS,
            Rotation2d(),
            [
                self.front_left.get_position(),
                self.front_right.get_position(),
                self.back_left.get_position(),
                self.back_right.get_position()
            ]
        )

    def periodic(self) -> None:
        self.odometry.update(
            self.get_rotation2d(),
            [
                self.front_left.get_position(),
                self.front_right.get_position(),
                self.back_left.get_position(),
                self.back_right.get_position()
            ]
        )

# in our old drive sub file we next had def zero_heading_after_delay here

    def get_pose(self) -> Pose2d:
        """Returns the currently-estimated pose of the robot.

        :returns: The pose.
        """
        return self.odometry.getPose() # should be in meters and radians respecively

    def reset_odometry(self, pose: Pose2d):
        self.odometry.resetPosition(
            self.get_rotation2d(),
            (
                self.front_left.get_position(),
                self.front_right.get_position(),
                self.back_left.get_position(),
                self.back_right.get_position(),
            ),
            pose,
        )

    def drive(self, x_speed: float, y_speed: float, rot: float, field_relative: bool) -> None:
        """Method to drive the robot using joystick info.

        :param x_speed:        Speed of the robot in the x direction (forward).
        :param y_speed:        Speed of the robot in the y direction (strafe).
        :param rot:            Angular rate of the robot.
        :param field_relative: Whether the provided x and y speeds are relative to the
                        field.
        :param rate_limit:     Whether to enable rate limiting for smoother control.
        """
        x_speed_delivered = x_speed * DriveConstants.K_PHYSICAL_MAX_SPEED_METERS_PER_SECOND
        y_speed_delivered = y_speed * DriveConstants.K_PHYSICAL_MAX_SPEED_METERS_PER_SECOND
        rot_delivered = rot * DriveConstants.K_TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND

        if field_relative:
            chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                x_speed_delivered,
                y_speed_delivered,
                rot_delivered,
                Rotation2d.fromDegrees(self.gyro.getAngle())
            )
            print 
        else:
            chassis_speeds = ChassisSpeeds(
                x_speed_delivered,
                y_speed_delivered,
                rot_delivered
            ) 

        swerve_module_states = DriveConstants.KINEMATICS.toSwerveModuleStates(chassis_speeds)
        
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerve_module_states, DriveConstants.K_PHYSICAL_MAX_SPEED_METERS_PER_SECOND
            )

        fl, fr, bl, br = swerve_module_states
        print(f"FL: {fl}, FR: {fr}, BL: {bl}, BR: {br}")


        self.front_left.set_desired_state(fl)
        self.front_right.set_desired_state(fr)
        self.back_left.set_desired_state(bl)
        self.back_right.set_desired_state(br)
        #print (f"sms{swerve_module_states}")

    def set_x_command(self) -> None:
        """Sets the wheels into an X formation to prevent movement."""
        self.front_left.set_desired_state(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.front_right.set_desired_state(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.back_left.set_desired_state(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.back_right.set_desired_state(SwerveModuleState(0, Rotation2d.fromDegrees(45)))        

    def set_module_states(self, desired_states: typing.Tuple[
            SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
            ]) -> None:
        """Sets the swerve ModuleStates.

        :param desired_states: The desired SwerveModule states.
        """
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desired_states, DriveConstants.K_PHYSICAL_MAX_SPEED_METERS_PER_SECOND
            )

        fl, fr, bl, br = desired_states
        
        self.front_left.set_desired_state(fl)
        self.front_right.set_desired_state(fr)
        self.back_left.set_desired_state(bl)
        self.back_right.set_desired_state(br)
     
    # def get_module_positions_old(self) -> tuple[SwerveModulePosition, SwerveModulePosition,SwerveModulePosition,SwerveModulePosition]:
    #     return (
    #         SwerveModulePosition(self.front_left.get_position(), Rotation2d(self.front_left.get_absolute_encoder_rad())),
    #         SwerveModulePosition(self.front_right.get_position(), Rotation2d(self.front_right.get_absolute_encoder_rad())),
    #         SwerveModulePosition(self.back_left.get_position(), Rotation2d(self.back_left.get_absolute_encoder_rad())),
    #         SwerveModulePosition(self.back_right.get_position(), Rotation2d(self.back_right.get_absolute_encoder_rad()))
    #     )
    
# in our old drive sub file we next had def getModuleStates & getChassisSpeeds & driveChassisSpeeds here

    def reset_encoders(self) -> None:
        self.front_left.reset_encoders()
        self.front_right.reset_encoders()
        self.back_left.reset_encoders()
        self.back_right.reset_encoders()

    def zero_heading_command(self) -> None:
        """Zeroes the heading of the robot."""
        self.gyro.reset()

    def get_heading(self) -> float:
        """Returns the heading of the robot.

        :returns: the robot's heading in degrees, from -180 to 180
        """
        return Rotation2d.fromDegrees(self.gyro.getAngle()).degrees()

    def get_rotation2d(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.get_heading())
    
    def set_heading(self, angle):
        self.gyro.reset()
        self.gyro.setAngleAdjustment(angle)

    def get_turn_rate(self) -> float:
        """Returns the turn rate of the robot.

        :returns: The turn rate of the robot, in degrees per second
        """
        return self.gyro.getRate() * (-1.0 if RobotConstants.K_GYRO_REVERSED else 1.0)

# in our old drive sub file we next had def stop & publishSwerveStates & getModuleStates here



    
