# Type: Subsystem
''' 
This file contains the DriveSubsystem class, which represents the drive subsystem of the robot.

'''
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

from constants import RobotConstants, DriveConstants
from wpilib import SmartDashboard as sd
import swerveutils
from subsystems.MAXSwerveModule import MAXSwerveModule
from navx import AHRS
import navx
from ntcore import NetworkTableInstance, StructPublisher, StructArrayPublisher
import time
import commands2




last_print_time = 0  # Global variable to track last print time





class DriveSubsystem(Subsystem):
    """Represents the drive subsystem of the robot. """

    def __init__(self) -> None:
        super().__init__()

        self.nt = NetworkTableInstance.getDefault()
        self.swerveTable = self.nt.getTable("Swerve")

                #  **Create Struct Publishers for AdvantageScope**
        self.moduleStatesPublisher: StructArrayPublisher = (
            self.swerveTable.getStructArrayTopic("Drive/Modules/States", SwerveModuleState).publish()
        )
        self.desiredModuleStatesPublisher: StructArrayPublisher = (
            self.swerveTable.getStructArrayTopic("Drive/Modules/DesiredStates", SwerveModuleState).publish()
        )
        self.chassisSpeedsPublisher: StructPublisher = (
            self.swerveTable.getStructTopic("Drive/ChassisSpeeds", ChassisSpeeds).publish()
        )

        # Create swerve modules
        self.front_left = MAXSwerveModule(RobotConstants.K_FRONT_LEFT_DRIVE_ID, 
                                                   RobotConstants.K_FRONT_LEFT_TURN_ID, 
                                                   RobotConstants.K_FRONT_LEFT_CHASSIS_ANGULAR_OFFSET
)
        
        self.front_right = MAXSwerveModule(RobotConstants.K_FRONT_RIGHT_DRIVE_ID, 
                                                    RobotConstants.K_FRONT_RIGHT_TURN_ID, 
                                                    RobotConstants.K_FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET
)
        
        self.back_left = MAXSwerveModule(RobotConstants.K_BACK_LEFT_DRIVE_ID, 
                                                  RobotConstants.K_BACK_LEFT_TURN_ID, 
                                                  RobotConstants.K_BACK_LEFT_CHASSIS_ANGULAR_OFFSET 
)
        
        self.back_right = MAXSwerveModule(RobotConstants.K_BACK_RIGHT_DRIVE_ID, 
                                                   RobotConstants.K_BACK_RIGHT_TURN_ID, 
                                                   RobotConstants.K_BACK_RIGHT_CHASSIS_ANGULAR_OFFSET

)

        #self.gyro = wpilib.ADXRS450_Gyro()
        self.gyro = AHRS(comType=navx._navx.AHRS.NavXComType.kMXP_SPI)

        # Slew rate filter variables for controlling lateral acceleration
        # self.currentRotation = 0.0
        # self.currentTranslationDir = 0.0
        # self.currentTranslationMag = 0.0

        # self.magLimiter = SlewRateLimiter(DrivingConstants.kMagnitudeSlewRate)
        # self.rotLimiter = SlewRateLimiter(DrivingConstants.kRotationalSlewRate)
        # self.prevTime = wpilib.Timer.getFPGATimestamp()

        # Odometry class for tracking robot pose
        self.odometry = SwerveDrive4Odometry(
            RobotConstants.KINEMATICS,
            Rotation2d(),
            [
                self.front_left.get_position(),
                self.front_right.get_position(),
                self.back_left.get_position(),
                self.back_right.get_position()
            ]
        )

        thread = threading.Thread(target = self.zero_heading_after_delay)

        thread.start()

    def periodic(self) -> None:
        sd.putNumber("Gyro", self.getHeading())
        self.odometry.update(
            self.getRotation2d(), 
                (       
                    self.front_left.get_position(),
                    self.front_right.get_position(),
                    self.back_left.get_position(),
                    self.back_right.get_position()   
                )
            )
        
        # sd.putString("Robot Odometer", str(self.getModulePositionsOld()))
        sd.putString("Robot Location, x", str(self.get_pose().X()))
        sd.putString("Robot Location, y", str(self.get_pose().Y()))
        sd.putString("Robot Location, rotation", str(self.get_pose().rotation().degrees()))    



        # Publish swerve module states and chassis speeds for AdvantageScope
        #self.publishSwerveStates()



    def zero_heading_after_delay(self):
        try:
            time.sleep(1)
            self.gyro.reset()
        except Exception as e:
            print(f"Error resetting gyro: {e}")

    
    
    
    def get_pose(self) -> Pose2d:
        """Returns the currently-estimated pose of the robot.

        :returns: The pose.
        """
        return self.odometry.getPose()

    def resetOdometry(self, pose: Pose2d):
        self.odometry.resetPosition(
            self.getRotation2d(),
            (
                self.front_left.get_position(),
                self.front_right.get_position(),
                self.back_left.get_position(),
                self.back_right.get_position(),
            ),
            pose,
        )

    def drive(self, xSpeed: float, ySpeed: float, rot: float, fieldRelative: bool) -> None:
            """Method to drive the robot using joystick info.

            :param xSpeed:        Speed of the robot in the x direction (forward).
            :param ySpeed:        Speed of the robot in the y direction (strafe).
            :param rot:           Angular rate of the robot.
            :param fieldRelative: Whether the provided x and y speeds are relative to the
                                field.
            :param rateLimit:     Whether to enable rate limiting for smoother control.
            """

            print(f"Drive() Called - X:{xSpeed:.2f}, Y:{ySpeed:.2f}, Rot:{rot:.2f}")


            # if rateLimit:
            #     # Convert XY to polar for rate limiting
            #     inputTranslationDir = math.atan2(ySpeed, xSpeed)
            #     inputTranslationMag = math.hypot(xSpeed, ySpeed)

            #     # Calculate the direction slew rate based on an estimate of the lateral acceleration
            #     if self.currentTranslationMag != 0.0:
            #         directionSlewRate = abs(
            #             DrivingConstants.kDirectionSlewRate / self.currentTranslationMag
            #         )
            #     else:
            #         directionSlewRate = 500.0
            #         # some high number that means the slew rate is effectively instantaneous

            #     currentTime = wpilib.Timer.getFPGATimestamp()
            #     elapsedTime = currentTime - self.prevTime
            #     angleDif = swerveutils.angleDifference(
            #         inputTranslationDir, self.currentTranslationDir
            #     )
            #     if angleDif < 0.45 * math.pi:
            #         self.currentTranslationDir = swerveutils.stepTowardsCircular(
            #             self.currentTranslationDir,
            #             inputTranslationDir,
            #             directionSlewRate * elapsedTime,
            #         )
            #         self.currentTranslationMag = self.magLimiter.calculate(
            #             inputTranslationMag
            #         )

            #     elif angleDif > 0.85 * math.pi:
            #         # some small number to avoid floating-point errors with equality checking
            #         # keep currentTranslationDir unchanged
            #         if self.currentTranslationMag > 1e-4:
            #             self.currentTranslationMag = self.magLimiter.calculate(0.0)
            #         else:
            #             self.currentTranslationDir = swerveutils.wrapAngle(
            #                 self.currentTranslationDir + math.pi
            #             )
            #             self.currentTranslationMag = self.magLimiter.calculate(
            #                 inputTranslationMag
            #             )

            #     else:
            #         self.currentTranslationDir = swerveutils.stepTowardsCircular(
            #             self.currentTranslationDir,
            #             inputTranslationDir,
            #             directionSlewRate * elapsedTime,
            #         )
            #         self.currentTranslationMag = self.magLimiter.calculate(0.0)

            #     self.prevTime = currentTime

            #     xSpeedCommanded = self.currentTranslationMag * math.cos(
            #         self.currentTranslationDir
            #     )
            #     ySpeedCommanded = self.currentTranslationMag * math.sin(
            #         self.currentTranslationDir
            #     )
            #     self.currentRotation = self.rotLimiter.calculate(rot)

            # else:
            #     self.currentRotation = rot

            # Convert the commanded speeds into the correct units for the drivetrain
            x_speed_delivered = xSpeed * DriveConstants.K_PHYSICAL_MAX_SPEED_METERS_PER_SECOND
            y_speed_delivered = ySpeed * DriveConstants.K_PHYSICAL_MAX_SPEED_METERS_PER_SECOND
            rot_delivered = rot * DriveConstants.K_TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND

            swerveModuleStates = RobotConstants.KINEMATICS.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    x_speed_delivered,
                    y_speed_delivered,
                    rot_delivered,
                    Rotation2d.fromDegrees(self.gyro.getAngle()),
                )
                if fieldRelative
                else ChassisSpeeds(x_speed_delivered, y_speed_delivered, rot_delivered)
            )
            # fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            #     swerveModuleStates, DrivingConstants.kTeleDriveMaxSpeedMetersPerSecond
            # )
        
            SwerveDrive4Kinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.K_PHYSICAL_MAX_SPEED_METERS_PER_SECOND
            )

            fl, fr, bl, br = swerveModuleStates
            self.front_left.setDesiredState(fl)
            self.front_right.setDesiredState(fr)
            self.back_left.setDesiredState(bl)
            self.back_right.setDesiredState(br)
 
    def setX(self) -> None:
        """Sets the wheels into an X formation to prevent movement."""
        self.front_left.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.front_right.setDesiredState(
            SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        )
        self.back_left.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.back_right.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))        

    def setModuleStates(
        self,
        desiredStates: typing.Tuple[
            SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
        ],
    ) -> None:
        """Sets the swerve ModuleStates.

        :param desiredStates: The desired SwerveModule states.
        """
        
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.K_PHYSICAL_MAX_SPEED_METERS_PER_SECOND
            )

        fl, fr, bl, br = desiredStates

        self.front_left.setDesiredState(fl)
        self.front_right.setDesiredState(fr)
        self.back_left.setDesiredState(bl)
        self.back_right.setDesiredState(br)


    # def getModulePositionsOld(self) -> tuple[SwerveModulePosition, SwerveModulePosition,SwerveModulePosition,SwerveModulePosition]:
    #     return (
    #             SwerveModulePosition(self.front_left.get_position(), Rotation2d(self.front_left.get_position())),
    #             SwerveModulePosition(self.front_right.get_position(), Rotation2d(self.front_right.get_position())),
    #             SwerveModulePosition(self.back_left.get_position(), Rotation2d(self.back_left.get_position())),
    #             SwerveModulePosition(self.back_right.get_position(), Rotation2d(self.back_right.get_position()))
    #             )
    # def getModuleStates(self) -> tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]:
    #     return (
    #         SwerveModuleState(self.front_left.get_position(), Rotation2d(self.front_left.getAbsoluteEncoderRad())),
    #             SwerveModuleState(self.front_right.get_position(), Rotation2d(self.front_right.getAbsoluteEncoderRad())),
    #             SwerveModuleState(self.back_left.get_position(), Rotation2d(self.back_left.getAbsoluteEncoderRad())),
    #             SwerveModuleState(self.back_right.get_position(), Rotation2d(self.back_right.getAbsoluteEncoderRad()))
    #     )

    # def getChassisSpeeds(self):
    #     return DrivingConstants.kinematics.toChassisSpeeds(self.getModuleStates())
    
    # def driveChassisSpeeds(self, chassisSpeeds: ChassisSpeeds):
    #     self.setModuleStates(
    #         DrivingConstants.kinematics.toSwerveModuleStates(chassisSpeeds)
    #     )



    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        self.front_left.resetEncoders()
        self.front_right.resetEncoders()
        self.back_left.resetEncoders()
        self.back_right.resetEncoders()


    def zeroHeading(self) -> None:
        """Zeroes the heading of the robot."""
        self.gyro.reset()

    def getHeading(self) -> float:
        """Returns the heading of the robot.

        :returns: the robot's heading in degrees, from -180 to 180
        """
        return Rotation2d.fromDegrees(self.gyro.getAngle()).degrees()
        #return Rotation2d.fromDegrees(self.gyro.getAngle() * (-1.0 if DrivingConstants.KGyroReversed else 1.0)).degrees()


    def getRotation2d(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.getHeading())
    
        #^used when we are able to adjust gyro with apriltags
    def setHeading(self, angle):
        self.gyro.reset()
        self.gyro.setAngleAdjustment(angle)


    def getTurnRate(self) -> float:
        """Returns the turn rate of the robot.

        :returns: The turn rate of the robot, in degrees per second
        """
        return self.gyro.getRate() * (-1.0 if RobotConstants.K_GYRO_REVERSED else 1.0)
    

    def stop(self) -> None:
        """Stops the module."""
        self.drive(0, 0, 0, False)


    # def publishSwerveStates(self):
    #     """Publishes swerve module states and chassis speeds to AdvantageScope using correct struct format."""
        
    #     global last_print_time
    #     current_time = time.time()

    #     # Get all module states
    #     moduleStates = [
    #         self.frontLeft.getState(),
    #         self.frontRight.getState(),
    #         self.backLeft.getState(),
    #         self.backRight.getState(),
    #     ]

    #     #  **Publish `SwerveModuleState[]` as a structured array**
    #     self.moduleStatesPublisher.set(moduleStates)

    #     self.desiredModuleStatesPublisher.set([
    #         self.frontLeft.getDesiredState(),
    #         self.frontRight.getDesiredState(),
    #         self.backLeft.getDesiredState(),
    #         self.backRight.getDesiredState()
    #     ])

    #     #  **Publish `ChassisSpeeds` as a structured object**
    #     chassisSpeeds = DrivingConstants.kinematics.toChassisSpeeds(tuple(moduleStates))
    #     self.chassisSpeedsPublisher.set(chassisSpeeds)

    #     #  **Print only once per second**
    #     if current_time - last_print_time >= 1.0:
    #         print("Published Swerve Module States & Chassis Speeds (Struct Format)")
    #         print(f"Publishing Swerve Module States: {moduleStates}")
    #         print(f"Publishing Chassis Speeds: {chassisSpeeds}")
    #         last_print_time = current_time  # Update last print time

    #     #chassisSpeeds = DrivingConstants.kinematics.toChassisSpeeds(*moduleStates)



    #     #  Use setStruct to publish structured data for AdvantageScope
    #     self.swerveTable.getStructArrayTopic("ModuleStates", SwerveModuleState).publish().set(moduleStates)
    #     self.swerveTable.getStructTopic("ChassisSpeeds", ChassisSpeeds).publish().set(chassisSpeeds)


    def getModuleStates(self):
        """Returns the current states of all swerve modules."""
        return [
            self.front_left.getState(),
            self.front_right.getState(),
            self.back_left.getState(),
            self.back_right.getState(),
        ]






