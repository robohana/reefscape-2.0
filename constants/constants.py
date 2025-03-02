# #!/usr/bin/env python3


from math import pi
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath import kinematics, units



class DriveConstants:
    # Constants for the swerve module 

    # Motion and Drive control limiters
    # Slew rate limits for different drive parameters
    K_DIRECTION_SLEW_RATE = 1.2  # radians per second
    K_MAGNITUDE_SLEW_RATE = 1.8  # percent per second (1 = 100%)
    K_ROTATIONAL_SLEW_RATE = 2.0  # percent per second (1 = 100%)

    # Speed limiters for drive and rotation
    K_DRIVING_SPEED_LIMITER = 1  # Adjust this value as needed
    K_ROTATION_SPEED_LIMITER = 1  # Adjust this value as needed

    # Voltage limiter for controlling motor voltage
    K_VOLTAGE_LIMITER = SlewRateLimiter(0.5)  # Assuming SlewRateLimiter takes a rate value (in volts or time scale)

# Drivetrain PID Constants
    K_DRIVE_P = 0.1
    K_DRIVE_I = 0
    K_DRIVE_D = 0

    K_DRIVE_KS = 0.1
    K_DRIVE_KV = 1

    K_TURN_P = 0.01
    K_TURN_I = 0
    K_TURN_D = 0

    K_TURN_KS = 0.1
    K_TURN_KV = 0.227

    kDrivingMotorPinionTeeth = 14 

    K_WHEEL_DIAMETER_METERS = 0.0762
    K_WHEEL_RADIUS_METERS = K_WHEEL_DIAMETER_METERS / 2 # ~ 0.0508
    kWheelCircumferenceMeters = K_WHEEL_DIAMETER_METERS * pi

   
    kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
    kFreeSpeedRpm = 5676
    kDrivingMotorFreeSpeedRps = kFreeSpeedRpm / 60

    kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction   
    
    # I don't know what to call these yet
    # Conversion from encoder rotations to meters (drive)
    K_DRIVE_ENCODER_ROT2METER = kDrivingMotorReduction * pi * K_WHEEL_DIAMETER_METERS

    # Conversion from encoder rotations to radians (turn)
    # K_TURN_ENCODER_ROT2RAD = K_TURN_ENCODER_GEAR_RATIO * pi * 2  

    # Conversion from encoder RPM to meters per second (drive)
    K_DRIVE_ENCODER_RPM2METER_PER_SEC = K_DRIVE_ENCODER_ROT2METER / 60

    # Conversion from encoder RPM to radians per second (turn)
    # K_TURN_ENCODER_RPM2RAD_PER_SEC = K_TURN_ENCODER_ROT2RAD / 60


    # Movement Constants
    # Physical robot movement limits (in feet and meters)
    K_PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 4.8

    # Physical max angular velocity (in meters per second)
    K_PHYSICAL_MAX_ANGULAR_VELOCITY_METERS_PER_SECOND = K_PHYSICAL_MAX_SPEED_METERS_PER_SECOND / K_WHEEL_RADIUS_METERS  # rad/s

    # Teleop drive limits (adjustments for speed, angular speed, and acceleration)
    K_TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND = K_PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 1  # to be adjusted
    K_TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = (K_PHYSICAL_MAX_SPEED_METERS_PER_SECOND * 2 / K_WHEEL_DIAMETER_METERS) / 2  # rad/s
    K_TELE_DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3  # to be adjusted
    K_TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 3  # to be adjusted 




class OIConstants:
    # Constants for the Operator Interface

    kDriverControllerPort = 0
    kOperatorControllerPort = 1
    deadzone = 0.1


class RobotConstants:

        # Robot Dimemsions
    K_TRACK_WIDTH = units.inchesToMeters(23.375) # ~0.584m
     # Distance between centers of right and left wheels
    K_WHEEL_BASE = units.inchesToMeters(23.375) # ~0.584m
     # Distance between centers of front and back wheels


    K_GYRO_REVERSED = False

    
    # Front left module constants
    K_FRONT_LEFT_DRIVE_ID = 23
    K_FRONT_LEFT_TURN_ID = 22
    K_FRONT_LEFT_DRIVING_MOTOR_REVERSED = False
    K_FRONT_LEFT_TURNING_MOTOR_REVERSED = False
    K_FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = pi / 4

    # Front right module constants
    K_FRONT_RIGHT_DRIVE_ID = 21
    K_FRONT_RIGHT_TURN_ID = 20
    K_FRONT_RIGHT_DRIVING_MOTOR_REVERSED = True
    K_FRONT_RIGHT_TURNING_MOTOR_REVERSED = False
    K_FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = -pi / 4

    # Back left module constants
    K_BACK_LEFT_DRIVE_ID = 25
    K_BACK_LEFT_TURN_ID = 24
    K_BACK_LEFT_DRIVING_MOTOR_REVERSED = False
    K_BACK_LEFT_TURNING_MOTOR_REVERSED = False
    K_BACK_LEFT_CHASSIS_ANGULAR_OFFSET = (3 * pi) / 4

    # Back right module constants
    K_BACK_RIGHT_DRIVE_ID = 27
    K_BACK_RIGHT_TURN_ID = 26
    K_BACK_RIGHT_DRIVING_MOTOR_REVERSED = True
    K_BACK_RIGHT_TURNING_MOTOR_REVERSED = False
    K_BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = -(3 * pi) / 4


    K_FRONT_LEFT_LOCATION = Translation2d(K_WHEEL_BASE / 2 , K_TRACK_WIDTH / 2) 
    K_FRONT_RIGHT_LOCATION = Translation2d(K_WHEEL_BASE / 2 , -K_TRACK_WIDTH / 2)
    K_BACK_LEFT_LOCATION = Translation2d(-K_WHEEL_BASE / 2 , K_TRACK_WIDTH / 2)
    K_BACK_RIGHT_LOCATION =  Translation2d(-K_WHEEL_BASE / 2 , -K_TRACK_WIDTH / 2)

        # Kinematics Constants
    KINEMATICS = SwerveDrive4Kinematics(
        K_FRONT_LEFT_LOCATION,
        K_FRONT_RIGHT_LOCATION,
        K_BACK_LEFT_LOCATION,
        K_BACK_RIGHT_LOCATION
    )



class AutoConstants:
    kMaxSpeedMetersPerSecond = 3
    kMaxAccelerationMetersPerSecondSquared = 3
    kMaxAngularSpeedRadiansPerSecond = pi
    kMaxAngularSpeedRadiansPerSecondSquared = pi
    
