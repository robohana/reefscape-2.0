from wpimath import units
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.filter import SlewRateLimiter

from math import pi

from constants.module_constants import ModuleConstants


"""
This file should contain constants specific to the robot’s drivetrain and movement. These constants define things like maximum speed, deadzones, and drivetrain-specific settings.

Examples:
 - Drivetrain PID constants (if using PID control for driving)
 - Speed limits (e.g., max forward, rotational speed)
 - Field-relative driving settings

"""

class DriveConstants:

    K_WHEEL_DIAMETER_METERS = units.inchesToMeters(4) # ~ 0.1016 # wheel listed as "Wheel, Billet, 4"OD x 1.5"W (MK4/4i)""
    K_WHEEL_RADIUS_METERS = K_WHEEL_DIAMETER_METERS / 2 # ~ 0.0508

    K_DRIVE_MOTOR_GEAR_RATIO = 1 / 6.12  
    K_TURN_ENCODER_GEAR_RATIO = 1 / (150 / 7)  

    KINEMATICS = SwerveDrive4Kinematics(
        ModuleConstants.K_FRONT_LEFT_LOCATION,
        ModuleConstants.K_FRONT_RIGHT_LOCATION,
        ModuleConstants.K_BACK_LEFT_LOCATION,
        ModuleConstants.K_BACK_RIGHT_LOCATION
    )

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

# I don't know what to call these yet
    # Conversion from encoder rotations to meters (drive)
    K_DRIVE_ENCODER_ROT2METER = K_DRIVE_MOTOR_GEAR_RATIO * pi * K_WHEEL_DIAMETER_METERS

    # Conversion from encoder rotations to radians (turn)
    K_TURN_ENCODER_ROT2RAD = K_TURN_ENCODER_GEAR_RATIO * pi * 2  

    # Conversion from encoder RPM to meters per second (drive)
    K_DRIVE_ENCODER_RPM2METER_PER_SEC = K_DRIVE_ENCODER_ROT2METER / 60

    # Conversion from encoder RPM to radians per second (turn)
    K_TURN_ENCODER_RPM2RAD_PER_SEC = K_TURN_ENCODER_ROT2RAD / 60

# Movement Constants
    # Physical robot movement limits (in feet and meters)
    K_PHYSICAL_MAX_SPEED_FEET_PER_SECOND = 16.6  # ft/s from SwerveSpecialties website
    K_PHYSICAL_MAX_SPEED_METERS_PER_SECOND = K_PHYSICAL_MAX_SPEED_FEET_PER_SECOND / 3.281  # ~5.05968 m/s

    # Physical max angular velocity (in meters per second)
    K_PHYSICAL_MAX_ANGULAR_VELOCITY_METERS_PER_SECOND = K_PHYSICAL_MAX_SPEED_METERS_PER_SECOND / K_WHEEL_RADIUS_METERS  # rad/s

    # Teleop drive limits (adjustments for speed, angular speed, and acceleration)
    K_TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND = K_PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 1  # to be adjusted
    K_TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = (K_PHYSICAL_MAX_SPEED_METERS_PER_SECOND * 2 / K_WHEEL_DIAMETER_METERS) / 2  # rad/s
    K_TELE_DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3  # to be adjusted
    K_TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 3  # to be adjusted    