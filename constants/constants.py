# #!/usr/bin/env python3

from math import pi
# from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath import units

class DriveConstants:
    # Constants for the swerve module 

    # Motion and Drive control limiters
    # Slew rate limits for different drive parameters
    # K_DIRECTION_SLEW_RATE = 1.2  # radians per second
    # K_MAGNITUDE_SLEW_RATE = 1.8  # percent per second (1 = 100%)
    # K_ROTATIONAL_SLEW_RATE = 2.0  # percent per second (1 = 100%)

    # # Speed limiters for drive and rotation
    # K_DRIVING_SPEED_LIMITER = 1  # Adjust this value as needed
    # K_ROTATION_SPEED_LIMITER = 1  # Adjust this value as needed

    # # Voltage limiter for controlling motor voltage
    # K_VOLTAGE_LIMITER = SlewRateLimiter(0.5)  # Assuming SlewRateLimiter takes a rate value (in volts or time scale)

# Drivetrain PID Constants
    K_DRIVE_P = 0.1
    K_DRIVE_I = 0
    K_DRIVE_D = 0

    K_DRIVE_KS = 0.1
    K_DRIVE_KV = 1

    K_TURN_P = 1
    K_TURN_I = 0
    K_TURN_D = 0

    K_TURN_KS = 0.1
    K_TURN_KV = 1

    K_DRIVING_MOTOR_PINION_TEETH = 13

    K_WHEEL_DIAMETER_METERS = 0.0762
    K_WHEEL_RADIUS_METERS = K_WHEEL_DIAMETER_METERS / 2 # ~ 0.0508
    K_WHEEL_CIRCUMFERENCE_METERS = K_WHEEL_DIAMETER_METERS * pi
   
    K_DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (K_DRIVING_MOTOR_PINION_TEETH * 15)
    K_FREE_SPEED_RPM = 5676 # drive motor
    K_DRIVING_MOTOR_FREE_SPEED_RPM = K_FREE_SPEED_RPM / 60

    K_DRIVE_WHEEL_FREE_SPEED_RPS = (K_DRIVING_MOTOR_FREE_SPEED_RPM * K_WHEEL_CIRCUMFERENCE_METERS) / K_DRIVING_MOTOR_REDUCTION   
    
    # Movement Constants
    # Physical robot movement limits (in feet and meters)
    K_MAX_SPEED_METERS_PER_SECOND = 4.46
    K_MAX_ANGULAR_SPEED = 2 * pi

    # Physical max angular velocity (in meters per second)
    # K_PHYSICAL_MAX_ANGULAR_VELOCITY_METERS_PER_SECOND = K_PHYSICAL_MAX_SPEED_METERS_PER_SECOND / K_WHEEL_RADIUS_METERS  # rad/s

    # Teleop drive limits (adjustments for speed, angular speed, and acceleration)
    # K_TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND = K_PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 1  # to be adjusted
    # K_TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = (kMaxSpeedMetersPerSecond * 2 / K_WHEEL_DIAMETER_METERS) / 2  # rad/s
    # K_TELE_DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3  # to be adjusted
    # K_TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 3  # to be adjusted 

class OIConstants:
    # Constants for the Operator Interface
    K_DRIVER_CONTROLLER_PORT = 0
    K_OPERATOR_CONTROLLER_PORT = 1
    DEADZONE = 0.15

class RobotConstants:
    # Robot Dimemsions
    K_TRACK_WIDTH = units.inchesToMeters(23.375) # ~23 3/8 in, ~0.593725m
     # Distance between centers of right and left wheels
    K_WHEEL_BASE = units.inchesToMeters(23.375) # ~23 3/8 in, ~0.593725m
     # Distance between centers of front and back wheels

    K_GYRO_REVERSED = False
    
    # Front left module constants
    K_FRONT_LEFT_DRIVE_ID = 23
    K_FRONT_LEFT_TURN_ID = 22
    K_FRONT_LEFT_DRIVING_MOTOR_REVERSED = False
    K_FRONT_LEFT_TURNING_MOTOR_REVERSED = False
    K_FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -pi / 2

    # Front right module constants
    K_FRONT_RIGHT_DRIVE_ID = 21
    K_FRONT_RIGHT_TURN_ID = 20
    K_FRONT_RIGHT_DRIVING_MOTOR_REVERSED = True
    K_FRONT_RIGHT_TURNING_MOTOR_REVERSED = False
    K_FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0

    # Back left module constants
    K_BACK_LEFT_DRIVE_ID = 25
    K_BACK_LEFT_TURN_ID = 24
    K_BACK_LEFT_DRIVING_MOTOR_REVERSED = False
    K_BACK_LEFT_TURNING_MOTOR_REVERSED = False
    K_BACK_LEFT_CHASSIS_ANGULAR_OFFSET = pi

    # Back right module constants
    K_BACK_RIGHT_DRIVE_ID = 27
    K_BACK_RIGHT_TURN_ID = 26
    K_BACK_RIGHT_DRIVING_MOTOR_REVERSED = True
    K_BACK_RIGHT_TURNING_MOTOR_REVERSED = False
    K_BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = pi / 2


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

class Setpoint:
    K_CORAL_STATION = "Coral Station"
    K_ZERO = "0"
    K_LEVEL_2 = "2"
    K_LEVEL_3 = "3"
    K_POP = "Pop"

    class Arm:
        K_CORAL_STATION = -13.5 # 33 for rev
        K_ZERO = 0
        K_LEVEL_2 = -43 # 2 for rev
        K_LEVEL_3 = -43 # 2 for rev
        K_POP = -14

    class Elevator:
        K_CORAL_STATION = 0 
        K_ZERO = 0
        K_LEVEL_2 = 1
        K_LEVEL_3 = 30   # 100 for rev 
        K_POP = 0

    class Intake:
        K_FORWARD = 0.5 # 0.7 if we have issues
        K_REVERSE = -0.4

    # class Hang:
    #     # K_FORWARD = 0.1
    #     # K_REVERSE = -0.1
    #     K_UP_POSITION = 1
    #     K_DOWN_POSITION = -1  

    class Algae:
        K_LOAD_POSITION = -26
        K_LOCKED_POSITION = 0
        K_LAUNCH_POSITION = -17

        K_LOAD_POWER = -1
        K_SCORE_POWER = 0.5
        K_LOCKED_POWER = 0.0

class CoralSubsystemConstants:
    # can Ids
    K_ARM_MOTOR_CHANNEL = 31
    K_ELEVATOR_MOTOR_CHANNEL = 32 
    K_INTAKE_MOTOR_CHANNEL = 33 

    class Arm:
        K_P = 0.06
        K_I = 0
        K_D = 0

    class Elevator:
        K_P = 0.1
        K_I = 0
        K_D = 0
        K_F = 0.2

class AlgaeSubsystemConstants:
    #can Ids
    K_ANGLE_MOTOR_CHANNEL = 34
    K_ROLLER_MOTOR_CHANNEL = 35
    # PID for the angle motor
    K_P = 0.8
    K_I = 0
    K_D = 0

# class HangSubSystemConstants:
#     K_HANG_MOTOR_CHANNEL = 34
#     K_ENCODER_CONVERSION_FACTOR = 32.1 # 8192 / 255

#     K_P = 0.25
#     K_I = 0
#     K_D = 0    

class AutoConstants:
    K_MAX_SPEED_METERS_PER_SECOND = 2
    K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3
    K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = pi
    K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = pi
    
class BlinkinColor:
    ORANGE = 0.63
    GREEN = 0.77
    RED = 0.61
    BLUE = 0.85
    STROBE_BLUE = -0.09
    STROBE_RED = -0.11
