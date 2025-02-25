from wpimath.geometry import Translation2d

from math import pi

from constants.robot_constants import RobotConstants


"""
This file should contain constants that are specific to individual subsystems (e.g., drive modules, arms, elevators, etc.). These constants define configurations, limits, and specifications for components within a particular subsystem.

Examples:
 - Motor constants for individual motors (e.g., max speeds, motor type)
 - Pneumatic constants (e.g., solenoid channels)
 - Sensor constants (e.g., encoder resolutions)

"""

class ModuleConstants:

# Wheel Positions
    K_FRONT_LEFT_LOCATION = Translation2d(RobotConstants.K_WHEEL_BASE / 2 , RobotConstants.K_TRACK_WIDTH / 2) 
    K_FRONT_RIGHT_LOCATION = Translation2d(RobotConstants.K_WHEEL_BASE / 2 , -RobotConstants.K_TRACK_WIDTH / 2)
    K_BACK_LEFT_LOCATION = Translation2d(-RobotConstants.K_WHEEL_BASE / 2 , RobotConstants.K_TRACK_WIDTH / 2)
    K_BACK_RIGHT_LOCATION =  Translation2d(-RobotConstants.K_WHEEL_BASE / 2 , -RobotConstants.K_TRACK_WIDTH / 2)



# Sensor Constants
    K_ENCODER_RESOLUTION = 4096 # from thrifty bot abs enc user manual



# Indiviual Module Constants

    # Front left module constants
    K_FRONT_LEFT_DRIVE_ID = 20
    K_FRONT_LEFT_TURN_ID = 31
    K_FRONT_LEFT_ABSOLUTE_ENCODER_ID = 3
    K_FRONT_LEFT_DRIVING_MOTOR_REVERSED = False
    K_FRONT_LEFT_TURNING_MOTOR_REVERSED = False
    K_FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET = -pi / 2  # recalibrate this value
    K_FRONT_LEFT_ABSOLUTE_ENCODER_REVERSED = False  # TODO: check if this is correct -JL on 2/3/25
    K_FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -pi / 2

    # Front right module constants
    K_FRONT_RIGHT_DRIVE_ID = 23
    K_FRONT_RIGHT_TURN_ID = 22
    K_FRONT_RIGHT_ABSOLUTE_ENCODER_ID = 0
    K_FRONT_RIGHT_DRIVING_MOTOR_REVERSED = True
    K_FRONT_RIGHT_TURNING_MOTOR_REVERSED = False
    K_FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET = -pi / 2  # recalibrate this value
    K_FRONT_RIGHT_ABSOLUTE_ENCODER_REVERSED = False  # TODO: check if this is correct -JL on 2/3/25
    K_FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = -pi / 2

    # Back left module constants
    K_BACK_LEFT_DRIVE_ID = 24
    K_BACK_LEFT_TURN_ID = 30
    K_BACK_LEFT_ABSOLUTE_ENCODER_ID = 2
    K_BACK_LEFT_DRIVING_MOTOR_REVERSED = False
    K_BACK_LEFT_TURNING_MOTOR_REVERSED = False
    K_BACK_LEFT_ABSOLUTE_ENCODER_OFFSET = -pi / 2  # recalibrate this value
    K_BACK_LEFT_ABSOLUTE_ENCODER_REVERSED = False  # TODO: check if this is correct -JL on 2/3/25
    K_BACK_LEFT_CHASSIS_ANGULAR_OFFSET = -pi / 2

    # Back right module constants
    K_BACK_RIGHT_DRIVE_ID = 21
    K_BACK_RIGHT_TURN_ID = 29
    K_BACK_RIGHT_ABSOLUTE_ENCODER_ID = 1
    K_BACK_RIGHT_DRIVING_MOTOR_REVERSED = True
    K_BACK_RIGHT_TURNING_MOTOR_REVERSED = False
    K_BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET = -pi / 2  # recalibrate this value
    K_BACK_RIGHT_ABSOLUTE_ENCODER_REVERSED = False  # TODO: check if this is correct -JL on 2/3/25
    K_BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = -pi / 2

