from wpimath import units

"""
This file should contain constants related to the overall robot setup and configuration. These constants define key components that are used across multiple subsystems or throughout the entire robot.

Examples:
 - Robot CAN IDs for motors, pneumatics, and other devices
 - Robot dimensions (e.g., robot width, length, height)
 - General robot configurations (e.g., robot weight, battery voltage limits)

"""
class RobotConstants:

    # Robot Dimemsions
    K_TRACK_WIDTH = units.inchesToMeters(23) # ~0.584m
     # Distance between centers of right and left wheels
    K_WHEEL_BASE = units.inchesToMeters(23) # ~0.584m
     # Distance between centers of front and back wheels


    K_GYRO_REVERSED = False
