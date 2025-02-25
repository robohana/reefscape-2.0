from math import pi

"""
AutoConstants

The constants you use during autonomous mode
"""


class AutoConstants:
    # Maximum linear speed (in meters per second)
    K_MAX_SPEED_METERS_PER_SECOND = 3.0

    # Maximum linear acceleration (in meters per second squared)
    K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3.0

    # Maximum angular speed (in radians per second)
    K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = pi

    # Maximum angular acceleration (in radians per second squared)
    K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = pi