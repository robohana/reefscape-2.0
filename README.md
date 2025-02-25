# 2025
Ohana Robotics code for 2025 FRC season

## Language & Version
Python 3.11.9

## Naming Conventions
We use the standard PEP8 style from Python.

Folders: \
```/commands``` # Holds command-based structures \
```/subsystems``` # Each subsystem gets their own file \
```/utils``` # Helper functions & utility classes \
```/vision``` # For Limelight or OpenCV processing \
```/constant```s # All constant values \


File Names: \
```drivetrain.py``` # Subsystem for drivetrain \ 
```arm.py``` # Subsystem for arm  \
```auto_routines.py``` # Autonomous command sequences  \
```teleop.py``` # Teleop controls \
```limelight.py``` # Limelight vision processing \
```utils.py``` # Utility functions \

Variable Naming (**snake_case**): \
 • Descriptive but concise. \
 • Avoid single-letter names (except for loop indices like i, j). \
 • Use _ as a separator. \

Class Naming (**PascalCase**:) \
 • ```subsystems```, ```commands```, and helper classes should be PascalCase. \
 • If a class represents a command, use Command at the end. \


## Motor Information
> This all needs to be filled in **AFTER** we update all the motor controllers

| Motor Name        | Can ID    |
| :---              |   :---:   |
| roboRIO           | 0         |
| PDH               | _         |
| FRONT_LEFT_DRIVE  | _         |
| FRONT_LEFT_STEER  | _         |
| FRONT_RIGHT_DRIVE | _         |
| FRONT_RIGHT_STEER | _         |
| BACK_LEFT_DRIVE   | _         |
| BACK_LEFT_STREER  | _         |
| BACK_RIGHT_DRIVE  | _         |
| BACK_RIGHT_STEER  | _         |
