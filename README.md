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

## Important Notes
### Constants Files/Folder
> All the constants that we will use in the actual code live inside of constants.constants \
> When using constants in other files, you import from constants.constants only what you need

### Imports
> Import only what you need so that our imports can run smoothly

## Motor Information
> Turn Sparks have even IDs \
> Drive Sparks have odd IDs

| Motor Name          | Can ID    |
| :---                |   :---:   |
| roboRIO             | 0         |
| PDH                 | 1         |
| FRONT_LEFT_DRIVE    | 23        |
| FRONT_LEFT_STEER    | 22        |
| FRONT_RIGHT_DRIVE   | 21        |
| FRONT_RIGHT_STEER   | 20        |
| BACK_LEFT_DRIVE     | 25        |
| BACK_LEFT_STEER     | 24        |
| BACK_RIGHT_DRIVE    | 27        |
| BACK_RIGHT_STEER    | 26        |
| INTAKE_MOTOR        | 33        |
| ARM_MOTOR           | 31        |
| ELEVATOR_MOTOR      | 32        |
| ANGLE_MOTOR (algee) | 34        |
| ROLLER_MOTOR (algee)| 35        |

