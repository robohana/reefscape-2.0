import math

import commands2
from commands2 import Command, CommandScheduler, cmd, InstantCommand
import wpimath
import wpilib


from wpimath.controller import PIDController, ProfiledPIDControllerRadians
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import (
    TrajectoryConfig,
    TrajectoryGenerator,
    TrapezoidProfileRadians,
)
from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)

from subsystems.drivetrain import DriveSubsystem
from constants.oi_constants import OIConstants
from commands.SwerveJoystickCmd import SwerveJoystickCmd


import commands.auto_routines


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self.robot_drive = DriveSubsystem()

        # Other subsystems here

        self.driver_controller = wpilib.XboxController(OIConstants.K_DRIVER_CONTROLLER_PORT)


        # self.robot_drive.setDefaultCommand(
        #     RunCommand(
        #         lambda: self.robot_drive.drive(
        #             -wpimath.applyDeadband(self.driver_controller.getLeftY(), OIConstants.DEADZONE),
        #             -wpimath.applyDeadband(self.driver_controller.getLeftX(), OIConstants.DEADZONE),
        #             -wpimath.applyDeadband(self.driver_controller.getRightX(), OIConstants.DEADZONE),
        #             True,
        #         ),
        #         self.robot_drive
        #     )
        # )
        self.robot_drive.setDefaultCommand(SwerveJoystickCmd(self.robot_drive, self.driver_controller))
        
        
        
        self.configure_button_bindings()

            

    # def debug_drive(self):
    #     y = -wpimath.applyDeadband(self.driver_controller.getLeftY(), OIConstants.DEADZONE)
    #     x = -wpimath.applyDeadband(self.driver_controller.getLeftX(), OIConstants.DEADZONE)
    #     rotation = -wpimath.applyDeadband(self.driver_controller.getRightX(), OIConstants.DEADZONE)

    #     #print(f"Driving: Y={y}, X={x}, Rotation={rotation}")
    #     self.robot_drive.drive(y, x, rotation, True)
    #     #print ("in containers")
        
    def configure_button_bindings(self) -> None:
        pass
        # if abs(self.driver_controller.getLeftY()) > 0.1:  # Threshold for when the stick is held
        #     self.robot_drive.set_x_command()  # Trigger the command based on stick position

    def get_autonomous_command(self) -> commands2.Command:
        return commands.auto_routines
       

