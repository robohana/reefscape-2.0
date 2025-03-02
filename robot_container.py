# Type: RobotContainer
'''
# The RobotContainer is where you will configure your robot. The container is where you will define your subsystems, commands, and button bindings. 
#The container is created in robot.py and passed to the TimedCommandRobot. 
#The TimedCommandRobot will call the getAutonomousCommand method to get the command to run during autonomous. 
#The container will also set the default commands for the subsystems

 '''

import math

import commands2
from commands2 import Command, CommandScheduler, cmd, InstantCommand, RunCommand
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

from constants import AutoConstants, OIConstants
from subsystems.drivesubsystem import DriveSubsystem
from TeleopCommands.SwerveJoystickCmd import SwerveJoystickCmd

import AutoCommands.SimpleAuto


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:

        self.driver_controller = wpilib.XboxController(OIConstants.kDriverControllerPort)
        self.operatorController = wpilib.XboxController(OIConstants.kOperatorControllerPort)
        # The robot's subsystems
        self.robot_drive = DriveSubsystem()
        self.setDefaultCommand()
        self.configureButtonBindings()


        # self.robotDrive.setDefaultCommand(
        #     SwerveJoystickCmd(self.robotDrive, self.driverController)
        # )

        # # Configure default commands
        # self.robotDrive.setDefaultCommand(
        #     SwerveJoystickCmd(
        #         robotDrive = self.robotDrive, driverController = self.driverController
        #     )
        # )
        
        # Configure the button bindings

    def setDefaultCommand (self) -> None:
        self.robot_drive.setDefaultCommand(
            RunCommand(
                lambda: self.robot_drive.drive(
                    -wpimath.applyDeadband(self.driver_controller.getLeftY(), OIConstants.deadzone),
                    -wpimath.applyDeadband(self.driver_controller.getLeftX(), OIConstants.deadzone),
                    -wpimath.applyDeadband(self.driver_controller.getRightX(), OIConstants.deadzone),
                    True
                ),
                self.robot_drive 
            ) 
        )


    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        # """
        # self.robotDrive.setDefaultCommand(SwerveJoystickCmd(self.robotDrive, self.driverController))

    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""
        pass

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        pass
        #return AutoCommands.SimpleAuto(self.robotDrive)
