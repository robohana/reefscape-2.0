# Type: RobotContainer
'''
# The RobotContainer is where you will configure your robot. The container is where you will define your subsystems, commands, and button bindings. 
#The container is created in robot.py and passed to the TimedCommandRobot. 
#The TimedCommandRobot will call the getAutonomousCommand method to get the command to run during autonomous. 
#The container will also set the default commands for the subsystems

 '''


import commands2
from commands2 import Command, CommandScheduler, cmd, InstantCommand, RunCommand, button
import wpimath
import wpilib
from wpilib import XboxController

from subsystems.coral_subsystem import CoralSubsystem
from constants import AutoConstants, OIConstants, Setpoint
from subsystems.drivetrain import DriveSubsystem
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


        self.driver_controller = wpilib.XboxController(OIConstants.K_DRIVER_CONTROLLER_PORT)
        self.operator_controller = wpilib.XboxController(OIConstants.K_OPERATOR_CONTROLLER_PORT)
        # The robot's subsystems
        self.robot_drive = DriveSubsystem()
        self.coral = CoralSubsystem()
        self.set_default_command()
        self.configure_button_bindings()


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

    def set_default_command (self) -> None:

        self.robot_drive.setDefaultCommand(
            RunCommand(
                lambda: self.robot_drive.drive(
                    wpimath.applyDeadband(self.driver_controller.getLeftY() * -1, OIConstants.DEADZONE),
                    wpimath.applyDeadband(self.driver_controller.getLeftX() * -1, OIConstants.DEADZONE),
                    wpimath.applyDeadband(self.driver_controller.getRightX() * -1, OIConstants.DEADZONE),
                    True
                ),
                self.robot_drive 
            ) 
        )


    def configure_button_bindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        # """
        # self.robotDrive.setDefaultCommand(SwerveJoystickCmd(self.robotDrive, self.driverController))
        if self.operator_controller.getLeftBumper(): CoralSubsystem.run_intake_command()
        if self.operator_controller.getRightBumper(): CoralSubsystem.reverse_intake_command()   

        if self.operator_controller.getBButton(): CoralSubsystem.setSetpointCommand(Setpoint.K_CORAL_STATION)
        if self.operator_controller.getAButton(): CoralSubsystem.setSetpointCommand(Setpoint.K_LEVEL_2) 
        if self.operator_controller.getYButton(): CoralSubsystem.setSetpointCommand(Setpoint.K_LEVEL_3) 

        # if self.driver_controller.start(): self.robot_drive.zero_heading


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
