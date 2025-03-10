# Type: RobotContainer

'''
# The RobotContainer is where you will configure your robot. The container is where you will define your subsystems, commands, and button bindings. 
# The container is created in robot.py and passed to the TimedCommandRobot. 
# The TimedCommandRobot will call the getAutonomousCommand method to get the command to run during autonomous. 
# The container will also set the default commands for the subsystems

 '''

from commands2 import RunCommand, Command
from commands2.button import CommandXboxController
from wpimath import applyDeadband
from subsystems.coral_subsystem import CoralSubsystem
from subsystems.hang_subsystem import HangSubsystem
from subsystems.drivetrain import DriveSubsystem
from constants import AutoConstants, OIConstants, Setpoint
from commands.SwerveJoystickCmd import SwerveJoystickCmd
from commands.hang_cmd import HangCmd
from commands.auto_routines import SimpleAuto


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:

        # The robot's subsystems
        self.robot_drive = DriveSubsystem()
        self.coral = CoralSubsystem()
        self.hang = HangSubsystem()

        # Controllers
        self.driver_controller = CommandXboxController(OIConstants.K_DRIVER_CONTROLLER_PORT)
        self.operator_controller = CommandXboxController(OIConstants.K_OPERATOR_CONTROLLER_PORT)  

        self.hang_lift_command = HangCmd(self.hang, self.driver_controller, Setpoint.Hang.K_UP_POSITION)
        self.hang_lower_command = HangCmd(self.hang, self.driver_controller, Setpoint.Hang.K_DOWN_POSITION)

        # Configure default commands
        self.set_default_command()
        # Configure the button bindings
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
        # The left stick controlls translantion of the robot.
        # Turning is controled by the X axis of the right stick

        self.robot_drive.setDefaultCommand(
            RunCommand(
                lambda: self.robot_drive.drive(
                    applyDeadband(self.driver_controller.getLeftY() * -1, OIConstants.DEADZONE),
                    applyDeadband(self.driver_controller.getLeftX() * -1, OIConstants.DEADZONE),
                    applyDeadband(self.driver_controller.getRightX() * -1, OIConstants.DEADZONE),
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
        
        # OPERATOR Left Bumper -> Run Coral Intake
        self.operator_controller.leftBumper().whileTrue(self.coral.run_intake_command())
        # OPERATOR Right Bumper -> Run Coral Intake in Reverse
        self.operator_controller.rightBumper().whileTrue(self.coral.reverse_intake_command()) 

        # OPERATOR B Button -> Elevator/Arm to human player position
        self.operator_controller.b().onTrue(self.coral.setSetpointCommand(Setpoint.K_CORAL_STATION))
        # OPERATOR X Button -> Elevator/Arm to level 2 position
        self.operator_controller.a().onTrue(self.coral.setSetpointCommand(Setpoint.K_LEVEL_2))
        # OPERATOR Y Button -> Elevator/Arm to level 3 position
        self.operator_controller.y().onTrue(self.coral.setSetpointCommand(Setpoint.K_LEVEL_3))

        # DRIVER Start Button -> Zero swerve heading
        # self.driver_controller.start().onTrue(self.robot_drive.zero_heading())
        
        # DRIVER Y Button -> Lift robot using power only
        # self.driver_controller.y().onTrue(self.hang.lift_command_power())
        # # DRIVER X Button -> Lower robot using power only
        # self.driver_controller.x().onTrue(self.hang.lower_command_power())

        # # DRIVER B Button -> Lift robot using setpoint only
        # self.driver_controller.b().onTrue(self.hang_lift_command())
        # # DRIVER A Button -> Lower robot using setpoint only
        # self.driver_controller.a().onTrue(self.hang_lower_command())

        self.driver_controller.y().onTrue(self.hang_lift_command)  # Press Y to go up
        self.driver_controller.a().onTrue(self.hang_lower_command)  # Press A to go down     

        # self.driver_controller.rightBumper(self.hang.power_zero())


    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""
        pass

    def getAutonomousCommand(self) -> Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        pass
        #return AutoCommands.SimpleAuto(self.robotDrive)
