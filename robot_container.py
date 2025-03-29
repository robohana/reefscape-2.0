# Type: RobotContainer

'''
# The RobotContainer is where you will configure your robot. The container is where you will define your subsystems, commands, and button bindings. 
# The container is created in robot.py and passed to the TimedCommandRobot. 
# The TimedCommandRobot will call the getAutonomousCommand method to get the command to run during autonomous. 
# The container will also set the default commands for the subsystems

 '''

from commands2 import RunCommand, Command, InstantCommand
from commands2.button import CommandXboxController

from wpimath import applyDeadband
from wpimath.filter import SlewRateLimiter
from wpilib import SmartDashboard as sd, SendableChooser
from ntcore import NetworkTableInstance

from subsystems.coral_subsystem import CoralSubsystem
from subsystems.drivetrain import DriveSubsystem
from subsystems.algae_subsystem import AlgaeSubsystem

from constants.constants import OIConstants

from commands.SwerveJoystickCmd import SwerveJoystickCmd
from commands.auto_routines import SimpleAuto
from commands.coral_cmds import IntakeToZero, ScoreCoralL2, ScoreCoralL3, IntakeCoralStation
from commands.algae_cmds import AlgaeLoadCommand, AlgaeScoreCommand, AlgaeZeroCommand, AlgaeOnCommand
from commands.auto_routine_with_camera import SimpleScoreAutoRIGHT, SimpleScoreAutoLEFT
from commands.auto_align_scoring_cmd import AutoAlignScoringCommand
from commands.teleop_align_scoring_cmd import TeleopAlignScoringCommand


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
        self.algae = AlgaeSubsystem()

        # Controllers
        self.driver_controller = CommandXboxController(OIConstants.K_DRIVER_CONTROLLER_PORT)
        self.operator_controller = CommandXboxController(OIConstants.K_OPERATOR_CONTROLLER_PORT)  

        # Slew Rate Limiter to make joystick inputs more gental 1/3 sec from 0 to 1
        self.x_speed_limiter = SlewRateLimiter(3)
        self.y_speed_limiter = SlewRateLimiter(3)
        self.rot_limiter = SlewRateLimiter(3)

        self.table = NetworkTableInstance.getDefault().getTable("limelight")

        
        # April Tag Chooser
        self.targetAprilTagChooser = SendableChooser()
        self.targetAprilTagChooser.setDefaultOption("Tag 9", 9)
        self.targetAprilTagChooser.addOption("Tag 7", 7)
        self.targetAprilTagChooser.addOption("Tag 11", 11)
        sd.putData("Target AprilTag", self.targetAprilTagChooser)


        # Configure default commands
        self.set_default_command()

        # Configure the button bindings
        self.configure_button_bindings()

                                        
        
    # Set default command
    def set_default_command (self) -> None:
        # The left stick controlls translantion of the robot.
        # Turning is controled by the X axis of the right stick

        self.robot_drive.setDefaultCommand(
            RunCommand(
                lambda: self.robot_drive.drive(
                    # Apply Slew Rate Limiters then Apply Deadband
                    self.x_speed_limiter.calculate(applyDeadband(self.driver_controller.getLeftY() * -1, OIConstants.DEADZONE)),
                    self.y_speed_limiter.calculate(applyDeadband(self.driver_controller.getLeftX() * -1, OIConstants.DEADZONE)),
                    self.rot_limiter.calculate(applyDeadband(self.driver_controller.getRightX() * -1, OIConstants.DEADZONE)),
                    True
                ),
                self.robot_drive 
            ) 
        )

    # Configure the button bindings
    def configure_button_bindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        # """  
      
        """Coral Roller Controls"""        
        # OPERATOR Left Bumper -> Run Coral Intake
        self.operator_controller.leftBumper().whileTrue(self.coral.run_intake_command())
        # OPERATOR Right Bumper -> Run Coral Intake in Reverse
        self.operator_controller.rightBumper().whileTrue(self.coral.reverse_intake_command()) 

        """Elevator and Arm Controls for various positions"""
        # OPERATOR A Button -> Elevator/Arm to human player position
        self.operator_controller.a().onTrue(IntakeCoralStation(self.coral))
        # OPERATOR X Button -> Elevator/Arm to level 1 position
        self.operator_controller.x().onTrue(IntakeToZero(self.coral))
        # OPERATOR Y Button -> Elevator/Arm to level 2 position
        self.operator_controller.y().onTrue(ScoreCoralL2(self.coral))
        # OPERATOR B Button -> Elevator/Arm to level 3 position
        self.operator_controller.b().onTrue(ScoreCoralL3(self.coral))

        """Algae Controls"""    
        # OPERATOR D-PAD DOWN -> Algae Grabber/Roller to load position
        self.operator_controller.povDown().onTrue(AlgaeLoadCommand(self.algae))
        # OPERATOR D-PAD UP -> Algae Grabber/Roller to zero position
        self.operator_controller.povUp().onTrue(AlgaeZeroCommand(self.algae))
        # OPERATOR D PAD RIGHT -> Algae Grabber/Roller to load position
        self.operator_controller.povRight().onTrue(AlgaeScoreCommand(self.algae))
        # OPERATOR LEFT TRIGGER -> Algae Roller On
        self.operator_controller.leftTrigger().whileTrue(AlgaeOnCommand(self.algae))

        """Zero Swerve Heading Control"""
        # DRIVER A Button -> Zero swerve heading
        self.driver_controller.a().onTrue(InstantCommand(lambda: self.robot_drive.zero_heading(), self.robot_drive))

        """Auto Align Controls"""
        # Bind scoring left to x:
        self.driver_controller.x().whileTrue(
            TeleopAlignScoringCommand(self.robot_drive, 'left', 2.5, 1.25)
        )

        # Bind scoring right to b:
        self.driver_controller.b().whileTrue(
            TeleopAlignScoringCommand(self.robot_drive, 'right', 2.5, 1.25)
        )


    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""
        pass

    def getAutonomousCommand(self) -> Command:
        """Selects and returns the autonomous command based on the chooser settings."""
        targetTag = self.targetAprilTagChooser.getSelected()



        return SimpleScoreAutoLEFT(self.robot_drive, self.coral, targetTag)
    
    def periodic(self) -> None:
        current_tv = self.table.getNumber("tv", 0)
        sd.putBoolean("I see April Tag", current_tv)
