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

# from ntcore import NetworkTableInstance

from subsystems.coral_subsystem import CoralSubsystem
from subsystems.drivetrain import DriveSubsystem
from subsystems.algae_subsystem import AlgaeSubsystem

from constants.constants import OIConstants

from commands.coral_cmds import IntakeToZero, ScoreCoralL2, ScoreCoralL3, IntakeCoralStation
from commands.algae_cmds import AlgaeLoadCommand, AlgaeScoreCommand, AlgaeZeroCommand, AlgaeOnCommand, AlgaeCoralCommand
from commands.auto_routine_with_camera import SimpleScoreAutoRIGHT, SimpleScoreAutoLEFT
from commands.teleop_align_scoring_cmd import TeleopAlignScoringCommand
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
        self.drivetrain = DriveSubsystem()
        self.coral = CoralSubsystem()
        self.algae = AlgaeSubsystem()

        # Controllers
        self.driver_controller = CommandXboxController(OIConstants.K_DRIVER_CONTROLLER_PORT)
        self.operator_controller = CommandXboxController(OIConstants.K_OPERATOR_CONTROLLER_PORT)  

        # Slew Rate Limiter to make joystick inputs more gental 1/3 sec from 0 to 1
        self.x_speed_limiter = SlewRateLimiter(3)
        self.y_speed_limiter = SlewRateLimiter(3)
        self.rot_limiter = SlewRateLimiter(3)

        # self.table = NetworkTableInstance.getDefault().getTable("limelight")

        # April Tag Chooser
        # self.targetAprilTagChooser = SendableChooser()
        # self.targetAprilTagChooser.setDefaultOption("Tag 9 - Red(RL)", 9) # RL: Robot Left
        # self.targetAprilTagChooser.addOption("Tag 11 - Red(RR)", 11) # RR: Robot Right
        # self.targetAprilTagChooser.addOption("Tag 20 - Blue(RR)", 20)
        # self.targetAprilTagChooser.addOption("Tag 22 - Blue(RL)", 22)
        # sd.putData("Target AprilTag", self.targetAprilTagChooser)
        # targetTag = self.targetAprilTagChooser.getSelected()

        self.auto_chooser = SendableChooser()
        self.auto_chooser.setDefaultOption("9 - Drive and Score", SimpleScoreAutoLEFT(self.drivetrain, self.coral, 9)) # Auto align score Left and l2 from april tag 9
        self.auto_chooser.addOption("11 - Drive and Score", SimpleScoreAutoLEFT(self.drivetrain, self.coral, 11)) # Auto align score Left and l2 from april tag 11 
        self.auto_chooser.addOption("20 - Drive and Score", SimpleScoreAutoLEFT(self.drivetrain, self.coral, 20)) # Auto align score Left and l2 from april tag 20
        self.auto_chooser.addOption("22 - Drive and Score", SimpleScoreAutoLEFT(self.drivetrain, self.coral, 22)) # Auto align score Left and l2 from april tag 22
        self.auto_chooser.addOption("Drive", SimpleAuto(self.drivetrain)) # drive backwards for 4 seconds
        sd.putData("Auto Chooser", self.auto_chooser)

        # print ("target", targetTag)

        # Configure default commands
        self.set_default_command()

        # Configure the button bindings
        self.configure_button_bindings()
                           
    # Set default command
    def set_default_command (self) -> None:
        # The left stick controlls translantion of the robot.
        # Turning is controled by the X axis of the right stick

        self.drivetrain.setDefaultCommand(
            RunCommand(
                lambda: self.drivetrain.drive(
                    # Apply Slew Rate Limiters then Apply Deadband
                    self.x_speed_limiter.calculate(applyDeadband(self.driver_controller.getLeftY() * -1, OIConstants.DEADZONE)),
                    self.y_speed_limiter.calculate(applyDeadband(self.driver_controller.getLeftX() * -1, OIConstants.DEADZONE)),
                    self.rot_limiter.calculate(applyDeadband(self.driver_controller.getRightX() * -1, OIConstants.DEADZONE)),
                    True
                ),
                self.drivetrain 
            ) 
        )

    # Configure the button bindings
    def configure_button_bindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """  
      
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
        # OPERATOR D PAD RIGHT -> Algae Grabber/Roller to score position
        self.operator_controller.povRight().onTrue(AlgaeScoreCommand(self.algae))
        # OPERATOR D PAD Left -> Algae Grabber/Roller to launch coral position
        self.operator_controller.povLeft().onTrue(AlgaeCoralCommand(self.algae))
        # OPERATOR LEFT TRIGGER -> Algae Roller On
        self.operator_controller.leftTrigger().whileTrue(AlgaeOnCommand(self.algae))

        """Zero Swerve Heading Control"""
        # DRIVER A Button -> Zero swerve heading
        self.driver_controller.a().onTrue(InstantCommand(lambda: self.drivetrain.zero_heading(), self.drivetrain))

        """Auto Align Controls"""
        # DRIVER X Button -> Align with left reef pole
        self.driver_controller.x().whileTrue(TeleopAlignScoringCommand(self.drivetrain, 'left', 2.5, 1.25))
        # DRIVER X Button -> Align with right reef pole
        self.driver_controller.b().whileTrue(TeleopAlignScoringCommand(self.drivetrain, 'right', 2.5, 1.25))

    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""
        pass

    def getAutonomousCommand(self) -> Command:
        """Selects and returns the autonomous command based on the chooser settings."""
        auto = self.auto_chooser.getSelected()
        return auto
    
    # def periodic(self) -> None:
    #     current_tv = self.table.getNumber("tv", 0)
    #     sd.putBoolean("I see April Tag", current_tv)
