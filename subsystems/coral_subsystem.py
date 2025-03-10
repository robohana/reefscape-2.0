import commands2
from commands2 import Command, StartEndCommand

from constants import CoralSubsystemConstants, Setpoint, OIConstants

from wpilib import RobotController, XboxController, DigitalInput
from wpilib import SmartDashboard as sd
from rev import SparkMax, SparkMaxConfig, SparkBase, SparkBaseConfig, ClosedLoopConfig, SparkFlex, SparkFlexConfig, MAXMotionConfig, LimitSwitchConfig, SparkLowLevel


class CoralSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()

        # arm_motor:      Angles the intake,              Spark Max,  Brushless, Closed Loop Controller, Relative Encoder
        # elevator_motor: Raises and lowers the elevator, Spark Flex, Brushless, Closed Loop Controller, Absolute Encoder
        # intake_motor:   Controllers the intake rollers, Spark Max,  Brushless, None,                   None
        self.arm_motor = SparkMax(CoralSubsystemConstants.K_ARM_MOTOR_CHANNEL, SparkMax.MotorType.kBrushless)
        self.elevator_motor = SparkMax(CoralSubsystemConstants.K_ELEVATOR_MOTOR_CHANNEL, SparkFlex.MotorType.kBrushless)
        self.intake_motor = SparkMax(CoralSubsystemConstants.K_INTAKE_MOTOR_CHANNEL, SparkMax.MotorType.kBrushless)

        # Setup encoders for the motors. Only need arm and elevator. The elevator will be pulling the through bore absolute encoder in. RN just set to how rev has it
        self.arm_encoder = self.arm_motor.getEncoder()
        self.elevator_encoder = self.elevator_motor.getEncoder()

        # the intake won't have a closed loop or any controller
        self.arm_closed_loop_controller = self.arm_motor.getClosedLoopController()
        self.elevator_closed_loop_controller = self.elevator_motor.getClosedLoopController()

        arm_motor_config = SparkMaxConfig()
        arm_maxmotion_config = MAXMotionConfig()
        elevator_motor_config = SparkFlexConfig()
        elevator_maxmotion_config = MAXMotionConfig()
        intake_motor_config = SparkMaxConfig()
        
        self.operator_controller = XboxController(OIConstants.K_OPERATOR_CONTROLLER_PORT)

# arm_motor_config
        """Configure the closed loop contoller. We want to make sure we set the feedback sensor as the primary encoder"""
        # Configure basic settings of the arm motor
        arm_motor_config.setIdleMode(SparkBaseConfig.IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12)
        arm_motor_config.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        # Set PID values for position control
        arm_motor_config.closedLoop.pid(CoralSubsystemConstants.Arm.K_P, CoralSubsystemConstants.Arm.K_I, CoralSubsystemConstants.Arm.K_D)
        arm_motor_config.closedLoop.outputRange(-1,1)
        # Set MAXMotion paraneters for position control
        arm_maxmotion_config.maxVelocity(2000)
        arm_maxmotion_config.maxAcceleration(10000)
        arm_maxmotion_config.allowedClosedLoopError(0.25)

# elevator_motor_config
        """Configure the closed loop controller. We want to make sure we set the feedback sensor as the primary encoder"""
        # Configure basic settings of the elevator motor
        elevator_motor_config.setIdleMode(SparkBaseConfig.IdleMode.kCoast).smartCurrentLimit(50).voltageCompensation(12)
        elevator_motor_config.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        # Set PID values for position control
        elevator_motor_config.closedLoop.pidf(CoralSubsystemConstants.Elevator.K_P, CoralSubsystemConstants.Elevator.K_I, CoralSubsystemConstants.Elevator.K_D, CoralSubsystemConstants.Elevator.K_F)
        elevator_motor_config.closedLoop.outputRange(-1, 1)
        """Configure the reverse limit switch for the elevator. By enabling the limit switch, this will prevent any actuation of the elevator in the reverse direction of the limit switch is pressed."""
        elevator_motor_config.limitSwitch.reverseLimitSwitchEnabled(True)
        elevator_motor_config.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        # set the MAXMotion for position control
        elevator_maxmotion_config.maxVelocity(4200)
        elevator_maxmotion_config.maxAcceleration(6000)
        elevator_maxmotion_config.allowedClosedLoopError(0.5)

#intake_motor_config
        # Configure basic settings of the intake motor
        intake_motor_config.inverted(True).setIdleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40)
        
# configure motors
        # arm_motor_config.apply(arm_maxmotion_config)
        self.arm_motor.configure(arm_motor_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        # elevator_motor_config.apply(elevator_maxmotion_config)
        self.elevator_motor.configure(elevator_motor_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters) 
        self.intake_motor.configure(elevator_motor_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters) 

        # reset encoders to 0
        self.arm_encoder.setPosition(0)
        self.elevator_encoder.setPosition(0)    

        self.arm_current_target = Setpoint.Arm.K_CORAL_STATION
        self.elevator_current_target = Setpoint.Elevator.K_CORAL_STATION

        self.was_reset_by_limit = False
        self.was_reset_by_button = False

        self.coral_sensor = DigitalInput(0)
        
    def move_to_setpoint(self) -> None:
        """ Drive the arm and elevator motors to their respective setpoints. This will use MAXMotion position control which will allow for a smooth acceleration and deceleration to the mechanisms' setpoints """
        self.arm_closed_loop_controller.setReference(self.arm_current_target, SparkLowLevel.ControlType.kMAXMotionPositionControl)  
        self.elevator_closed_loop_controller.setReference(self.elevator_current_target, SparkLowLevel.ControlType.kMAXMotionPositionControl)  

    def zero_elevator_on_limit_switch(self) -> None:
        """ Zero the encoder when the limit switch is pressed """
        if not self.was_reset_by_limit and self.elevator_motor.getReverseLimitSwitch().get():
            self.elevator_encoder.setPosition(0)
            self.was_reset_by_limit = True
        else:
            self.was_reset_by_limit = False

    def zero_on_user_button(self) -> None:
        """ Zero the arm and elevator encoders when the user button is preesed on the roboRIO """    
        if not self.was_reset_by_button and RobotController.getUserButton():
            self.was_reset_by_button = True
            self.arm_encoder.setPosition(0)
            self.elevator_encoder.setPosition(0)   
        else:
            self.was_reset_by_button = False

    def set_intake_power(self, power: float) -> None:
        self.intake_motor.set(power) 

    class setSetpointCommand(Command):  
        """ Command to set the subsystem setpoint. This will set the arm and elevator to their predefined positions for the given setpoint """ 
        def __init__(self, setpoint):
            super().__init__()
            self.setpoint = setpoint

        def initialize(self):
            setpoint_map = {
                Setpoint.K_CORAL_STATION: (Setpoint.Arm.K_CORAL_STATION, Setpoint.Elevator.K_CORAL_STATION),
                Setpoint.K_LEVEL_1: (Setpoint.Arm.K_LEVEL_1, Setpoint.Elevator.K_LEVEL_1),
                Setpoint.K_LEVEL_2: (Setpoint.Arm.K_LEVEL_2, Setpoint.Elevator.K_LEVEL_2),
                Setpoint.K_LEVEL_3: (Setpoint.Arm.K_LEVEL_3, Setpoint.Elevator.K_LEVEL_3),
                Setpoint.K_POP: (Setpoint.Arm.K_POP, Setpoint.Elevator.K_POP),
            }    

            if self.setpoint in setpoint_map:
                self.arm_current_target, self.elevator_current_target = setpoint_map[self.setpoint]

        def isFinished(self):
            return True
    
    def run_intake_command(self):
        """ Command to run the intake motor. When the command is interrupted, e.g. the button is released, the motor will stop """
        return StartEndCommand(
            lambda: self.set_intake_power(Setpoint.Intake.K_FORWARD),
            lambda: self.set_intake_power(0.0)
        ) 
    
    def run_intake_power(self):
        self.set_intake_power(Setpoint.Intake.K_FORWARD) 

    def stop_intake(self):
        self.set_intake_power(0.0)      

    def reverse_intake_command(self):
        """ Command to reverse the intake motor. When the command is interrupted, e.g. the button is released, the motor will stop """
        return StartEndCommand(
            lambda: self.set_intake_power(Setpoint.Intake.K_REVERSE),
            lambda: self.set_intake_power(0.0)
        ) 
    
    def reverse_intake_power(self):
        self.set_intake_power(Setpoint.Intake.K_REVERSE)
    
    def pop_intake(self):
        # Add code to raise intake mechanism (e.g., solenoid or motor)
        self.setSetpointCommand(Setpoint.K_POP)
        print("Intake popped up!")  # Placeholder

    def is_object_detected(self):
        return not self.coral_sensor.get()  # Adjust based on sensor logic (True when detected)
    
    def periodic(self):
        self.move_to_setpoint()
        self.zero_elevator_on_limit_switch()
        self.zero_on_user_button()

        # Display subsystem values
        sd.putNumber("Coral/Arm/Target Position", self.arm_current_target)
        sd.putNumber("Coral/Arm/Actual Position", self.arm_encoder.getPosition())
        sd.putNumber("Coral/Elevator/Target Position", self.elevator_current_target)
        sd.putNumber("Coral/Elevator/Actual Position", self.elevator_encoder.getPosition())
        sd.putNumber("Coral/Intake/Applied Output", self.intake_motor.getAppliedOutput())
