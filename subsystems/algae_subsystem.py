import commands2

from constants.constants import AlgaeSubsystemConstants

# from wpilib import SmartDashboard as sd

from rev import SparkMax, SparkMaxConfig, SparkBase, SparkBaseConfig, ClosedLoopConfig, MAXMotionConfig

class AlgaeSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        self.angle_motor = SparkMax(AlgaeSubsystemConstants.K_ANGLE_MOTOR_CHANNEL, SparkMax.MotorType.kBrushless)
        self.roller_motor = SparkMax(AlgaeSubsystemConstants.K_ROLLER_MOTOR_CHANNEL, SparkMax.MotorType.kBrushless)

        self.angle_encoder = self.angle_motor.getEncoder()

        self.angle_closed_loop_controller = self.angle_motor.getClosedLoopController()

        angle_motor_config = SparkMaxConfig()
        angle_maxmotion_config = MAXMotionConfig()
        roller_motor_config = SparkMaxConfig()

        # angle_motor_config
        angle_motor_config.setIdleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12)
        angle_motor_config.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        angle_motor_config.closedLoop.pid(AlgaeSubsystemConstants.K_P, AlgaeSubsystemConstants.K_I, AlgaeSubsystemConstants.K_D)
        angle_motor_config.closedLoop.outputRange(-0.5,0.5)
        angle_maxmotion_config.maxVelocity(2000)
        angle_maxmotion_config.maxAcceleration(10000)
        angle_maxmotion_config.allowedClosedLoopError(0.5)

        roller_motor_config.setIdleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40)

        self.angle_motor.configure(angle_motor_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        self.roller_motor.configure(roller_motor_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters) 

        self.angle_encoder.setPosition(0)

    def move_to_setpoint(self, angle_current_target):
        self.angle_current_target = angle_current_target
        self.angle_closed_loop_controller.setReference(self.angle_current_target, SparkBase.ControlType.kPosition)

    def set_roller_power(self, power: float):
        self.roller_motor.set(power)

