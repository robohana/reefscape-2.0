from math import pi

import commands2

from rev import SparkMax, SparkMaxConfig, SparkBase, SparkBaseConfig, ClosedLoopConfig, ClosedLoopSlot

from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

from constants.constants import DriveConstants

class MAXSwerveModule(commands2.SubsystemBase):
    def __init__(self, drive_motor_channel: int, turn_motor_channel: int,  chassis_angular_offset: float) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor, drive encoder, and turning encoder."""
        super().__init__()
        # # Force NetworkTables to start
        # self.nt = NetworkTableInstance.getDefault()
        # self.debugTable = self.nt.getTable("SwerveDebug")
        # print("NetworkTables started: SwerveDebug should be available")

        driving_factor = DriveConstants.K_WHEEL_DIAMETER_METERS * pi / DriveConstants.K_DRIVING_MOTOR_REDUCTION
        turning_factor = 2 * pi
        driving_velocity_feed_forward = 1 / DriveConstants.K_DRIVE_WHEEL_FREE_SPEED_RPS

        self.drive_motor = SparkMax(drive_motor_channel, SparkMax.MotorType.kBrushless)
        self.turn_motor = SparkMax(turn_motor_channel, SparkMax.MotorType.kBrushless)

         # Setup encoders for the drive and turning motors.
        self.drive_encoder = self.drive_motor.getEncoder()
        self.turn_encoder = self.turn_motor.getAbsoluteEncoder()

        self.driving_closed_loop_controller = self.drive_motor.getClosedLoopController()
        self.turning_closed_loop_controller = self.turn_motor.getClosedLoopController()

        drive_motor_config = SparkMaxConfig()
        turn_motor_config = SparkMaxConfig()

        drive_motor_config.setIdleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(50)
        drive_motor_config.encoder.positionConversionFactor(driving_factor) # Meters
        drive_motor_config.encoder.velocityConversionFactor(driving_factor / 60.0) # Meters per second
        drive_motor_config.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        drive_motor_config.closedLoop.pid(DriveConstants.K_DRIVE_P, DriveConstants.K_DRIVE_I, DriveConstants.K_DRIVE_D)
        drive_motor_config.closedLoop.velocityFF(driving_velocity_feed_forward, ClosedLoopSlot.kSlot0) # Wants ff:The velocity feedforward gain value, slot: The closed loop slot to set the values for. 
                                            # set to slot 0 because that is the default
        drive_motor_config.closedLoop.outputRange(-1, 1)

        turn_motor_config.setIdleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(20)
        turn_motor_config.absoluteEncoder.positionConversionFactor(turning_factor)
        turn_motor_config.absoluteEncoder.velocityConversionFactor(turning_factor / 60.0)
        #Invert the turning encoder, since the output shaft rotates in the opposite direction of the steering motor in the MAXSwerve Module.
        turn_motor_config.absoluteEncoder.inverted(True)  
        turn_motor_config.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
        turn_motor_config.closedLoop.pid(DriveConstants.K_TURN_P, DriveConstants.K_TURN_I, DriveConstants.K_TURN_D)
        turn_motor_config.closedLoop.outputRange(-1, 1)
        #Enable PID wrap around for the turning motor. This will allow the PID controller to go through 0 to get to the setpoint i.e. going from 350 degrees to 10 degrees will go through 0 rather than the other direction which is a longer route.
        turn_motor_config.closedLoop.positionWrappingEnabled(True) # I don't know if this is a correct value - LC 2/23/25
        turn_motor_config.closedLoop.positionWrappingInputRange(0, turning_factor)

        self.drive_motor.configure(drive_motor_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        self.turn_motor.configure(turn_motor_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters) 

        self.desiredState = SwerveModuleState(0.0, Rotation2d())
        self.chassis_angular_offset = chassis_angular_offset
        # self.desiredState.angle = Rotation2d(self.turningEncoder.getPosition())
        self.drive_encoder.setPosition(0)

    def getState(self) -> SwerveModuleState:
        """Returns the current state of the module."""
        return SwerveModuleState(self.drive_encoder.getVelocity(), Rotation2d(self.turn_encoder.getPosition() - self.chassis_angular_offset))
    
    def get_position(self) -> SwerveModulePosition:
        return SwerveModulePosition(
            self.drive_encoder.getPosition(),
            Rotation2d(self.turn_encoder.getPosition() - self.chassis_angular_offset)
        )

    def set_desired_state(self, desired_state: SwerveModuleState):
        """Sets the desired state of the module, using PID and feedforward for the drive motor."""
        corrected_desired_state = SwerveModuleState()
        corrected_desired_state.speed = desired_state.speed
        corrected_desired_state.angle = desired_state.angle + Rotation2d(self.chassis_angular_offset)
        corrected_desired_state.optimize(Rotation2d(self.turn_encoder.getPosition()))

        self.driving_closed_loop_controller.setReference(corrected_desired_state.speed, SparkBase.ControlType.kVelocity)
        self.turning_closed_loop_controller.setReference(corrected_desired_state.angle.radians(), SparkBase.ControlType.kPosition)
        
        self.desired_state = desired_state

    def stop(self):
        """Stops the module."""
        self.drive_motor.stopMotor()
        self.turn_motor.stopMotor()
        
