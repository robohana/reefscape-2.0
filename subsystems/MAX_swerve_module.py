import math
import commands2

from wpimath.controller import PIDController, SimpleMotorFeedforwardMeters, SimpleMotorFeedforwardRadians
from constants import DriveConstants

from rev import SparkMax, SparkMaxConfig, SparkBase, SparkBaseConfig

from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from ntcore import NetworkTableInstance


class MAXSwerveModule(commands2.SubsystemBase):
    def __init__(self, driveMotorChannel: int, turningMotorChannel: int,  chassis_angular_offset: float) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor, drive encoder, and turning encoder."""
        super().__init__()
        # Force NetworkTables to start
        self.nt = NetworkTableInstance.getDefault()
        self.debugTable = self.nt.getTable("SwerveDebug")
        print("NetworkTables started: SwerveDebug should be available")


        self.driveMotor = SparkMax(driveMotorChannel, SparkMax.MotorType.kBrushless)
        self.turningMotor = SparkMax(turningMotorChannel, SparkMax.MotorType.kBrushless)

        driveMotorConfig = SparkMaxConfig()
        turnMotorConfig = SparkMaxConfig()

        driveMotorConfig.encoder.positionConversionFactor(DriveConstants.K_DRIVE_ENCODER_ROT2METER)
        driveMotorConfig.encoder.velocityConversionFactor(DriveConstants.K_DRIVE_ENCODER_RPM2METER_PER_SEC) 

        turnMotorConfig.encoder.positionConversionFactor(DriveConstants.K_TURN_ENCODER_ROT2RAD)
        turnMotorConfig.encoder.velocityConversionFactor(DriveConstants.K_TURN_ENCODER_RPM2RAD_PER_SEC)

        self.driveMotor.configure(driveMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        self.turningMotor.configure(turnMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

        # Setup encoders for the drive and turning motors.
        self.driveEncoder = self.driveMotor.getEncoder()
        self.turningEncoder = self.turningMotor.getAbsoluteEncoder()


        # Setup PID controllers for the driving and turning SPARKS MAX.
        self.drivePIDController = self.driveMotor.getClosedLoopController()
        self.turningPIDController = PIDController(DriveConstants.K_TURN_P, DriveConstants.K_TURN_I, DriveConstants.K_TURN_D)
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

        self.driveFeedforward = SimpleMotorFeedforwardMeters(DriveConstants.K_DRIVE_KS, DriveConstants.K_DRIVE_KV)
        self.turnFeedforward = SimpleMotorFeedforwardRadians(DriveConstants.K_TURN_KS, DriveConstants.K_TURN_KV)

        # Set the idle mode for the motors to brake.
        driveMotorConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        turnMotorConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)

        self.desiredState = SwerveModuleState()
        self.chassis_angular_offset = chassis_angular_offset
        self.desiredState.angle = Rotation2d(self.turningEncoder.getPosition())
        self.resetEncoders()

    def resetEncoders(self):
        """Resets the drive encoder to zero."""
        self.driveEncoder.setPosition(0)

    def getState(self) -> SwerveModuleState:
        """Returns the current state of the module."""
        return SwerveModuleState(self.driveEncoder.getVelocity(), Rotation2d(self.turningEncoder.getPosition() - self.chassis_angular_offset))
    
    def get_position(self):
        return SwerveModulePosition(
            self.driveEncoder.getPosition(),
            Rotation2d(self.turningEncoder.getPosition() - self.chassis_angular_offset)
        )
 

    def setDesiredState(self, state: SwerveModuleState):
        """Sets the desired state of the module, using PID and feedforward for the drive motor."""
        self.desiredState = state
        state.optimize(self.getState().angle)
        if math.isclose(state.speed, 0):
            self.driveMotor.stopMotor()
        else:
            self.drivePIDController.setReference(state.speed, SparkBase.ControlType.kVelocity)

        try:
            # For the turning motor, we typically use PID control.
            turningOutput = self.turningPIDController.calculate(self.getState().angle, state.angle.radians())
            self.turningMotor.set(turningOutput)
        except Exception as e:
            print(f"Turning motor set error: {e}")
            self.turningMotor.set(0)


    def stop(self):
        """Stops the module."""
        self.driveMotor.stopMotor()
        self.turningMotor.stopMotor()
        
