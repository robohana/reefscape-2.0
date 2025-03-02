from math import pi
import commands2

from wpimath.controller import PIDController, SimpleMotorFeedforwardMeters, SimpleMotorFeedforwardRadians
from constants import DriveConstants

from rev import SparkMax, SparkMaxConfig, SparkBase, SparkBaseConfig, ClosedLoopConfig, ClosedLoopSlot, SparkClosedLoopController

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


        drivingFactor = DriveConstants.K_WHEEL_DIAMETER_METERS * pi / DriveConstants.kDrivingMotorReduction
        turningFactor = 2 * pi
        drivingVelocityFeedForward = 1 / DriveConstants.kDriveWheelFreeSpeedRps


        self.driveMotor = SparkMax(driveMotorChannel, SparkMax.MotorType.kBrushless)
        self.turningMotor = SparkMax(turningMotorChannel, SparkMax.MotorType.kBrushless)

            # Setup encoders for the drive and turning motors.
        self.driveEncoder = self.driveMotor.getEncoder()
        self.turningEncoder = self.turningMotor.getAbsoluteEncoder()

        self.drivingClosedLoopController = self.driveMotor.getClosedLoopController()
        self.turningClosedLoopController = self.turningMotor.getClosedLoopController()

        driveMotorConfig = SparkMaxConfig()
        turnMotorConfig = SparkMaxConfig()

        driveMotorConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(50)
        driveMotorConfig.encoder.positionConversionFactor(drivingFactor) # Meters
        driveMotorConfig.encoder.velocityConversionFactor(drivingFactor / 60.0) # Meters per second
        driveMotorConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        driveMotorConfig.closedLoop.pid(DriveConstants.K_DRIVE_P, DriveConstants.K_DRIVE_I, DriveConstants.K_DRIVE_D)
        driveMotorConfig.closedLoop.velocityFF(drivingVelocityFeedForward, ClosedLoopSlot.kSlot0) # Wants ff:The velocity feedforward gain value, slot: The closed loop slot to set the values for. 
                                            # set to slot 0 because that is the default
        driveMotorConfig.closedLoop.outputRange(-1, 1)

        turnMotorConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(20)
        turnMotorConfig.absoluteEncoder.positionConversionFactor(turningFactor)
        turnMotorConfig.absoluteEncoder.velocityConversionFactor(turningFactor / 60.0)
        #Invert the turning encoder, since the output shaft rotates in the opposite direction of the steering motor in the MAXSwerve Module.
        turnMotorConfig.absoluteEncoder.inverted(True)  
        turnMotorConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
        turnMotorConfig.closedLoop.pid(DriveConstants.K_TURN_P, DriveConstants.K_TURN_I, DriveConstants.K_TURN_D)
        turnMotorConfig.closedLoop.outputRange(-1, 1)
        #Enable PID wrap around for the turning motor. This will allow the PID controller to go through 0 to get to the setpoint i.e. going from 350 degrees to 10 degrees will go through 0 rather than the other direction which is a longer route.
        turnMotorConfig.closedLoop.positionWrappingEnabled(True) # I don't know if this is a correct value - LC 2/23/25
        turnMotorConfig.closedLoop.positionWrappingInputRange(0, turningFactor)

        self.driveMotor.configure(driveMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        self.turningMotor.configure(turnMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters) 


        self.desiredState = SwerveModuleState()
        self.chassis_angular_offset = chassis_angular_offset
        self.desiredState.angle = Rotation2d(self.turningEncoder.getPosition())
        self.driveEncoder.setPosition(0)


    def getState(self) -> SwerveModuleState:
        """Returns the current state of the module."""
        return SwerveModuleState(self.driveEncoder.getVelocity(), Rotation2d(self.turningEncoder.getPosition() - self.chassis_angular_offset))
    
    def get_position(self):
        return SwerveModulePosition(
            self.driveEncoder.getPosition(),
            Rotation2d(self.turningEncoder.getPosition() - self.chassis_angular_offset)
        )
    
 

    def setDesiredState(self, desiredState: SwerveModuleState):
        """Sets the desired state of the module, using PID and feedforward for the drive motor."""
        correctedDesiredState = SwerveModuleState()
        correctedDesiredState.speed = desiredState.speed
        correctedDesiredState.angle = desiredState.angle + Rotation2d(self.chassis_angular_offset)
        correctedDesiredState.optimize(Rotation2d(self.turningEncoder.getPosition()))

        self.drivingClosedLoopController.setReference(correctedDesiredState.speed, SparkBase.ControlType.kVelocity)
        self.turningClosedLoopController.setReference(correctedDesiredState.angle.radians(), SparkBase.ControlType.kPosition)
        
        self.desiredState = desiredState

    def stop(self):
        """Stops the module."""
        self.driveMotor.stopMotor()
        self.turningMotor.stopMotor()
        
