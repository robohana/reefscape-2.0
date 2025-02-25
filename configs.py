import rev
# import math
from rev import ClosedLoopConfig, ClosedLoopSlot, SparkMaxConfig, SparkBaseConfig, SparkFlexConfig
from math import pi
from constants.module_constants import ModuleConstants
from constants.drive_constants import DriveConstants



class MAXSwerveModule:
    drive_config = SparkMaxConfig()
    turn_config = SparkMaxConfig()

    drive_config.setIdleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(50)
    drive_config.encoder.positionConversionFactor(DriveConstants.K_DRIVE_ENCODER_ROT2METER) # Meters
    drive_config.encoder.velocityConversionFactor(DriveConstants.K_DRIVE_ENCODER_ROT2METER / 60.0) # Meters per second
    drive_config.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
    drive_config.closedLoop.pid(DriveConstants.K_DRIVE_P, DriveConstants.K_DRIVE_I, DriveConstants.K_DRIVE_D)
    drive_config.closedLoop.velocityFF(DriveConstants.K_DRIVE_KS, ClosedLoopSlot.kSlot0) # Wants ff:The velocity feedforward gain value, slot: The closed loop slot to set the values for. 
                                        # set to slot 0 because that is the default
    drive_config.closedLoop.outputRange(-1, 1)

    turn_config.setIdleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(20)
    turn_config.absoluteEncoder.positionConversionFactor(DriveConstants.K_TURN_ENCODER_ROT2RAD)
    turn_config.absoluteEncoder.velocityConversionFactor(DriveConstants.K_TURN_ENCODER_ROT2RAD / 60.0)
    #Invert the turning encoder, since the output shaft rotates in the opposite direction of the steering motor in the MAXSwerve Module.
    turn_config.absoluteEncoder.inverted(True)  
    turn_config.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
    turn_config.closedLoop.pid(DriveConstants.K_TURN_P, DriveConstants.K_TURN_I, DriveConstants.K_TURN_D)
    turn_config.closedLoop.outputRange(-1, 1)
    #Enable PID wrap around for the turning motor. This will allow the PID controller to go through 0 to get to the setpoint i.e. going from 350 degrees to 10 degrees will go through 0 rather than the other direction which is a longer route.
    turn_config.closedLoop.positionWrappingEnabled(True) # I don't know if this is a correct value - LC 2/23/25
    turn_config.closedLoop.positionWrappingInputRange(0, DriveConstants.K_TURN_ENCODER_ROT2RAD)        

class CoralSubsytem:
    arm_config = SparkMaxConfig()
    elevator_config = SparkMaxConfig()
    intake_config = SparkMaxConfig()

    arm_config.setIdleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12)
    # fill in - LC 2/23/25

    elevator_config.setIdleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12)
    # fill in - LC 2/23/25

    intake_config.setIdleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40)
    # fill in - LC 2/23/25

    

# Put other subsystem configs down here like Coral or Hang - LC 2/23/25    

     