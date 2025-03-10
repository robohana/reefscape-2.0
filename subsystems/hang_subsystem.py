import commands2
from commands2 import StartEndCommand, InstantCommand
from constants import HangSubSystemConstants, Setpoint
from rev import SparkMax, SparkMaxConfig, SparkBase, SparkBaseConfig, ClosedLoopConfig, MAXMotionConfig, SparkLowLevel
from wpilib import SmartDashboard as sd

class HangSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()

        self.hang_motor = SparkMax(HangSubSystemConstants.K_HANG_MOTOR_CHANNEL, SparkMax.MotorType.kBrushless)
        self.hang_encoder = self.hang_motor.getAbsoluteEncoder()

        hang_motor_config = SparkMaxConfig()
        hang_maxmotion_config = MAXMotionConfig()

        hang_motor_config.setIdleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12)
        hang_motor_config.absoluteEncoder.positionConversionFactor(HangSubSystemConstants.K_ENCODER_CONVERSION_FACTOR)
        hang_motor_config.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
        hang_motor_config.closedLoop.pid(HangSubSystemConstants.K_P, HangSubSystemConstants.K_I, HangSubSystemConstants.K_D)
        hang_motor_config.closedLoop.outputRange(-1, 1)
        # hang_maxmotion_config.maxVelocity(2000) # this is the same maxV as the elevator, might need to change
        # hang_maxmotion_config.maxAcceleration(10000) # this is the same maxV as the elevator, might need to change
        hang_maxmotion_config.allowedClosedLoopError(0.25) # this is the same maxV as the elevator, might need to change

        self.hang_motor.configure(hang_motor_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        self.hang_closed_loop_controller = self.hang_motor.getClosedLoopController()

    def move_to_position(self, target_position):
        """Move the hang mechanism to a target position using motion profiling."""
        sd.putString("HangSubsystem", f"Moving to {target_position}")
        self.hang_closed_loop_controller.setReference(target_position, SparkLowLevel.ControlType.kMAXMotionPositionControl)

    def get_current_position(self):
        """Return the current encoder position (already converted to meaningful units)."""
        return self.hang_encoder.getPosition()
    # def load_zero_offset(self):
    #     """Load the last stored zero offset from persistent memory (or default to 0)."""
    #     return self.hang_motor.getPersistentParameter("ZeroOffset") or 0.0

    # def save_zero_offset(self, offset):
    #     """Save the zero offset persistently so it remains after a reboot."""
    #     self.hang_motor.setPersistentParameter("ZeroOffset", offset)

    # def set_zero_position(self):
    #     """Set the current position as the zero position."""
    #     self.zero_offset = self.hang_encoder.getPosition()
    #     self.save_zero_offset(self.zero_offset)

    # def get_relative_position(self):
    #     """Get the current position relative to the saved zero offset."""
    #     return self.hang_encoder.getPosition() - self.zero_offset

    # def set_hang_power(self, power: float) -> None:
    #     self.hang_motor.set(power) 

    # def set_hang_position(self, target_position: float) -> None:
    #     """Move hang mechanism to a specific encoder position using closed-loop control."""
    #     self.hang_closed_loop_controller.setReference(target_position, SparkMax.ControlType.kPosition)   
        
    # def lift_command_power(self):
    #     """ Command to run the hang motor so that we lift ourselves up. When the command is interrupted, e.g. the button is released, the motor will stop """
    #     return StartEndCommand(
    #         lambda: self.set_hang_power(Setpoint.Hang.K_FORWARD),
    #         lambda: self.set_hang_power(0.0)
    #     )  

    # def power_zero(self):
    #     self.set_hang_power(0.0)

    # def zero_position(self):
    #     return InstantCommand(
    #         lambda: self.set_hang_position(0.0), 
    #         self
    #     )   

    # def lower_command_power(self):
    #     """ Command to run the hang motor so that we lower ourselves up. When the command is interrupted, e.g. the button is released, the motor will stop """
    #     return StartEndCommand(
    #         lambda: self.set_hang_power(Setpoint.Hang.K_REVERSE),
    #         lambda: self.set_hang_power(0.0)
    #     )   
    
    # def lift_to_setpoint(self):
    #     """Command to lift to the UP position using setpoint """
    #     return InstantCommand(
    #         lambda: self.set_hang_position(Setpoint.Hang.K_UP_POSITION), 
    #         self
    #     )

    # def lower_to_setpoint(self):
    #     """Command to lower to the DOWN position using setpoint """
    #     return InstantCommand(
    #         lambda: self.set_hang_position(Setpoint.Hang.K_DOWN_POSITION), 
    #         self
    #     )
    
    # def stop (self):
    #     return InstantCommand(
    #         lambda: self.power_zero(), 
    #         self
    #     )
