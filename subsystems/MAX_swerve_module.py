import math
import wpilib
import time
import threading
import wpimath.kinematics
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory
from wpimath.geometry import Rotation2d
from wpimath import units
from wpimath.trajectory import TrajectoryConfig
from wpilib import AnalogEncoder
import commands2

from wpimath.controller import PIDController, ProfiledPIDController, SimpleMotorFeedforwardMeters, SimpleMotorFeedforwardRadians
from constants.module_constants import ModuleConstants

from rev import SparkMax, SparkMaxConfig, SparkBase, SparkBaseConfig
from math import radians, pi



from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from ntcore import NetworkTableInstance
from commands2 import CommandScheduler
import configs



class MAXSwerveModule(commands2.SubsystemBase):
    def __init__(self, drive_can_id: int, turn_can_id: int, chassis_angular_offset: float) -> None:
        super().__init__()
	 
        self.drive_motor = SparkMax(drive_can_id, SparkMax.MotorType.kBrushless)
        self.turn_motor = SparkMax(turn_can_id, SparkMax.MotorType.kBrushless)
		
		# Setup encoders for the drive and turning motors.
        self.drive_encoder = self.drive_motor.getEncoder()
        self.turn_encoder = self.turn_motor.getAbsoluteEncoder()
		
        self.drive_motor.configure(
			configs.MAXSwerveModule.drive_config,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
		)
        self.turn_motor.configure(
			configs.MAXSwerveModule.turn_config,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
		)

        self.desired_state = SwerveModuleState()  # Make sure to initialize it

        self.chassis_angular_offset = chassis_angular_offset
        self.desired_state.angle = Rotation2d(self.turn_encoder.getPosition())
        self.drive_encoder.setPosition(0)

    def get_state(self):
        return SwerveModuleState(
            self.drive_encoder.getVelocity(),
            Rotation2d(self.turn_encoder.getPosition() - self.chassis_angular_offset)
        )
    
    def get_position(self):
        return SwerveModulePosition(
            self.drive_encoder.getPosition(),
            Rotation2d(self.turn_encoder.getPosition() - self.chassis_angular_offset)
        )
    
    def set_desired_state(self, desired_state: SwerveModuleState):
        corrected_desired_state = SwerveModuleState()
        corrected_desired_state.speed = desired_state.speed
        corrected_desired_state.angle = desired_state.angle + Rotation2d.radians(self.chassis_angular_offset) # double check Rotation2d.radians, in java its Rotation2d.fromRadians. My serveal minutes of looking makes me belive that they serve the same function - LC 2/23/25

        corrected_desired_state.optimize(Rotation2d(self.turn_encoder.getPosition()))

        self.desired_state = desired_state

    def reset_encoders(self):
        self.drive_encoder.setPosition(0)

