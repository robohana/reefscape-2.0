#!/usr/bin/env python3
'''

This is the main class for the robot. This class is the glue that binds the robot together. All of the robot-wide systems
(such as the Field2d, RobotContainer, and RobotContainer) should be instantiated here. The autonomous routine should
be in the autonomousInit() method, and the teleop control should be in the teleopPeriodic() method. The robotInit() and
robotPeriodic() methods at the bottom of this file should generally remain unchanged. 

'''

from commands2 import CommandScheduler, TimedCommandRobot
from wpilib import DataLogManager, DriverStation, run, SmartDashboard as sd, Spark
from robot_container import RobotContainer
from constants.constants import BlinkinColor
# from subsystems.drivetrain import DriveSubsystem

class MyRobot(TimedCommandRobot):
    def robotInit(self) -> None:
        # Instantiate our RobotContainer.  
        self.container = RobotContainer()
        # self.hang = HangSubsystem()
        #self.autonomousCommand = None

        #self.driver_controller = self.container.driver_controller

        #self.drive = self.Container.drive
        #self.robotDrive = self.container.robot_drive
        #CommandScheduler.getInstance().registerSubsystem(self.robotDrive)

        # global robotDrive #expose the drive subsystem globally (nothing can go wrong, right?)

        # robotDrive = self.robotDrive

        self.blinkin = Spark(6)
        # self.blinkin.setSafetyEnabled(False)
        self.blinkin.set(BlinkinColor.ORANGE)

        DataLogManager.start()  # Start logging
        DriverStation.startDataLog(DataLogManager.getLog())  # Log joystick inputs
        print("WPILib Data Logging Enabled")


    def robotPeriodic(self) -> None:
        CommandScheduler.getInstance().run()
        # position = self.hang.get_current_position()
        # sd.putNumber("Hang Position", position)
        self.blinkin.set(BlinkinColor.ORANGE)


    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        #pass
        #print("Starting Autonomous... ")
        self.autonomousCommand = self.container.getAutonomousCommand()
        if self.autonomousCommand is not None:
            self.autonomousCommand.schedule()
       
    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        #pass
        print("Starting TeleOp... ")
        # Reset the odometry using the current gyro reading
        # self.robot_drive.reset_odometry(self.robot_drive.get_pose())
        # SwerveModule.resetEncoders()
        # if self.autonomousCommand is not None:
        #     self.autonomousCommand.cancel()

    def teleopPeriodic(self) -> None:
        pass

    def testInit(self) -> None:
        CommandScheduler.getInstance().run()

    def testPeriodic(self) -> None:
        pass



if __name__ == "__main__":
    run(MyRobot)
