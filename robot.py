#!/usr/bin/env python3
'''

This is the main class for the robot. This class is the glue that binds the robot together. All of the robot-wide systems
(such as the Field2d, RobotContainer, and RobotContainer) should be instantiated here. The autonomous routine should
be in the autonomousInit() method, and the teleop control should be in the teleopPeriodic() method. The robotInit() and
robotPeriodic() methods at the bottom of this file should generally remain unchanged. 

'''

import wpilib

from commands2 import CommandScheduler, TimedCommandRobot

from robot_container import RobotContainer


class MyRobot(TimedCommandRobot):
    def robotInit(self) -> None:
        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        print("Robot is initializing...")  # Debug output
        self.container = RobotContainer()
        print("RobotContainer initialized!")  
        self.autonomous_command = self.container.get_autonomous_command()

    def robotPeriodic(self) -> None:
        CommandScheduler.getInstance().run()
        pass

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        pass
        #print("Starting Autonomous... ")
        if self.autonomous_command is not None:
            self.autonomous_command.schedule()

    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        print("Teleop is running!")  # Debug output
        #CommandScheduler.getInstance().run()


    def teleopPeriodic(self) -> None:
        pass
    
    def testInit(self) -> None:
        CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self) -> None:
        pass



if __name__ == "__main__":
    wpilib.run(MyRobot)
