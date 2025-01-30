# 2025-Robot-Code

import wpilib
import wpilib.drive
from wpilib.shuffleboard import Shuffleboard
import rev
import navx
from math import *

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """ 

        self.pdh = wpilib.PowerDistribution()
        self.light = True
        self.precision = 1
        """self.lightStrobe = 10
        self.lightStrobeStatus = True"""

        self.leftDrive = rev.SparkMax(1, rev.SparkMax.MotorType.kBrushed)
        self.rightDrive = rev.SparkMax(4, rev.SparkMax.MotorType.kBrushed)
        self.leftDriveTheSequel = rev.SparkMax(2, rev.SparkMax.MotorType.kBrushed)
        self.rightDriveTheSequel = rev.SparkMax(3, rev.SparkMax.MotorType.kBrushed)
        self.left = wpilib.MotorControllerGroup(self.leftDrive,self.leftDriveTheSequel)
        self.right = wpilib.MotorControllerGroup(self.rightDrive,self.rightDriveTheSequel)
        self.robotDrive = wpilib.drive.DifferentialDrive(
            self.left, self.right
        )
        self.elevatorHeightMotor = rev.SparkMax(5, rev.SparkMax.MotorType.kBrushed)
        self.coralDispensorMotor = rev.SparkMax(6, rev.SparkMax.MotorType.kBrushed)
        self.controller = wpilib.XboxController(0)
        self.kPT = 0.5
        self.kPS = 0.05
        self.gyro = navx.AHRS.create_spi()
        Shuffleboard.getTab("Example tab").add(self.gyro)
        self.timer = wpilib.Timer()

        # We need to invert one side of the drivetrain so that positive voltages
        # result in both sides moving forward. Depending on how your robot's
        # gearbox is constructed, you might have to invert the left side instead.
        self.rightDrive.setInverted(True)
        self.rightDriveTheSequel.setInverted(True)
        self.heading = 0
        self.driveAlternate = 0

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.timer.restart()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""

        # Drive for two seconds
        if self.timer.get() < 2.0:
            # Drive forwards half speed, make sure to turn input squaring off
            self.robotDrive.arcadeDrive(0.5, 0, squareInputs=False)
        else:
            self.robotDrive.stopMotor()  # Stop robot

    def teleopInit(self):
        """This function is called once each time the robot enters teleoperated mode."""
        self.gyro.zeroYaw()
        self.gyro.reset()
        self.heading = 0
        self.joystickRYDrift = max(min(self.controller.getRightY(),-0.0078125),0.0078125)

    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""
        if abs(self.controller.getRightY()) > abs(self.joystickRYDrift):
            print("joystick right y: " + str(self.controller.getRightY()))
            print("limit: " + str(self.joystickRYDrift))
            self.heading = (self.gyro.getAngle() - (self.controller.getRightY()-self.joystickRYDrift)*self.precision*2)
            error = (self.heading - self.gyro.getAngle()) * self.kPT
        else:
            print("error w/o Correction: " + str(self.heading - self.gyro.getAngle()))
            """error = min(max(self.heading - self.gyro.getAngle(),-0.5),0.5)"""
            """error = cbrt(self.heading - self.gyro.getAngle())"""
            error = (self.heading - self.gyro.getAngle()) * self.kPS

        print("Heading: " + str(self.heading))
        print("error: " + str(error))
        self.robotDrive.arcadeDrive(
            self.controller.getLeftY()*self.precision, error, True
        )
        voltage = self.pdh.getVoltage()
        print("Voltage: " + str(voltage))
        print()
        if voltage < 9.5:
            self.pdh.setSwitchableChannel(False)
        elif voltage >= 9.5:
            self.pdh.setSwitchableChannel(True)
        """if self.controller.getStartButtonReleased():
            if self.light:
                self.pdh.setSwitchableChannel(False)
                self.light = False
            else:
                self.pdh.setSwitchableChannel(True)
                self.light = True"""
        if self.controller.getBackButtonReleased():
            if self.precision == 1:
                self.precision = 0.5
            elif self.precision == 0.5:
                self.precision = 1
        dpadDirection = self.controller.getPOV()
        #dpadX is vertical while dpadY is horizontal because the getPOV() is centered with up = 0degrees and right = 90 degrees
        if dpadDirection != -1:
            dpadX = 0.5*cos(radians(dpadDirection))
            dpadY = 0.5*sin(radians(dpadDirection))
        else:
            dpadX = 0
            dpadY = 0
        self.elevatorHeightMotor.set(dpadX)
        if self.controller.getLeftBumperButton():
            self.coralDispensorMotor.set(0.5)
        else:
            self.coralDispensorMotor.set(-0.5*self.controller.getLeftTriggerAxis())
        """if self.light:
            if self.lightStrobe <= 0:
                self.lightStrobe = 10
                if self.lightStrobeStatus:
                    self.lightStrobeStatus = False
                    self.pdh.setSwitchableChannel(False)
                else:
                    self.lightStrobeStatus = True
                    self.pdh.setSwitchableChannel(True)
            else:
                self.lightStrobe -= 1"""

    def testInit(self):
        """This function is called once each time the robot enters test mode."""

    def testPeriodic(self):
        """This function is called periodically during test mode."""


if __name__ == "__main__":
    wpilib.run(MyRobot)
