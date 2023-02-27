# Use the robot as a python object
# Motor guide: https://pybricks.com/ev3-micropython/ev3devices.html#motors
# Ultrasonic sensor guide: https://pybricks.com/ev3-micropython/ev3devices.html#motors
# Color sensor guide: https://pybricks.com/ev3-micropython/ev3devices.html#ultrasonic-sensor

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from debugFeatures import Logger
from threading import *



class ROBOT():    

    def __init__(self, leftMotorPort, rightMotorPort, ColorSensorPort, FrontSensorPort, 
                LeftSensorPort, RightSensorPort, debugMode = False, overrideSafetyFeatures=False, movementTestMode=True):
        """
    def __init__(self, leftMotorPort, rightMotorPort, M3Port, ColorSensorPort, FrontSensorPort, 
            LeftSensorPort, RightSensorPort, debugMode = False, overrideSafetyFeatures=False):

            
        """
        

        self.wheelDiameter = 1 # the diameter of the wheels
        self.axleTrack = 1 # the horizontal distance between the two wheels, practically the width of the robot  
        # TODO: get diameter of the two wheels, and horizontal distance between the two wheels

        self.safeMode = not overrideSafetyFeatures # safety features should only be turned off while testing and debugging

        self.obstacleDetected = False # when the robot comes to a complete stop because of saftey features, this boolean is set to true
        # NOTE: when the obstacle isn't a problem anymore, the program has to manually set it to false

        self.debugMode = debugMode # enables certain features to test the robot.

        self.logger = None
        if movementTestMode == False:
            self.logger = Logger(self.motor,
                                    self.sensorOutput,
                                    self.LeftWheel,
                                    self.RightWheel,
                                    None,
            #                     self.m3,
                                    self.isObstacleDetected
                                    )

        self.brick = EV3Brick()
        self.slowDownDistance = 500 # slows down when it is near this distance
        self.stopDistance = 200 # comes to a complete stop when it reaches this distance

        self.LeftWheel = Motor(leftMotorPort)
        self.RightWheel = Motor(rightMotorPort)
        self.motor = DriveBase(self.LeftWheel, self.RightWheel, self.wheelDiameter, self.axleTrack) # The class used to drive robots


        self.colorSensor = None
        self.frontSensor = None
        self.leftSensor = None
        self.rightSensor = None
        self._statThread = None

        if movementTestMode == True:
            # self.m3 = Motor(M3Port) # TODO: find use of the third motor
            self.colorSensor = ColorSensor(ColorSensorPort) # Should be used to track the lines
            self.frontSensor = UltrasonicSensor(FrontSensorPort)
            self.leftSensor = UltrasonicSensor(LeftSensorPort)
            self.rightSensor = UltrasonicSensor(RightSensorPort)
            self._statThread = Thread(target=self._statFunc)
            self._statThread.setDaemon(True)
        
        
    def forward(self, distance):
        self.motor.straight(distance)

    def right(self, degrees):
        self.motor.turn(degrees)

    def left(self, degrees):
        self.motor.turn(-degrees)

    def backward(self, distance):
        self.motor.straight(-distance)

    

    def followLine(self, speed, threshold) -> int:
        
        """ 
        This function should be put in a loop in order to fully work, 
        and when finished make sure to end with self.motor.stop().\n
        
        threshold should be calculated like:\n
        a = light reflected by the black line\n
        b = light reflected elsewhere\n
        c = another value of light reflected elsewhere\n
        d = tuple(a, b, c\n
        sum(d/ length(d)\n
        
        speed is milimeters per second\n
        should return 0 if it performed fine, or -1 if something happened
        """
        _, _, fS, reflection = self.sensorOutput()
        deviation = reflection - threshold
        percentage = 1
        turn = 1.2 * deviation
        if self.safeMode == True:
            if fS <= self.slowDownDistance:
                percentage = (self.slowDownDistance - fS) / (self.slowDownDistance - self.stopDistance)

        if percentage <= 0.1:
            self.motor.stop()
            return -1
        speed *= percentage
        self.motor.drive(speed, turn)
        return 0

    def sensorOutput(self):
        """
        returns: distance between the left side and and an object, 
        distance between the right side and and an object,
        distance between the front side and and an object, and the light reflected in the color sensor.
        
        """


        leftSide = self.leftSensor.distance()
        rightSide = self.rightSensor.distance()
        frontSide = self.frontSensor.distance()
        lightReflected = self.colorSensor.reflection()
        return leftSide, rightSide, frontSide, lightReflected
    
    def moveAroundObstacle(self):
        pass
    

    def _statFunc(self):
    
        while True:
            self.brick.screen.clear()
            _, robotSpeed, _, _ = self.motor.state()
            robotSpeed = "IDLE" if robotSpeed == 0 else str(robotSpeed)
            text = f"""
                    INGLEBOROUGH ROBOT\n
                       |{robotSpeed}|
                    """
            # TODO: add more info on the display
            self.brick.screen.print(text)
            


    def displayStats(self): ## displays information on the screen of the brick in the background
        self._statThread.start()
        

    def isObstacleDetected(self): # Provides information to other programs in other files
        return self.obstacleDetected