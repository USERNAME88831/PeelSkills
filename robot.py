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




class ROBOT(): #TODO: finish making basic functions, and add a line tracking system here.
    def __init__(self, M1Port, M2Port, M3Port, ColorSensorPort, FrontSensorPort, LeftSensorPort, RightSensorPort):
        """
        """
        self.wheelDiameter = 1 # the diameter of the wheels
        self.axleTrack = 1 # the horizontal distance between the two wheels, practically the width of the robot  
        # TODO: get diameter of the two wheels, and distance between the two wheels

        self.safeDistance = 300 # avoid the obstacle when it reaches this distance(uses mm)
        self.brick = EV3Brick()
        self.LeftWheel = Motor(M1Port)
        self.RightWheel = Motor(M2Port)
        self.motor = DriveBase(self.LeftWheel, self.RightWheel, self.wheelDiameter, self.axleTrack) # The class used to drive robots
        self.M3 = Motor(M3Port) # TODO: find use of the third motor
        self.colorSensor = ColorSensor(ColorSensorPort) # Should be used to track the lines
        self.frontSensor = UltrasonicSensor(FrontSensorPort)
        self.leftSensor = UltrasonicSensor(LeftSensorPort)
        self.rightSensor = UltrasonicSensor(RightSensorPort)
        
    def forward(self, distance):
        self.motor.straight(distance)

    def right(self, degrees):
        self.motor.turn(degrees)

    def left(self, degrees):
        self.motor.turn(-degrees)

    def backward(self, distance):
        self.motor.straight(-distance)

    def followLine(self, speed, threshold):
        
        """ 
        This function should be put in a loop in order to fully work, and when finished make sure to end with self.motor.stop().\n
        
        threshold should be calculated like:
        a = light reflected by the black line
        b = light reflected elsewhere
        c = another value of light reflected elsewhere
        d = tuple(a, b, c)
        (a+b+c) / length(d)\n
        
        speed is milimeters per second
        """
        _, _, _, reflection = self.sensorOutput()
        deviation = reflection - threshold
        turn = 1.2 * deviation
        self.motor.drive(speed, turn)

    def sensorOutput(self):
        """
        returns: distance between the left side and and an object, distance between the right side and and an object,
        distance between the front side and and an object, and the light reflected in the color sensor.
        
        """


        leftSide = self.leftSensor.distance()
        rightSide = self.rightSensor.distance()
        frontSide = self.frontSensor.distance()
        lightReflected = self.colorSensor.reflection()
        return leftSide, rightSide, frontSide, lightReflected

    
