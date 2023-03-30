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
from ports import *

d = [11, 80]

class ROBOT():    

    def __init__(self, leftMotorPort, rightMotorPort, M3Port, ColorSensorPort, FrontSensorPort, LeftSensorPort, RightSensorPort, debugMode = False, overrideSafetyFeatures=False, thirdMotorOn=True):
        """
    def __init__(self, leftMotorPort, rightMotorPort, M3Port, ColorSensorPort, FrontSensorPort, 
            LeftSensorPort, RightSensorPort, debugMode = False, overrideSafetyFeatures=False):

            
        """
       


        self.wheelDiameter = 60 # the diameter of the wheels
        self.axleTrack = 125 # the horizontal distance between the two wheels, practically the width of the robot  
        # TODO: get diameter of the two wheels, and horizontal distance between the two wheels

        self.safeMode = not overrideSafetyFeatures # safety features should only be turned off while testing and debugging

        self.obstacleDetected = False # when the robot comes to a complete stop because of saftey features, this boolean is set to true
        # NOTE: when the obstacle isn't a problem anymore, the program has to manually set it to false
        self.brick = EV3Brick()
        self.slowDownDistance = 500 # slows down when it is near this distance
        self.stopDistance = 200 # comes to a complete stop when it reaches this distance
    
        self.LeftWheel = Motor(leftMotorPort)
        self.RightWheel = Motor(rightMotorPort)

        self.motor = DriveBase(self.LeftWheel, self.RightWheel, self.wheelDiameter, self.axleTrack) # The class used to drive robots
        
        self.m3 = None
        if thirdMotorOn:
            self.m3 = Motor(M3Port) # TODO: find use of the third motor
        self.colorSensor = ColorSensor(ColorSensorPort) # Should be used to track the lines
        self.frontSensor = UltrasonicSensor(FrontSensorPort)
        try:
            self.leftSensor = UltrasonicSensor(LeftSensorPort)
        except Exception as e:
            print(str(e))
        self.rightSensor = UltrasonicSensor(RightSensorPort)
        self.debugMode = debugMode # enables certain features to test the robot.
        self.hasThirdMotor = thirdMotorOn
        self.logger = Logger(self.motor,
                             self.sensorOutput,
                             self.LeftWheel,
                             self.RightWheel,
                             self.m3,
                             self.isObstacleDetected,
                             thirdMotorOn
                             )

        self.position = [0,0]
        self._statThread = Thread(target=self._statFunc)
        self._statThread.daemon = True

        self.pos = [[0,0],[0,1],[1,1],[1,0]]
        self.pos_1 = 0
        self.direction = [0,0] # (0,0) = Forward, (1,1) = back, (1,0)= left, (0,1)=right
        
    def forward(self, distance):
        self.motor.straight(distance)

    def right(self, degrees):
        self.motor.turn(degrees)

    def left(self, degrees):
        self.motor.turn(-degrees)

    def backward(self, distance):
        self.motor.straight(-distance)

    def resetLift(self):
        self.m3.run_target(100, 0)

    def right_box(self):
        self.right(90)
        pos_1 = self.pos_1 + 1
        if pos_1 == 4:
            pos_1 = 0
        self.direction = self.pos[pos_1]
        self.pos_1 = pos_1
        
    def left_box(self):
        self.left(90)
        pos_1 = self.pos_1 - 1
        if pos_1 == -1:
            pos_1 = 3
        self.direction = self.pos[pos_1]
        self.pos_1 = pos_1

    def up_box(self, distance):
        line = 50

        if self.direction == [0,0]:
            a = self.position[1] + distance
            self.position[1] = a
        elif self.direction == [0,1]:
            a = self.position[0] + distance
            self.position[0] = a 
        elif self.direction == [1,0]:
            a = self.position[0] - distance
            print(self.position)
            self.position[0] = a
        else:
            a = self.position[1] - distance
            self.position[1] = a

        self.forward(line*distance)


    def down_box(self, distance):
        line = 50
        if self.direction == [0,0]:
            self.position[1] -= distance
        elif self.direction == [0,1]:
            self.position[0] -= distance
        elif self.direction == [1,0]:
            self.position[0] += distance
        else:
            self.position[1] += distance

    
        self.backward(line*distance)

    def liftUp(self, angle) -> int:
        maxAngle = 90

        minAngle = 0

        if self.hasThirdMotor:
            print(self.m3.angle()+angle)
            if angle/-angle == 1:
                if (self.m3.angle()+angle) >= minAngle:
                    self.m3.run_angle(100, angle)
                else:
                    return -1
            else:
                if (self.m3.angle()+angle) <= maxAngle:
                    self.m3.run_angle(100, angle)
                else:
                    return -1
        else:
            return -1


    def goTo(self, coordinate):
        coord_a = self.position
        coord_b = lookUp(coordinate)
        coord_c = (coord_a[0] - coord_b[0], coord_a[1] - coord_b[1])
        print(coord_b)
        if abs(coord_c[0])/-coord_c[0] == 1:
            while True:
        
                if self.direction == [0,1]:
                    break
                if self.pos_1 == 0:
                    self.right_box()
                elif self.pos_1 == 2:
                    self.right_box()
                else:
                    self.left_box()
            self.up_box(abs(coord_c[0]))
        else:
            while True:
                if self.direction == [1,0]:
                    break
                if self.pos_1 == 0:
                    self.left_box()
                elif self.pos_1 == 1:
                    self.right_box()
                else:
                    self.right_box()
            self.up_box(abs(coord_c[0]))

        if abs(coord_c[1])/-coord_c[1] == 1:
            while True:
                if self.direction == [0,0]:
                    break
                self.left_box()
            self.up_box(abs(coord_c[1]))
        else:
            while True:
                if self.direction == [1,1]:
                    break
                self.right_box()
            self.up_box(abs(coord_c[1]))
        

    def followLine(self, speed) -> int:
        
        """ 
        This function should be put in a loop in order to fully work, 
        and when finished make sure to end with self.motor.stop().\n
        speed is milimeters per second\n
        should return 0 if it performed fine, or -1 if something happened
        """
        _, _, fS, reflection = self.sensorOutput()

        threshold = sum(d) / len(d)

        
        score = (reflection - threshold) / (threshold // 2) # checks if its an outlier
        if abs(score) < 3:
            d.append(reflection)
        

        percentage = 1

        deviation = reflection - threshold
        turn = 1.2 * deviation
        if self.safeMode == True:
            if fS <= self.slowDownDistance:
                percentage = (self.slowDownDistance - fS) / (self.slowDownDistance - self.stopDistance)

        if percentage <= 0.1:
            self.motor.stop()
            return -1

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
            text = """
                    INGLEBOROUGH ROBOT\n
                       |{}|
                    """.format(robotSpeed)
            # TODO: add more info on the display
            self.brick.screen.print(text)
            


    def displayStats(self): ## displays information on the screen of the brick in the background
        self._statThread.start()
        

    def isObstacleDetected(self): # Provides information to other programs in other files
        return self.obstacleDetected