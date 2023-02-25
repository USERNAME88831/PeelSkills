#!/usr/bin/env pybricks-micropython
# This file is the program that will be run by the brick
from robot import ROBOT
from ports import *

"""
motor ports are A, B, C, and D
sensor ports are S1, S2, S3, and S4
"""



# TODO: replace these values with the actual ports in the brick
leftWheelPort = A
rightWheelPort = B
thirdMotorPort = C # should be given a name after we find out the use of it

leftSensorPort = S1
rightSensorPort = S2
frontSensorPort = S3
colorSensorPort = S4

bill = ROBOT()