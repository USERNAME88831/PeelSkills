#!/usr/bin/env pybricks-micropython
# This file is the program that will be run by the brick
# TODO: Comment out all references to third motor, as we have no current use for it.
from robot import ROBOT
from ports import *
"""
motor ports are A, B, C, and D
sensor ports are S1, S2, S3, and S4
"""



# TODO: replace these values with the actual ports in the brick
leftWheelPort = A
rightWheelPort = D
thirdMotorPort = B # should be given a name after we find out the use of it

leftSensorPort = S1
rightSensorPort = S2
frontSensorPort = S4
colorSensorPort = S3


bill = ROBOT(leftWheelPort, rightWheelPort, thirdMotorPort,colorSensorPort, frontSensorPort, leftSensorPort, rightSensorPort,debugMode=True, overrideSafetyFeatures=False)

# Waiting until a button is pressed
while True:
    buttonsPressed = bill.brick.buttons.pressed()
    if CENTERBUTTON in buttonsPressed:
        break 
buttonsPressed = None
while True:

    a = bill.followLine(100)
    buttonsPressed = bill.brick.buttons.pressed()
    if RIGHT in buttonsPressed or a==-1:
        break


# Main program starts here

