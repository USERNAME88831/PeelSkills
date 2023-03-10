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
leftWheelPort = B
rightWheelPort = C
# thirdMotorPort = D # should be given a name after we find out the use of it

leftSensorPort = S1
rightSensorPort = S2
frontSensorPort = S3
colorSensorPort = S4


"""
bill = ROBOT(leftWheelPort, rightWheelPort, thirdMotorPort,
             colorSensorPort, frontSensorPort, leftSensorPort, rightSensorPort,
             debugMode=True, overrideSafetyFeatures=False)
"""

bill = ROBOT(leftWheelPort, rightWheelPort,
             colorSensorPort, frontSensorPort, leftSensorPort, rightSensorPort,
             debugMode=True, overrideSafetyFeatures=False)

# Waiting until a button is pressed
while True:
    buttonsPressed = bill.brick.buttons.pressed()
    if CENTERBUTTON in buttonsPressed:
        break

# Main program starts here

bill.displayStats()