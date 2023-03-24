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
thirdMotorPort = D # should be given a name after we find out the use of it

leftSensorPort = S1
rightSensorPort = S2
frontSensorPort = S3
colorSensorPort = S4
d = [11, 80]

bill = ROBOT(leftWheelPort, rightWheelPort, thirdMotorPort,colorSensorPort, frontSensorPort, leftSensorPort, rightSensorPort,debugMode=True, overrideSafetyFeatures=False, thirdMotorOn=False)

# Waiting until a button is pressed
while True:
    buttonsPressed = bill.brick.buttons.pressed()
    if CENTERBUTTON in buttonsPressed:
        break 
buttonsPressed = None
while True:
    _, _, _, reflection = bill.sensorOutput()
    threshold = sum(d) / len(d)
    score = (reflection - threshold) / (threshold // 2) # checks if its an outlier
    if abs(score) < 3:
        d.append(reflection)
         

    bill.followLine(100, threshold)
    buttonsPressed = bill.brick.buttons.pressed()
    if RIGHT in buttonsPressed:
        break
print(threshold)

# Main program starts here

