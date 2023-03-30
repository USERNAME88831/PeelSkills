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

        



# Main program starts here

startLocaction = "P5"

bill.position = lookUp(startLocaction)

# CHALLENGE 2 

bill.goTo("C25")
bill.liftUp(90)
bill.up_box(1)
bill.resetLift()
bill.down_box(2)


# CHALLENGE 5

bill.liftUp(90)
bill.goTo("T24")
bill.resetLift()
bill.goTo("V1")
bill.liftUp(90)
bill.down_box(2)

bill.goTo("T25")
bill.resetLift()
bill.goTo(startLocaction)
bill.liftUp(90)
bill.down_box(2)

bill.goTo("R25")
bill.resetLift()
bill.goTo("V1")
bill.liftUp(90)
bill.down_box(2)


bill.goTo("R24")
bill.resetLift()
bill.goTo(startLocaction)
bill.liftUp(90)
bill.down_box(2)


# CHALLENGE 3

bill.goTo("N31")
bill.resetLift()
bill.goTo("C41")
bill.liftUp(90)
bill.down_box(3)

# CHALLENGE  1

bill.goTo("B13")
bill.resetLift()
bill.goTo("E34")
bill.liftUp(90)

bill.goTo("F11")
bill.resetLift()
bill.goTo("F33")
bill.liftUp(90)

bill.goTo("F15")
bill.resetLift()
bill.goTo("G33")
bill.liftUp(90)

bill.down_box(3)

bill.goTo(startLocaction)
#  G32, G31 is where the mangos should be placed