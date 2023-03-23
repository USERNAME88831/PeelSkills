# TODO: Create features that are useful in debug mode. 
from pybricks.tools import DataLog, StopWatch, wait
from threading import * 


class Logger:
    def __init__(self, motor, sensorFunction, leftMotor, rightMotor, thirdMotor, obstacleDetectFunc, thirdMotorOn=True): # TODO: Change name when we find the use of the third motor
        """
        The logger logs everything that the robot is doing in a file \n
        
        """
        self.motor = motor
        self.sensorFunction = sensorFunction
        self.leftMotor = leftMotor
        self.rightMotor = rightMotor
        self.thirdMotor = None
        if thirdMotorOn:
            self.thirdMotor = thirdMotor
        self.obstacleDetectFunc = obstacleDetectFunc
        
        self.data = None

        if thirdMotorOn:
            self.data = DataLog("leftSensorDistance", 
                                "rightSensorDistance",
                                "frontSensorDistance",
                                "colorSensorReflection"
                                "leftWheelAngle", # Individual wheel
                                "rightWheelAngle", # Individual wheel
                                "thirdMotorAngle",
                                "robotSpeed",
                                "obstacleDetected"
                                )
        else:
            self.data = DataLog("leftSensorDistance", 
                                "rightSensorDistance",
                                "frontSensorDistance",
                                "colorSensorReflection"
                                "leftWheelAngle", # Individual wheel
                                "rightWheelAngle", # Individual wheel
                                "robotSpeed",
                                "obstacleDetected"
                                )

        self.thread = Thread(target=self._logFunc)
        self.thread.setDaemon(True) # Since it runs in the background, it will immediatly end when the non-dameon threads end.



    def _logFunc(self):
        """
        this function is to log stuff during the entire program 
        """
        while True:
            leftSide, rightSide, frontSide, lightReflected = self.sensorFunction()
            leftWheelAngle = self.leftMotor.angle()
            rightWheelAngle = self.rightMotor.angle()
            if self.thirdMotor != None:
                thirdMotorAngle = self.thirdMotor.angle()
            
            _, robotSpeed, _, _ = self.motor.state()
            obstacleDetected = self.obstacleDetectFunc()
            
            if self.thirdMotor != None:
                self.data.log(leftSide,
                            rightSide,
                            frontSide,
                            lightReflected,
                            leftWheelAngle,
                            rightWheelAngle,
                            thirdMotorAngle,
                            robotSpeed,
                            obstacleDetected
                            )
            else:
                self.data.log(leftSide,
                            rightSide,
                            frontSide,
                            lightReflected,
                            leftWheelAngle,
                            rightWheelAngle,
                            robotSpeed,
                            obstacleDetected
                            )
        
    def startLogging(self):
        self.thread.start()
