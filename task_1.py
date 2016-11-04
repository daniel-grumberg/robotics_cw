import brickpi
import time
import math

interface = brickpi.Interface()
interface.initialize()

motors = [0, 1]
robotWheelRadius = 2.0 + 0.1
robotWheelDistance = 13.5 + 1.15

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 6.0
motorParams.maxRotationSpeed = 12.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 18.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255
motorParams.pidParameters.k_p = 300
motorParams.pidParameters.k_i = 650
motorParams.pidParameters.k_d = 5

def rotationDistance(rad, wheelDistance):
  return rad / 2 * wheelDistance

def turnLeft(rad):
  distance = rotationDistance(rad, robotWheelDistance)
  angle = distanceToAngle(distance, robotWheelRadius)
  interface.increaseMotorAngleReferences(motors, [-angle, angle])
  waitUntilReached()

def turnRight(rad):
  turnLeft(-rad)

def distanceToAngle(distance, wheelRadius):
  return distance / wheelRadius

def moveForward(distance):
  angle = distanceToAngle(distance, robotWheelRadius)
  interface.increaseMotorAngleReferences(motors, [angle, angle])
  waitUntilReached()

def waitUntilReached():
  while not interface.motorAngleReferencesReached(motors):
    motorAngles = interface.getMotorAngles(motors)
    if motorAngles :
      print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
    time.sleep(0.1)
    print "Destination reached!"

interface.setMotorAngleControllerParameters(motors[0],motorParams)
interface.setMotorAngleControllerParameters(motors[1],motorParams)

moveForward(40)
for i in range(3):
  turnLeft(math.pi/2)
  moveForward(40)


interface.terminate()
