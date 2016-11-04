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

touch_ports = [0, 1]
interface.sensorEnable(touch_ports[0], brickpi.SensorType.SENSOR_TOUCH)
interface.sensorEnable(touch_ports[1], brickpi.SensorType.SENSOR_TOUCH)

motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 6.0
motorParams.maxRotationSpeed = 6.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 18.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255
motorParams.pidParameters.k_p = 80
motorParams.pidParameters.k_i = 400
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

def moveBackward(distance):
  moveForward(-distance)

def waitUntilReached():
  while not interface.motorAngleReferencesReached(motors):
    motorAngles = interface.getMotorAngles(motors)
    if motorAngles :
      print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
    time.sleep(0.1)
    print "Destination reached!"

def startMotors(speed):
  interface.setMotorRotationSpeedReferences(motors, [speed, speed])

def stopMotors():
  interface.setMotorPwm(motors[0], 0)
  interface.setMotorPwm(motors[1], 0)
  time.sleep(0.1)

interface.setMotorAngleControllerParameters(motors[0],motorParams)
interface.setMotorAngleControllerParameters(motors[1],motorParams)

while(1):
  startMotors(8.0);
  left_touched = interface.getSensorValue(touch_ports[0])[0]
  right_touched = interface.getSensorValue(touch_ports[1])[0]
  if(left_touched):
    stopMotors()
    moveBackward(3)
    turnRight(math.pi/8)
  elif(right_touched):
    stopMotors()
    moveBackward(3)
    turnLeft(math.pi/8)

interface.terminate()
