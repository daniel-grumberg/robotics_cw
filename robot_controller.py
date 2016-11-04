import brickpi
import time
import math

class RobotState:
  def __init__(self, x_pos=0, y_pos=0, rotation=0):
    self.x = x_pos
    self.y = y_pos
    self.rot = rotation

  def __str__(self):
    return '''\
x: {0}
y: {1}
theta: {2}'''.format(self.x, self.y, self.rot)

  def motionUpdate(self, distance, e=0, f=0):
    self.x += math.cos(self.rot)*(distance + e)
    self.y += math.sin(self.rot)*(distance + e)
    self.rot += f

  def rotationUpdate(self, rotation, g):
    self.rot = (self.rot + rotation + g) % (2*math.pi)

class Robot:
  def __init__(self,
               interface,
               motorParams,
               motors,
               wheelRad,
               wheelDist,
               sensors):

    self.interface = interface
    self.motors = motors
    self.wheelRadius = wheelRad
    self.wheelDistance = wheelDist
    self.state = RobotState()
    #enable the motors
    for mot in motors:
      interface.motorEnable(mot)
      #set the controller parameters
      interface.setMotorAngleControllerParameters(mot, motorParams)

  #with statement entry
  def __enter__(self):
    return self
  #with statement exit
  def __exit__(self, exc_type, exc_value, traceback):
    self.interface.terminate()

  def rotationDistance(self, rad):
    return (rad / 2) * self.wheelDistance

  def turnLeft(self, rad):
    distance = self.rotationDistance(rad, self.wheelDistance)
    angle = self.distanceToAngle(distance, self.wheelRadius)
    self.interface.increaseMotorAngleReferences(self.motors,
                                                [-angle, angle])
    self.waitUntilReached()

  def turnRight(self, rad):
    self.turnLeft(-rad)

  def distanceToAngle(self, distance):
    return distance / self.wheelRadius

  def moveForward(self, distance):
    angle = self.distanceToAngle(distance)
    self.interface.increaseMotorAngleReferences(self.motors,
                                                [angle, angle])
    self.waitUntilReached()

  def motion(self, distance, e=0, f=0):
    self.moveForward(distance)
    self.state.motionUpdate(distance, e, f)

  def rotate(self, rad, g=0):
    self.turnLeft(rad)
    self.state.rotationUpdate(rad, g)

  def waitUntilReached(self):
    while not self.interface.motorAngleReferencesReached(self.motors):
      motorAngles = self.interface.getMotorAngles(self.motors)
      if motorAngles :
        print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
      time.sleep(0.1)
      print "Destination reached!"
