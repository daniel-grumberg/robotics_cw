import brickpi
import time
import math

class Particle:
  def __init__(self, x_pos=0, y_pos=0, rotation=0, w=1):
    self.state = RobotState(x_pos, y_pos, rotation);
    self.weight = w

  def print(self)
      return (self.state.x, self.state.y, self.state.rot, self.weight)

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

  def particle(self):
    xOffset = 100
    yOffset = 520
    return (xOffset+int(round(12*self.x)),
            yOffset-int(round(12*self.y)),
            int(round(self.rot*360/(2*math.pi))))

  def motionUpdate(self, distance, e=0, f=0):
    self.x += math.cos(self.rot)*(distance + e)
    self.y += math.sin(self.rot)*(distance + e)
    self.rot += f

  def rotationUpdate(self, rotation, g):
    self.rot = (self.rot + rotation + g) % (2*math.pi)

  def updateState(self, new_x, new_y, new_rot):
    self.x = new_x
    self.y = new_y
    self.rot = new_rot

class Robot:
  def __init__(self,
               interface,
               motorParams,
               motors,
               wheelRad,
               wheelDist,
               sensors,
               particles):

    self.interface = interface
    self.motors = motors
    self.wheelRadius = wheelRad
    self.wheelDistance = wheelDist
    self.state = RobotState()
    self.particles = [Particle(0, 0, 0, 1/particles) for _ in
                      range(particles)]
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
    distance = self.rotationDistance(rad)
    angle = self.distanceToAngle(distance)
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

  def moveTo(self, x_coord, y_coord):
    d_x = x_coord - self.state.x
    d_y = y_coord - self.state.y
    print(d_x)
    print(d_y)

    d_rot = math.atan2(d_y, d_x) - self.state.rot
    print(d_rot)
    d_rot = (d_rot + math.pi)%(2*math.pi)-math.pi # Takes smallest angle
    print(d_rot)
    r = math.hypot(d_x, d_y)
    self.rotate(d_rot)
    if (r <= 20):
      self.motion(r)
    else:
      self.motion(20)
      moveTo(self, x_coord, y_coord)

  def waitUntilReached(self):
    while not self.interface.motorAngleReferencesReached(self.motors):
      motorAngles = self.interface.getMotorAngles(self.motors)
      #if motorAngles :
        #print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
      time.sleep(0.1)
      #print "Destination reached!"
