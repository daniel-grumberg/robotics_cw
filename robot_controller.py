import brickpi
import time
import math
import copy
import random
import particleDataStructures

sigma = 2
K = 0.2

class Particle:
  def __init__(self, x_pos=0, y_pos=0, rotation=0, w=1):
    self.state = RobotState(x_pos, y_pos, rotation);
    self.weight = w

  def toTuple(self):
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
               x,
               y,
               r,
               numParticles):

    self.interface = interface
    self.motors = motors
    self.wheelRadius = wheelRad
    self.wheelDistance = wheelDist
    self.state = RobotState(x, y, r)
    self.particles = [Particle(x, y, r, 1.0/numParticles) for _ in range(numParticles)]
    #enable the motors
    for mot in motors:
      interface.motorEnable(mot)
      #set the controller parameters
      interface.setMotorAngleControllerParameters(mot, motorParams)

      interface.sensorEnable(0, brickpi.SensorType.SENSOR_ULTRASONIC)

  #with statement entry
  def __enter__(self):
    return self
  #with statement exit
  def __exit__(self, exc_type, exc_value, traceback):
    self.interface.terminate()

  def rotationDistance(self, rad):
    return (rad / 2) * self.wheelDistance

  def turn(self, rad):
    distance = self.rotationDistance(rad)
    angle = self.distanceToAngle(distance)
    self.interface.increaseMotorAngleReferences(self.motors,
                                                [-angle, angle])
    self.waitUntilReached()

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
    self.turn(rad)
    self.state.rotationUpdate(rad, g)

  def moveTo(self, x_coord, y_coord):
    d_x = x_coord - self.state.x
    d_y = y_coord - self.state.y

    d_rot = math.atan2(d_y, d_x) - self.state.rot
    d_rot = (d_rot + math.pi)%(2*math.pi)-math.pi # Takes smallest angle
    r = math.hypot(d_x, d_y)
    self.rotate(d_rot)

    r_prime = min(r, 20)
    self.motion(r_prime)
    self.updateParticles(d_rot, r_prime)
    self.particles = self.resample()
    self.updateRobotState()

    if (r > 20):
      self.moveTo(x_coord, y_coord)

  def updateRobotState(self):
    x_cum = 0
    y_cum = 0
    rot_cumx = 0
    rot_cumy = 0
    N = len(self.particles)

    for particle in self.particles:
      x_cum += particle.state.x
      y_cum += particle.state.y
      rot_cumx += math.cos(particle.state.rot)
      rot_cumy += math.sin(particle.state.rot)

    rot_avg = math.atan2(rot_cumy/N, rot_cumx/N)
    self.state.updateState(x_cum/N, y_cum/N, rot_avg)

  def gaussian(self, a, b, c, x):
    return a * math.exp(-((x-b)**2/(2*c**2)))

  def calculateLikelihood(self, particle, z):
    m = self.calculateM(particle)
    return self.gaussian(1, m, sigma, z) + K

  def updateParticles(self, d_rot, r):
    usReading = self.interface.getSensorValue(0)[0]
    for particle in self.particles:
      particle.state.rotationUpdate(d_rot,random.gauss(0,0.02))
      particle.state.motionUpdate(r, random.gauss(0,0.1), random.gauss(0,0.01))
      self.updateWeight(particle, usReading)

  def updateWeight(self, particle, z):
    return particle.weight * self.calculateLikelihood(particle, z)

  def calculateM(self, particle):
    for wall, label in particleDataStructures.mymap.walls:
      Ax = min(wall[0], wall[2])
      Ay = min(wall[1], wall[3])
      Bx = max(wall[0], wall[2])
      By = max(wall[1], wall[3])

      x = particle.state.x
      y = particle.state.y
      theta = particle.state.rot
      m = ((By - Ay)*(Ax - x) - (Bx - Ax)*(Ay - y))/((By - Ay)*math.cos(theta) - (Bx - Ax)*math.sin(theta))
      mx = x + m * math.cos(theta)
      my = y + m * math.sin(theta)
      if (m > 0 and mx >= Ax and mx <= By and my >= Ay and my <= By):
        return m
    return -1

  def resample(self):
    newParticles = []
    weight_cum = 0
    new_weight = 1.0 / len(self.particles)
    for particle in self.particles:
      weight_cum += particle.weight
    for i in range(len(self.particles)):
      r = random.uniform(0, weight_cum)
      cum = 0
      for particle in self.particles:
        cum += particle.weight
        if r < cum:
          newParticle = copy.deepcopy(particle)
          newParticle.weight = new_weight
          newParticles.append(newParticle)
          break
    return newParticles

  def waitUntilReached(self):
    while not self.interface.motorAngleReferencesReached(self.motors):
      motorAngles = self.interface.getMotorAngles(self.motors)
      #if motorAngles :
        #print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
      time.sleep(0.1)
      #print "Destination reached!"
