import brickpi
import time
import math
import copy
import random
import particleDataStructures

#sigma = 1.5
sigma = 4.5
K = 0.01
x_diff = 0.5

e = 0.5
f = 0.01
g = 0.02

eps = 0.01
bottle_eps = 5 * math.pi / 180

NUM=30

THRESHHOLD=0.04

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
               numParticles,
               usMotor=None,
               usParams=None,
               touchports=None):

    self.timeout = None
    self.checkTouch = False
    self.isArea = False
    self.interface = interface
    self.motors = motors
    self.usMotor = usMotor
    self.wheelRadius = wheelRad
    self.wheelDistance = wheelDist
    self.state = RobotState(x, y, r)
    self.coordinateUsTheta = r
    self.usTheta = 0
    self.particles = [Particle(x, y, r, 1.0/numParticles) for _ in range(numParticles)]
    self.touchports = touchports
    #enable the motors
    for mot in motors:
      interface.motorEnable(mot)
      #set the controller parameters
      interface.setMotorAngleControllerParameters(mot, motorParams)

    if (usMotor != None):
      interface.motorEnable(usMotor)
      if (usParams == None):
        interface.setMotorAngleControllerParameters(usMotor, motorParams)
      else:
        interface.setMotorAngleControllerParameters(usMotor, usParams)
    if (touchports != None):
      for tp in touchports:
        interface.sensorEnable(tp, brickpi.SensorType.SENSOR_TOUCH)

    interface.sensorEnable(self.usMotor, brickpi.SensorType.SENSOR_ULTRASONIC)

  #with statement entry
  def __enter__(self):
    return self
  #with statement exit
  def __exit__(self, exc_type, exc_value, traceback):
    print 'Exit'
    self.faceAhead()
    self.interface.terminate()

  def rotationDistance(self, rad):
    return (rad / 2) * self.wheelDistance

  def turn(self, rad, usTheta, selfUsTheta=None):
    print 'coordinateUsTheta = ' + str(self.coordinateUsTheta)
    print 'selfUsTheta = ' + str(self.usTheta)
    motors = [self.motors[0], self.motors[1]]

    distance = self.rotationDistance(rad)
    angle = self.distanceToAngle(distance)

    angles = [-angle, angle]
    self.coordinateUsTheta = (self.coordinateUsTheta + rad + math.pi) % (2*math.pi) - math.pi #TODO: Add 'g' parameter for variability
    if (usTheta == None and selfUsTheta != None):
      usTheta = self.coordinateUsTheta - self.usTheta + selfUsTheta
    if (usTheta != None):
      motors.append(self.usMotor)
      angles.append(-self.lookAt(usTheta))
      self.coordinateUsTheta = usTheta

    self.interface.increaseMotorAngleReferences(motors,
                                                angles)
    self.fastWaitUntilReached(motors)


  def distanceToAngle(self, distance):
    return distance / self.wheelRadius

  def angleToDistance(self, angle):
    return angle * self.wheelRadius

  #def moveArc(self, r=19):
  #  motors = [self.motors[0],self.motors[1],self.usMotor]
  #  self.interface.increaseMotorAngleReferences(motors,
  #      [self.distanceToAngle(r-8.125)*math.pi,
  #        self.distanceToAngle(r+8.125)*math.pi,
  #        math.pi/2])
  #  self.fastWaitUntilReached(motors)

  def returnArc(self, r=30-7.5, usDiff=None):
    #usTheta=math.pi #should be set to make face look backwards
    motors = [self.motors[0], self.motors[1]]
    cons = 2#2.08
    angles = [self.distanceToAngle(-(r-8.125))*math.pi/cons, self.distanceToAngle(-(r+8.125))*math.pi/cons]
    if (usDiff != None):
      motors.append(self.usMotor)
      angles.append(usDiff)
    self.interface.increaseMotorAngleReferences(motors, angles)
    self.fastWaitUntilReached(self.motors)

  def startArc(self, x, y, r=25-0.5, timeout=3.5):
    self.timeout=timeout
    #self.rotate(0, None, selfUsTheta=math.pi/2)
    angle = -self.lookAt(self.coordinateUsTheta - self.usTheta - math.pi / 2)
    motors = [self.motors[0], self.motors[1], self.usMotor]
    cons = 2-0.04#2.08
    angles = [self.distanceToAngle((r-8.125))*math.pi/cons, self.distanceToAngle((r+8.125))*math.pi/cons, angle]
    self.interface.increaseMotorAngleReferences(motors, angles)
    self.fastWaitUntilReached(self.motors)

    #self.rotate(0, math.pi / 2)
    #self.returnArc(r)
    self.state.updateState(x, y, self.state.rot + math.pi/2)
    self.coordinateUsTheta = 0

  def stopMotors(self, third=False):
    self.interface.setMotorPwm(self.motors[0], 0)
    self.interface.setMotorPwm(self.motors[1], 0)
    if (third):
      self.interface.setMotorPwm(self.usMotor, 0)
    time.sleep(0.02)

  def returnSequence(self):
    #FUCK THE STATE OF THE ROBOT WHEN THIS IS CALLED

    rad = (-math.pi/2 - self.state.rot + math.pi) % (2*math.pi) - math.pi
    print rad
    self.rotate(rad, None, selfUsTheta=0)

    angle = self.distanceToAngle(210)
    self.interface.increaseMotorAngleReferences(self.motors,[angle, angle])
    while (not (self.interface.getSensorValue(self.touchports[0])[0]
             and self.interface.getSensorValue(self.touchports[1])[0])):
      time.sleep(0.1)
    time.sleep(0.1) #ensures robot is fully rammed in wall
    self.stopMotors()
    time.sleep(0.1) #ensures robot is fully rammed in wall
    self.returnArc()

    usReading = self.interface.getSensorValue(self.usMotor)[0]
    val = (usReading - 84) / 20
    print '\tusReading: ' + str(usReading)
    self.moveForward((usReading-(84+val))*2, stopAt=84+val)

  def moveForward(self, distance, stopAt=None):
    angle = self.distanceToAngle(distance)
    self.interface.increaseMotorAngleReferences(self.motors,
                                                [angle, angle])
    if (stopAt == None):
      return self.waitUntilReached(self.motors)
    else:
      self.finalWaitUntilReached(self.motors, stopAt)

  def motion(self, distance, e=0, f=0):
    if self.moveForward(distance):
      self.state.motionUpdate(distance, e, f)

  def rotate(self, rad, usTheta, selfUsTheta=None):
    self.turn(rad, usTheta, selfUsTheta=selfUsTheta)
    self.state.rotationUpdate(rad, 0)

  def faceAhead(self):
    self.rotate(0, self.coordinateUsTheta - self.usTheta)

  def lookAt(self, theta):
    diff = (theta - self.coordinateUsTheta + math.pi) % (2*math.pi) - math.pi
    #if (self.usTheta + diff >= -math.pi/2 - eps and self.usTheta + diff <= math.pi/2 + eps):
    self.usTheta += diff
    return diff
    #else:
    #  print "ERROR: Bad theta value!"
    #  return 0

  def calculateMotion(self, startMotorAngles):
    angles = self.interface.getMotorAngles(self.motors)
    return (self.angleToDistance(angles[0][0]) + self.angleToDistance(angles[1][0]) - self.angleToDistance(startMotorAngles[0][0]) - self.angleToDistance(startMotorAngles[1][0])) / 2

  def moveTo(self, x_coord, y_coord, usTheta=None, isArea=False, checkLocation=False):
    if checkLocation:
      usReading = self.interface.getSensorValue(self.usMotor)
      if usReading and usReading[0] != 255:
        usReading = usReading[0]
        print "___________"
        print usReading
        m = self.calculateM(self, isRobot=True)
        print m
        print "------------"
        self.motion(usReading - m)

    self.isArea = isArea
    if isArea:
      self.checkTouch = True
    d_x = x_coord - self.state.x
    d_y = y_coord - self.state.y

    d_rot = math.atan2(d_y, d_x) - self.state.rot# + math.pi
    d_rot = (d_rot + math.pi)%(2*math.pi)-math.pi # Takes smallest angle
    r = math.hypot(d_x, d_y)
    self.rotate(d_rot, usTheta)

    #r_prime = min(r, 20)
    #self.motion(r_prime)
    #self.updateParticles(d_rot, r_prime)
    self.motion(r)
    #self.motion(-r)
    #self.updateParticles(d_rot, r)
    #self.normalise()
    #particleDataStructures.canvas.drawParticles([p.toTuple() for p in self.particles])
    #time.sleep(0.3)
    #self.particles = self.resample()
    #particleDataStructures.canvas.drawParticles([p.toTuple() for p in self.particles])
    #self.updateRobotState()

    #if (r > 20):
    #  self.moveTo(x_coord, y_coord)

  def normalise(self):
    w_cum = 0
    for particle in self.particles:
      w_cum += particle.weight
    for particle in self.particles:
      particle.weight /= w_cum

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

  def calculateLikelihood(self, particle, z, isRobot=False):
    m = self.calculateM(particle, isRobot=isRobot)
    if (m == -1):
      return K
    if (z != 255 and z > m):
      return 1
    return self.gaussian(1, m, sigma, z) + K

  def updateParticles(self, d_rot, r):
    usReadings = []
    i = 0
    while i < 10:
      usReading = self.interface.getSensorValue(self.usMotor)
      if (usReading):
        usReadings.append(usReading[0])
        i += 1
      time.sleep(0.001)
    usReadings.sort()
    usReading = usReadings[len(usReadings) / 2] + x_diff

    for particle in self.particles:
      particle.state.rotationUpdate(d_rot,random.gauss(0, g))
      particle.state.motionUpdate(r, random.gauss(0, e), random.gauss(0, f))
      self.updateWeight(particle, usReading)

  def updateWeight(self, particle, z):
    p = self.calculateLikelihood(particle, z)
    particle.weight = p# * particle.weight

  def calculateM(self, particle, isRobot=False):
    min_m = -1
    min_label = 'undefined'
    ms = []
    for wall, label in particleDataStructures.mymap.walls:
      Ax = min(wall[0], wall[2])
      Ay = min(wall[1], wall[3])
      Bx = max(wall[0], wall[2])
      By = max(wall[1], wall[3])

      x = particle.state.x
      y = particle.state.y
      theta = particle.state.rot
      if (isRobot):
        theta = particle.coordinateUsTheta
      if (theta == 0):
        theta = eps
      m = ((By - Ay)*(Ax - x) - (Bx - Ax)*(Ay - y))/((By - Ay)*math.cos(theta) - (Bx - Ax)*math.sin(theta))
      mx = x + m * math.cos(theta)
      my = y + m * math.sin(theta)
      ms.append((mx, my))
      if (m > 0 and mx >= Ax - eps and mx <= Bx + eps and my >= Ay - eps and my <= By + eps):
        if (min_m == -1 or m < min_m):
          min_m = m
          min_label = label
    if (min_m == -1):
      pass
      #print particle.state.x
      #print particle.state.y
      #print particle.state.rot
      #print ms
      #print
    #print("Looking at wall:\t" + min_label)
    return min_m

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

  def waitUntilReached(self, motors):
    startMotorAngles = self.interface.getMotorAngles(motors)
    diff_pos = 0
    weight = 0
    num = 0
    isBottle = False
    bottle_dist = 0

    start_x, start_y, start_rot = self.state.x, self.state.y, self.state.rot

    while not self.interface.motorAngleReferencesReached(motors):
      if self.isArea:
        usReadings = []
        i = 0
        while i < 10:
          usReading = self.interface.getSensorValue(self.usMotor)
          if (usReading):
            usReadings.append(usReading[0])
            i += 1
          time.sleep(0.001)
        usReadings.sort()
        usReading = usReadings[len(usReadings) / 2] + x_diff
        likelihood = self.calculateLikelihood(self, usReading, isRobot=True)

        #if likelihood < THRESHHOLD:
        #  if (not isBottle):
        #    isBottle = True
        #  diff_pos += self.calculateMotion(startMotorAngles)
        #  num += 1
        #else:
        #  if isBottle:
        #    self.isArea = False
        #    print 'Found Bottle'
        #    isBottle = False
        #    print '.'
        #    self.stopMotors()
        #    print '.'
        #    pos = self.calculateMotion(startMotorAngles)
        #    print '.'
        #    self.state.motionUpdate(pos)
        #    print '.'
        #    diff_pos /= num
        #    self.motion(-(pos - diff_pos))
        #    print '.'
        #    self.turn(self.coordinateUsTheta - self.state.rot - bottle_eps, None, selfUsTheta=0)
        #    print '.'
        #    self.motion(210)
        #    print '.'
        #    return False
        print '\t\tusReading: ' + str(usReading) + ", " + str(likelihood)
        if likelihood < THRESHHOLD:
          if weight < NUM:
            if usReading == 255:
              weight += 5
            else:
              weight += 10*math.exp(-usReading / 15)
            num +=1
            bottle_dist += usReading
          else:
            self.isArea = False
            print 'Found Bottle'
            isBottle = False
            print '.'
            self.stopMotors()
            print '.'
            pos = self.calculateMotion(startMotorAngles)
            print '.'
            self.state.motionUpdate(pos)
            print '.'
            #diff_pos /= num
            #self.motion(-(pos - diff_pos))
            #print '.'
            ang = (self.coordinateUsTheta - self.state.rot - bottle_eps + math.pi) % (2*math.pi) - math.pi
            self.rotate(ang, None, selfUsTheta=0)
            print '.'
            bottle_dist = bottle_dist / num + 10
            self.motion(bottle_dist)
            print '.'
            return False
        else:
          if weight > 0:
            weight -= 2

      if self.checkTouch:
        if (self.interface.getSensorValue(self.touchports[0])[0]
            or self.interface.getSensorValue(self.touchports[1])[0]):
          print 'Touched'
          print 'coordinateUsTheta = ' + str(self.coordinateUsTheta)
          print 'selfUsTheta = ' + str(self.usTheta)
          self.checkTouch = False
          print '|'
          print '__'
          self.stopMotors()
          print '__'
          print '|'
          print 'Angle = ' + str(self.state.rot)
          print self.state.x
          print self.state.y
          self.state.motionUpdate(self.calculateMotion(startMotorAngles))
          print 'Angle = ' + str(self.state.rot)
          print self.state.x
          print self.state.y
          print '|'
          self.motion(-12)
          return False

      #usReading = self.interface.getSensorValue(self.usMotor)
      #if (usReading):
      #  usReading = usReading[0]
      #  print self.calculateLikelihood(self, usReading, isRobot=True)
    if self.isArea and self.checkTouch:
      self.isArea = False
      print 'Didn\'t find Bottle'
      isBottle = False
      print '.'
      self.stopMotors()
      print '.'
      pos = self.calculateMotion(startMotorAngles)
      print '.'
      self.state.motionUpdate(pos)
      print '.'
      ang = (self.coordinateUsTheta - self.state.rot - bottle_eps + math.pi) % (2*math.pi) - math.pi
      self.rotate(ang, None, selfUsTheta=0)
      print '.'
      self.motion(210)
      print '.'
      return False

    return True

  def fastWaitUntilReached(self, motors):
    startTime = time.time()
    while not self.interface.motorAngleReferencesReached(motors):
      print time.time() - startTime
      if (self.timeout != None and time.time() - startTime > self.timeout):
        self.timeout = None
        print 'Stop motors now'
        self.stopMotors(third=True)
        break
      time.sleep(0.1)

  def finalWaitUntilReached(self, motors, stopAt):
    while not self.interface.motorAngleReferencesReached(motors):
      usReading = self.interface.getSensorValue(self.usMotor)[0]
      print '\tusReading: ' + str(usReading)
      if (abs(usReading - stopAt) < 3):
        self.stopMotors()
        break
      time.sleep(0.01)
