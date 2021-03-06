import brickpi
import time
import math
import sys
import random
from robot_controller import Robot, RobotState

interface = brickpi.Interface()
interface.initialize()

carpet = 1

motors = [0, 1]
robotWheelRadius = 2.0 + 0.1 + carpet*0.3
robotWheelDistance = 13.5 + 1.38 + carpet*2.85

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

#Variation parameters
e=0.1
f=0.01
g=0.02

def getRandom(x):
  return random.gauss(0, x)


def main():

  states = [RobotState() for _ in range(100)]
  particles = [states[i].particle() for i in range(100)]

  print "drawParticles:" + str(particles)

  line1 = (100, 20, 100, 520)
  line2 = (100, 520, 600, 520)
  print "drawLine:" + str(line1)
  print "drawLine:" + str(line2)

#will automagically terminate the interface upon exit
  with Robot(interface, motorParams, motors, robotWheelRadius,
      robotWheelDistance, 0) as robot:
    for i in range(4):
      for j in range(4):
        robot.motion(10)
        for k in range(100):
          states[k].motionUpdate(10, getRandom(e), getRandom(f))
          particles[k] = states[k].particle()
        print "drawParticles:" + str(particles)

      robot.rotate(math.pi/2)
      for l in range(100):
        states[l].rotationUpdate(math.pi/2, getRandom(g))
        particles[l] = states[l].particle()
      print "drawParticles:" + str(particles)

if __name__ == "__main__":
  main()
