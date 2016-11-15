import brickpi
import time
import math
import sys
import random
from robot_controller import Robot, RobotState
import particleDataStructures as dp


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

numberOfParticles=100

def getRandom(x):
  return random.gauss(0, x)

def drawParticles(ps):
  dp.canvas.drawParticles([p.print() for p in ps])

def main():

  s_x = 0
  s_y = 0
  s_rot = 0
  s_w = 1/numberOfParticles

  particles = [Particle(s_x, s_y, s_rot, s_w) for _ in range(numberOfParticles)]

  drawParticles(particles)

#will automagically terminate the interface upon exit
  with Robot(interface, motorParams, motors, robotWheelRadius,
      robotWheelDistance, 0, numberOfParticles) as robot:

    points = [(84, 30), (180, 30), (180,54), (138, 54), (138, 168), (114, 168),
              (114, 84), (84, 84), (84, 30)]

    for x, y in points:
      robot.moveTo(x, y)



if __name__ == "__main__":
  main()
