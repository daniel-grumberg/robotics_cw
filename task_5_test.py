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
robotWheelRadius = 2.0 + 0.1 + carpet*0.0
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

def drawParticles(ps):
  dp.canvas.drawParticles([p.toTuple() for p in ps])

def main():

  start_point = (0,0)

  #will automagically terminate the interface upon exit
  with Robot(interface, motorParams, motors, robotWheelRadius,
             robotWheelDistance, 0, start_point[0], start_point[1], 0, numberOfParticles) as robot:
    for_ in range(8):
      robot.turn(math.pi/2)


if __name__ == "__main__":
  main()
