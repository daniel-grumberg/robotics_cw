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

#will automagically terminate the interface upon exit
  with Robot(interface, motorParams, motors, robotWheelRadius,
      robotWheelDistance, 0) as robot:

    while(1):

      print(robot.state)
      #print(states[0])


      x_coord = input("Enter x coord (m): ")*100
      y_coord = input("Enter y coord (m): ")*100

      robot_motion = robot.moveTo(x_coord, y_coord)

      x_cum = 0
      y_cum = 0
      rot_cumx = 0
      rot_cumy = 0

      for i in range(100):
        states[i].rotationUpdate(robot_motion[0], getRandom(g))
        states[i].motionUpdate(robot_motion[1], getRandom(e), getRandom(f))
        x_cum += states[i].x
        y_cum += states[i].y
        rot_cumx += math.cos(states[i].rot)
        rot_cumy += math.sin(states[i].rot)

      rot_avg = math.atan2(rot_cumy/100, rot_cumx/100)

      robot.state.updateState(x_cum/100, y_cum/100, rot_avg)

if __name__ == "__main__":
  main()
