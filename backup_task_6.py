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
usMotor = 2
touchports = [0,1]
robotWheelRadius = 2.0 + 0.1 + carpet*0.0
robotWheelDistance = 13.5 + 1.38 + carpet*0.8

motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 7#6.0
motorParams.maxRotationSpeed = 15#12.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 18.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255
motorParams.pidParameters.k_p = 300
motorParams.pidParameters.k_i = 650
motorParams.pidParameters.k_d = 5

usParams = interface.MotorAngleControllerParameters()
usParams.maxRotationAcceleration = 7.0
usParams.maxRotationSpeed = 15.0
usParams.feedForwardGain = 255/20.0
usParams.minPWM = 18.0
usParams.pidParameters.minOutput = -255
usParams.pidParameters.maxOutput = 255
usParams.pidParameters.k_p = 100
usParams.pidParameters.k_i = 250
usParams.pidParameters.k_d = 2

numberOfParticles=100

def drawParticles(ps):
  dp.canvas.drawParticles([p.toTuple() for p in ps])

def main():
  start_point = (84, 30, 0)
  #points = [(84, 68, 0), (15, 68, math.pi/2), (84, 90, math.pi/4), (158, 90, math.pi/2), start_point]
  #points = [(84, 48, math.pi/2, False), (74, 48, math.pi/2, False), (20, 48, math.pi/2, True), (84, 90, 0, False), (148, 90, math.pi/2, True), (122, 74, -math.pi/2, False), (122, 20, 0, True)]
  #points = [(69, 48, math.pi/2, False), (20, 48, math.pi/2, True), (84, 90, 0, False), (148, 90, math.pi/2, True), (122, 74, -math.pi/2, False), (122, 20, 0, True)]
  #points = [(64, 48, math.pi, False), (64, 148, math.pi, True), (64, 90, 0, False), (148, 90, math.pi/2, True), (122, 74, -math.pi/2, False), (122, 20, 0, True)]
  points = [(64, 148, math.pi, True), (64, 90, 0, False), (104, 90, 0, False), (104, 190, 0, True), (102, 64, 0, False), (102, 20, 0, True)]

  #will automagically terminate the interface upon exit
  with Robot(interface, motorParams, motors, robotWheelRadius,
             robotWheelDistance, 0, start_point[0], start_point[1], 0, numberOfParticles, usMotor=usMotor, usParams=usParams, touchports=touchports) as robot:
    try:
      robot.startArc(64, 10)
      print 'Start arc finished'
      for x, y, theta, isArea in points:
        robot.moveTo(x, y, usTheta=theta, isArea=isArea)
      print 'Return sequence'
      robot.returnSequence()
    except KeyboardInterrupt:
      pass

if __name__ == "__main__":
  main()
