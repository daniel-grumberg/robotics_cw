import brickpi
import time
import math
from robot_controller import Robot, RobotState

interface = brickpi.Interface()
interface.initialize()

carpet = 0

motors = [0, 1]
robotWheelRadius = 2.0 + 0.1 + carpet*0.3
robotWheelDistance = 13.5 + 1.38 + carpet*4.8

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


def main():
#will automagically terminate the interface upon exit
  with Robot(interface, motorParams, motors, robotWheelRadius,
      robotWheelDistance, 0) as robot:

    print(robot.state)
    robot.motion(10)

if __name__ == "__main__":
  main()
