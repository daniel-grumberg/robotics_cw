
#!/usr/bin/env python

from BrickPi import *   #import BrickPi.py file to use BrickPi operations

BrickPiSetup()  # setup the serial port for communication

Color_Sensor_Port = PORT_4										# Setup the sensor on Port 1.

while True:
  BrickPi.SensorType[Color_Sensor_Port] = TYPE_SENSOR_COLOR_GREEN   #Set the type of sensor, set it to function as a Red LED.
  BrickPiSetupSensors()   #Send the properties of sensors to BrickPi
  BrickPi.SensorType[Color_Sensor_Port] = TYPE_SENSOR_COLOR_BLUE   #Set the type of sensor, set it to function as a Red LED.
  BrickPiSetupSensors()   #Send the properties of sensors to BrickPi
  BrickPi.SensorType[Color_Sensor_Port] = TYPE_SENSOR_COLOR_RED   #Set the type of sensor, set it to function as a Red LED.
  BrickPiSetupSensors()   #Send the properties of sensors to BrickPi
  BrickPi.SensorType[Color_Sensor_Port] = TYPE_SENSOR_COLOR_FULL   #Set the type of sensor, set it to function as a Red LED.
  BrickPiSetupSensors()   #Send the properties of sensors to BrickPi
  time.sleep(0.5)
