#!/usr/bin/python

import serial

PORT = "/dev/ttyUSB0"
BAUD = 9600
ser = serial.Serial(PORT, BAUD)

print(ser.write("129"))
print(ser.write("4"))
print(ser.write("60"))
print(ser.write())
