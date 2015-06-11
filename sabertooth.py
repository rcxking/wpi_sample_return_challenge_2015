#!/usr/bin/python

'''
sabertooth.py - This script provides a REPL to allow a user to send commands
to a Sabertooth 2x25 motor driver.

RPI Rock Raiders
6/10/15

Last Updated: Bryant Pong: 6/10/15 - 4:20 PM     
'''
import serial
ser = serial.Serial("/dev/ttyUSB0", 9600)

while True:
	nextCmd = str(raw_input("Please enter your next command.  Commands are in <addr> <cmd> <data>: ")).split()
	print("Your command (serialized) is: " + str(nextCmd))

	chksum = (int(nextCmd[0]) + int(nextCmd[1]) + int(nextCmd[2])) & 127
	
	print("Your command is: " + str(nextCmd[0]) + " " + str(nextCmd[1]) + \
	    " " + str(nextCmd[2]) + " " + str(chksum))	 

	ser.write(chr(int(nextCmd[0])))
	ser.write(chr(int(nextCmd[1])))
	ser.write(chr(int(nextCmd[2])))
	ser.write(chr(int(chksum)))
