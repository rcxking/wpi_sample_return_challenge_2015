#!/usr/bin/python

# Open a serial port, presumably containing an xbee, and print serial number

from xbee_config import XBeeConfig as XBee
import sys

port = '/dev/ttyUSB0'
baud = 9600

if len(sys.argv) >= 2:
  port = sys.argv[1]

if len(sys.argv) >= 3:
  baud = sys.argv[2]

#print 'Connecting to:', port, 'at baud:', baud
xbee = XBee(port, baud)
assert xbee.isOpen()

#print 'Entering command mode'
if xbee.commandMode():
  # set defaults
  xbee.serialSend('ATRE\r')
  print xbee.getReply()
  # digital input on pin 1
  xbee.serialSend('ATD03\r')
  print xbee.getReply()
  # digital output enable
  xbee.serialSend('ATIU1\r')
  print xbee.getReply()
  # 1 sample till transmit
  xbee.serialSend('ATIT1\r')
  print xbee.getReply()
  # force sampling
  xbee.serialSend('ATIS\x01\r')
  print xbee.getReply()
  # sample rate
  xbee.serialSend('ATIR\x01\x00\r')
  print xbee.getReply()
  # change detect on 1
  xbee.serialSend('ATIC\x01\r')
  print xbee.getReply()
  # write
  xbee.serialSend('ATWR\r')
  print xbee.getReply()
  # disconnect
  xbee.serialSend('ATCN\r')
  xbee.close()
else:
  print 'Could not enter command mode'

