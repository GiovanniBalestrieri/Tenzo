#!/usr/bin/env python 
import serial
import sys
import time

port = '/dev/ttyACM6'
portRfid = '/dev/ttyUSB0'

try:
        #ser = serial.Serial(port,115200)
	ser = serial.Serial('/dev/ttyUSB0',2400)
except:
	print "Unable to open serial port" + port       

while True:
	try:
		line = ser.readline()
	        print line
	except:
	        print "Unable to read line..."
                sys.exit(0)

