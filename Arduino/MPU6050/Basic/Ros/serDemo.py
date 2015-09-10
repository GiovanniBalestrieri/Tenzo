#!/usr/bin/env python
import serial
import sys
import time

port = '/dev/ttyACM6'

try:
        #ser = serial.Serial(port,115200);
	ser = serial.Serial('/dev/ttyACM6',115200)
except:
	print "Unable to open serial port" + port       

while True:
	try:
		line = ser.readline()
	        print line
	except:
	        print "Unable to read line..."
                sys.exit(0)

