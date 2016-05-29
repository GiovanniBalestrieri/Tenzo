#!/usr/bin/env python
import serial
import time
ser = serial.Serial('/dev/rfcomm0', 9600, timeout=1)

while True:
    ser.write("a")
    if (ser.inWaiting()>0): 
    	string = ser.readline()
	line = string.split(",") 
    	#linelen = len(line)
	print 'Receiving:  (a,b,c) ' ,line[1],',',line[2],',',line[3]
	time.sleep(0.1)
