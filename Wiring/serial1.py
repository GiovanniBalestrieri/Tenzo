#!/usr/bin/python

import serial 

ser = serial.Serial('/dev/tty0', 57600)

while True:
    try:
        response = ser.readline()
        print response

    except KeyboardInterrupt:
        break

ser.close()
