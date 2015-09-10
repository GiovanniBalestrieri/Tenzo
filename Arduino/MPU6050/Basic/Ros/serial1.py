#!/usr/bin/env python
import serial
import sys
import time
from threading import Thread

portCOM = '/dev/ttyACM6'

def receiving(ser):
    global last_received

    buffer_string = ''
    while True:
        buffer_string = buffer_string + ser.read(ser.inWaiting())
        if '\n' in buffer_string:
            lines = buffer_string.split('\n') 
	    # Guaranteed to have at least 2 entries
            last_received = lines[-2]
            #If the Arduino sends lots of empty lines, you'll lose the
            #last filled line, so you could make statement conditional
            #like so: if lines[-2]: last_received = lines[-2]
            buffer_string = lines[-1]

def receiving1(ser):
        global last_received
        buffer = ''

        while True:
                # last_received = ser.readline()
                buffer += ser.read(ser.inWaiting())
                if '\n' in buffer:
                        last_received = buffer.split('\n')[-2:]

if __name__ ==  '__main__':
    ser = serial.Serial(
        port=portCOM,
        baudrate=115200,
        timeout=0.1,
        xonxoff=0,
        rtscts=0,
        interCharTimeout=None
    )

    Thread(target=receiving, args=(ser,)).start()
