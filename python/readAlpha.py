#!/usr/bin/env python
import serial
import time
import math
i = 0
while True:
    #ser.write("a")
    x = 37*math.cos(0.001*i - 0.1) 
    y = 180*math.cos(0.001*i - 0.1)
    z = 74*math.cos(0.002*i + 0.1) 
    print ' (Temp,Cond,Hz) ' ,round(x,2),',',round(y,2),',',round(z,2) 
    i = i+1
    time.sleep(0.1)
