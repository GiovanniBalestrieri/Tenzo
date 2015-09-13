import serial
import time

serial = serial.Serial("/dev/ttyUSB0", baudrate=2400)
while True:
    if serial.inWaiting() > 0:
        read_result = serial.read(12)
        print("Read card {0}" . format(read_result))
        print("Sleeping 2 seconds")
        time.sleep(2)
        serial.flushInput() # ignore errors, no data
