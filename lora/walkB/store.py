from datetime import datetime

NOME_FILE = "loraWalk_Gep.txt"

import serial
ser = serial.Serial('/dev/ttyACM0', 9600)

while ser.inWaiting()!=0:
    trash = ser.readline()

while(True):

    while ser.inWaiting()!=0:

        incoming = ser.readline().decode("utf-8")
        print(incoming)

        parsed = str(incoming).split(",")

        time = datetime.now().strftime("%H:%M:%S")
        if len(parsed)>=4:
	      data = parsed[0] +"," + parsed[1] +"," + parsed[2] +"," + parsed[3] + "," + time + "\n"

 	      with open(NOME_FILE, "a+") as f:

        	   f.write(data)
	# print(data)
