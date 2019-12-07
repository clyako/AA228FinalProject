import serial
import time

port = serial.Serial('/dev/ttyACM1', 115200, timeout = 1)
time.sleep(1)
print("serial init")

def grasped():
	sendVal = True
	while True:
		if sendVal:
			val = "9" # don't make this a 1 or 0
			port.write(str.encode(val))
			time.sleep(1)
			sendVal = False
		else:
			status = port.read().decode("ascii")
			if (status == "0" or status == "1"):
				if status == "0":
					# print("Fail!")
					return False
				if status == "1":
					# print("Success!")
					return True

def serialClose():
	port.close()