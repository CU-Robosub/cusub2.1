import serial
import sys

ser = serial.Serial('/dev/ttyACM0', 9600)
print("connected")
ser.close()
