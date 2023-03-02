"""
    AUTHOR: XAVIER O'KEEFE
    CONTACT: xaok7569@colorado.edu
    PURPOSE: Create class for basic motor control functions
"""
import serial
import time
import sys
from Maestro import maestro

class motorController:
    
    def __init__(self):
        # initialize serial port , set baud rate, set timeout
        self.serial = serial.Serial('/dev/ttyACM0', 9600, timeout=1) 
        

    def run(self, channels, target, duration=-1):
        targetPWM = round(4 * (1490 + 410*target)) # Multiply by 4 for Maestro
        targetPWM = targetPWM
        targetBytes = [(targetPWM & 0x7F), ((targetPWM >> 7) & 0x7F)]
        for channel in channels: # loop through channels
            finalCommand = [0x84, channel] + targetBytes # Send 4 byte command to maestro
            self.serial.write(bytearray(finalCommand))
        if duration != -1: # if duration parameter is passed
            time.sleep(duration)
            self.killAll(channels)

    def killAll(self, channels):
        target = 4*1490 # neutral target
        targetBytes = [(target & 0x7F), ((target >> 7) & 0x7F)]
        for channel in channels: # kill all channels
            finalCommand = [0x84, channel] + targetBytes
            self.serial.write(bytearray(finalCommand))

    def testFunc(val):
        finalVal = 1490 + val*410
        print("received val: ", finalVal)



 