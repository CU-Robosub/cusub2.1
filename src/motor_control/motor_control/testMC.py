"""
    AUTHOR: XAVIER OKEEFE
    CONTACT: xaok7569@colorado.edu
    PURPOSE: Test code to test motor controller
"""
from motorController import motorController
from submodules.Maestro import maestro
import time

# All of this servo code is needed to clear maestro errors
servo = maestro.Controller()
servo.setSpeed(0,1900)     #set speed of servo 1
x = servo.getPosition(1) #get the current position of servo 1
servo.sendCmd(chr(0x21))
time.sleep(1)
x = servo.usb.read()
print(x.hex())
servo.close()


# Code for controlling motors
channels = {9} # Channels to command
mc = motorController()
mc.run(channels,1200,1) # run motors at set speed for set time (seconds)

motors = {0,1,2,3,4,5,6,7,8}
mc.run(motors, 1490 * 4,1)

