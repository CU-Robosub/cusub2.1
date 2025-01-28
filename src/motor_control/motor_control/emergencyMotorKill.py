"""
    AUTHOR: JAKE TUCKER
    CONTACT: jatu9146@colorado.edu
    PURPOSE: Send neutral PWM to all motors
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
channels = {1,2,3,4,5,6,7} # Channels to command
mc = motorController()
mc.run(channels,1490, raw_pwm=True) # run motors at set speed for set time (seconds)

light = {9}
mc.run(light, 1100/4, raw_pwm=True)
print("Killed Motors")
