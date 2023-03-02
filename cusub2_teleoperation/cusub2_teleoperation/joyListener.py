"""
    AUTHOR: XAVIER O'KEEFE
    CONTACT: xaok7569@colorado.edu
    PURPOSE: Create subscriber for teleoperation
"""
import rclpy
from rclpy.node import Node
from motorController import motorController # Class with motor control functions
from Maestro import maestro
import time

from sensor_msgs.msg import Joy

class JoyListener(Node):

    def __init__(self):
        super().__init__('joyListener')
        self.subscription = self.create_subscription(
            Joy, # msg type
            '/joy', # topic to listen to
            self.listener_callback, #callback fxn
            10 # overflow queue
        )
        self.subscription

    def listener_callback(self, msg): # test fxn for joy_node
        mc = motorController()
        if(msg.axes[1] != 0): # trigger button
            channels = [0,1,2,3,4,5,6,7] # dummy channel list
            mc.run(channels,msg.axes[1])

def main(args=None):
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
    channels = {0} # Channels to command
    mc = motorController()
    mc.run(channels,1600,1) # run motors at set speed for set time (seconds)

    rclpy.init(args=args)

    jl = JoyListener()
    rclpy.spin(jl)



if __name__ == '__main__':
    main()
