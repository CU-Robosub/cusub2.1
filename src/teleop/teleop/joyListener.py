"""
    AUTHOR: XAVIER O'KEEFE
    CONTACT: xaok7569@colorado.edu
    PURPOSE: Create subscriber for teleoperation
"""
import rclpy
from rclpy.node import Node
import teleop.motorController # Class with motor control functions

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
    rclpy.init(args=args)

    jl = JoyListener()
    rclpy.spin(jl)



if __name__ == '__main__':
    main()
