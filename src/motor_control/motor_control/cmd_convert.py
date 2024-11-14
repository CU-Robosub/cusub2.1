import rclpy
from rclpy.node import Node
from .motorController import motorController # Class with motor control functions
from .PID_controller import PID
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import numpy as np

DEPTH_TOLERANCE = 0.1
# import testMC # importing this clears the motors for use
"""
AUTHOR: JAKE TUCKER
EMAIL: jakob.tucker@colorado.edu
PURPOSE: Subscribe to cmd_vel, publish value to motorController to send to servos, as well as find the depth level
"""
class cmd_convert(Node):

    def __init__(self):
        super().__init__('cmd_convert')
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.experimental_callback,
            10)
        self.last_msg_time = self.get_clock().now()
        self.pose_sub = self.create_subscription(
            Pose,
            'pose',
            self.pose_callback,
            10)
        self.goal_pose_sub = self.create_subscription(
            Pose,
            'goal_pose',
            self.goal_pose_callback,
            10)
        self.mc = motorController()
        self.pid = PID()
        self.current_pose = Pose()
        self.goal_pose = Pose()

    def goal_pose_callback(self, msg):
        self.goal_pose = msg

    def pose_callback(self, msg):
        self.current_pose = msg

    def listener_callback(self, msg): # test fxn for joy_node
        if(msg.linear.x > 0): # only send a command when vel is not 0
            channels = [0,1,2,7] # dummy channel list
            self.mc.run(channels,msg.linear.x, INVERT=False, raw_pwm=False)
        elif(msg.linear.x < 0):
            channels = [0,1,2,7] # dummy channel list
            self.mc.run(channels,msg.linear.x, INVERT=False, raw_pwm=False)
        if(msg.linear.y > 0): # only send a command when vel is not 0
            forward_channels = [7,1] # dummy channel list
            backward_channels = [2,0] # dummy channel list
            self.mc.run(forward_channels,msg.linear.y, INVERT=False)
            self.mc.run(backward_channels,msg.linear.y, INVERT=True)
        elif(msg.linear.y < 0): # only send a command when vel is not 0
            forward_channels = [2,0] # dummy channel list
            backward_channels = [7,1] # dummy channel list
            self.mc.run(forward_channels,msg.linear.y, INVERT=True)
            self.mc.run(backward_channels,msg.linear.y, INVERT=False)
        if(msg.linear.x == 0 and msg.linear.y == 0 and msg.angular.z == 0):
            channels = [0,1,2,7]
            self.mc.killAll(channels)
        if (self.current_pose.position.z > self.goal_pose.position.z + DEPTH_TOLERANCE or self.current_pose.position.z < self.goal_pose.position.z - DEPTH_TOLERANCE):
            channels = [3,4,5,6]
            self.mc.runDepthPID(channels, self.current_pose.position.z, self.goal_pose.position.z, self.pid)
        else:
            channels = [3,4,5,6]
            self.mc.killAll(channels)
        if(msg.angular.z > 0): # spin left?
            forward_channels = [7,2] # dummy channel list
            backward_channels = [0,1] # dummy channel list
            self.mc.run(forward_channels,msg.angular.z, INVERT=True)
            self.mc.run(backward_channels,msg.angular.z, INVERT=False)
        elif(msg.angular.z < 0): # spin right?
            forward_channels = [0,1] # dummy channel list
            backward_channels = [7,2] # dummy channel list
            self.mc.run(forward_channels,msg.angular.z, INVERT=False)
            self.mc.run(backward_channels,msg.angular.z, INVERT=True)
        
    
    def experimental_callback(self, msg):
        z_channels = [3,4,5,6]
        
        xmsg = msg.linear.x
        ymsg = msg.linear.y
        zmsg = msg.linear.z
        azmsg = msg.angular.z
        
        x_targetPWM = self.convert_to_PWM(xmsg)
        y_targetPWM = self.convert_to_PWM(ymsg)
        y_inv_targetPWM = self.convert_to_PWM(ymsg, invert=True)
        z_targetPWM = self.convert_to_PWM(zmsg, invert=True)
        az_targetPWM = self.convert_to_PWM(azmsg, invert=True)
        az_inv_targetPWM = self.convert_to_PWM(azmsg)
        
        motors = {0 : self.calculate_motor_PWM(np.array([x_targetPWM, y_inv_targetPWM, az_inv_targetPWM])),
                  1 : self.calculate_motor_PWM(np.array([x_targetPWM, y_targetPWM, az_inv_targetPWM])),
                  2 : self.calculate_motor_PWM(np.array([x_targetPWM, y_inv_targetPWM, az_targetPWM])),
                  7 : self.calculate_motor_PWM(np.array([x_targetPWM, y_targetPWM, az_targetPWM]))}
        
        for motor in motors:
            self.mc.run([motor], motors[motor], raw_pwm=True)
            # self.get_logger().info(f"Motor {motor} PWM: {motors[motor]}")
        self.mc.run(z_channels, z_targetPWM, raw_pwm=True)
        
        
    def convert_to_PWM(self, target, multiplier=30, invert=False):
        if invert:
            return round(1490 - (target * multiplier))
        return round(1490 + (target * multiplier))
    
    def calculate_motor_PWM(self, pwm_set):
        neutral = 1490
        return max(min(round(neutral + np.sum(pwm_set - neutral)), 1650), 1330)


def main(args=None):
    rclpy.init(args=args)

    convert = cmd_convert()

    rclpy.spin(convert)

    convert.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
