"""
    AUTHOR: JAKE TUCKER
    CONTACT: jatu9146@colorado.edu
    PURPOSE: Create subscriber for teleoperation
"""
import rclpy
from rclpy.node import Node
# from .submodules import motorController # Class with motor control functions
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool
import yaml
from .motorController import motorController

MAXVEL_X = 0
MAXVEL_Y = 0
MAXVEL_Z = 0
MAXVEL_AZ = 0
DEPTH_K = 0
DEPTH_GOAL_STEP = 0
DEPTH_TOLERANCE = 0
with open('src/cfg/sub_properties.yaml') as f:
    file = yaml.safe_load(f)
    MAXVEL_X = file['max_vel_x']
    MAXVEL_Y = file['max_vel_y']
    MAXVEL_Z = file['max_vel_z']
    MAXVEL_AZ = file['max_vel_az']
    DEPTH_GOAL_STEP = file['depth_goal_step']
    DEPTH_K = file['kp']
    DEPTH_TOLERANCE = file['depth_tolerance']

class JoyListener(Node):

    def __init__(self):
        super().__init__('joyListener')
        
        self.goalPose = Pose()
        self.currentpose = self.create_subscription(Pose, 'pose', self.current_pose_callback, 10)
        self.timer = self.create_timer(0.2, self.publish_goal)
        self.jlinear_x = 0
        self.jlinear_y = 0
        self.jlinear_z = 0
        self.jangular_z = 0
        self.slinear_x = 0
        self.slinear_y = 0
        self.slinear_z = 0
        self.sangular_z = 0
        self.mc = motorController()
        self.fmc_pressed = False
        self.fmc_val = 1
        self.light_pressed = False
        self.light_on = False
        self.setPoint = 0
        self.goal = Pose()
        self.currentPosition = Pose()
        self.zaxis = 0
        self.autonamous_status = False
        
        self.joy_sub = self.create_subscription(
            Joy, # msg type
            '/joy', # topic to listen to
            self.listener_callback, #callback fxn
            10 # overflow queue
        )
        self.cmd_sub = self.create_subscription(
            Twist, # msg type
            '/sys_cmd_vel', # topic to listen to
            self.sys_cmd_vel_callback, #callback fxn
            10 # overflow queue
        )
        self.goalPoseSub = self.create_subscription(
            Pose,
            '/goal_pose',
            self.goal_pose_callback,
            10)
        self.autonamousSub = self.create_subscription(
            Bool,
            '/autonamous_status',
            self.autonamous_status_callback,
            10)
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        self.goalPosePub = self.create_publisher(
            Pose,
            '/goal_pose',
            100 # made it 100 cuz we might want backlog
        )
        self.autonamous = self.create_publisher(
            Bool,
            '/autonamous_status',
            100 # made it 100 cuz we might want backlog
        )


    def publish_goal(self):
        goalPose = self.goalPose
        if self.zaxis != 0.0:
            goalPose.position.z = float(self.get_setpoint() + DEPTH_GOAL_STEP*self.zaxis/abs(self.zaxis))
            self.lastPose = float(self.get_setpoint()  + DEPTH_GOAL_STEP*self.zaxis/abs(self.zaxis))
        else:
            goalPose.position.z = float(self.setPoint)
        
        self.goalPosePub.publish(goalPose)
        real = self.currentPosition.position.z
        goal3 = self.goal.position.z
        if real+DEPTH_TOLERANCE < goal3  or goal3 < real - DEPTH_TOLERANCE:
            zCommand = -DEPTH_K* (self.currentPosition.position.z - self.goal.position.z)
            zCommand = max(-5.0, min(5.0, zCommand))
            self.jlinear_z = zCommand
            self.slinear_z = zCommand
        else:
            self.jlinear_z = 0.0
            self.slinear_z = 0.0
        

    def autonamous_status_callback(self, msg):
        self.autonamous_status = msg.data
    def controller_callback(self, msg):
        self.goalPose = msg
    def current_pose_callback(self, msg):
        self.currentPosition = msg
    # def listener_callback(self, msg): # test fxn for joy_node
    #     mc = motorController()
    #     if(msg.axes[1] != 0): # trigger button
    #         channels = [0,1,2,3,4,5,6,7] # dummy channel list
    #         mc.run(channels,msg.axes[1])

    # def publish_goal(self, msg):
    #     goalPose = self.goal()

    def listener_callback(self, msg):
        # Need to adjust values
        # implement logic to dermine max values to send
        x = msg.axes[1] # forward/backward
        y = msg.axes[0] # side to side
        z = msg.axes[5] * MAXVEL_Z # depth control (need point implementation)
        self.zaxis = msg.axes[5]
        az = msg.axes[2] # yah

        # goalPose = Pose()
        # goalPose.position.x = 0.0
        # goalPose.position.y = 0.0

        # tempZ = float(msg.axes[5])
        # if tempZ != 0.0:
        #     goalPose.position.z = float(self.get_setpoint() + 3*tempZ/abs(tempZ))
        #     self.lastPose = float(self.get_setpoint()  + 3*tempZ/abs(tempZ))
        # else:
        #     goalPose.position.z = float(self.setPoint)

        # goalPose.orientation.x = 0.0
        # goalPose.orientation.y = 0.0
        # goalPose.orientation.z = 0.0
        # goalPose.orientation.w = 0.0

        # self.goalPosePub.publish(goalPose)
        # k = 1
        # real = self.currentPosition.position.z
        # goal3 = self.goal.position.z
        # if real+10 < goal3  or goal3 < real - 10:
        #     zCommand = -k* (self.currentPosition.position.z - self.goal.position.z)
        #     zCommand = max(-5.0, min(5.0, zCommand))
        #     self.jlinear_z = zCommand
        #     self.slinear_z = zCommand
        # else:
        #     self.jlinear_z = 0.0
            # self.slinear_z = 0.0

        
        # proportion logic
        sum_ax = abs(x) + abs(y) + abs(az)
        if sum_ax < 1: sum_ax = 1
        self.jlinear_x = x / sum_ax * MAXVEL_X * self.fmc_val # forward/backward
        self.jlinear_y = y / sum_ax * MAXVEL_Y * self.fmc_val # side to side
        self.jlinear_z = z * self.fmc_val # depth control (need point implementation)
        self.jangular_z = az / sum_ax * MAXVEL_AZ # yaw
        
        self.mc.run([9],self.convert_to_PWM(msg.axes[3]), raw_pwm=True)
        if (not int(msg.buttons[0])):
            self.mc.run([8],1200, 1, raw_pwm=True)
        else:
            self.mc.run([8],1800, 1, raw_pwm=True)
        if(int(msg.buttons[1]) and not self.fmc_pressed):
            self.toggle_fmc()
        if(int(msg.buttons[2]) and not self.light_pressed):
            self.light_on = not self.light_on
            if self.light_on:
                self.get_logger().info(f"Light ON")
            else:
                self.get_logger().info(f"Light OFF")
            
        self.fmc_pressed = bool(msg.buttons[1])
        self.light_pressed = bool(msg.buttons[2])
        # self.get_logger().info(f"Right before publish cmd")
        self.publish_cmd()
        self.send_light_pwm()

    def toggle_fmc(self):
        if (self.fmc_val == 1):
            self.get_logger().info(f"Fine Motor Control ON")
            self.fmc_val = 0.66
        else:
            self.get_logger().info(f"Fine Motor Control OFF")
            self.fmc_val = 1
    
    def send_light_pwm(self):
        if (self.light_on):
            self.mc.run([10],1900)
        else:
            self.mc.run([10],1100)
    
    def convert_to_PWM(self, axis):
        return round(1500 + 500 * axis)

    def sys_cmd_vel_callback(self, msg):
        self.slinear_x = msg.linear.x
        self.slinear_y = msg.linear.y
        self.slinear_z = msg.linear.z
        self.sangular_z = msg.angular.z
        self.publish_cmd()

    def goal_pose_callback(self, msg):
        self.setPoint = msg.position.z
    
    def get_setpoint(self):
        return self.setPoint

    def publish_cmd(self):
        # Create Twist message
        twist_msg = Twist()
        
        # Publish jlinear data if available, otherwise use slinear data
        if self.jlinear_x is not None and self.jlinear_x != 0:
            twist_msg.linear.x = float(self.jlinear_x)
            self.autonamous_status = False
        else:
            twist_msg.linear.x = float(self.slinear_x)
            
        if self.jlinear_y is not None and self.jlinear_y != 0:
            twist_msg.linear.y = float(self.jlinear_y)
            self.autonamous_status = False
        else:
            twist_msg.linear.y = float(self.slinear_y)
            
        # if self.jlinear_z is not None and self.jlinear_z != 0:
        #     twist_msg.linear.z = float(self.jlinear_z)
        # else:
        twist_msg.linear.z = float(self.slinear_z)
            
        if self.jangular_z is not None and self.jangular_z != 0:
            twist_msg.angular.z = float(self.jangular_z)
            self.autonamous_status = False
        else:
            twist_msg.angular.z = float(self.sangular_z)
            
        self.publisher.publish(twist_msg)
        self.autonamous.publish(Bool(data=self.autonamous_status))

def main(args=None):
    rclpy.init(args=args)

    jl = JoyListener()
    rclpy.spin(jl)



if __name__ == '__main__':
    main()
