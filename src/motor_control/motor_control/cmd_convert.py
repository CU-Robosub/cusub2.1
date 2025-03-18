import math

import yaml
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from .motorController import motorController # Class with motor control functions
from .PID_controller import PID
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from sensor_msgs.msg import Joy
from custom_interfaces.srv import SetPIDValues
from custom_interfaces.msg import ThrusterValues
import numpy as np

DEPTH_TOLERANCE = 0.1

with open('src/cfg/sub_properties.yaml') as f:
    file = yaml.safe_load(f)
    CHANNEL_FL = file['front_left_id']
    CHANNEL_FR = file['front_right_id']
    CHANNEL_BL = file['back_left_id']
    CHANNEL_BR = file['back_right_id']
    CHANNEL_V_FL = file['vertical_front_left_id']
    CHANNEL_V_FR = file['vertical_front_right_id']
    CHANNEL_V_BL = file['vertical_back_left_id']
    CHANNEL_V_BR = file['vertical_back_right_id']
    H_THRUSTER_R = file['h_thruster_radius']
    H_THRUSTER_A = file['h_thruster_angle']
    H_THRUSTER_TA = file['h_thruster_torque_angle']

# import testMC # importing this clears the motors for use
"""
AUTHOR: JAKE TUCKER
EMAIL: jakob.tucker@colorado.edu
PURPOSE: Subscribe to cmd_vel, publish value to motorController to send to servos, as well as find the depth level
"""

def quaternion_to_euler(quat: Quaternion) -> tuple[float, float, float]:
    # Convert quaternion to Euler angles
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw

def normalize_motor_outputs(outputs: tuple[float, float, float, float], max_magnitude: float) -> tuple[float, float, float, float]:
    max_output = max([abs(x) for x in outputs])
    
    if max_output > max_magnitude:
        # Scale the outputs so the maximum output has a magnitude of max_magnitude
        ratio = max_magnitude/max_output
        outputs = [x * ratio for x in outputs]
    
    return outputs

class cmd_convert(Node):

    # type hinting for member variables
    motor_pub: Publisher
    motor_values: list[float]

    force_pubs: list[Publisher] | None

    cmd_vel: Twist
    velocity: Twist

    def __init__(self):
        super().__init__('cmd_convert')

        self.is_sim = self.declare_parameter("is_simulation", False).value

        if(self.is_sim):
            self.get_logger().info('running in simulation')

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
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
        
        self.vel_sub = self.create_subscription(
            Twist,
            'velocity',
            self.velocity_callback,
            10)
        
        # publish the values we send to the motor controller
        self.motor_pub = self.create_publisher(
            ThrusterValues,
            'cmd_thruster_values',
            10)        
        
        # Services for changing PID constants on the fly
        self.change_pid_depth_values = self.create_service(SetPIDValues, "set_pid_depth_values", lambda request, response : self.change_pid_values_callback(request, response, self.pid_depth))
        self.change_pid_roll_values = self.create_service(SetPIDValues, "set_pid_roll_values", lambda request, response : self.change_pid_values_callback(request, response, self.pid_roll))
        self.change_pid_pitch_values = self.create_service(SetPIDValues, "set_pid_pitch_values", lambda request, response : self.change_pid_values_callback(request, response, self.pid_pitch))
        self.change_pid_vel_x_values = self.create_service(SetPIDValues, "set_pid_vel_x_values", lambda request, response : self.change_pid_values_callback(request, response, self.pid_vel_x))
        self.change_pid_vel_y_values = self.create_service(SetPIDValues, "set_pid_vel_y_values", lambda request, response : self.change_pid_values_callback(request, response, self.pid_vel_y))

        self.mc = motorController()
        self.current_pose = Pose()

        self.cmd_vel = Twist()
        self.velocity = Twist()
        
        self.goal_pose = Pose()
        self.goal_pose.position = Point(x=0.0, y=0.0, z=-3.0)
        self.goal_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # clear motors
        # self.mc.clearMotors() # TODO This call throws an error
        
        self.pid_depth = PID(1, 0, 0)
        self.pid_pitch = PID(1, 0, 0) # pitch  angular y
        self.pid_roll = PID(1, 0, 0)  # roll   angular x
        self.pid_vel_x = PID(1, 0, 0) # x velocity
        self.pid_vel_y = PID(1, 0, 0) # y velocity
        
        # initialize motor_values
        self.motor_values = [0.0 for _ in range(8)]

    def goal_pose_callback(self, msg):
        self.goal_pose = msg

    def pose_callback(self, msg):
        self.current_pose = msg
        self.control_loop()
    
    def change_pid_values_callback(self, request, response, pid):
        if(request.kp != -1):
            pid.setKP(request.kp)
        if(request.kd != -1):
            pid.setKD(request.kd)
        if(request.ki != -1):
            pid.setKI(request.ki)

        response.success = True
        return response
    
    def velocity_callback(self, msg):
        self.velocity = msg

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
            self.mc.runDepthPID(channels, self.current_pose.position.z, self.goal_pose.position.z, self.pid_depth)
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
    
    def control_loop(self):
        outputs = [0.0 for _ in range(8)]

        self.stability_loop(outputs)
        self.depth_loop(outputs)

        normalize_motor_outputs(outputs, 1)

        self.velocity_loop(outputs)

        for i, output in enumerate(outputs):
            self.run_motor(i, output)

    def stability_loop(self, outputs: list[int]):

        euler = quaternion_to_euler(self.current_pose.orientation)
        
        # calculate outputs
        roll_output = self.pid_roll.calculateOutput(euler[0], 0)
        pitch_output = self.pid_pitch.calculateOutput(euler[1], 0)

        contributions = [0.0 for _ in range(8)]

        # +roll_output  -> +left motors  -right motors -> increased roll
        # +pitch_output -> +front motors -back motors  -> increased pitch
        contributions[CHANNEL_V_FL] += roll_output + pitch_output
        contributions[CHANNEL_V_FR] += -roll_output + pitch_output
        contributions[CHANNEL_V_BL] += roll_output - pitch_output
        contributions[CHANNEL_V_BR] += -roll_output - pitch_output

        normalize_motor_outputs(contributions, 1)

        for i in range(8):
            outputs[i] += contributions[i]

    
    def depth_loop(self, outputs: list[int]):

        current_depth = self.current_pose.position.z
        goal_depth = self.goal_pose.position.z

        # calculate output, invert
        depth_output = max(min(-self.pid_depth.calculateOutput(current_depth, goal_depth), 1), -1)

        # add outputs
        outputs[CHANNEL_V_FL] += depth_output
        outputs[CHANNEL_V_FR] += depth_output
        outputs[CHANNEL_V_BL] += depth_output
        outputs[CHANNEL_V_BR] += depth_output


    def velocity_loop(self, outputs: list[int]):
        # calculate output
        x_vel = self.velocity.linear.x
        y_vel = self.velocity.linear.y

        x_vel_output = self.pid_vel_x.calculateOutput(x_vel, self.cmd_vel.linear.x)
        y_vel_output = self.pid_vel_y.calculateOutput(y_vel, self.cmd_vel.linear.y)

        force = Twist()
        force.linear.x = x_vel_output
        force.linear.y = y_vel_output

        forces = self.calc_forces(force)

        contributions = [0.0 for _ in range(8)]

        # add outputs
        contributions[CHANNEL_FL] += forces[0]
        contributions[CHANNEL_FR] += forces[1]
        contributions[CHANNEL_BL] += forces[2]
        contributions[CHANNEL_BR] += forces[3]

        normalize_motor_outputs(contributions, 1)

        for i in range(8):
            outputs[i] += contributions[i]


    def calc_forces(self, cmd_force: Twist):
        
        x = cmd_force.linear.x
        y = cmd_force.linear.y

        force_1_4 = 0 # summed force of thrusters 1 and 4 (FL and BR)
        force_2_3 = 0 # summed force of thrusters 2 and 3 (FR and BL)

        # handle divide by zero case
        if x == 0:
            force_1_4 = 1 if y > 0 else -1
            force_2_3 = 1 if y < 0 else -1
        else:

            r = y / x # simply the tan of the force vector's angle
            if r == 0: # Only x movement
                force_1_4 = 1 if x > 0 else -1
                force_2_3 = 1 if x > 0 else -1
            elif r < 0: # Force vector in 2nd or 4th quadrants
                force_2_3 = 1 if x > 0 else -1 # constrain 2 & 3 to +-1
                force_1_4 = ( (-force_2_3 * math.tan(H_THRUSTER_A) - force_2_3 * r) / (r - math.tan(H_THRUSTER_A)) )
            else: # Force vector in 1st or 3rd quadrants
                force_1_4 = 1 if y > 0 else -1 # constrain 1 & 4 to +- 1
                force_2_3 = ( (force_1_4 * math.tan(H_THRUSTER_A) - force_1_4 * r) / (r + math.tan(H_THRUSTER_A)) )

        # Determine magnitude of calculated force, and scale to match the desired force magnitude
        result_mag = math.sqrt((math.cos(H_THRUSTER_A) * (force_1_4 + force_2_3)) ** 2 + (math.sin(H_THRUSTER_A) * (force_1_4 - force_2_3)) ** 2)
        scale = math.sqrt(x**2 + y**2) / result_mag

        force_1_4 *= scale
        force_2_3 *= scale

        # Calculate forces of each pair to produce half the desired torque each
        az = cmd_force.angular.z
        
        force_1: float = (force_1_4 + ((az / (2 * H_THRUSTER_R)) / math.sin(H_THRUSTER_TA))) / 2
        force_4: float = force_1_4 - force_1

        force_2: float = (force_2_3 + ((az / (2 * H_THRUSTER_R)) / math.sin(-H_THRUSTER_TA))) / 2
        force_3: float = force_2_3 - force_2

        return (force_1, force_2, force_3, force_4)
    

    def run_motor(self, motor_channel: int, value: float):
        self.mc.run([motor_channel], value)
        self.motor_values[motor_channel] = float(value)
        self.publish_motor_values()


    def publish_motor_values(self):
        msg = ThrusterValues()

        msg.front_left = float(self.motor_values[CHANNEL_FL])
        msg.front_right = float(self.motor_values[CHANNEL_FR])
        msg.back_left = float(self.motor_values[CHANNEL_BL])
        msg.back_right = float(self.motor_values[CHANNEL_BR])
        msg.top_front_left = float(self.motor_values[CHANNEL_V_FL])
        msg.top_front_right = float(self.motor_values[CHANNEL_V_FR])
        msg.top_back_left = float(self.motor_values[CHANNEL_V_BR])
        msg.top_back_right = float(self.motor_values[CHANNEL_V_BL])

        self.motor_pub.publish(msg)


    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg
        self.get_logger().info(f"Received cmd_vel: {msg}")

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
