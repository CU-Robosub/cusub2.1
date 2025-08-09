import math
import threading
import yaml
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from .motorController import motorController # Class with motor control functions
from .PID_controller import PID
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Joy
from custom_interfaces.srv import SetPIDValues
from custom_interfaces.msg import ThrusterValues
from std_msgs.msg import Bool
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

    def __init__(self):
        super().__init__('cmd_convert')

        self.is_sim = self.declare_parameter("is_simulation", False).value

        if(self.is_sim):
            self.get_logger().info('running in simulation')

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.experimental_callback,
            10)
        self.last_msg_time = self.get_clock().now()
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'pose',
            self.pose_callback,
            10)
        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_pose_callback,
            10)

        self.kill_motors_sub = self.create_subscription(Bool, '/shutdown_motors', self.shutdown_callback, 10)
        
        
        # publish the values we send to the motor controller
        self.motor_pub = self.create_publisher(
            ThrusterValues,
            'cmd_thruster_values',
            10)        
        
        # Services for changing PID constants on the fly
        self.change_pid_depth_values = self.create_service(SetPIDValues, "set_pid_depth_values", lambda request, response : self.change_pid_values_callback(request, response, self.pid_depth))
        self.change_pid_roll_values = self.create_service(SetPIDValues, "set_pid_roll_values", lambda request, response : self.change_pid_values_callback(request, response, self.pid_roll))
        self.change_pid_pitch_values = self.create_service(SetPIDValues, "set_pid_pitch_values", lambda request, response : self.change_pid_values_callback(request, response, self.pid_pitch))
        
        self.mc = motorController()
        self.pid_depth = PID(10.0, 0.0, 0.0)
        self.current_pose = PoseStamped()
        self.goal_pose = PoseStamped()

        self.last_cmd_vel = Twist()
        self.last_cmd_vel.linear.x = 0.0
        self.last_cmd_vel.linear.y = 0.0
        self.last_cmd_vel.linear.z = 0.0
        self.last_cmd_vel.angular.x = 0.0
        self.last_cmd_vel.angular.y = 0.0
        self.last_cmd_vel.angular.z = 0.0
        self.cmd_lock = threading.Lock()

        self.control_timer = self.create_timer(0.05, self.calculate_motor_inputs)  # 20 Hz
        
        # clear motors
        # self.mc.clearMotors() # TODO This call throws an error
        

        self.pid_pitch = PID(100.0, 0.0, 0.0) # pitch  angular y

        self.pid_roll = PID(0.0, 0.0, 0.0)  # I dn't think the motors have the ability to affect this
        
        # initialize motor_values
        self.motor_values = [0.0 for _ in range(8)]

    def shutdown_callback(self, msg):
        if msg.data:
            self.get_logger().warn("Shutdown signal received. Shutting down node.")
            rclpy.shutdown()

    def goal_pose_callback(self, msg):
        self.goal_pose = msg

    def pose_callback(self, msg):
        self.current_pose = msg
        # self.stability_loop() # keep sub level <- super broken rn
    
    def change_pid_values_callback(self, request, response, pid):
        if(request.kp != -1):
            pid.setKP(request.kp)
        if(request.kd != -1):
            pid.setKD(request.kd)
        if(request.ki != -1):
            pid.setKI(request.ki)

        response.success = True
        return response

    def stability_loop(self) -> dict[int, float]:
        """Returns motor corrections for pitch and roll stabilization."""
        # self.get_logger().info("Stability loop executing")
        euler = quaternion_to_euler(self.current_pose.pose.orientation)

        roll_output = self.pid_roll.calculateOutput(euler[0], 0)
        pitch_output = self.pid_pitch.calculateOutput(euler[1], 0)

        # Access z from position of the pose inside PoseStamped
        depth_output = self.pid_depth.calculateOutput(
            self.current_pose.pose.position.z,
            self.goal_pose.pose.position.z
        )  # experimental

        fl_output = roll_output + pitch_output + depth_output
        fr_output = -roll_output + pitch_output + depth_output
        bl_output = roll_output - pitch_output + depth_output
        br_output = -roll_output - pitch_output + depth_output

        [fl_output, fr_output, bl_output, br_output] = normalize_motor_outputs(
            [fl_output, fr_output, bl_output, br_output], 1
        )
        
        # self.get_logger().info("Stability loop executed")

        return {
            CHANNEL_V_FL: fl_output,
            CHANNEL_V_FR: fr_output,
            CHANNEL_V_BL: bl_output,
            CHANNEL_V_BR: br_output
        }





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

    def pwm_to_percent(self, pwm, neutral=1490, scale=30):
        return (pwm - neutral) / scale

    def experimental_callback(self, msg):
        with self.cmd_lock:
            self.last_cmd_vel = msg
    
    def calculate_motor_inputs(self):
        try:
            # self.get_logger().info("Timer triggered: calculate_motor_inputs() running")

            # Always use last_cmd_vel (initialized to zero in __init__)
            xmsg = self.last_cmd_vel.linear.x
            ymsg = self.last_cmd_vel.linear.y
            zmsg = self.last_cmd_vel.linear.z
            azmsg = self.last_cmd_vel.angular.z

            # self.get_logger().info(f"Current base cmd_vel: {self.last_cmd_vel}")

            # Convert commands to PWM
            x_targetPWM = self.convert_to_PWM(xmsg)
            y_targetPWM = self.convert_to_PWM(ymsg)
            y_inv_targetPWM = self.convert_to_PWM(ymsg, invert=True)
            z_targetPWM = self.convert_to_PWM(zmsg, invert=True)
            az_targetPWM = self.convert_to_PWM(azmsg, invert=True)
            az_inv_targetPWM = self.convert_to_PWM(azmsg)

            motors = {
                CHANNEL_BL: self.calculate_motor_PWM(np.array([x_targetPWM, y_inv_targetPWM, az_inv_targetPWM])),
                CHANNEL_BR: self.calculate_motor_PWM(np.array([x_targetPWM, y_targetPWM, az_inv_targetPWM])),
                CHANNEL_FL: self.calculate_motor_PWM(np.array([x_targetPWM, y_inv_targetPWM, az_targetPWM])),
                CHANNEL_FR: self.calculate_motor_PWM(np.array([x_targetPWM, y_targetPWM, az_targetPWM]))
            }

            for motor, pwm in motors.items():
                self.mc.run([motor], pwm, raw_pwm=True)
                self.motor_values[motor] = pwm

            # Stabilization corrections
            stability_outputs = self.stability_loop()
            self.get_logger().info(f"Stability outputs: {stability_outputs}")

            base_z_pwm = self.convert_to_PWM(zmsg, invert=True)

            for motor in [CHANNEL_V_FL, CHANNEL_V_FR, CHANNEL_V_BL, CHANNEL_V_BR]:
                correction = stability_outputs.get(motor, 0.0)
                correction_pwm = int(correction * 30)
                combined_pwm = max(min(base_z_pwm + correction_pwm, 1650), 1330)
                self.mc.run([motor], combined_pwm, raw_pwm=True)
                self.motor_values[motor] = combined_pwm

            self.publish_motor_values()
            # self.get_logger().info("Motor values published")

        except Exception as e:
            import traceback
            self.get_logger().error(f"Exception in calculate_motor_inputs: {e}")
            self.get_logger().error(traceback.format_exc())




        
        
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