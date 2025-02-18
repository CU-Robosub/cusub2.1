
import math
import yaml

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from std_msgs.msg import Float64
from custom_interfaces.msg import ThrusterValues

"""
AUTHOR: CURTIS PREEDOM
EMAIL: curtis.preedom@colorado.edu
PURPOSE: Converts motor values to thrust forces and publishes to the simulation
"""

# Read motor ids from config
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

class SimThruster(Node):

    motor_values_sub: Subscription

    force_FL_pub: Publisher
    force_FR_pub: Publisher
    force_BL_pub: Publisher
    force_BR_pub: Publisher
    force_TFL_pub: Publisher
    force_TFR_pub: Publisher
    force_TBL_pub: Publisher
    force_TBR_pub: Publisher

    def __init__(self):
        super().__init__("SimThruster")

        # subscribe to 'cmd_thruster_values' to get the motor values
        self.motor_values_sub = self.create_subscription(ThrusterValues, 'cmd_thruster_values', self.motor_values_callback, 10)

        # publishers for thrust force topics
        self.force_FL_pub = self.create_publisher(Float64, 'sim/force_FL', 1)
        self.force_FR_pub = self.create_publisher(Float64, 'sim/force_FR', 1)
        self.force_BL_pub = self.create_publisher(Float64, 'sim/force_BL', 1)
        self.force_BR_pub = self.create_publisher(Float64, 'sim/force_BR', 1)
        self.force_TFL_pub = self.create_publisher(Float64, 'sim/force_TFL', 1)
        self.force_TFR_pub = self.create_publisher(Float64, 'sim/force_TFR', 1)
        self.force_TBL_pub = self.create_publisher(Float64, 'sim/force_TBL', 1)
        self.force_TBR_pub = self.create_publisher(Float64, 'sim/force_TBR', 1)
    
    def motor_values_callback(self, msg: ThrusterValues):

        # Publish thrust forces to the simulation
        self.force_FL_pub.publish(Float64(data=self.calculate_thrust(msg.front_left)))
        self.force_FR_pub.publish(Float64(data=self.calculate_thrust(msg.front_right)))
        self.force_BL_pub.publish(Float64(data=self.calculate_thrust(msg.back_left)))
        self.force_BR_pub.publish(Float64(data=self.calculate_thrust(msg.back_right)))
        self.force_TFL_pub.publish(Float64(data=self.calculate_thrust(msg.top_front_left)))
        self.force_TFR_pub.publish(Float64(data=self.calculate_thrust(msg.top_front_right)))
        self.force_TBL_pub.publish(Float64(data=self.calculate_thrust(msg.top_back_left)))
        self.force_TBR_pub.publish(Float64(data=self.calculate_thrust(msg.top_back_right)))
    
    def calculate_thrust(self, thruster_value):
        # TODO Better force calculation using data from BlueRobotics
        return math.copysign(3.5 * (thruster_value**2), thruster_value) # Force ~= sign(motor_percent) * 3.5 * motor_percent ^ 2

def main(args=None):
    rclpy.init(args=args)

    sim_thruster = SimThruster()

    rclpy.spin(sim_thruster)

    sim_thruster.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
