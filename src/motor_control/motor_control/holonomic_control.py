import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool, Float64
import yaml
import math

H_THRUSTER_R = 0
H_THRUSTER_A = 0
H_THRUSTER_TA = 0

with open('src/cfg/sub_properties.yaml') as f:
    file = yaml.safe_load(f)
    H_THRUSTER_R = file['h_thruster_radius']
    H_THRUSTER_A = file['h_thruster_angle']
    H_THRUSTER_TA = file['h_thruster_torque_angle']

class HolonomicControl(Node):
    
    def __init__(self):
        super().__init__('holonomic_control')
        self.cmd_force_sub = self.create_subscription(Twist, 'cmd_force', self.cmd_force_callback, 10)
        
        self.FL_pub = self.create_publisher(Float64, 'thrusters/force_FL', 1)
        self.FR_pub = self.create_publisher(Float64, 'thrusters/force_FR', 1)
        self.BL_pub = self.create_publisher(Float64, 'thrusters/force_BL', 1)
        self.BR_pub = self.create_publisher(Float64, 'thrusters/force_BR', 1)

    def cmd_force_callback(self, cmd_force):
        (force_1, force_2, force_3, force_4) = self.calc_forces(cmd_force)

        msg_1 = Float64()
        msg_1.data = force_1
        self.FL_pub.publish(msg_1)

        msg_2 = Float64()
        msg_2.data = force_2
        self.FR_pub.publish(msg_2)

        msg_3 = Float64()
        msg_3.data = force_3
        self.BL_pub.publish(msg_3)

        msg_4 = Float64()
        msg_4.data = force_4
        self.BR_pub.publish(msg_4)
        
    def calc_forces(self, cmd_force):
        
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

            if r < 0: # Force vector in 2nd or 4th quadrants
                force_2_3 = 1 if x else -1 # constrain 2 & 3 to +-1
                force_1_4 = ( (-force_2_3 * math.tan(H_THRUSTER_A) - force_2_3 * r) / (r - math.tan(H_THRUSTER_A)) )
            else: # Force vector in 1st or 3rd quadrants
                force_1_4 = 1 if y > 0 else -1 # constrain 1 & 4 to +- 1
                force_2_3 = ( (force_1_4 * math.tan(H_THRUSTER_A) - force_1_4 * r) / (r + math.tan(H_THRUSTER_A)) )

        # Determine magnitude of calculated force, and scale to match the desired force magnitude
        result_mag = math.sqrt((math.cos(H_THRUSTER_A * (force_1_4 + force_2_3))) ** 2 + (math.sin(H_THRUSTER_A * (force_1_4 - force_2_3))) ** 2)
        scale = math.sqrt(x**2 + y**2) / result_mag

        force_1_4 *= scale
        force_2_3 *= scale

        # Calculate forces of each pair to produce half the desired torque each
        az = cmd_force.angular.z
        
        force_1 = (force_1_4 + ((az / 2) / math.sin(H_THRUSTER_TA))) / 2
        force_4 = force_1_4 - force_1

        force_2 = (force_2_3 + ((az / 2) / math.sin(-H_THRUSTER_TA))) / 2
        force_3 = force_2_3 - force_2

        return (force_1, force_2, force_3, force_4)

def main(args=None):
    rclpy.init(args=args)
    holonomic_control = HolonomicControl()
    rclpy.spin(holonomic_control)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        