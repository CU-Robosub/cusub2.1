import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool
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

    def cmd_force_callback(self, cmd_force):
        (force_1, force_2, force_3, force_4) = self.calc_paired_forces(cmd_force)

        
        
    def calc_forces(cmd_force):
        
        a_x = cmd_force.linear.x
        a_y = cmd_force.linear.y

        force_1_4 = 0 # summed force of thrusters 1 and 4 (FL and BR)
        force_2_3 = 0 # summed force of thrusters 2 and 3 (FR and BL)

        # handle divide by zero case
        if a_x == 0:
            force_1_4 = 1 if a_y > 0 else -1
            force_2_3 = 1 if a_y < 0 else -1
        else:

            r = a_y / a_x # simply the tan of the force vector's angle

            if r < 0: # Force vector in 2nd or 4th quadrants
                force_2_3 = 1 if a_x else -1 # constrain 2 & 3 to +-1
                force_1_4 = ( (-force_2_3 * math.tan(H_THRUSTER_A) - force_2_3 * r) / (r - math.tan(H_THRUSTER_A)) )
            else: # Force vector in 1st or 3rd quadrants
                force_1_4 = 1 if a_y > 0 else -1 # constrain 1 & 4 to +- 1
                force_2_3 = ( (force_1_4 * math.tan(H_THRUSTER_A) - force_1_4 * r) / (r + math.tan(H_THRUSTER_A)) )

        # Determine magnitude of calculated force, and scale to match the desired force magnitude
        result_mag = math.sqrt((math.cos(H_THRUSTER_A * (force_1_4 + force_2_3))) ** 2 + (math.sin(H_THRUSTER_A * (force_1_4 - force_2_3))) ** 2)
        scale = 2 * math.sqrt(a_x**2 + a_y**2) / result_mag

        force_1_4 *= scale
        force_2_3 *= scale

        # Calculate forces of each pair to produce half the desired torque each
        a_w = cmd_force.angular.z
        
        force_1 = (force_1_4 + (a_w / math.sin(H_THRUSTER_TA))) / 2
        force_4 = force_1_4 - force_1

        force_2 = (force_2_3 + (a_w / math.sin(-H_THRUSTER_TA))) / 2
        force_3 = force_2_3 - force_2

        return (force_1, force_2, force_3, force_4)

def main(args=None):
    rclpy.init(args=args)
    holonomic_control = HolonomicControl()
    rclpy.spin(holonomic_control)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        