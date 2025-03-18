import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from geometry_msgs.msg import Twist, Pose, PoseArray
import numpy as np

"""
AUTHOR: CURTIS PREEDOM
EMAIL: curtis.preedom@colorado.edu
PURPOSE: Acts a drop-in replacement of the dvl_data.DVL_subpub node for Gazebo simulation. Publishes to the 'velocity' topic.
"""

class SimVelocity(Node):

    sim_pose_sub: Subscription
    velocity_pub: Publisher

    prev_time: float
    prev_pose: Pose

    vel_filter: Twist = Twist()
    vel_filter_ratio: float = 0.5

    def __init__(self):
        super().__init__("SimVelocity")

        self.sim_pose_sub = self.create_subscription(
                PoseArray,
                'sim/pose',
                self.sim_pose_callback,
                10)

        self.velocity_pub = self.create_publisher(Twist, 'velocity', 10)
        self.prev_time = self.get_clock().now().nanoseconds
        self.prev_pose = Pose()
    
    def quaternion_to_rotation_matrix(self, q):
        """
        Convert a quaternion into a rotation matrix.
        """
        qx, qy, qz, qw = q
        return np.array([
            [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
        ])

    def sim_pose_callback(self, msg: PoseArray):
        if(msg.poses):
            pose = msg.poses[0]

            # Transform from ENU to NED coordinate systems
            pose.orientation.y = -pose.orientation.y
            pose.orientation.z = -pose.orientation.z

            pose.position.y = -pose.position.y
            pose.position.z = -pose.position.z


            time_now = self.get_clock().now().nanoseconds
            time_diff = (time_now - self.prev_time) / 1e9

            # Calculate velocity if the time since last data is short enough
            if(0.5 > time_diff > 0):
                # Calculate velocity in world frame
                vel_world = np.array([
                    (pose.position.x - self.prev_pose.position.x) / time_diff,
                    (pose.position.y - self.prev_pose.position.y) / time_diff,
                    (pose.position.z - self.prev_pose.position.z) / time_diff
                ])

                # Get the orientation quaternion
                orientation = pose.orientation
                quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]

                # Convert quaternion to rotation matrix
                rotation_matrix = self.quaternion_to_rotation_matrix(quaternion)

                # Transform velocity to vehicle frame
                vel_vehicle = np.dot(rotation_matrix.T, vel_world)

                # Apply low-pass filter
                self.vel_filter.linear.x = (self.vel_filter.linear.x * (1 - self.vel_filter_ratio)) + (vel_vehicle[0] * self.vel_filter_ratio)
                self.vel_filter.linear.y = (self.vel_filter.linear.y * (1 - self.vel_filter_ratio)) + (vel_vehicle[1] * self.vel_filter_ratio)
                self.vel_filter.linear.z = (self.vel_filter.linear.z * (1 - self.vel_filter_ratio)) + (vel_vehicle[2] * self.vel_filter_ratio)

                self.velocity_pub.publish(self.vel_filter)

            self.prev_time = time_now
            self.prev_pose = pose
        else:
            self.get_logger().warning("received an empty pose array from the simulation")

def main(args=None):
    rclpy.init(args=args)

    sim_velocity = SimVelocity()

    rclpy.spin(sim_velocity)

    sim_velocity.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
