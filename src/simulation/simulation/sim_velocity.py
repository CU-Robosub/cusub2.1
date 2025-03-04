
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from geometry_msgs.msg import Twist, Pose, PoseArray

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
    
    def sim_pose_callback(self, msg: PoseArray):
        if(msg.poses):
            pose = msg.poses[0]

            # Transform from ENU to NED coordinate systems
            pose.position.y = -pose.position.y
            pose.position.z = -pose.position.z

            time_now = self.get_clock().now().nanoseconds
            time_diff = (time_now - self.prev_time) / 1e9

            # Calculate velocity if the time since last data is short enough
            if(0.5 > time_diff > 0):
                velocity = Twist()
                velocity.linear.x = (pose.position.x - self.prev_pose.position.x) / time_diff
                velocity.linear.y = (pose.position.y - self.prev_pose.position.y) / time_diff
                velocity.linear.z = (pose.position.z - self.prev_pose.position.z) / time_diff

                self.velocity_pub.publish(velocity)

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
