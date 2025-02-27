
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from geometry_msgs.msg import Pose, PoseArray

"""
AUTHOR: CURTIS PREEDOM
EMAIL: curtis.preedom@colorado.edu
PURPOSE: Acts a drop-in replacement of the dvl_data.DVL_subpub node for Gazebo simulation. Publishes to the 'pose' topic.
"""

class SimPose(Node):

    sim_pose_sub: Subscription
    pose_pub: Publisher

    def __init__(self):
        super().__init__("SimPose")

        self.sim_pose_sub = self.create_subscription(
                PoseArray,
                'sim/pose',
                self.sim_pose_callback,
                10)

        self.pose_pub = self.create_publisher(Pose, 'pose', 10)
    
    def sim_pose_callback(self, msg: PoseArray):
        if(msg.poses):
            pose = msg.poses[0]

            # Transform from ENU to NED coordinate systems
            pose.orientation.y = -pose.orientation.y
            pose.orientation.z = -pose.orientation.z

            pose.position.y = -pose.position.y
            pose.position.z = -pose.position.z

            self.pose_pub.publish(pose)
        else:
            self.get_logger().warning("received an empty pose array from the simulation")

def main(args=None):
    rclpy.init(args=args)

    sim_pose = SimPose()

    rclpy.spin(sim_pose)

    sim_pose.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
