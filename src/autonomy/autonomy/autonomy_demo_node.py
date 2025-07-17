import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.duration import Duration


class AutonomyDemoNode(Node):
    def __init__(self):
        super().__init__('autonomy_demo_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

    def publish_for_duration(self, twist_msg, duration):
        start_time = self.get_clock().now()
        end_time = start_time + Duration(seconds=duration)

        while rclpy.ok() and self.get_clock().now() < end_time:
            self.publisher_.publish(twist_msg)
            rclpy.spin_once(self, timeout_sec=0.03)

    def run_sequence(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.3

        turn_cmd = Twist()
        turn_cmd.angular.z = 0.5
        turn_cmd.linear.x = 0.0
        turn_cmd.linear.y = 0.0
        turn_cmd.linear.z = 0.0


        stop_cmd = Twist()

        self.get_logger().info("Moving forward (1st) for 3 seconds")
        self.publish_for_duration(move_cmd, 3.0)

        self.get_logger().info("Turning for 15.5 seconds")
        self.publish_for_duration(turn_cmd, 15.5)

        self.get_logger().info("Moving forward (2nd) for 3 seconds")
        self.publish_for_duration(move_cmd, 3.0)

        self.get_logger().info("Stopping")
        self.publisher_.publish(stop_cmd)
        rclpy.spin_once(self, timeout_sec=0.1)  # Make sure stop command is sent


def main():
    rclpy.init()
    node = AutonomyDemoNode()
    node.run_sequence()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
