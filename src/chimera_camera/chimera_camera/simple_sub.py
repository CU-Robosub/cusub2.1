import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class SimpleSub(Node):
    def __init__(self):
        super().__init__('simple_sub')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub = self.create_subscription(Image, '/camera_0/image_raw', self.callback, qos)
        self.create_timer(10.0, lambda: self.get_logger().info("Node alive"))

    def callback(self, msg):
        self.get_logger().info("Image received!")

def main():
    rclpy.init()
    node = SimpleSub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
