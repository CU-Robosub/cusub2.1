import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys
import threading
import time

class MultiCameraPublisher(Node):
    def __init__(self, camera_ports):
        super().__init__('multi_camera_publisher')
        self.camera_ports = camera_ports
        self.publishers = {}
        self.captures = {}
        self.bridge = CvBridge()
        self.running = True

        for port in self.camera_ports:
            cap = cv2.VideoCapture(port)
            if not cap.isOpened():
                self.get_logger().error(f'Failed to open camera port {port}')
                continue

            topic_name = f'/image_{port}'
            pub = self.create_publisher(Image, topic_name, 10)
            self.publishers[port] = pub
            self.captures[port] = cap
            self.get_logger().info(f'Camera {port} publishing to {topic_name}')

        # Start the thread
        self.thread = threading.Thread(target=self.publish_loop)
        self.thread.start()

    def publish_loop(self):
        rate = self.create_rate(10)  # 10 Hz
        while rclpy.ok() and self.running:
            for port, cap in self.captures.items():
                ret, frame = cap.read()
                if not ret:
                    self.get_logger().warning(f'Failed to read frame from camera {port}')
                    continue
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.publishers[port].publish(msg)
            rate.sleep()

    def stop(self):
        self.running = False
        self.thread.join()
        for cap in self.captures.values():
            cap.release()

def main(args=None):
    rclpy.init(args=args)

    # Parse camera ports from CLI arguments
    if len(sys.argv) < 2:
        print("Usage: ros2 run <package_name> multi_camera_publisher.py <camera_port1> <camera_port2> ...")
        return

    camera_ports = list(map(int, sys.argv[1:]))

    node = MultiCameraPublisher(camera_ports)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
