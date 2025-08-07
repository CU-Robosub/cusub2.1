import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys
import threading
import time

class TimeSlicedMultiCameraPublisher(Node):
    def __init__(self, camera_ports):
        super().__init__('time_sliced_multi_camera_publisher')
        self.camera_ports = camera_ports
        self.camera_publishers = {}
        self.bridge = CvBridge()
        self.running = True
        self.current_index = 0  # Track which camera is active

        # Create publishers (one per camera)
        for port in self.camera_ports:
            topic_name = f'/image_{port}'
            pub = self.create_publisher(Image, topic_name, 10)
            self.camera_publishers[port] = pub
            self.get_logger().info(f'Publisher created for camera {port} on topic {topic_name}')

        # Start background thread
        self.thread = threading.Thread(target=self.publish_loop)
        self.thread.start()

    def publish_loop(self):
        rate_hz = 10  # Total loop rate (e.g., 10 Hz shared between all cameras)
        rate = self.create_rate(rate_hz)

        while rclpy.ok() and self.running:
            port = self.camera_ports[self.current_index]

            cap = cv2.VideoCapture(port)
            if not cap.isOpened():
                self.get_logger().warning(f'Failed to open camera {port}')
            else:
                ret, frame = cap.read()
                if ret:
                    msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    msg.header.stamp = self.get_clock().now().to_msg()
                    self.camera_publishers[port].publish(msg)
                else:
                    self.get_logger().warning(f'Failed to read frame from camera {port}')
                cap.release()

            # Move to next camera
            self.current_index = (self.current_index + 1) % len(self.camera_ports)
            rate.sleep()

    def stop(self):
        self.running = False
        self.thread.join()

def main(args=None):
    rclpy.init()

    camera_ports = [0, 2, 4, 6]

    node = TimeSlicedMultiCameraPublisher(camera_ports)

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
