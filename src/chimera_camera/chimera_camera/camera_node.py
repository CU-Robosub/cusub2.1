import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self, camera_id):
        super().__init__(f'camera_{camera_id}_node')
        self.camera_id = camera_id
        self.device_path = f"/dev/video{camera_id}"
        self.bridge = CvBridge()
        self.enabled = False

        self.publisher = self.create_publisher(Image, f'/camera_{camera_id}/image_raw', 10)
        self.subscription = self.create_subscription(
            Bool,
            f'/camera_{camera_id}/enable',
            self.enable_callback,
            10
        )

        # Keep camera open
        self.cap = cv2.VideoCapture(self.device_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open {self.device_path}")

        self.timer = self.create_timer(0.05, self.capture_frame)  # 20 FPS max

    def enable_callback(self, msg: Bool):
        self.enabled = msg.data
        state = "ENABLED" if self.enabled else "DISABLED"
        self.get_logger().info(f"Camera {self.camera_id} {state}")

    def capture_frame(self):
        if not self.enabled:
            return
        ret, frame = self.cap.read()
        if ret:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(img_msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    import sys
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Usage: camera_node.py <camera_id>")
        return
    camera_id = int(sys.argv[1])
    node = CameraNode(camera_id)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
