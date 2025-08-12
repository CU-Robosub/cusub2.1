import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import time

class CameraNode(Node):
    def __init__(self, camera_id):
        super().__init__(f'camera_{camera_id}_node')
        self.camera_id = camera_id
        self.device_path = f"/dev/video{camera_id}"
        self.bridge = CvBridge()
        self.enabled = False
        self.cap = None

        self.publisher = self.create_publisher(Image, f'/camera_{camera_id}/image_raw', 1)
        self.subscription = self.create_subscription(
            Bool,
            f'/camera_{camera_id}/enable',
            self.enable_callback,
            10
        )

        self.timer = self.create_timer(1.0 / 15.0, self.capture_frame)

    def enable_callback(self, msg: Bool):
        new_state = msg.data
        if new_state and not self.enabled:
            # Enable: open camera
            self.cap = cv2.VideoCapture(self.device_path)
            if not self.cap.isOpened():
                self.get_logger().error(f"Could not open {self.device_path}")
                self.cap = None
                return

            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 15)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            self.get_logger().info(f"Camera {self.camera_id} ENABLED and opened")
        elif not new_state and self.enabled:
            # Disable: release camera
            if self.cap is not None:
                self.cap.release()
                time.sleep(1.0)
                self.cap = None
            self.get_logger().info(f"Camera {self.camera_id} DISABLED and released")
        self.enabled = new_state

    def capture_frame(self):
        if not self.enabled or self.cap is None:
            return
        ret, frame = self.cap.read()
        if ret:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(img_msg)

    def destroy_node(self):
        if self.cap is not None:
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
