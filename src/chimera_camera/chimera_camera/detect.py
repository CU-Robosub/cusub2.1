import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose, ObjectHypothesis, Pose2D, Point2D
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import os


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        self.model_path = self.declare_parameter('model_path', 'src/chimera_camera/chimera_camera/weights.pt').get_parameter_value().string_value
        self.image_path = self.declare_parameter('image_path', 'images.jpeg').get_parameter_value().string_value
        self.publish_topic = self.declare_parameter('detection_topic', 'yolo/detections').get_parameter_value().string_value

        if not os.path.exists(self.model_path):
            self.get_logger().error(f'model path is {os.getcwd()}')
            self.get_logger().error(f'Model file not found: {self.model_path}')
            rclpy.shutdown()
            return
        self.model = YOLO(self.model_path)

        self.detections_pub = self.create_publisher(Detection2DArray, self.publish_topic, 10)
        
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(Image, 'image_raw', self.image_callback, 10)

        # Uncomment this line to test with fixed image instead of subscriber
        # self.run_on_fixed_image()

    def run_on_fixed_image(self):
        """Run YOLO on a fixed image file"""
        if not os.path.exists(self.image_path):
            self.get_logger().error(f"Image file not found: {self.image_path}")
            rclpy.shutdown()
            return

        self.process_yolo_detection(self.image_path)

    def process_yolo_detection(self, image_input):
        """
        Process YOLO detection on either:
        - image_input: file path (string) for fixed images
        - image_input: OpenCV image (numpy array) for live images
        """
        results = self.model.predict(image_input, save=False, show=False)

        detection_array_msg = Detection2DArray()
        detection_array_msg.header.frame_id = "camera_frame"
        detection_array_msg.header.stamp = self.get_clock().now().to_msg()

        if len(results) == 0:
            self.get_logger().warn("No detections found in image.")
        else:
            result = results[0]
            boxes = result.boxes

            for box in boxes:
                det = Detection2D()
                det.header.frame_id = "camera_frame"
                det.header.stamp = self.get_clock().now().to_msg()

                xmin, ymin, xmax, ymax = box.xyxy[0].cpu().numpy()
                confidence = box.conf[0].cpu().numpy()
                class_id = int(box.cls[0].cpu().numpy())
                
                center_x = (xmin + xmax) / 2.0
                center_y = (ymin + ymax) / 2.0
                width = xmax - xmin
                height = ymax - ymin

                center_pose = Pose2D()
                center_pose.position = Point2D()
                center_pose.position.x = float(center_x)
                center_pose.position.y = float(center_y)
                center_pose.theta = 0.0
                
                det.bbox.center = center_pose
                
                det.bbox.size_x = float(width)
                det.bbox.size_y = float(height)

                obj_hyp = ObjectHypothesisWithPose()
                obj_hyp.hypothesis = ObjectHypothesis()
                
                class_name = self.model.names[class_id] if class_id < len(self.model.names) else f"unknown_{class_id}"
                obj_hyp.hypothesis.class_id = class_name
                obj_hyp.hypothesis.score = float(confidence)
                det.results.append(obj_hyp)
                
                det.id = class_name

                detection_array_msg.detections.append(det)

                self.get_logger().info(f"Detected {class_name} (class {class_id}) with confidence {confidence:.2f} at ({center_x:.1f}, {center_y:.1f})")

        self.detections_pub.publish(detection_array_msg)
        self.get_logger().info(f"Published {len(detection_array_msg.detections)} detections")

    def image_callback(self, msg: Image):
        """Callback for processing incoming image messages"""
        self.get_logger().info("Received image topic")
        
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
        
        self.process_yolo_detection(cv_img)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()