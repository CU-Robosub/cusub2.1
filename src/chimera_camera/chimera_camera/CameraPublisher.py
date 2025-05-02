import rclpy
from custom_interfaces.srv import DetectObjects
from ultralytics import YOLO
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np


class Camera(Node):
    def __init__(self):
        super().__init__('CameraPublisher')

        self.get_logger().info("Initializing Camera Node")

        # Sets up the service with the custom type in ../srv/DetectObjects.srv
        self.srv = self.create_service(DetectObjects, 'detect_objects', self.detect_objects)

        self.model = YOLO("../models/yolo11m.pt")

        # Initiate the CvBridge to convert OpenCV images into ROS images
        self.bridge = CvBridge()

        # parameter for displaying output
        self.declare_parameter('display_output', False)
        self.display_output = self.get_parameter('display_output').get_parameter_value().bool_value

        # valid camera topic names for camera publishers, subscribers, and service requests
        self.sub_map = {
            "front": "/image/front/image_raw",
            "rear": "/image/rear/image_raw",
            "top": "/image/top/image_raw",
            "bottom": "/image/bottom/image_raw"
        }

        self.pub_map = {
            "front": "/image/front/image_annotated",
            "rear": "/image/rear/image_annotated",
            "top": "/image/top/image_annotated",
            "bottom": "/image/bottom/image_annotated"
        }

        # save all images from subscribed camera topics
        self.image_cache = {}

        # save publishers for each camera topic
        self.pubs = {}

        # find all valid running cameras
        camera_topics = [n for n, t in self.get_topic_names_and_types() if n in self.sub_map.values()]

        # does not read from /dev/ to prevent conflicts with cameraCompressed
        # subscribes to all camera topics according to convention
        for cam_id in self.sub_map.keys():
            if self.sub_map[cam_id] in camera_topics:
                self.create_subscription(CompressedImage, self.sub_map[cam_id], self.create_callback(self.sub_map[cam_id]), 10)
                p = self.create_publisher(CompressedImage, self.pub_map[cam_id], 10)
                self.pubs[cam_id] = p
        if not camera_topics:
            self.get_logger().error(f'No active camera topics found!')
        else:
            self.get_logger().info(f"Detect Objects Service Successfully Initialized, subscribed to {camera_topics}")
    
    # callback retains access to topic to cache the returned msg
    def create_callback(self, topic):
        def callback(msg):
            self.image_cache[topic] = msg
        return callback

    def detect_objects(self, request, response):

        # recieve requested camera topic in DetectObjects.camera_topic
        camera_id = request.camera_id

        camera_topic = self.sub_map.get(camera_id)

        if not camera_topic:
            self.get_logger().error(f"Invalid camera_id: {camera_id}. Select from 'front, rear, top, bottom")
            return response
        elif camera_topic not in self.image_cache.keys():
            self.get_logger().error(f"Unavailable camera_id: {camera_id}")
            return response

        # retrieve latest frame from image cache
        msg = self.image_cache[camera_topic]
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Perform object detection on the frame using YOLO11m
        results = self.model(frame)

        object_names = []
        object_positions_x = []
        object_positions_y = []
        confidences = []
        top_left = []
        bottom_right = []
    # Draw bounding boxes and labels
        for result in results:
            for detection in result.boxes:
                
                # Get the coords for the bounding box, confidence scores, and labels. The .cpu().numpy() moves the tensors to cpu memory where they can
                #be converted to numpy arrays. Tensors can be converted to numpy in the GPU.
        
                xyxy = detection.xyxy.cpu().numpy()[0]
                confidence = detection.conf.cpu().numpy()[0]
                label = self.model.names[int(detection.cls.cpu().numpy()[0])]  

        
                start_point = (int(xyxy[0]), int(xyxy[1]))  # Top-left corner
                end_point = (int(xyxy[2]), int(xyxy[3]))    # Bottom-right corner
                cv2.rectangle(frame, start_point, end_point, (0, 255, 0), 2)  # Green box with thickness 2

                # Put the label and confidence on the frame
                text = f"{label} {confidence:.2f}"
                cv2.putText(frame, text, (int(xyxy[0]), int(xyxy[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Calculate center of bounding box
                x_center = (xyxy[0] + xyxy[2]) / 2
                y_center = (xyxy[1] + xyxy[3]) / 2

                # Store detection data for response
                if float(confidence) > 0.75:
                    top_left.extend([float(xyxy[0]), float(xyxy[1])])
                    bottom_right.extend([float(xyxy[2]), float(xyxy[3])])
                    object_names.append(label)
                    object_positions_x.append(float(x_center))
                    object_positions_y.append(float(y_center))
                    confidences.append(float(confidence))


        if self.display_output:
            # Resize the image
            img_resized = cv2.resize(frame, (640, 480))  # Adjust the size as needed

            # Convert the OpenCV image to a ROS compressed image message
            try:
                image_msg = self.bridge.cv2_to_compressed_imgmsg(img_resized, dst_format='jpeg')  # Use JPEG compression
            except CvBridgeError as e:
                rclpy.logerr(e)
                self.destroy_node()
                return

            # Publish the compressed ROS image message
            self.pubs[camera_id].publish(image_msg)


        # Populate the response
        response.object_names = object_names
        response.object_positions_x = object_positions_x
        response.object_positions_y = object_positions_y
        response.confidence = confidences
        response.top_left = top_left
        response.bottom_right = bottom_right

        return response

def main(args=None):
    rclpy.init(args=args)
    image = Camera()
    rclpy.spin(image)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
