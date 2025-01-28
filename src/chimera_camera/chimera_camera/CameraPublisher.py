import rclpy
from custom_interfaces.srv import DetectObjects
from ultralytics import YOLO
from rclpy.node import Node
#from cv_bridge import CvBridge
import cv2
import numpy as np


CAMERA_PORT = 0

class Camera(Node):
    def __init__(self):
        super().__init__('CameraPublisher')

        self.get_logger().info("Initializing Camera Node")

        # Sets up the service with the custom type in ../srv/DetectObjects.srv
        self.srv = self.create_service(DetectObjects, 'detect_objects', self.detect_objects)

        self.model = YOLO("../models/yolo11m.pt")

        # Initiate the CvBridge to convert OpenCV images into ROS images
        #self.bridge = CvBridge()

        # Opens up the camera port to be read from
        self.cam_feed = cv2.VideoCapture(CAMERA_PORT)

        # If camera does not open, shut down the node because it is useless
        if not self.cam_feed.isOpened():
            self.get_logger().error("Failed to open camera")
            rclpy.shutdown()
            return

        # OpenCV keeps a buffer of frames, we only need the most recent frame
        self.cam_feed.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.get_logger().info("Camera Node Successfully Initialized")

        

    def cleanup(self):
        self.get_logger().info("Cleaning up resources...")
        if self.cam_feed.isOpened():
            self.cam_feed.release()
        cv2.destroyAllWindows()

    def detect_objects(self, request, response):

        unused = self.cam_feed.read()  # Clear buffer
        ret, frame = self.cam_feed.read()  # Get the latest frame

        if not ret:
            self.get_logger().error("Could not read camera")
            return response

        # Perform object detection on the frame using YOLO11m
        results = self.model(frame)

        object_names = []
        object_positions_x = []
        object_positions_y = []
        confidences = []
        top_left = []
        bottom_right = []
    # Draw bounding boxes and labels
        for detection in results[0].boxes:
            
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



        ''' Uncomment if you want to see bounding boxes 
        # Display the resulting frame with bounding boxes
        cv2.imshow('Webcam Feed with Detections', frame)

        # Wait for a key press to keep the window responsive
        cv2.waitKey(0)  
        '''
        # Populate the response
        response.object_names = object_names
        response.object_positions_x = object_positions_x
        response.object_positions_y = object_positions_y
        response.confidence = confidences
        response.top_left = top_left
        response.bottom_right = bottom_right

        return response

    def __del__(self):
        self.cleanup()  # Ensure cleanup if the object is deleted

def main(args=None):
    rclpy.init(args=args)
    image = Camera()
    rclpy.spin(image)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
