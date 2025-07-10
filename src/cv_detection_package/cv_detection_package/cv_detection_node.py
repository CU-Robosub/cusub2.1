#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from flask import Flask, Response
import cv2
from ultralytics import YOLO
import time
import threading
import socket
import torch
from cv_detection_interfaces.msg import CVData, CVDataPrimitive
from std_msgs.msg import Header

print(torch.cuda.is_available())  # Should be True
print(torch.cuda.get_device_name(0))  # Should print Jetson GPU

# Get the Jetson's IP address
def get_ip_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Doesn't need to be reachable
    s.connect(('8.8.8.8', 1))
    ip_address = s.getsockname()[0]
    s.close()
    return ip_address

class CVDetectionNode(Node):
    def __init__(self):
        super().__init__('cv_detection_node')
        
        # Create publisher
        self.cv_publisher = self.create_publisher(CVData, '/cv_detections', 10)
        
        # Initialize YOLO model
        self.model = YOLO("yolov8n.pt")
        
        # Global frame variables
        self.global_frame = None
        self.frame_lock = threading.Lock()
        
        # Detection ID counter
        self.detection_id = 1
        
        # Initialize Flask app
        self.app = Flask(__name__)
        self.setup_flask_routes()
        
        self.get_logger().info('CV Detection Node initialized')

    def setup_flask_routes(self):
        @self.app.route('/video_feed')
        def video_feed():
            def generate_frames():
                while True:
                    with self.frame_lock:
                        if self.global_frame is not None:
                            # Encode the frame as JPEG
                            _, buffer = cv2.imencode('.jpg', self.global_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                            frame_bytes = buffer.tobytes()
                            
                            # Yield the frame in the response
                            yield (b'--frame\r\n'
                                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                    
                    # Add a small delay
                    time.sleep(0.03)  # ~30 FPS
            
            return Response(generate_frames(),
                            mimetype='multipart/x-mixed-replace; boundary=frame')

        @self.app.route('/')
        def index():
            ip_address = get_ip_address()
            return f"""
            <html>
            <head>
                <title>YOLO Live Stream - ROS2</title>
                <style>
                    body {{ font-family: Arial, sans-serif; text-align: center; }}
                    img {{ max-width: 100%; height: auto; }}
                </style>
            </head>
            <body>
                <h1>YOLO Live Stream from Jetson Nano Orin (ROS2)</h1>
                <img src="/video_feed" />
                <p>Streaming from: {ip_address}</p>
                <p>Publishing detections to ROS2 topic: /cv_detections</p>
            </body>
            </html>
            """

    def capture_frames(self):
        # Initialize webcam
        cap = cv2.VideoCapture(0)
        
        if not cap.isOpened():
            self.get_logger().error("Cannot open camera")
            return
        
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        prev_time = 0
        
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                break
                
            start = time.time()
            results = self.model(frame, verbose=False)
            end = time.time()
            self.get_logger().debug(f"Inference time: {end - start:.3f}s")

            frame_height, frame_width = frame.shape[:2]
            
            annotated_frame = results[0].plot()
            
            # Create ROS2 message
            cv_msg = CVData()
            cv_msg.header = Header()
            cv_msg.header.stamp = self.get_clock().now().to_msg()
            cv_msg.header.frame_id = "camera_frame"
            cv_msg.detections = []

            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        cls = int(box.cls[0])
                        class_name = result.names[cls]
                        confidence = float(box.conf[0])
                        
                        # Only process detections with reasonable confidence
                        if confidence > 0.5:
                            x1, y1, x2, y2 = box.xyxy[0].tolist()
                            
                            # Create CV data primitive message
                            cv_primitive = CVDataPrimitive()
                            cv_primitive.id = self.detection_id
                            cv_primitive.name = class_name
                            cv_primitive.x = int(x1)
                            cv_primitive.y = int(y1)
                            cv_primitive.width = int(x2 - x1)
                            cv_primitive.height = int(y2 - y1)
                            
                            cv_msg.detections.append(cv_primitive)
                            self.detection_id += 1
                            
                            # Special handling for bottles (from your original code)
                            if class_name == "bottle":
                                center_x = (x1 + x2) / 2
                                center_y = (y1 + y2) / 2
                                
                                if center_x < 200:
                                    h_position = "left"
                                elif center_x > 440:
                                    h_position = "right"
                                elif center_y < 100:
                                    h_position = "up"
                                elif center_y > 380:
                                    h_position = "down"
                                else:
                                    h_position = "straight"
                                
                                position_text = f"Bottle: {h_position}"
                                text_position = (int(x1), int(y1) - 10)
                                cv2.putText(annotated_frame, position_text, text_position, 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                                
                                self.get_logger().info(f"Bottle detected at: {h_position}")
            
            # Publish the CV data message
            if len(cv_msg.detections) > 0:
                self.cv_publisher.publish(cv_msg)
                self.get_logger().info(f"Published {len(cv_msg.detections)} detections")
            
            # Calculate and display FPS
            current_time = time.time()
            fps = 1 / (current_time - prev_time)
            prev_time = current_time
            cv2.putText(annotated_frame, f'FPS: {int(fps)}', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Update the global frame with lock
            with self.frame_lock:
                self.global_frame = annotated_frame

        # Release the camera
        cap.release()

    def run_flask(self):
        ip_address = get_ip_address()
        self.get_logger().info(f"Starting Flask server on http://{ip_address}:5000")
        self.app.run(host='0.0.0.0', port=5000, threaded=True)

def main(args=None):
    rclpy.init(args=args)
    
    node = CVDetectionNode()
    
    try:
        # Start the frame capture thread
        capture_thread = threading.Thread(target=node.capture_frames)
        capture_thread.daemon = True
        capture_thread.start()
        
        # Start Flask in a separate thread
        flask_thread = threading.Thread(target=node.run_flask)
        flask_thread.daemon = True
        flask_thread.start()
        
        # Spin the ROS2 node
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()