"""
    AUTHOR: JAKE TUCKER
    CONTACT: jakob.tucker.edu
    PURPOSE: Camera node to subscribe to raw camera data
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class Camera(Node):
    def __init__(self):
        super().__init__('CameraPublisherRaw')

        self.declare_parameter('camera_id', 'front')
        CAMERA_ID = self.get_parameter('camera_id').get_parameter_value().string_value

        # valid camera topic names for camera publishers, subscribers, and service requests
        self.topic_map = {
            "front": "/image/front/image_raw",
            "rear": "/image/rear/image_raw",
            "top": "/image/top/image_raw",
            "bottom": "/image/bottom/image_raw"
        }
        camera_topic = self.topic_map[CAMERA_ID]
        self.get_logger().info(f'Camera id is set to: {CAMERA_ID}')

        # publisher based on camera id given in launch file
        self.publisher = self.create_publisher(CompressedImage, camera_topic, 10)

        # initialize camera to /dev/video0; configurable thru launch files
        # the idea is to launch this node for each submarine camera in different namespaces
        self.declare_parameter('camera_port', 0)
        CAMERA_PORT = self.get_parameter('camera_port').get_parameter_value().integer_value
        self.get_logger().info(f'Camera port is set to: {CAMERA_PORT}')

        self.bridge = CvBridge()
        self.cam_feed = cv2.VideoCapture(CAMERA_PORT)

        # If camera does not open, shut down the node because it is useless
        if not self.cam_feed.isOpened():
            self.get_logger().error("Failed to open camera")
            rclpy.shutdown()
            return

        # OpenCV keeps a buffer of frames, we only need the most recent frame
        self.cam_feed.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.get_logger().info("Camera Node Successfully Initialized")

        # tick length parameter for ticking the behavior tree
        self.declare_parameter('fps', 30)
        fps = self.get_parameter('fps').get_parameter_value().integer_value

        TICK_LENGTH = 1 / fps

        self.timer = self.create_timer(TICK_LENGTH, self.compress_and_publish_image)

    # the topic to publish to is handled by the namespace assigned in the launch file
    def compress_and_publish_image(self):
        ret, img = self.cam_feed.read()

        if not ret:
            self.get_logger().error("Could not read camera")
            return 
        
        # Resize the image
        img_resized = cv2.resize(img, (640, 480))  # Adjust the size as needed

        # Convert the OpenCV image to a ROS compressed image message
        try:
            image_msg = self.bridge.cv2_to_compressed_imgmsg(img_resized, dst_format='jpeg')  # Use JPEG compression
        except CvBridgeError as e:
            rclpy.logerr(e)
            self.destroy_node()

        # Publish the compressed ROS image message
        self.publisher.publish(image_msg)

    def cleanup(self):
        self.get_logger().info("Cleaning up resources...")
        if self.cam_feed.isOpened():
            self.cam_feed.release()
        cv2.destroyAllWindows()

    def __del__(self):
        self.cleanup()  # Ensure cleanup if the object is deleted

def main(args=None):
    rclpy.init(args=args)
    image = Camera()
    
    # Use image_transport for publishing compressed images
    #image_transport = image.create_publisher(CompressedImage, 'image', 10)
    
    rclpy.spin(image)
    image.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
