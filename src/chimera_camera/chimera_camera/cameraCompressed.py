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
    def __init__(self, topic):
        super().__init__('CameraPublisherRaw')
        self.publisher = self.create_publisher(CompressedImage, topic, 10)

        # initialize camera to /dev/video0; configurable at runtime with --ros-args -p
        self.declare_parameter('camera_port', 0)
        CAMERA_PORT = self.get_parameter('camera_port').get_parameter_value().integer_value
        self.get_logger().info(f'Camera port is set to: {CAMERA_PORT}')

        self.bridge = CvBridge()
        self.cam_feed = cv2.VideoCapture(CAMERA_PORT)

        # tick length parameter for ticking the behavior tree
        self.declare_parameter('fps', 30)
        fps = self.get_parameter('fps').get_parameter_value().integer_value

        TICK_LENGTH = 1 / fps

        self.timer = self.create_timer(TICK_LENGTH, self.compress_and_publish_image)

    def compress_and_publish_image(self, topic='image_compressed'):
        ret, img = self.cam_feed.read()
        
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

def main(args=None):
    rclpy.init(args=args)
    image = Camera(topic='image')
    
    # Use image_transport for publishing compressed images
    #image_transport = image.create_publisher(CompressedImage, 'image', 10)
    
    rclpy.spin(image)
    image.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
