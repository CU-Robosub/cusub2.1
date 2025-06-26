#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_detection_interfaces.msg import CVData, CVDataPrimitive

class CVDetectionSubscriber(Node):
    def __init__(self):
        super().__init__('cv_detection_subscriber')
        
        self.subscription = self.create_subscription(
            CVData,
            '/cv_detections',
            self.cv_detection_callback,
            10
        )
        
        self.get_logger().info('CV Detection Subscriber started. Waiting for messages...')

    def cv_detection_callback(self, msg):
        self.get_logger().info(f"Received {len(msg.cv_data_primative)} detections")
        
        for detection in msg.cv_data_primative:
            self.get_logger().info(f"Detection ID: {detection.id}")
            self.get_logger().info(f"  Name: {detection.name}")
            self.get_logger().info(f"  Position: ({detection.x}, {detection.y})")
            self.get_logger().info(f"  Size: {detection.width}x{detection.height}")
            self.get_logger().info("---")

def main(args=None):
    rclpy.init(args=args)
    
    subscriber = CVDetectionSubscriber()
    
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()