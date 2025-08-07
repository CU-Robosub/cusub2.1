# kill_motors_service.py
import time
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import KillMotors  # Replace with your actual package name
from .motorController import motorController 
from std_msgs.msg import Bool

class KillMotorsService(Node):
    def __init__(self):
        super().__init__('kill_motors_service')
        self.shutdown_pub = self.create_publisher(Bool, '/shutdown_motors', 10)
        self.srv = self.create_service(KillMotors, 'kill_motors', self.kill_motors_callback)
        self.mc = motorController()
        self.channels = [0,1,2,3,4,5,6,7]

    def kill_motors_callback(self, request, response):
        try:
            self.mc.run(self.channels, 1490, raw_pwm=True)
            response.success = True
            self.shutdown_pub.publish(Bool(data=True))
            self.get_logger().info("Motors killed (set to 1490).")
            time.sleep(0.5)

            # Reset shutdown = False
            self.shutdown_pub.publish(Bool(data=False))
        except Exception as e:
            self.get_logger().error(f"Failed to kill motors: {e}")
            response.success = False
        return response

def main(args=None):
    rclpy.init(args=args)
    node = KillMotorsService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
