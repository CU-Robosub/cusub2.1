# kill_motors_service.py
import time
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import SendPWM  # Replace with your actual package name
from .motorController import motorController 
from std_msgs.msg import Bool

class KillMotorsService(Node):
    def __init__(self):
        super().__init__('send_pwm_service')
        self.shutdown_pub = self.create_publisher(Bool, '/shutdown_motors', 10)
        self.srv = self.create_service(SendPWM, 'send_pwm', self.send_pwm_callback)
        self.mc = motorController()

    def send_pwm_callback(self, request, response):
        try:
            self.mc.run(request.channels, int(request.value), raw_pwm=True)
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Failed to send pwm to motors: {e}")
            response.success = False
        return response

def main(args=None):
    rclpy.init(args=args)
    node = KillMotorsService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
