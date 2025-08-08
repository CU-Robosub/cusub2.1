import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class CameraCoordinator(Node):
    def __init__(self):
        super().__init__('camera_coordinator')
        self.camera_ids = [0, 1, 2, 3]
        self.pub_map = {
            cam_id: self.create_publisher(Bool, f'/camera_{cam_id}/enable', 10)
            for cam_id in self.camera_ids
        }


        self.current_pair_index = 0
        self.switch_interval = 1.0  # seconds
        self.timer = self.create_timer(self.switch_interval, self.switch_cameras)

        self.switch_cameras()

    def switch_cameras(self):
        # Disable all
        for pub in self.pub_map.values():
            pub.publish(Bool(data=False))

        # Pick two to enable
        pair = [
            self.camera_ids[self.current_pair_index % len(self.camera_ids)],
            self.camera_ids[(self.current_pair_index + 1) % len(self.camera_ids)]
        ]
        for cam_id in pair:
            self.pub_map[cam_id].publish(Bool(data=True))

        self.get_logger().info(f"Enabled cameras: {pair}")
        self.current_pair_index = (self.current_pair_index + 2) % len(self.camera_ids)


def main(args=None):
    rclpy.init(args=args)
    node = CameraCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
