import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class CameraCoordinator(Node):
    def __init__(self):
        super().__init__('camera_coordinator')

        # === Argument parsing, MUST be here before self.camera_ids usage ===
        if '--ros-args' in sys.argv:
            ros_index = sys.argv.index('--ros-args')
            args_only = sys.argv[1:ros_index]
        else:
            args_only = sys.argv[1:]

        if len(args_only) == 1:
            # One arg containing spaces
            self.camera_ids = args_only[0].split()
        else:
            self.camera_ids = args_only

        # Convert to ints
        try:
            self.camera_ids = [int(cid) for cid in self.camera_ids]
        except ValueError:
            self.get_logger().error(f"Invalid camera ID(s) in arguments: {self.camera_ids}")
            rclpy.shutdown()
            return
        # === End arg parsing ===

        # Now it's safe to use self.camera_ids
        self.pub_map = {
            cam_id: self.create_publisher(Bool, f'/camera_{cam_id}/enable', 10)
            for cam_id in self.camera_ids
        }

        self.current_pair_index = 0
        self.switch_interval = 1.0  # seconds

        if len(self.camera_ids) < 3:
            self.get_logger().info(
                f"Less than 3 cameras ({len(self.camera_ids)}). Keeping all cameras enabled without switching."
            )
            self.enable_all_cameras()
        else:
            self.timer = self.create_timer(self.switch_interval, self.switch_cameras)
            self.switch_cameras()


    def enable_all_cameras(self):
        for pub in self.pub_map.values():
            pub.publish(Bool(data=True))
        self.get_logger().info(f"All cameras enabled: {list(self.camera_ids)}")

    def switch_cameras(self):
        # Disable all cameras first
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

        # Move to next pair
        self.current_pair_index = (self.current_pair_index + 2) % len(self.camera_ids)



def main(args=None):
    rclpy.init(args=args)
    node = CameraCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
