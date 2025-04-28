import sys
import rclpy
import rclpy.node as rcl

import behavior_tree.BehaviorTree as BehaviorTree


class BTClient(rcl.Node, BehaviorTree.Action):
    def __init__(self, name : str, srv_obj, srv_name: str, **kwargs):
        rcl.Node.__init__(self, name)
        BehaviorTree.Action.__init__(self, name, self.action, **kwargs)

        self.cli = self.create_client(srv_obj, srv_name)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'waiting for {srv_name}...')
        self.req = srv_obj.Request()

        self._future = None
        self._sent_request = False
    
    # request object is not populated
    def send_request(self):
        return self.cli.call_async(self.req)
    
    def action(self, response):
        for field in response.get_fields_and_field_types().keys():
            self.blackboard[field] = getattr(response, field)

    def evaluate(self):
        if not self._sent_request:
            self._future = self.send_request()
            self.get_logger().info(f'Sending service request')
            self._sent_request = True
        
        while not self._future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        try:
            response = self._future.result()
            self.get_logger().info(f'Called service')
            self.action(response)
            self._sent_request = False
            return True

        except Exception as e:
            self.get_logger().error(f'Error calling service: {e}')
            self._sent_request = False
            return False

def main(args=None):
    from example_interfaces.srv import SetBool
    rclpy.init(args=args)

    # a global blackboard may be passed
    bt_client = BTClient('get_bool', SetBool, 'set_boolean')

    bt_client.evaluate()

    bt_client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()







