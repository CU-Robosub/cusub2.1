import sys
import rclpy
import rclpy.node as rcl

import behavior_tree.BehaviorTree as BehaviorTree

# dummy service to use for debugging; the BT only calls services, not hosts them
class BTDebugService(rcl.Node, BehaviorTree.Action):
    def __init__(self, name:str, srv_obj, srv_name: str, **kwargs):
        rcl.Node.__init__(self, name)
        BehaviorTree.Action.__init__(self, name, self.action, **kwargs)
        self.srv = self.create_service(srv_obj, srv_name, self.action)
    
    # service functions are not defined in behavior tree
    def action(self, request, response):
        response.success = True
        response.message = 'bruh'
        self.get_logger().info('Service requested')
        return response

def main(args=None):
    from example_interfaces.srv import SetBool
    rclpy.init(args=args)
    
    bt_service = BTDebugService('return_bool', SetBool, 'set_boolean')

    rclpy.spin(bt_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

