####################################################################################################
# how the behavior tree is run from root

# implements equivalent functionality as in:
#   cortex.controller.computeHeadingTest
# without relying on other nodes running

# mostly tests propagation of the blackboard through the tree

# run setup.py executables for other examples
####################################################################################################


import rclpy
import rclpy.node as rcl

# behaviortree package
import behavior_tree.BehaviorTree as BehaviorTree

# ROS node wrapper around behavior tree for ticking
class BTRoot(rcl.Node):
    def __init__(self, name='bt_root'):
        rcl.Node.__init__(self, name)
        
        # initialize blackboard (TODO: from yaml)

        self.blackboard = BehaviorTree.Blackboard()
        self.blackboard['object_name'] = 'item'


        self.root = BehaviorTree.Iterator(name='try_until_detection', maxAttempts=3, blackboard=self.blackboard)
        
        seq = BehaviorTree.Sequence(name='seq')

        # hacky implementation of obj detection without service call
        # these would be custom behavior
        def get():
            try:
                self.blackboard['object_position'] = [15, 15]
                return True
            except Exception as e:
                print('Exception {e}')
                return False 
        get_obj = BehaviorTree.Action(name='get_obj', action=get)

        # 'computing' something
        def compute():
            try:
                theta_x, theta_y = self.blackboard['object_position']
                self.blackboard['object_heading'] = [theta_x, theta_y]
                return True
            except Exception as e:
                print(f'Exception {e}')
                return False
        compute_heading = BehaviorTree.Action(name='compute_heading', action=compute)

        # IMPORTANT: top down, left to right tree init to make blackboard propagate properly
        self.root.child = seq
        seq.add_child(get_obj)
        seq.add_child(compute_heading)

        # tick length parameter for ticking the behavior tree
        self.declare_parameter('tick_length', 1.0)
        TICK_LENGTH = self.get_parameter('tick_length').get_parameter_value().double_value

        self.timer = self.create_timer(TICK_LENGTH, self.evaluate)

    def evaluate(self):
        # this is a terrible way to do this
        result = self.root.evaluate()
        if result:
            self.get_logger().info(f'Root node {self.root.name} reached stopping condition')
            self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    root = BTRoot()

    # behavior tree root node is ticked by internal timer
    # returning true makes the node self destruct
    rclpy.spin(root)

    # TODO: idk why the node isn't shutting down

    root.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()