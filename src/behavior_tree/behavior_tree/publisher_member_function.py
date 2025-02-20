# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import rclpy.node as rcl

from std_msgs.msg import String

import behavior_tree.BehaviorTree as BehaviorTree


class BTPublisher(rcl.Node, BehaviorTree.Action):

    def __init__(self, name : str, topic_name : str, action): # action function must return string
        rcl.Node.__init__(self, name)                    # Node init
        BehaviorTree.Action.__init__(self, name, action) # Action init
        self.publisher_ = self.create_publisher(String, topic_name, 10)
        self.i = 0 # keeps track of number of times published

    def evaluate(self):
        print(f"Executing Action: {self.name}")
        msg = String()

        try:
            msg.data = self.action()
            self.publisher_.publish(msg)
            self.get_logger().info('Published: "%s"' % msg.data)
            self.i += 1
            return True
        
        except Exception as e:
            self.get_logger().error(f'Error publishing message: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)

    bt_publisher = BTPublisher('first_publisher', 'topic', 1)

    print('calling evaluate...')
    bt_publisher.evaluate()
    print('done!')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bt_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
