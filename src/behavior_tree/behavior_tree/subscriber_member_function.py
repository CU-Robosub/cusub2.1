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


class BTSubscriber(rcl.Node, BehaviorTree.Condition):

    def __init__(self, name : str, topic_name : str, condition): # condition must take 1 argument (subscriber message) and return true or false
        rcl.Node.__init__(self, name)                       # Node init
        BehaviorTree.Condition.__init__(self, name, condition) # Action init
        self.subscription = self.create_subscription(
            String,
            topic_name,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.conditionMet = False # attribute stores if condition was met (false by default)

    def listener_callback(self, msg):
        self.get_logger().info('Message Recieved: "%s"' % msg.data)
        self.conditionMet = self.condition(msg.data)

    def evaluate(self):
        rclpy.spin_once(self)
        result = self.conditionMet
        print(f"Checking Condition: {self.name} -> {result}")
        return result


def main(args=None):
    rclpy.init(args=args)

    bt_subscriber = BTSubscriber('first_subscriber', 'topic', lambda x : x == 'hello world')

    bt_subscriber.evaluate()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bt_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
