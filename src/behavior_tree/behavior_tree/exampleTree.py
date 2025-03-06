import behavior_tree.BehaviorTree as BehaviorTree
from behavior_tree.publisher_member_function import BTPublisher
from behavior_tree.subscriber_member_function import BTSubscriber

import rclpy

import random

####################################################################################################
# this script is an example of how a behavior tree can be used to guide a robo-sub through a       #
# task: centering a target in the camera view                                                      #
####################################################################################################


# define initial direction of the sub and target direction
currentDirection = 0
targetDirection = random.choice(range(0, 356, 5)) # succeeds 50% of the time
print(f'Target direction: {targetDirection}')


# functions used by the tree to carry out actions or check conditions

def checkBattery():
    # returns true (battery charged) with 80% chance
    return random.random() <= 0.8

def rotate():
    global currentDirection
    currentDirection += 10
    return str(currentDirection)


def main(args=None):
    rclpy.init(args=args)


    # define nodes
    root = BehaviorTree.Sequence("Root_Sequence")

    check_battery = BehaviorTree.Condition("Check_Battery", checkBattery)

    rotate_sub = BehaviorTree.Iterator("Rotate_Sub", maxAttempts=36)

    update_direction = BehaviorTree.Sequence("Update_Direction")

    publish_current_direction = BTPublisher('Publish_Direction', 'direction', rotate)

    check_aligned = BTSubscriber('Check_Aligned', 'direction', lambda curr_dir : curr_dir == str(targetDirection))


    # define node relations
    root.add_child(check_battery)
    root.add_child(rotate_sub)

    rotate_sub.child = update_direction
    update_direction.parent = rotate_sub

    update_direction.add_child(publish_current_direction)
    update_direction.add_child(check_aligned)


    # visualize tree
    root.display(save_name='./src/behavior_tree/visualization/exampleTree')


    print('calling evaluate...')
    root.evaluate()
    print('done!')

    # Destroy the nodes explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publish_current_direction.destroy_node()
    check_aligned.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
