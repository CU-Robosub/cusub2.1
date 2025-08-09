"""
    AUTHOR: JAKE TUCKER
    CONTACT: jakob.tucker@colorado.edu
    PURPOSE: Get and publish data from our DVL to ROS
"""

# package requries additional setup, see documentation from https://github.com/waterlinked/dvl-python/tree/master/serial

import rclpy
from rclpy.node import Node
# from wldvl import WlDVL
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import serial
import math
from .dvl_tcp_parser import datareader
import time
import json

class DVL(Node):

    def __init__(self):
        super().__init__('DVLPublisher')
        self.rawrecpub = self.create_publisher(String, 'dvl_raw_deadrec_data', 10)
        self.posepub = self.create_publisher(PoseStamped, 'pose', 10)
        self.goalposepub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.dvl = datareader()
        self.dvl.connect_dvl()
        #if (self.dvl.is_connected()):
        #    self.get_logger.info("Calibrating gyro...",str(self.dvl.calibrate_gyro()))
        #    self.get_logger.info("Resetting dead reckoning...",str(self.dvl.calibrate_gyro()))
        self.posemsg = PoseStamped()
        self.didinitialpose = False

        self.timer_ = self.create_timer(0.2, self.publish_data)  # Call data_callback every 0.2 seconds (dead reckoning report update cycle is 5hz)
    
    def publish_data(self):
        msgstr = String()
        if (self.dvl.is_connected()):
            msg = self.dvl.read_data("dead_reckoning")
            if (msg is not None):
                msgstr.data = str(msg)
                self.posepub.publish(
                    self.convert_to_pose(
                        msg)
                )
                self.rawrecpub.publish(msgstr)
                if (not self.didinitialpose):
                    self.goalposepub.publish(
                        self.convert_to_pose(
                            msg)
                    )
                    self.didinitialpose = True
            else:
                self.get_logger().info("no data")
                msgstr.data = "no data"
                self.rawrecpub.publish(msgstr)
        else:
            self.get_logger().info("DVL not connected")


    def convert_to_pose(self, data):
        if (data is None):
             return
        parsed_data = json.loads(data)
        self.posemsg.header.stamp = self.get_clock().now().to_msg()
        self.posemsg.header.frame_id = 'map'
        self.posemsg.pose.position.x = parsed_data['x']
        self.posemsg.pose.position.y = parsed_data['y']
        self.posemsg.pose.position.z = parsed_data['z']
        angdata = self.euler_to_quaternion(parsed_data['roll'],parsed_data['pitch'],parsed_data['yaw'])
        self.posemsg.pose.orientation.x = angdata[0]
        self.posemsg.pose.orientation.y = angdata[1]
        self.posemsg.pose.orientation.z = angdata[2]
        self.posemsg.pose.orientation.w = angdata[3]
        return self.posemsg


    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qw, qx, qy, qz

def main(args=None):
    rclpy.init(args=args)

    dvl_publish = DVL()

    rclpy.spin(dvl_publish)
    dvl_publish.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
