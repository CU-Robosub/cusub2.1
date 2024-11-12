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
from geometry_msgs.msg import Pose, Twist
import serial
import math
from .dvl_tcp_parser import datareader
import time
import json

class DVL(Node):

    def __init__(self):
        super().__init__('DVLPublisher')
        self.rawrecpub = self.create_publisher(String, 'dvl_raw_deadrec_data', 10)
        self.posepub = self.create_publisher(Pose, 'pose', 10)
        self.velocity_publisher = self.create_publisher(Twist, 'velocity', 10)
        self.goalposepub = self.create_publisher(Pose, 'goal_pose', 10)
        self.dvl = datareader()
        self.dvl.connect_dvl()
        #if (self.dvl.is_connected()):
        #    self.get_logger.info("Calibrating gyro...",str(self.dvl.calibrate_gyro()))
        #    self.get_logger.info("Resetting dead reckoning...",str(self.dvl.calibrate_gyro()))
        self.latestPose = Pose()
        self.didinitialpose = False

        self.timer_ = self.create_timer(0.2, self.publish_data)  # Call data_callback every 0.2 seconds (dead reckoning report update cycle is 5hz)
    

    def publish_data(self):
        msgstr = String()

        if (not self.dvl.is_connected()):
            self.get_logger("Could not read data, DVL not connected")
            return
        
        msg = self.dvl.read_data("dead_reckoning")

        if (msg is None):
            self.get_logger("DVL connected but provided no data")
            msgstr.data = "no data"
            self.rawrecpub.publish(msgstr)
            return
        
        msgstr.data = str(msg)

        #Convert raw data supplied from dvl into a pose message
        pose = self.convert_to_pose(msg)

        self.posepub.publish(pose)

        #Publish raw dvl data for testing
        self.rawrecpub.publish(msgstr)

    def publish_data2(self):
        msgstr = String()

        if (not self.dvl.is_connected()):
            self.get_logger("Could not read data, DVL not connected")
            return
        
        msg = self.dvl.read_data("dead_reckoning")

        if (msg is None):
            self.get_logger("DVL connected but provided no data")
            msgstr.data = "no data"
            self.rawrecpub.publish(msgstr)
            return
        
        if msg["type"] == "position_local":
            pose = self.convert_to_pose(msg)
            self.posepub.publish(pose)
        elif msg["type"] == "velocity":
            velocity = self.convert_to_twist(msg)
            self.velocity_publisher.publish(velocity)
        elif msg["type"] == "response":
            self.get_logger(msg["response_to"] + ": " + "FAILED" if msg["success"] == False else "SUCCESS")
        
            
        
        
        msgstr.data = str(msg)

        #Convert raw data supplied from dvl into a pose message

        #Publish raw dvl data for testing
        self.rawrecpub.publish(msgstr)
            


    def convert_to_pose(self, data):
        if (data is None):
             return
    
        parsed_data = json.loads(data)
        posemsg = Pose()
        posemsg.position.x = parsed_data['x']
        posemsg.position.y = parsed_data['y']
        posemsg.position.z = parsed_data['z']
        angdata = self.euler_to_quaternion(parsed_data['roll'],parsed_data['pitch'],parsed_data['yaw'])
        posemsg.orientation.x = angdata[0]
        posemsg.orientation.y = angdata[1]
        posemsg.orientation.z = angdata[2]
        posemsg.orientation.w = angdata[3]
        return posemsg
    
    def convert_to_twist(self, data):
        velocity_msg = Twist()
        velocity_msg.linear([data["vx"], data["vy"], data["vz"]])
        return velocity_msg


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
