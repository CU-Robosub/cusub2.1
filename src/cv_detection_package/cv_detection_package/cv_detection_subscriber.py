#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_detection_interfaces.msg import CVData, CVDataPrimitive, NamedPoseArray, NamedPose
import tf_transformations
import numpy as np
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Quaternion, Point
import os
import yaml

class ObjectEstimator(Node):
    def __init__(self):
        super().__init__('cv_detection_subscriber')
        
        self.subscription = self.create_subscription(
            CVData,
            '/cv_detections',
            self.cv_detection_callback,
            10
        )
        
        self.pose_subscription = self.create_subscription(
            Pose,
            '/pose',
            self.pose_callback,
            10
        )
        
        self.robot_pose = None
        self.publisher = self.create_publisher(NamedPoseArray, '/object_poses', 10)
        
        self.get_logger().info('CV Detection Subscriber started. Waiting for messages...')
        objects_yaml_path = os.path.join(get_package_share_directory("cv_detection_package"), "config", "objects.yaml")
        cameras_yaml_path = os.path.join(get_package_share_directory("cv_detection_package"), "config", "cameras.yaml")
        self.objects_yaml = self.read_yaml(objects_yaml_path)
        self.cameras_yaml = self.read_yaml(cameras_yaml_path)
            
    def read_yaml(self, path: str) -> dict | None:
        yml = None
        try:
            with open(path, "r") as file:
                yml = yaml.safe_load(file)
        except FileNotFoundError:
            self.get_logger().warn(f"Error: The file '{path}' was not found.")
        except yaml.YAMLError as e:
            self.get_logger().warn(f"Error parsing YAML file: {e}")
        return yml
    
    def pose_callback(self, msg):
        self.robot_pose = msg.data
        

    def cv_detection_callback(self, msg):
        # self.get_logger().info(f"Received {len(msg.detections)} detections")
        pose_arr = NamedPoseArray()
        if (len(msg.detections)):
            for detection in msg.detections:
                if detection.results.score < 0.3: continue # skip this object, since we have low confidence
                o_width, o_height = self.get_object_physical_properties(detection.results.id)
                camera_q = self.get_camera_orientation_quaternion(msg.header.frame_id)
                camera_fl = self.get_camera_focal_length(msg.header.frame_id)
                cx, cy = self.get_camera_pxpy(msg.header.frame_id)
                cx /= 2
                cy /= 2
                cx, cy = int(cx), int(cy)
                obj_pose = self.compute_object_position(self.robot_pose, camera_q, camera_fl, detection.bbox.center.x, detection.bbox.center.y,
                                                        detection.bbox.size_x, detection.bbox.size_y, o_width, o_height, cx, cy)
                
                named_pose = NamedPose()
                named_pose.pose = obj_pose
                named_pose.name = detection.name
                named_pose.id = detection.id
                pose_arr.append(named_pose)
            self.publisher.publish(pose_arr)
    
    def get_object_physical_properties(self, detection_id: int) -> tuple:
        """Performs a lookup with an object's id to get its physical size.

        Args:
            detection_id (int): id of the object (often returned from the topic /cv_detections)

        Returns:
            int, int: width and height of the object
        """
        try:
            width = self.objects_yaml[detection_id]["width"]
            height = self.objects_yaml[detection_id]["height"]
        except:
            width, height = None
            self.get_logger().warn("Error getting width or height from yaml")
        
        return width, height
    
    def kalman_filter_update(self):
        # TODO: Perform filter update on camera estimate
        pass
    
    def get_camera_orientation_quaternion(self, camera_frame: str) -> Quaternion:
        roll = self.cameras_yaml[camera_frame]["roll"]
        pitch = self.cameras_yaml[camera_frame]["pitch"]
        yaw = self.cameras_yaml[camera_frame]["yaw"]

        # Convert RPY to quaternion (x, y, z, w)
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw, axes='sxyz')

        # Return as a geometry_msgs.msg.Quaternion
        quat_msg = Quaternion()
        quat_msg.x = q[0]
        quat_msg.y = q[1]
        quat_msg.z = q[2]
        quat_msg.w = q[3]

        return quat_msg
    
    def get_camera_focal_length(self, camera_frame: str) -> int:
        return self.cameras_yaml[camera_frame]["focal_length"]
    
    def get_camera_pxpy(self, camera_frame: str) -> tuple[int, int]:
        return self.cameras_yaml[camera_frame]["px"], self.cameras_yaml[camera_frame]["py"]
        
        
    
    
    def compute_object_position(self, robot_pose: Pose, camera_quaternion: Quaternion, focal_length: int, x: int, y: int,
                                object_width: int, object_height: int, physical_width: float, 
                                physical_height: float, cx: int, cy: int) -> Pose:
        """Computes a physical position estimate for an object based on the position of the robot,
        where the camera is positioned relative to the orientation of the robot, the objects position
        and size in the camera frame, and its physical dimentions

        Args:
            robot_pose (Pose): position of the robot (usually obtained by subscribing to /pose)
            camera_quaterion (Quaternion): camera orientation relative to the robot's pose
            focal_length (int): Focal length of the camera
            x (int): x position of the center of the object in the camera
            y (int): y position of the center of the object in the camera
            object_width (int): width of the object in the camera
            object_height (int): height of the object in the camera
            physical_width (float): physical width of the object
            physical_height (float): physical height of the object

        Returns:
            Pose: physical position estimate of the object
        """

        # Estimate distance to object using similar triangles (based on height)
        if object_height > 0:
            distance = (physical_height * focal_length) / object_height
        else:
            self.get_logger().warn("No objects to estimate!")
            return Pose()

        # Compute object's center in image (assume image center is (cx, cy))
        dx = x + object_width / 2 - cx
        dy = y + object_height / 2 - cy

        # Convert image displacement to real-world coordinates in camera frame
        x_cam = (dx / focal_length) * distance
        y_cam = (dy / focal_length) * distance
        z_cam = distance  # Forward from camera

        # Object position in camera frame
        object_in_camera_frame = np.array([x_cam, y_cam, z_cam, 1.0])

        # Convert camera orientation (relative to robot) to transformation matrix
        cam_q = [camera_quaternion.x, camera_quaternion.y, camera_quaternion.z, camera_quaternion.w]
        T_camera_in_robot = tf_transformations.quaternion_matrix(cam_q)

        # Set translation if the camera has an offset from the robot (e.g., mounted above)
        # T_camera_in_robot[0:3, 3] = [self.camera_tx, self.camera_ty, self.camera_tz]  # Optional translation

        # Compute object's position in robot frame
        object_in_robot_frame = np.dot(T_camera_in_robot, object_in_camera_frame)

        # Convert robot orientation (in world frame) to transformation matrix
        robot_q = [robot_pose.orientation.x, robot_pose.orientation.y,
                robot_pose.orientation.z, robot_pose.orientation.w]
        T_robot_in_world = tf_transformations.quaternion_matrix(robot_q)
        T_robot_in_world[0:3, 3] = [robot_pose.position.x, robot_pose.position.y, robot_pose.position.z]

        # Compute object position in world frame
        object_in_world_frame = np.dot(T_robot_in_world, object_in_robot_frame)

        # Package result as a Pose
        object_pose = Pose()
        object_pose.position = Point(
            x=object_in_world_frame[0],
            y=object_in_world_frame[1],
            z=object_in_world_frame[2]
        )

        # Assume object has no orientation for now
        object_pose.orientation = Quaternion(x=0, y=0, z=0, w=1)

        return object_pose
        

def main(args=None):
    rclpy.init(args=args)
    
    estimator = ObjectEstimator()
    
    try:
        rclpy.spin(estimator)
    except KeyboardInterrupt:
        pass
    finally:
        estimator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()