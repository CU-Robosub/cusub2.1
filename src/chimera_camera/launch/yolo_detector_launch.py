from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chimera_camera',
            executable='yolo_node',
            name='yolo_node',
            output='screen',
            parameters=[
                {'model_path': 'src/chimera_camera/chimera_camera/weights.pt'},
                {'image_path': 'images.jpeg'},
                {'detection_topic': 'yolo/detections'}
            ],
            remappings=[
                ('image_raw', '/your_camera/image_raw')  # Adjust to your actual camera topic
            ]
        )
    ])
