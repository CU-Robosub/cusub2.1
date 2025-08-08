from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    camera_nodes = [
        Node(
            package='chimera_camera',
            executable='camera_node',
            name=f'camera_{i}_node',
            output='screen',
            arguments=[str(i)]
        )
        for i in range(4)
    ]


    return LaunchDescription(
        camera_nodes + [
            Node(
                package='chimera_camera',
                executable='coordinator_node',
                name='coordinator_node',
                output='screen'
            )
        ]
    )
