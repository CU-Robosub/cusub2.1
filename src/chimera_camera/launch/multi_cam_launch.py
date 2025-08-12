from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Resolve the launch argument to a string, split into ints
    camera_ids_str = LaunchConfiguration('camera_ids').perform(context)
    camera_ids = camera_ids_str.split()

    # Create one camera_node per ID
    camera_nodes = [
        Node(
            package='chimera_camera',
            executable='camera_node',
            name=f'camera_{cam_id}_node',
            output='screen',
            arguments=[cam_id]
        )
        for cam_id in camera_ids
    ]

    # Coordinator node gets the same list as arguments
    coordinator_node = Node(
        package='chimera_camera',
        executable='coordinator_node',
        name='coordinator_node',
        output='screen',
        arguments=camera_ids
    )

    return camera_nodes + [coordinator_node]

def generate_launch_description():
    camera_ids_arg = DeclareLaunchArgument(
        'camera_ids',
        default_value='0 2 4 6',
        description='Space-separated list of camera IDs for the coordinator and camera nodes'
    )

    return LaunchDescription([
        camera_ids_arg,
        OpaqueFunction(function=launch_setup)
    ])
