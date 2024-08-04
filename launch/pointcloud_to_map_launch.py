from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pointcloud_to_map_node = Node(
        package='pointcloud_to_map',
        executable='pointcloud_to_map_node',
        name='pointcloud_to_map_node'
    )

    return LaunchDescription([
        pointcloud_to_map_node
    ])
