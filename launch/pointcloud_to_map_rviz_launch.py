from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bag_exec = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '-r', '1.0',
             #'/data/kitti/raw/2011_09_29_drive_0071_sync_bag', '--clock']
             '/data/kitti/raw/2011_09_30_drive_0018_sync_bag', '--clock']
             #'/data/kitti/raw/2011_09_30_drive_0028_sync_bag', '--clock']
    )

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('kitti_urdf'), 'launch', 'kitti_urdf_launch.py'
            ])
        ])
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', join(get_package_share_directory('pointcloud_to_map'),
                              'rviz', 'pointcloud_to_map.rviz')]
    )

    gps_shift_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gps_imu_node'), 'launch', 'gps_shift_launch.py'
            ]),
        ]),
    )

    imu_rotate_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gps_imu_node'), 'launch', 'imu_rotate_launch.py'
            ])
        ])
    )

    pointcloud_to_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('pointcloud_to_map'), 'launch', 'pointcloud_to_map_launch.py'
            ])
        ])
    )

    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        parameters=[
            {'frame_id': 'map'},
            {'base_frame_id': 'base_link'},
            {'resolution': 0.4}],
        remappings=[
            ('cloud_in', 'kitti/velo')],
        output='screen'
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        bag_exec,
        robot_state_publisher_launch,
        rviz_node,
        gps_shift_launch,
        imu_rotate_launch,
        pointcloud_to_map_launch,
        octomap_server
    ])
