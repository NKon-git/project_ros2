from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    params = os.path.join(
        get_package_share_directory('car_bringup'),
        'config',
        'params.yaml'
    )

    lidar_node = Node(
        package='sensors',
        executable='lidar_node',
        name='lidar_node',
        parameters=[params]
    )

    ultrasound_node = Node(
        package='sensors',
        executable='ultrasound_node',
        name='ultrasound_node',
        parameters=[params]
    )

    control_node = Node(
        package='car_ctrl',
        executable='control_node',
        name='control_node',
        parameters=[params]
    )

    return LaunchDescription([
        lidar_node,
        ultrasound_node,
        control_node,
    ])

