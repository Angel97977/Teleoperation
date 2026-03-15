from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('imu_xarm_controller'),
        'config', 'master_slave_drag.yaml'
    )
    return LaunchDescription([
        Node(
            package='imu_xarm_controller',
            executable='master_slave',
            name='master_slave_node',
            output='screen',
            parameters=[config]
        )
    ])
