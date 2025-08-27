from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to your costmap YAML
    config_file = os.path.join(
        get_package_share_directory('antobot_nav2_costmap'),
        'config',
        'local_costmap.yaml'
    )

    return LaunchDescription([
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='costmap',        # ROS 2 node name
            output='screen',
            parameters=[config_file]
        )

    ])
