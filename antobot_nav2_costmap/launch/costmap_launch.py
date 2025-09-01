import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Path to your costmap YAML
    config_file = os.path.join(
        get_package_share_directory('antobot_nav2_costmap'),
        'config',
        'local_costmap.yaml'
    )

    # Costmap node
    costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='costmap',
        output='screen',
        parameters=[config_file]
    )

    # Lifecycle manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap',
        output='screen',
        parameters=[{
            'autostart': True,          # bring nodes up automatically
            'node_names': ['costmap/costmap'],  # list of managed lifecycle nodes
            'bond_timeout': 0.0,       
        }]
    )

    return LaunchDescription([
        costmap_node,
        lifecycle_manager
    ])
