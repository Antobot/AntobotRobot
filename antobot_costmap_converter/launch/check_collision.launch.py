#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('antobot_costmap_converter')
    cfg = os.path.join(share, 'config', 'costmap_converter.yaml')

    bridge = Node(
        package='antobot_costmap_converter',
        executable='occgrid_converter_bridge',
        name='occgrid_converter_bridge',
        output='screen',
        parameters=[
            {'converter_plugin': 'costmap_converter::CostmapToPolygonsDBSMCCH'},
            cfg,
            {'publish_obstacle_markers': True}
        ]
    )

    collision_checker = Node(
        package='antobot_costmap_converter',
        executable='collision_checker',
        name='collision_checker',
        output='screen',
        parameters=[
            {'clusters_topic': '/costmap_converter/obstacles'},
            {'footprint_topic': '/costmap/published_footprint'},
            {'target_frame': 'base_link'},
            {'use_tf': False},
            {'footprint_line_width': 0.06},
            {'collision_line_width_factor': 2.0},
            {'publish_collision_polygon': True},
            {'publish_collision_marker': False}
        ]
    )

    return LaunchDescription([bridge, collision_checker])
