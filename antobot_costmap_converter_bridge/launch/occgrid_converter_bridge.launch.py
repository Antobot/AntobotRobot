#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bridge = Node(
        package='antobot_costmap_converter_bridge',
        executable='occgrid_converter_bridge',
        name='occgrid_converter_bridge',
        output='screen',
        parameters=[{
            'converter_plugin': 'costmap_converter::CostmapToPolygonsDBSMCCH',
            'occupied_min_value': 60,
            'cluster_max_distance': 0.4,
            'cluster_min_pts': 2,
            'cluster_max_pts': 30,
            'convex_hull_min_pt_separation': 0.1
        }]
    )

    return LaunchDescription([bridge])
