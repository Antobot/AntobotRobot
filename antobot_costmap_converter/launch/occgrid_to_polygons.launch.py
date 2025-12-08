#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    node = Node(
        package='antobot_costmap_converter',
        executable='occgrid_to_polygons',
        name='occgrid_to_polygons',
        output='screen',
        parameters=[{
            'occupied_min_value': 60,
            'min_area_cells': 3,
            'dilate_kernel': 1,
            'approx_epsilon_cells': 1.0,
            'cluster_max_distance': 0.4,
            'cluster_min_pts': 2,
            'cluster_max_pts': 30,
            'convex_hull_min_pt_separation': 0.1,
            'use_costmap2d': True,
            'obstacles_topic': '/costmap_converter/obstacles_custom',
            'markers_topic': '/costmap_converter/polygon_markers_custom'
        }]
    )
    return LaunchDescription([node])
