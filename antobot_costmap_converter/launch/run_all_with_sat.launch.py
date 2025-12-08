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

    custom = Node(
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
            'use_plugin_core': False,
            'converter_plugin': 'costmap_converter::CostmapToPolygonsDBSMCCH',
            'obstacles_topic': '/costmap_converter/obstacles_custom',
            'markers_topic': '/costmap_converter/polygon_markers_custom'
        }]
    )

    # compare = Node(
    #     package='antobot_costmap_converter',
    #     executable='compare_polygons.py',
    #     name='compare_polygons',
    #     output='screen'
    # )

    # footprint = Node(
    #     package='antobot_costmap_converter',
    #     executable='footprint_pub.py',
    #     name='footprint_pub',
    #     output='screen',
    #     parameters=[{'topic': '/costmap/footprint'}]
    # )

    sat = Node(
        package='antobot_costmap_converter',
        executable='collision_sat_checker',
        name='collision_sat_checker',
        output='screen',
        parameters=[{
            'footprint_topic': '/costmap/published_footprint',
            'inflation_factor': 2.0,
            'inflated_topic': '/costmap/inflated_footprint',
            'clusters_topic': '/costmap_converter/obstacles_custom'
        }]
    )

    # return LaunchDescription([bridge, custom, compare, footprint, sat])
    return LaunchDescription([bridge, custom, sat])