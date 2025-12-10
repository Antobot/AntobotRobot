#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers.on_process_start import OnProcessStart
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
            {'footprint_line_width': 0.06},
            {'publish_collision_polygon': True},
            {'collision_strategy': "swept"}
        ]
    )

    # Delay start of `collision_checker` after `bridge`
    delay_collision_checker_node = RegisterEventHandler(
        OnProcessStart(
            target_action=bridge,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[collision_checker]
                )
            ]
        )
    )

    return LaunchDescription([bridge, delay_collision_checker_node])
