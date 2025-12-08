#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    checker = Node(
        package='antobot_costmap_converter',
        executable='collision_sat_checker',
        name='collision_sat_checker',
        output='screen',
        parameters=[{
            'footprint_topic': '/costmap/footprint',
            'clusters_topic': '/costmap_converter/obstacles_custom'
        }]
    )

    return LaunchDescription([checker])
