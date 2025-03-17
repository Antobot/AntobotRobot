#!/usr/bin/env python3
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    use_joy = DeclareLaunchArgument(
        'use_joy', default_value='true', description='Use joystick input'
    )

    # Define nodes
    joy_node = Node(
        condition=IfCondition(LaunchConfiguration('use_joy')),
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'dev': '/dev/input/js0'}],
        remappings=[('joy', 'joy_orig')]
    )

    # joy_remap_node = Node(
    #     condition=IfCondition(LaunchConfiguration('use_joy')),
    #     package='joy',
    #     executable='joy_remap.py',
    #     name='joy_remap',
    #     remappings=[('joy_in', 'joy_orig'), ('joy_out', 'joy')],
    #     #parameters=[{'gamepad_config': 'file:///path_to_your_gamepad.yaml'}]  # You can set the actual path here
    # )

    # teleop_node = Node(
    #     package='antobot_teleop',
    #     executable='teleop.py',
    #     name='teleop',
    #     output='screen'
    # )

    return LaunchDescription([
        use_joy,        # Declare the launch argument
        joy_node,       # Start joy node if use_joy is true
        #joy_remap_node, # Start joy remap node if use_joy is true
        #teleop_node,    # Start the teleop node
    ])
