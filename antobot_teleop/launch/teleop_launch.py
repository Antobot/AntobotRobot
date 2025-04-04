#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    use_joy = DeclareLaunchArgument(
        'use_joy', default_value='true', description='Use joystick input'
    )
    use_keyboard = DeclareLaunchArgument(
        'use_keyboard', default_value='true', description='Use Keyboard input'
    )

    # Get the path to the YAML configuration file
    joy_remap_config = os.path.join(
        get_package_share_directory('antobot_teleop'), 
        'config',
        'gamepad.yaml'  
    )
    print(joy_remap_config)

    # Define nodes
    joy_node = Node(
        condition=IfCondition(LaunchConfiguration('use_joy')),
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'dev': '/dev/input/js0'}],
        remappings=[('joy', 'joy_orig')],
    )

    joy_remap_node = Node(
        condition=IfCondition(LaunchConfiguration('use_joy')),
        package='antobot_teleop',
        executable='joy_remap',
        name='joy_remap',
        remappings=[('joy_in', 'joy_orig'), ('joy_out', 'joy')],
        parameters=[joy_remap_config] 

    )

    teleop_node = Node(
        condition=IfCondition(LaunchConfiguration('use_keyboard')),
        package='antobot_teleop',
        executable='teleop',
        name='teleop',
        prefix='xterm -e',
        output='screen'
    )

    return LaunchDescription([
        use_joy,use_keyboard,       # Declare the launch argument
        joy_node,       # Start joy node if use_joy is true
        joy_remap_node, # Start joy remap node if use_joy is true
        teleop_node,    # Start the teleop node
    ])
