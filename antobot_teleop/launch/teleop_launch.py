#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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

    # TODO
    joystick = DeclareLaunchArgument(
        'joystick', default_value='POCKETT', description='Joystick Type'
    )

    # Get the path to the YAML configuration file
    joy_remap_config = os.path.join(
        get_package_share_directory('antobot_teleop'), 
        'config',
        'gamepad.yaml'  
    )

    # Define nodes
    joy_node = Node(
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('use_joy'), "' == 'true' and '", 
            LaunchConfiguration('joystick'), "' == 'LogitechF710'"
        ])),
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'dev': '/dev/input/js0'}],
        remappings=[('joy', 'joy_orig')],
    )
	
    joy_remap_node = Node(
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('use_joy'), "' == 'true' and '", 
            LaunchConfiguration('joystick'), "' == 'LogitechF710'"
        ])),
        package='antobot_teleop',
        executable='joy_remap',
        name='joy_remap',
        remappings=[('joy_in', 'joy_orig'), ('joy_out', 'joy')],
        parameters=[joy_remap_config] 

    )

    joy_elrs_node_node = Node(
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('use_joy'), "' == 'true' and '", 
            LaunchConfiguration('joystick'), "' == 'POCKETT'"
        ])),
        package='antobot_devices_joy',
        executable='joy_elrs_node',
        name='joy_elrs_node'
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
        use_joy,use_keyboard,joystick,       # Declare the launch argument
        joy_node,       # Start joy node if use_joy is true
        joy_remap_node, # Start joy remap node if use_joy is true
        joy_elrs_node_node,
        teleop_node,    # Start the teleop node
    ])
