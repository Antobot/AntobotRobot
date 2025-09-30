#!/usr/bin/env python3

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    # get joystick type
    joystick_type = 'LogitechF710'
    use_keyboard = True

    try:
        package_path = get_package_share_directory('antobot_description')
        config_file = os.path.join(package_path, 'config', 'platform_config.yaml')

        with open(config_file, 'r') as file:
            params = yaml.safe_load(file)

        joystick_type = params["teleop"]["joystick"]
        use_keyboard = params["teleop"]["keyboard"]
    except:
        pass

    #print(joystick_type)

    if joystick_type == "LogitechF710":
        joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'dev': '/dev/input/js0'}],
            remappings=[('joy', 'joy_orig')]
        )
            
        joy_remap_config = os.path.join(
            get_package_share_directory('antobot_teleop'), 
            'config',
            'gamepad.yaml'  
        )

        joy_remap_node = Node(
            package='antobot_teleop',
            executable='joy_remap',
            name='joy_remap',
            remappings=[('joy_in', 'joy_orig'), ('joy_out', 'joy')],
            parameters=[joy_remap_config] 

        )

        ld.add_action(joy_node)
        ld.add_action(joy_remap_node)

    elif joystick_type == "Pocket":
        joy_elrs_node_node = Node(
            package='antobot_devices_joy',
            executable='joy_elrs_node',
            name='joy_elrs_node',
            parameters=[{'dev': '/dev/ttyUSB0'}]
        )
        ld.add_action(joy_elrs_node_node)
   
    if use_keyboard:
        teleop_node = Node(
            package='antobot_teleop',
            executable='teleop',
            name='teleop',
            prefix='xterm -e',
            output='screen'
        )
        ld.add_action(teleop_node)
    else:
        teleop_node = Node(
            package='antobot_teleop',
            executable='teleop',
            name='teleop',
            parameters=[{'use_keyboard': False}],
            output='screen'
        )
        ld.add_action(teleop_node)

    return ld