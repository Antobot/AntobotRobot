import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    
    # nav2_params = os.path.join(get_package_share_directory('antobot_navigation'), 'params', 'nav2_sim_params.yaml')
    controller_params = os.path.join(get_package_share_directory('antobot_navigation'), 'params', 'controller_params.yaml')
    planner_params = os.path.join(get_package_share_directory('antobot_navigation'), 'params', 'planners_params.yaml')
    costmap_common_sim_params = os.path.join(get_package_share_directory('antobot_navigation'),'params', 'costmap_common_sim_params.yaml')
    local_costmap_params = os.path.join(get_package_share_directory('antobot_navigation'), 'params', 'local_costmap_params.yaml')
    global_costmap_params = os.path.join(get_package_share_directory('antobot_navigation'), 'params', 'global_costmap_params.yaml')
    behaviour_server_params = os.path.join(get_package_share_directory('antobot_navigation'), 'params', 'behaviour_server_params.yaml')
    
    args = [
        DeclareLaunchArgument('cmd_vel_topic', default_value='/am_nav/cmd_vel'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_params],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_params],
    )

    local_costmap = Node(
        package='nav2_costmap_2d',
        executable='costmap_2d_node',
        name='local_costmap',
        parameters=[costmap_common_sim_params, local_costmap_params],
    )

    global_costmap = Node(
        package='nav2_costmap_2d',
        executable='costmap_2d_node',
        name='global_costmap',
        parameters=[costmap_common_sim_params, global_costmap_params],
    )

    behaviour_server = Node(
        package='nav2_behavior_server',
        executable='behavior_server',
        name='behaviour_server',
        output='screen',
        parameters=[behaviour_server_params],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'local_costmap',
                'global_costmap',
                'behaviour_server'
            ]
        }]
    )

    ld.add_action(args)
    ld.add_action(controller_server)
    ld.add_action(planner_server)
    ld.add_action(local_costmap)
    ld.add_action(global_costmap)
    ld.add_action(behaviour_server)
    ld.add_action(lifecycle_manager)
    return ld