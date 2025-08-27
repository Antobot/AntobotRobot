from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the YAML configuration file
    config_dir = os.path.join(
        get_package_share_directory('antobot_nav2_costmap'), 
        'config'
    )

    # Full Nav2 parameter file that disables global costmap, planners, BT navigator
    nav2_params_file = os.path.join(config_dir, 'nav2_local_only_params.yaml')

    return LaunchDescription([
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[nav2_params_file,{'use_sim_time': True}]
        )
    ])
