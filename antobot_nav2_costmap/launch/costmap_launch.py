import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
from antobot_com_postgresql.db_config_loader import get_robot_config

def generate_launch_description():

    platform_config = get_robot_config("platform_config")

    use_sim_time_value = not platform_config.get('robot_hardware', False)

    # Path to costmap YAML
    costmap_config_file = os.path.join(
        get_package_share_directory('antobot_nav2_costmap'),
        'config',
        'costmap_config.yaml'
    )

    with open(costmap_config_file, 'r') as f:
        costmap_config = yaml.safe_load(f)

    lidar_base_config = costmap_config['costmap/costmap']['ros__parameters']['lidar_base_conifg']

    observation_sources_list = []
    dynamic_params = {}
    for key, lidar_info in platform_config['lidar'].items():
        device_id = lidar_info['device_id']
        
        source_name = f"lidar_{device_id}"
        observation_sources_list.append(source_name)

        topic_name = f"/lidar_{device_id}/pointcloud"
        frame_name = f"lidar_{device_id}_frame"
        
        param_key = f"obstacle_layer.{source_name}"
        dynamic_params[param_key] = {
            **lidar_base_config,
            'topic': topic_name,
            'sensor_frame': frame_name
        }

    observation_sources_str = ' '.join(observation_sources_list)

    costmap_config_list = [ParameterFile(costmap_config_file, allow_substs=True),
        {
            'obstacle_layer.observation_sources': observation_sources_str,
            **dynamic_params,
            'use_sim_time': use_sim_time_value
        }
    ]

    # Costmap node
    costmap_node = Node(
        package='nav2_costmap_2d_test',
        executable='nav2_costmap_2d_test',
        name='costmap',
        output='screen',
        parameters=costmap_config_list
    )

    # Lifecycle manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap',
        output='screen',
        parameters=[{
            'autostart': True,          # bring nodes up automatically
            'node_names': ['costmap/costmap'],  # list of managed lifecycle nodes
            'bond_timeout': 0.0,       
        }]
    )

    return LaunchDescription([
        costmap_node,
        lifecycle_manager
    ])
