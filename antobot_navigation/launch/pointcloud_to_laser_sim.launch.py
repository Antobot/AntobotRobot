from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    back_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_back',
        output='screen',
        remappings=[
            ('cloud_in', '/mobile_base/zed2_back/point_cloud/cloud_registered'),
            ('scan', '/camera/scan/back'),
        ],
        parameters=[{
            'target_frame': 'zed2_back_camera_center',
            'transform_tolerance': 0.01,
            'min_height': -0.2,
            'max_height': 1.0,

            'angle_min': -3.14,
            'angle_max': 3.14,
            'angle_increment': 0.0087,
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 20.0,
            'use_inf': True,
            
            #concurrency_level affects number of pc queued for processing and the number of threadsused
            # 0: Detect number of cores
            # 1: Single threaded
            # 2: inf : Parallelism level
            'concurrency_level': 1,
        }]
    )

    front_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_front',
        output='screen',
        remappings=[
            ('cloud_in', '/mobile_base/zed2_front/point_cloud/cloud_registered'),
            ('scan', '/camera/scan/front'),
        ],
        parameters=[{
            'target_frame': 'zed2_front_camera_center',
            'transform_tolerance': 0.01,
            'min_height': -0.2,
            'max_height': 1.0,

            'angle_min': -3.14,
            'angle_max': 3.14,
            'angle_increment': 0.0087,
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 20.0,
            'use_inf': True,
            
            #concurrency_level affects number of pc queued for processing and the number of threadsused
            # 0: Detect number of cores
            # 1: Single threaded
            # 2: inf : Parallelism level
            'concurrency_level': 1,
        }]
    )

    ld.add_action(back_laserscan)
    ld.add_action(front_laserscan)
    return ld
