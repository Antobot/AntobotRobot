from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    args = [
        DeclareLaunchArgument('device_ip1', default_value='192.168.1.200'),
        DeclareLaunchArgument('msop_port1', default_value='2368'),
        DeclareLaunchArgument('difop_port1', default_value='2369'),
        DeclareLaunchArgument('device_ip2', default_value='192.168.1.201'),
        DeclareLaunchArgument('msop_port2', default_value='2370'),
        DeclareLaunchArgument('difop_port2', default_value='2371'),
        DeclareLaunchArgument('lidar_type1', default_value='c32'),
        DeclareLaunchArgument('lidar_type2', default_value='c16'),
        DeclareLaunchArgument('c32_type', default_value='c32_70'),
        DeclareLaunchArgument('pcl_type', default_value='false'),
        DeclareLaunchArgument('use_gps_ts', default_value='false'),
        DeclareLaunchArgument('packet_rate1', default_value='1695.0'),
        DeclareLaunchArgument('packet_rate2', default_value='1695.0'),
    ]

    c32_node = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        namespace='c32',
        output='screen',
        parameters=[{
            'packet_rate': LaunchConfiguration('packet_rate1'),
            'device_ip': LaunchConfiguration('device_ip1'),
            'msop_port': LaunchConfiguration('msop_port1'),
            'difop_port': LaunchConfiguration('difop_port1'),
            'pcl_type': LaunchConfiguration('pcl_type'),
            'lidar_type': LaunchConfiguration('lidar_type1'),
            'add_multicast': False,
            'group_ip': '224.1.1.2',
            'c32_type': LaunchConfiguration('c32_type'),
            'use_gps_ts': LaunchConfiguration('use_gps_ts'),
            'min_range': 0.15,
            'max_range': 200.0,
            'frame_id': 'laser_link_c32',
            'distance_unit': 0.4,
            'angle_disable_min': 0,
            'angle_disable_max': 0,
            'horizontal_angle_resolution': 0.18,
            'scan_num': 10,
            'read_once': False,
            'publish_scan': False,
            'pointcloud_topic': 'lslidar_point_cloud',
            'coordinate_opt': False,
        }]
    )

    c16_node = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        namespace='c16',
        output='screen',
        parameters=[{
            'packet_rate': LaunchConfiguration('packet_rate2'),
            'device_ip': LaunchConfiguration('device_ip2'),
            'msop_port': LaunchConfiguration('msop_port2'),
            'difop_port': LaunchConfiguration('difop_port2'),
            'pcl_type': LaunchConfiguration('pcl_type'),
            'lidar_type': LaunchConfiguration('lidar_type2'),
            'add_multicast': False,
            'group_ip': '224.1.1.2',
            'use_gps_ts': LaunchConfiguration('use_gps_ts'),
            'min_range': 0.15,
            'max_range': 200.0,
            'frame_id': 'laser_link_c16',
            'distance_unit': 0.4,
            'angle_disable_min': 0,
            'angle_disable_max': 0,
            'horizontal_angle_resolution': 0.18,
            'scan_num': 10,
            'read_once': False,
            'publish_scan': False,
            'pointcloud_topic': 'lslidar_point_cloud',
            'coordinate_opt': False,
        }]
    )

    ld.add_action(args)
    ld.add_action(c32_node)
    ld.add_action(c16_node)
    return ld