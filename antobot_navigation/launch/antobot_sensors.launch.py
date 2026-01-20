import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    gps_correction_node = Node(
        name="gpsCorrection",
        package="antobot_devices_gps",
        executable="gps_corrections.py",
        output='screen',
        condition=IfCondition('true'),
    )

    gps_manager_node = Node(
        name="gpsManager",
        package="antobot_devices_gps",
        executable="gpsManager.py",
        output='screen',
        condition=IfCondition('true'),
    )

    imu_launch_file = os.path.join(
        get_package_share_directory('am_bno055_imu'),
        'launch',
        'am_bno055_imu.launch.py' # Must be a .launch.py in ROS 2
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(imu_launch_file),
        condition=IfCondition('true'),
    )


    lidar_manager_node = Node(
        name="lidarManager",
        package="antobot_devices_lidar",
        executable="lidar_manager.py",
        output='screen',
        condition=IfCondition('true'),
    )

    ld.add_action(gps_correction_node)
    ld.add_action(gps_manager_node)
    ld.add_action(imu_launch)
    ld.add_action(lidar_manager_node)
    return ld