import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_bringup')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    serial_port = LaunchConfiguration('serial_port')
    lidar_port = LaunchConfiguration('lidar_port')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0',
                               description='ESP32 serial port'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1',
                               description='RPLIDAR A1 serial port'),

        Node(
            package='robot_bringup',
            executable='serial_bridge_node',
            name='serial_bridge',
            output='screen',
            parameters=[{
                'serial_port': serial_port,
                'publish_tf': False,   # ekf_filter_node publishes odom->base_link instead
            }],
        ),

        Node(
            package='robot_bringup',
            executable='imu_node',
            name='mpu6050_node',
            output='screen',
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
        ),

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': lidar_port,
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame'],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link'],
        ),
    ])
