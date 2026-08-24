import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_bringup')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    serial_port = LaunchConfiguration('serial_port')
    lidar_port = LaunchConfiguration('lidar_port')
    imu_flip_z = LaunchConfiguration('imu_flip_z')
    forward_only = LaunchConfiguration('forward_only')
    disable_tank_turns = LaunchConfiguration('disable_tank_turns')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/USB1',
                               description='ESP32 serial port'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/USB0',
                               description='RPLIDAR A1 serial port'),
        DeclareLaunchArgument('imu_flip_z', default_value='false',
                               description='Flip gyro Z sign if heading turns the wrong way'),
        DeclareLaunchArgument('forward_only', default_value='false',
                               description='Block reverse cmd_vel commands'),
        DeclareLaunchArgument('disable_tank_turns', default_value='false',
                               description='Disable in-place tank turns'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='robot_bringup',
            executable='serial_bridge_node',
            name='serial_bridge',
            output='screen',
            parameters=[{
                'port': serial_port,
                'imu_flip_z': imu_flip_z,
                'forward_only': forward_only,
                'disable_tank_turns': disable_tank_turns,
            }],
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
    ])
