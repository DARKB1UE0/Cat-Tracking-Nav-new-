from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('wheeltec_driver')
    
    # 参数文件路径
    default_params_file = os.path.join(pkg_dir, 'config', 'driver_params.yaml')
    
    # 声明参数
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the parameters file'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port device path'
    )
    
    # 驱动节点
    driver_node = Node(
        package='wheeltec_driver',
        executable='wheeltec_driver_node',
        name='wheeltec_driver_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'serial_port': LaunchConfiguration('serial_port')}
        ],
        remappings=[
            ('odom', '/odom'),
            ('cmd_vel', '/cmd_vel'),
            ('imu/data_raw', '/imu/data_raw'),
            ('battery_voltage', '/battery_voltage')
        ]
    )
    
    return LaunchDescription([
        params_file_arg,
        serial_port_arg,
        driver_node
    ])
