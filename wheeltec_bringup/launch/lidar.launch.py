from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包路径
    bringup_dir = get_package_share_directory('wheeltec_bringup')
    
    # 声明参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # MID360配置文件路径
    mid360_config = os.path.join(bringup_dir, 'config', 'MID360_config.json')
    
    # pointcloud_to_laserscan配置
    pc2ls_config = os.path.join(bringup_dir, 'config', 'pointcloud_to_laserscan.yaml')
    
    # Livox ROS2 驱动
    # 注意：需要先安装livox_ros_driver2
    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {'xfer_format': 0},           # 0: Pointcloud2
            {'multi_topic': 0},           # 0: 单话题
            {'data_src': 0},              # 0: 雷达数据
            {'publish_freq': 10.0},       # 发布频率
            {'output_type': 0},           # 0: 点云
            {'frame_id': 'laser_frame'},  # 坐标系
            {'lvx_file_path': ''},
            {'user_config_path': mid360_config},
            {'cmdline_input_bd_code': ''},
        ]
    )
    
    # pointcloud_to_laserscan节点
    # 使用 RELIABLE QoS 以兼容 slam_toolbox
    pc2ls_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        output='screen',
        parameters=[pc2ls_config],
        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('scan', '/scan')
        ],
        # 使用 --ros-args 强制设置 publisher QoS
        arguments=['--ros-args', '-p', 'reliability:=reliable']
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        livox_driver_node,
        pc2ls_node
    ])
