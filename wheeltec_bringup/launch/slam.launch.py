from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
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
    
    # SLAM参数文件
    slam_params_file = os.path.join(bringup_dir, 'config', 'slam_params.yaml')
    
    # QoS 覆盖文件
    qos_overrides_file = os.path.join(bringup_dir, 'config', 'qos_overrides.yaml')
    
    # slam_toolbox节点
    # 使用 BEST_EFFORT QoS 订阅 /scan 以兼容 pointcloud_to_laserscan
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        arguments=[
            '--ros-args',
            '--params-file', qos_overrides_file
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        slam_toolbox_node
    ])
