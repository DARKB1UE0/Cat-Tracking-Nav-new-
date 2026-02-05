from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包路径
    bringup_dir = get_package_share_directory('wheeltec_bringup')
    driver_dir = get_package_share_directory('wheeltec_driver')
    description_dir = get_package_share_directory('wheeltec_description')
    
    # 声明参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='slam',
        description='Mode: slam (for mapping) or nav (for navigation)'
    )
    
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file (required for nav mode)'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for wheeltec driver'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    # 获取参数值
    use_sim_time = LaunchConfiguration('use_sim_time')
    mode = LaunchConfiguration('mode')
    map_file = LaunchConfiguration('map')
    serial_port = LaunchConfiguration('serial_port')
    rviz_enabled = LaunchConfiguration('rviz')
    
    # RViz配置文件
    slam_rviz_config = os.path.join(bringup_dir, 'rviz', 'slam.rviz')
    nav_rviz_config = os.path.join(bringup_dir, 'rviz', 'navigation.rviz')
    
    # 1. 机器人描述
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_dir, 'launch', 'description.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # 2. 底盘驱动
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(driver_dir, 'launch', 'driver.launch.py')
        ),
        launch_arguments={'serial_port': serial_port}.items()
    )
    
    # 3. 激光雷达
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'lidar.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # 4a. SLAM模式
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'slam.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'slam'"]))
    )
    
    # 4b. 导航模式
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file
        }.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'nav'"]))
    )
    
    # 5a. SLAM模式RViz
    rviz_slam = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', slam_rviz_config],
        condition=IfCondition(
            PythonExpression(["'", rviz_enabled, "' == 'true' and '", mode, "' == 'slam'"])
        )
    )
    
    # 5b. 导航模式RViz
    rviz_nav = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', nav_rviz_config],
        condition=IfCondition(
            PythonExpression(["'", rviz_enabled, "' == 'true' and '", mode, "' == 'nav'"])
        )
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        mode_arg,
        map_arg,
        serial_port_arg,
        rviz_arg,
        description_launch,
        driver_launch,
        lidar_launch,
        slam_launch,
        nav_launch,
        rviz_slam,
        rviz_nav
    ])
