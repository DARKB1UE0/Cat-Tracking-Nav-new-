from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
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
    
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file'
    )
    
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    
    # 配置参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('nav2_params_file')
    autostart = LaunchConfiguration('autostart')
    
    # 展开地图路径中的 ~ 为用户主目录
    expanded_map = PythonExpression([
        "__import__('os').path.expanduser('", map_file, "')"
    ])
    
    # 重写参数文件以支持地图文件路径
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            'use_sim_time': use_sim_time,
            'yaml_filename': expanded_map
        },
        convert_types=True
    )
    
    # Nav2 生命周期节点 - map_server 和 amcl 先启动
    lifecycle_nodes = [
        'map_server',
        'amcl',
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother'
    ]
    
    # 控制器服务（输出到 cmd_vel_nav，经 velocity_smoother 平滑后发到 cmd_vel_smoothed）
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=[('cmd_vel', 'cmd_vel_nav')]
    )
    
    # 平滑器服务 (输入 cmd_vel_nav -> 输出 cmd_vel_smoothed)
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        output='screen',
        parameters=[configured_params],
        remappings=[('cmd_vel', 'cmd_vel')] # Changed to direct cmd_vel output
    )
    
    # 速度指令反转器已移除
    # 直接使用 cmd_vel 与 Teleop 保持一致
    
    # 规划器服务
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        output='screen',
        parameters=[configured_params]
    )
    
    # 行为服务
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        output='screen',
        parameters=[configured_params],
        remappings=[('cmd_vel', 'cmd_vel_nav')]
    )
    
    # 行为树导航器
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=[configured_params]
    )
    
    # 航点跟随
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        output='screen',
        parameters=[configured_params]
    )
    
    # 速度平滑器
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel')
        ]
    )
    
    # 地图服务器
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[configured_params]
    )
    
    # AMCL 定位（启动后在 RViz 中用 2D Pose Estimate 设置初始位置）
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        output='screen',
        parameters=[configured_params]
    )
    
    # 生命周期管理器
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        map_file_arg,
        params_file_arg,
        autostart_arg,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,

        map_server,
        amcl,
        lifecycle_manager
    ])
