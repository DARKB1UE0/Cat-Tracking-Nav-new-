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
    params_file = LaunchConfiguration('nav2_params_file')
    autostart = LaunchConfiguration('autostart')
    
    # 重写参数文件 (不需要 yaml_filename)
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            'use_sim_time': use_sim_time,
        },
        convert_types=True
    )
    
    # Nav2 生命周期节点 - 不包含 map_server 和 amcl (由 slam_toolbox 替代建图和定位)
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother'
    ]
    
    # 控制器服务
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=[('cmd_vel', 'cmd_vel_nav')]
    )
    
    # 平滑器服务
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        output='screen',
        parameters=[configured_params],
        remappings=[('cmd_vel', 'cmd_vel')] 
    )
    
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
    
    # SLAM参数文件
    slam_params_file = os.path.join(bringup_dir, 'config', 'slam_params.yaml')
    qos_overrides_file = os.path.join(bringup_dir, 'config', 'qos_overrides.yaml')
    
    # slam_toolbox节点
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        arguments=[
            '--ros-args',
            '--params-file', qos_overrides_file
        ]
    )
    
    # 启动 RViz2
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'navigation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
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
        params_file_arg,
        autostart_arg,
        
        # 启动底层底盘节点与雷达节点 (通过 bringup 统一管理)
        # 将 mode 设为 slam_nav，避免原 bringup 重复启动独立的 slam/nav 节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'bringup.launch.py')),
            launch_arguments={'mode': 'slam_nav'}.items()
        ),
        
        # 本 launch 文件内部负责管理 SLAM 和 导航 相关节点 :
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager,
        
        # SLAM 建图与定位节点
        slam_toolbox_node,
        
        # RViz 界面
        rviz_node
    ])