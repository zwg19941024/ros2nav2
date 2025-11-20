import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 获取与拼接默认路径
    isaac_nav2_dir = get_package_share_directory('isaac_nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(
        nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # 创建 Launch 配置
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='false')
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(isaac_nav2_dir, 'maps', 'home.yaml'))
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(isaac_nav2_dir, 'config', 'nav2_navigation_only_3d.yaml'))
    
    # 可选的点云到激光雷达转换launch
    pointcloud_to_laserscan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('isaac_nav2'),
                'launch',
                'pointcloud_to_laserscan_launch.py'
            ])
        ])
    )

    return launch.LaunchDescription([
        # 声明新的 Launch 参数
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
                                             description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                                             description='Full path to param file to load'),
        launch.actions.DeclareLaunchArgument('use_lifecycle_mgr', default_value='true',
                                             description='Whether to use lifecycle manager'),
        launch.actions.DeclareLaunchArgument('autostart', default_value='true',
                                             description='Automatically startup the nav2 stack'),

        # 可选：启用点云到激光雷达转换（如果需要）
        pointcloud_to_laserscan_launch,

        # 启动地图服务器（仅地图服务器，不启动AMCL）
        launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_param_path, {'yaml_filename': map_yaml_path}],
        ),

        # 启动Nav2导航堆栈（不包含AMCL定位）
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/navigation_launch.py']),
            # 使用 Launch 参数替换原有参数
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path,
                'use_lifecycle_mgr': launch.substitutions.LaunchConfiguration('use_lifecycle_mgr'),
                'autostart': launch.substitutions.LaunchConfiguration('autostart')}.items(),
        ),

        # 启动生命周期管理器
        launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': launch.substitutions.LaunchConfiguration('autostart')},
                        {'node_names': ['map_server']}]),
        
        # 启动RViz2进行可视化
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),

        launch_ros.actions.Node(
            package='isaac_nav2',
            executable='nav2_mqtt_client',
            name='nav2_mqtt_client',
            ),


    ])