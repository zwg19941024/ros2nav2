from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # DeclareLaunchArgument(
        #     name='scanner', default_value='',
        #     description='Namespace for sample topics'
        # ),
        # 上面的 DeclareLaunchArgument 可以注释掉，如果不再使用 scanner
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/localization_current_lidar'),  # 直接指定输入话题
                        ('scan', '/scan')],           # 直接指定输出话题
            parameters=[{
                'target_frame': 'lidar',
                'transform_tolerance': 0.1,
                'min_height': 0.1,
                'max_height': 0.4,
                'angle_min': -3.14159,  # -M_PI/2
                'angle_max': 3.14159,  # M_PI/2
                'angle_increment': 0.007,  # M_PI/360.0
                'scan_time': 0.1,
                'range_min': 0.4,
                'range_max': 100.0,
                'use_inf': False,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])