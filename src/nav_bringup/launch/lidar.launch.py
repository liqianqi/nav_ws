"""MID360S 雷达链路：驱动出点云 -> 压成 2D /scan 供 Nav2 使用。

单独启动查看点云:
    ros2 launch nav_bringup lidar.launch.py

本机雷达是 MID360S（SDK 上报 dev_type=35），配置必须用 MID360s_config.json，
其顶层 key 为 "Mid360s"；用 MID360_config.json 会导致设备发现一直失败。
该配置的 host_net_info 是数组，多台雷达也用同一个文件，
加雷达时跑 nx_setup/04_config_mid360.sh 传入多个 IP 即可。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('nav_bringup')
    livox_share = get_package_share_directory('livox_ros_driver2')

    mid360_params = os.path.join(pkg_share, 'config', 'mid360.yaml')
    p2l_params = os.path.join(pkg_share, 'config', 'pointcloud_to_laserscan.yaml')
    default_lidar_config = os.path.join(
        livox_share, 'config', 'MID360s_config.json')

    lidar_config = LaunchConfiguration('lidar_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_scan = LaunchConfiguration('publish_scan')
    use_description = LaunchConfiguration('use_description')

    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_config', default_value=default_lidar_config,
            description='Livox SDK 配置文件，决定雷达型号与网络参数'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='是否使用仿真时间'),
        DeclareLaunchArgument(
            'publish_scan', default_value='true',
            description='是否把点云转成 /scan'),
        DeclareLaunchArgument(
            'use_description', default_value='true',
            description='是否发布机器人 TF。点云要投到 base_link 才能转 /scan，'
                        '单独跑雷达时必须为 true；bringup 里已单独启动则传 false'),

        # 提供 livox_frame -> base_link 的 TF，否则 pointcloud_to_laserscan
        # 转换失败，/scan 不会有数据
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'description.launch.py')),
            condition=IfCondition(use_description),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=[
                mid360_params,
                {'user_config_path': lidar_config,
                 'use_sim_time': use_sim_time},
            ],
        ),

        Node(
            condition=IfCondition(publish_scan),
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[p2l_params, {'use_sim_time': use_sim_time}],
            remappings=[
                ('cloud_in', '/livox/lidar'),
                ('scan', '/scan'),
            ],
        ),
    ])
