"""Nav2 导航(不含定位链路):

    map_server + nav2 各服务器(controller/planner/behaviors/bt/velocity_smoother)
    定位由 localization.launch.py 提供 map->odom->base_link TF。

    ros2 launch nav_bringup navigation.launch.py
    ros2 launch nav_bringup navigation.launch.py rviz:=false   # 机上(NX)不开 RViz
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    nav_share = get_package_share_directory('nav_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    params_file = os.path.join(nav_share, 'config', 'nav2_params.yaml')
    map_yaml = os.path.join(nav_share, 'maps', 'map.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='是否打开 RViz'),
        DeclareLaunchArgument('map', default_value=map_yaml, description='地图 yaml 路径'),

        # Nav2 导航组件(不含 AMCL,定位由重定位包提供)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': 'False',
                'autostart': 'True',
            }.items(),
        ),

        # 静态地图(供 global costmap 使用)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file,
                        {'yaml_filename': LaunchConfiguration('map')}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'autostart': True, 'node_names': ['map_server']}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(nav_share, 'rviz', 'navigation.rviz')],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
