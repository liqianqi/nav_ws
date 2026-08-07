"""slam_toolbox 在线建图，输入是 lidar.launch.py 转出的 /scan。

建完图保存:
    ros2 run nav2_map_server map_saver_cli -f ~/nav_ws/src/nav_bringup/maps/my_map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('nav_bringup')
    slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='是否使用仿真时间'),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {'use_sim_time': use_sim_time}],
        ),
    ])
