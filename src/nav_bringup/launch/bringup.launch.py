"""一键导航: 雷达驱动 + 定位链路 + Nav2。

    ros2 launch nav_bringup bringup.launch.py                 # PC 调试(带 RViz)
    ros2 launch nav_bringup bringup.launch.py rviz:=false     # NX 机上运行

底盘控制另开一个终端(自动探测 CAN 口):
    ros2 launch omni_base_control base_control.launch.py

TF 链: map --(kiss_matcher/gicp 重定位)--> odom --(FAST-LIO)--> base_link --(URDF)--> livox_frame
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription


def generate_launch_description():
    nav_share = get_package_share_directory('nav_bringup')
    livox_share = get_package_share_directory('livox_ros_driver2')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='是否打开 RViz'),

        # Mid360S 雷达驱动
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(livox_share, 'launch_ROS2', 'msg_MID360s_launch.py')),
        ),

        # FAST-LIO 里程计 + 重定位 + scan 生成
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, 'launch', 'localization.launch.py')),
        ),

        # Nav2 + 地图 + RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, 'launch', 'navigation.launch.py')),
            launch_arguments={'rviz': LaunchConfiguration('rviz')}.items(),
        ),
    ])
