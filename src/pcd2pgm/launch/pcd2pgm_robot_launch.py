"""可视化对比: 点云图 + pgm 栅格地图 + 机器人底盘模型(不做点云转换)。

用法:
    ros2 launch pcd2pgm pcd2pgm_robot_launch.py
    ros2 launch pcd2pgm pcd2pgm_robot_launch.py x:=1.0 y:=-2.0 yaw:=1.57   # 把底盘摆到地图别处
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription

MAPS_DIR = '/home/ubuntu/AI/nav_ws/src/nav_bringup/maps'


def generate_launch_description():
    pcd2pgm_dir = get_package_share_directory('pcd2pgm')
    nav_bringup_dir = get_package_share_directory('nav_bringup')

    robot_xacro = os.path.join(nav_bringup_dir, 'urdf', 'robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', robot_xacro]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='0.0', description='底盘在 map 中的 x'),
        DeclareLaunchArgument('y', default_value='0.0', description='底盘在 map 中的 y'),
        DeclareLaunchArgument('z', default_value='0.09', description='底盘 base_link 离地高度'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='底盘朝向(弧度)'),

        # pgm 栅格地图
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': os.path.join(MAPS_DIR, 'map.yaml')}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            parameters=[{'autostart': True, 'node_names': ['map_server']}],
        ),
        # 点云图(处理好的 map_cloud.pcd,与栅格图同坐标系)
        Node(
            package='pcl_ros',
            executable='pcd_to_pointcloud',
            name='pcd_publisher',
            parameters=[{
                'file_name': os.path.join(MAPS_DIR, 'map_cloud.pcd'),
                'tf_frame': 'map',
                'interval': 3.0,
            }],
        ),
        # 机器人底盘模型
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
        ),
        # 把底盘摆进 map 坐标系,位置可用 launch 参数调整
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', LaunchConfiguration('x'),
                       '--y', LaunchConfiguration('y'),
                       '--z', LaunchConfiguration('z'),
                       '--yaw', LaunchConfiguration('yaw'),
                       '--frame-id', 'map',
                       '--child-frame-id', 'base_link'],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pcd2pgm_dir, 'rviz', 'pcd2pgm_robot.rviz')],
            output='screen',
        ),
    ])
