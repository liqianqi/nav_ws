"""发布机器人 URDF 与静态 TF 链 (base_footprint -> base_link -> livox_frame)。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('nav_bringup')
    default_model = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = ParameterValue(
        Command(['xacro ', model]), value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model', default_value=default_model,
            description='机器人 xacro/urdf 路径'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='是否使用仿真时间'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
