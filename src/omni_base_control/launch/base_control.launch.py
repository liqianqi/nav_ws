"""全向轮底盘控制链路启动：

    controller_manager (加载 hardware/OmniHardwareInterface, CAN 直连电机)
      + joint_state_broadcaster (轮子转角 -> /joint_states)
      + wheel_velocity_controller (四轮速度组控制器)
      + omni_kinematics (/cmd_vel -> 四轮转速)

    ros2 launch omni_base_control base_control.launch.py
    ros2 launch omni_base_control base_control.launch.py can_channel:=can1  # 手动指定

CAN 通道默认自动探测（取第一个处于 UP 状态的 canX 口）：PC 是 can0，NX 是 can1，
两台机器共用同一份代码，不需要改文件。

启动后用键盘遥控:
    ros2 run nav_bringup teleop_keyboard.py
"""

import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def detect_can_channel(default='can0'):
    """返回第一个 UP 状态的 CAN 口名（按名字排序），一个都没有就返回默认值。"""
    for path in sorted(glob.glob('/sys/class/net/can*')):
        try:
            with open(os.path.join(path, 'flags')) as f:
                if int(f.read().strip(), 16) & 0x1:  # IFF_UP
                    return os.path.basename(path)
        except (OSError, ValueError):
            continue
    return default


def generate_launch_description():
    pkg_share = get_package_share_directory('omni_base_control')

    control_urdf = os.path.join(pkg_share, 'urdf', 'omni_control.urdf.xacro')
    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')

    can_default = detect_can_channel()
    print(f'[base_control] CAN 通道: {can_default} (可用 can_channel:= 覆盖)')

    robot_description = ParameterValue(
        Command(['xacro ', control_urdf,
                 ' can_channel:=', LaunchConfiguration('can_channel')]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('can_channel', default_value=can_default,
                              description='SocketCAN 接口名，默认自动探测'),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                controllers_yaml,
            ],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['wheel_velocity_controller'],
            output='screen',
        ),
        Node(
            package='omni_base_control',
            executable='omni_kinematics.py',
            name='omni_kinematics',
            output='screen',
        ),
    ])
