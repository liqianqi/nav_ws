"""定位链路(不含雷达驱动、不含 Nav2):

    TF 链(仅此): map -> odom -> base_link -> livox_frame -> imu_link (+ 轮子)
    fastlio_mapping        直接发布 odom->base_link TF、/odom、/cloud_registered(odom)
    gicp_relocalization    C++ 多层 FFT 全局搜索 + GICP 验证/跟踪 -> map->odom
    pointcloud_to_laserscan /cloud_registered -> /scan (给 Nav2 costmap)
    robot_state_publisher  URDF TF (base_link->livox_frame / imu / 轮子)

单独调试:
    ros2 launch nav_bringup localization.launch.py
完整导航请用 bringup.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription


def generate_launch_description():
    nav_share = get_package_share_directory('nav_bringup')
    fastlio_share = get_package_share_directory('fast_lio')

    robot_xacro = os.path.join(nav_share, 'urdf', 'robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', robot_xacro]), value_type=str)

    prior_pcd = os.path.join(nav_share, 'maps', 'map_cloud.pcd')
    fastlio_frames = os.path.join(nav_share, 'config', 'fastlio_frames.yaml')

    return LaunchDescription([
        # URDF TF: base_link -> livox_frame / imu_link / 轮子
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
        ),

        # FAST-LIO 直接输出标准坐标：odom->base_link + /odom +
        # 稠密 /cloud_registered(odom)。本项目安装外参由 fastlio_frames.yaml 覆盖。
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            output='screen',
            parameters=[
                os.path.join(fastlio_share, 'config', 'mid360.yaml'),
                fastlio_frames,
                {'pcd_save.pcd_save_en': False,
                 'publish.path_en': False,
                 'publish.map_en': False,
                 'publish.dense_publish_en': True,
                 'publish.scan_imu_frame_pub_en': False,
                 'publish.publish_tf': True},
            ],
        ),

        # 纯 C++ 重定位：多高度层 FFT top-K 全局搜索 + 3D GICP 验证，
        # 成功后持续跟踪 map->odom；跟丢自动回到全局搜索。
        Node(
            package='gicp_relocalization',
            executable='gicp_relocalization_exec',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'num_threads': 4,
                'num_neighbors': 10,
                'global_leaf_size': 0.20,
                'registered_leaf_size': 0.20,
                'max_dist_sq': 0.36,
                'gicp_max_iterations': 50,
                'gicp_max_consecutive_failures': 3,
                'global.min_accum_sec': 8.0,
                'global.retry_period_sec': 5.0,
                'global.z_layers': [0.3, 0.8, 1.3, 1.8, 2.3],
                'global.grid_resolution': 0.10,
                'global.yaw_step_deg': 2.0,
                'global.match_threshold': 0.85,
                'global.top_k': 12,
                'global.peaks_per_yaw': 3,
                'global.peak_window_cells': 11,
                'global.candidate_min_translation': 0.7,
                'global.candidate_min_yaw_deg': 8.0,
                'global.min_occupied_cells': 1200,
                'validation_max_dist': 0.30,
                'candidate_min_inlier_ratio': 0.65,
                'candidate_max_rmse': 0.22,
                'candidate_ambiguity_ratio': 0.95,
                'candidate_max_correction_translation': 0.75,
                'candidate_max_correction_yaw_deg': 10.0,
                'tracking_min_inlier_ratio': 0.35,
                'tracking_max_rmse': 0.30,
                'tracking_max_jump_translation': 0.25,
                'tracking_max_jump_yaw_deg': 3.0,
                'force_planar': True,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'robot_base_frame': 'base_link',
                'prior_pcd_file': prior_pcd,
                'input_cloud_topic': '/cloud_registered',
            }],
        ),

        # 3D 点云 -> 2D scan,喂给 Nav2 costmap
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[os.path.join(nav_share, 'config', 'pointcloud_to_laserscan.yaml')],
            remappings=[
                ('cloud_in', '/cloud_registered'),
                ('scan', '/scan'),
            ],
        ),
    ])
