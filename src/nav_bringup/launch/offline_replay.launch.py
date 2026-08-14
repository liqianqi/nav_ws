"""离线调试 launch:回放 bag 时复现完整定位链路(FAST-LIO + gicp + 可选 RViz)。

用法:
    # 1. 另一个终端先回放 bag(循环 + 慢放)
    ros2 bag play ~/nav_ws_recordings/recording_60s --clock --loop -r 0.5

    # 2. 本 launch(所有节点 use_sim_time:=true,和 bag 时钟同步)
    ros2 launch nav_bringup offline_replay.launch.py

和实车 launch 的区别:
    - 不启动雷达驱动(bag 提供数据)
    - 所有节点 use_sim_time:=true
    - FAST-LIO / gicp 参数和实车一致
    - gicp 会额外发布 /prior_map_cloud(白色)方便和 /cloud_registered(红色)对比
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('nav_bringup')
    fastlio_share = get_package_share_directory('fast_lio')

    fastlio_frames = os.path.join(nav_share, 'config', 'fastlio_frames.yaml')
    prior_pcd = os.path.join(nav_share, 'maps', 'map_cloud.pcd')

    return LaunchDescription([
        DeclareLaunchArgument('use_gicp', default_value='true',
                              description='是否启动 gicp 重定位(发布 /prior_map_cloud 供对比)'),

        # FAST-LIO:和实车 launch 参数完全一致,use_sim_time:=true 用 bag 时钟
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
                 'publish.publish_tf': True,
                 'use_sim_time': True},
            ],
        ),

        # gicp 重定位:和实车参数一致,会发布 /prior_map_cloud 到 map frame
        Node(
            package='gicp_relocalization',
            executable='gicp_relocalization_exec',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'num_threads': 2,
                'num_neighbors': 10,
                'global_leaf_size': 0.20,
                'registered_leaf_size': 0.20,
                'max_dist_sq': 0.36,
                'gicp_max_iterations': 50,
                'gicp_max_consecutive_failures': 3,
                'global.min_accum_sec': 3.0,
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
                'use_sim_time': True,
            }],
        ),
    ])
