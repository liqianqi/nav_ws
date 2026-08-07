import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch

################### user configure parameters for ros2 start ###################
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
rviz_config_path = os.path.join(cur_config_path, 'display_point_cloud_ROS2.rviz')
user_config_path = os.path.join(cur_config_path, 'MID360s_config.json')


def _resolve_user_config(template_path):
    """启动时把 host_net_info.host_ip 自动换成与雷达同网段的本机 IP（详见 msg_MID360s_launch.py）。"""
    import json
    import socket
    try:
        with open(template_path) as f:
            cfg = json.load(f)
        lidar_ip = cfg['lidar_configs'][0]['ip']
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect((lidar_ip, 56000))
            host_ip = s.getsockname()[0]
        finally:
            s.close()
        if host_ip.rsplit('.', 1)[0] != lidar_ip.rsplit('.', 1)[0]:
            return template_path
        changed = False
        for section in cfg.values():
            if isinstance(section, dict) and isinstance(section.get('host_net_info'), list):
                for host in section['host_net_info']:
                    if host.get('host_ip') != host_ip:
                        host['host_ip'] = host_ip
                        changed = True
        if not changed:
            return template_path
        out = os.path.join('/tmp', os.path.basename(template_path).replace('.json', '_runtime.json'))
        with open(out, 'w') as f:
            json.dump(cfg, f, indent=2)
        print(f'[livox launch] host_ip 自动设为 {host_ip} -> {out}')
        return out
    except Exception as e:
        print(f'[livox launch] host_ip 自动探测失败({e})，使用模板配置')
        return template_path


user_config_path = _resolve_user_config(user_config_path)
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]


def generate_launch_description():
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
        )

    livox_rviz = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['--display-config', rviz_config_path]
        )

    return LaunchDescription([
        livox_driver,
        livox_rviz,
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=livox_rviz,
        #         on_exit=[
        #             launch.actions.EmitEvent(event=launch.events.Shutdown()),
        #         ]
        #     )
        # )
    ])
