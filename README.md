# nav_ws — ROS 2 全向底盘导航工作空间

基于 **MID360S 激光雷达 + FAST-LIO 里程计 + GICP 重定位 + Nav2** 的室内全向底盘建图 / 导航方案。

- 雷达：Livox MID360S（dev_type=35，需用 `MID360s_config.json`）
- 里程计：FAST-LIO（发布 `odom→base_link`，输出稠密 `/cloud_registered`）
- 重定位：`gicp_relocalization`（多高度层 FFT 全局搜索 + 3D GICP 精配，跟踪 `map→odom`）
- 导航：Nav2（`map_server + controller + planner + behaviors + velocity_smoother`）

TF 链：`map ──(gicp重定位)──▶ odom ──(FAST-LIO)──▶ base_link ──(URDF)──▶ livox_frame`

---

## 0. 前置准备

```bash
source /opt/ros/humble/setup.zsh
colcon build --symlink-install
source install/setup.zsh

# 一次性安装系统级依赖 small_gicp（编译并安装到系统）
bash third_party/install_deps.sh
```

---

## 1. 建图

本项目的“地图”由两部分组成，需要分别采集：

| 文件                          | 用途                | 来源                                              |
| ----------------------------- | ------------------- | ------------------------------------------------- |
| `maps/*.pgm` + `*.yaml`       | Nav2 二维全局代价图 | slam_toolbox                                      |
| `maps/map_cloud.pcd`          | GICP 重定位先验点云 | FAST-LIO 建图后用 `scripts/pcd2gridmap.py` 生成   |

### 1.1 二维栅格建图（slam_toolbox）

> 终端 1：雷达（驱动 + 压 `/scan`）

```bash
ros2 launch nav_bringup lidar.launch.py
```

> 终端 2：slam_toolbox 在线建图

```bash
ros2 launch nav_bringup slam.launch.py
```

> 终端 3：键盘遥控建图

```bash
ros2 run nav_bringup teleop_keyboard.py   # 或用底盘控制节点
```

建完保存（生成 `my_map.pgm` / `my_map.yaml`）：

```bash
ros2 run nav2_map_server map_saver_cli -f src/nav_bringup/maps/my_map
```

### 1.2 三维点云建图（FAST-LIO，作为重定位先验）

> 终端 1：FAST-LIO 单独建图（默认开启 `pcd_save`，退出时落到 `scans.pcd`）

```bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

完成后用脚本把 `scans.pcd` 扶正/裁切为重定位先验：

```bash
python3 src/nav_bringup/scripts/pcd2gridmap.py \
    scans.pcd \
    src/nav_bringup/maps/map_cloud.pcd
# 可选: --crop XMIN XMAX YMIN YMAX  在摆正坐标系下手动裁切(米)
# 可选: --no-yaw                    不做偏航摆正
```

> 注意：`map_cloud.pcd` 体积大（数十 MB），已被 `.gitignore` 排除，需在各机器上自行生成。

---

## 2. 导航

### 2.1 一键启动（雷达 + 定位 + Nav2）

```bash
# PC 调试（带 RViz）
ros2 launch nav_bringup bringup.launch.py

# NX 机上运行（不开 RViz）
ros2 launch nav_bringup bringup.launch.py rviz:=false
```

### 2.2 底盘控制（另开终端）

```bash
ros2 launch omni_base_control base_control.launch.py
```

### 2.3 指定地图

```bash
ros2 launch nav_bringup bringup.launch.py map:=src/nav_bringup/maps/my_map.yaml
```

---

## 3. 目录结构

```
nav_ws/
├── src/
│   ├── nav_bringup/         # 导航总装：launch / config / 地图 / URDF / 脚本
│   ├── gicp_relocalization/ # 多层 FFT 全局搜索 + GICP 重定位（自研）
│   ├── FAST_LIO_ROS2/       # FAST-LIO 里程计（ROS2 包名: fast_lio）
│   ├── omni_base_control/   # 全向底盘 CAN 控制
│   ├── hardware/            # 硬件层
│   ├── livox_ros_driver2/   # Livox MID360S 雷达驱动（第三方）
│   └── pcd2pgm/             # 点云转栅格工具（第三方）
├── third_party/
│   ├── small_gicp/          # GICP 算法库（第三方，系统级安装）
│   └── install_deps.sh      # 编译并安装 small_gicp
├── dds_env.sh               # ROS2 DDS 多机网络配置
├── sync_to_nx.sh            # 代码同步到 NX
└── tune_ros2_network.sh     # ROS2 网络调优
```

---

## 4. 常见问题

- **雷达发现失败**：本机是 MID360S（SDK 上报 `dev_type=35`），必须用 `MID360s_config.json`
  （顶层 key 是 `Mid360s`），用 `MID360_config.json` 会一直发现不到设备。
- **重定位不收敛**：检查 `maps/map_cloud.pcd` 是否已生成、与 `fastlio_frames.yaml` 外参是否一致。
- **NX 与 PC 多机通信**：先运行 `bash dds_env.sh` / `tune_ros2_network.sh` 配 DDS。

---

## 5. 关键 Launch 一览

| 命令                                              | 作用                                  |
| ------------------------------------------------- | ------------------------------------- |
| `ros2 launch nav_bringup lidar.launch.py`         | 仅雷达驱动 + 压 `/scan`               |
| `ros2 launch nav_bringup slam.launch.py`          | slam_toolbox 二维建图                 |
| `ros2 launch fast_lio mapping.launch.py`          | FAST-LIO 三维点云建图                 |
| `ros2 launch nav_bringup localization.launch.py`  | 仅定位链路（FAST-LIO + GICP + URDF）  |
| `ros2 launch nav_bringup navigation.launch.py`    | 仅 Nav2（不含定位 / 雷达）            |
| `ros2 launch nav_bringup bringup.launch.py`       | **一键导航**（雷达 + 定位 + Nav2）    |
| `ros2 launch omni_base_control base_control.launch.py` | 底盘 CAN 控制                    |
