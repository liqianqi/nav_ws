#!/usr/bin/env bash
# 把本地 nav_ws 的源码同步到 Orin NX 并重新编译。
# 在本地 PC 上运行: bash sync_to_nx.sh [--no-build]
set -euo pipefail

NX_HOST="${NX_HOST:-ubuntu@192.168.110.100}"
NX_PASS="${NX_PASS:-1}"
LOCAL_DIR="$(cd "$(dirname "$0")" && pwd)"

SSH_OPTS="-o StrictHostKeyChecking=no"
export SSHPASS="${NX_PASS}"

echo "[sync] 同步 src/ 与 nx_setup/ 到 ${NX_HOST}"
sshpass -e rsync -a --delete -e "ssh ${SSH_OPTS}" \
  --exclude '__pycache__' \
  "${LOCAL_DIR}/src/nav_bringup" "${NX_HOST}:~/nav_ws/src/"
sshpass -e rsync -a -e "ssh ${SSH_OPTS}" \
  "${LOCAL_DIR}/nx_setup/" "${NX_HOST}:~/nx_setup/"

# start_nav.sh 要 source ~/nav_ws/dds_env.sh，两边共用同一份
sshpass -e rsync -a -e "ssh ${SSH_OPTS}" \
  "${LOCAL_DIR}/dds_env.sh" "${LOCAL_DIR}/tune_ros2_network.sh" \
  "${NX_HOST}:~/nav_ws/"

if [ "${1:-}" = "--no-build" ]; then
  echo "[sync] 跳过编译"
  exit 0
fi

echo "[sync] 在 NX 上编译"
sshpass -e ssh ${SSH_OPTS} "${NX_HOST}" 'bash -lc "
set +u; source /opt/ros/humble/setup.bash; set -u
cd ~/nav_ws
colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
"'

echo "[sync] 完成"
