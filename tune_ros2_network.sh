#!/usr/bin/env bash
# ROS 2 跨机传输调优。本地 PC 和 Orin NX 两边都要各跑一次。
#
#   SUDO_PASS=1 bash tune_ros2_network.sh
#   SUDO_PASS=1 WIFI_IFACE=wlP1p1s0 bash tune_ros2_network.sh
#
# 背景：MID360S 单帧点云约 400KB，远超默认 208KB 的 UDP 缓冲区，
# DDS 的分片重组会失败，表现为话题能发现、但点云一帧都收不到。
set -euo pipefail

SUDO_PASS="${SUDO_PASS:-1}"
WIFI_IFACE="${WIFI_IFACE:-}"
CONF=/etc/sysctl.d/60-ros2-dds.conf

# 先缓存凭证，后面就能直接用 sudo（管道喂密码会和 heredoc 抢 stdin）
echo "${SUDO_PASS}" | sudo -S -v 2>/dev/null

TMP=$(mktemp)
cat > "${TMP}" <<'EOF'
# ROS 2 (DDS) 大消息跨机传输：点云单帧数百 KB，需要更大的 socket 缓冲
net.core.rmem_max = 134217728
net.core.rmem_default = 16777216
net.core.wmem_max = 134217728
net.core.wmem_default = 16777216
# UDP 分片重组缓冲，默认 4MB 在 10Hz 点云下会溢出
net.ipv4.ipfrag_high_thresh = 134217728
net.ipv4.ipfrag_low_thresh = 100663296
# 分片超时调短，避免残片长期占用重组缓冲
net.ipv4.ipfrag_time = 3
EOF

echo "[tune] 写入 ${CONF}"
sudo install -m 644 "${TMP}" "${CONF}"
rm -f "${TMP}"

# 只加载这一个文件：sysctl --system 会因其他文件里的无效键中断
sudo sysctl -q -p "${CONF}"

echo "[tune] 生效值:"
sysctl net.core.rmem_max net.core.wmem_max net.ipv4.ipfrag_high_thresh \
  | sed 's/^/    /'

if [ -n "${WIFI_IFACE}" ]; then
  echo "[tune] 关闭 ${WIFI_IFACE} 省电（省电会带来几百 ms 的延迟抖动）"
  if command -v iw >/dev/null; then
    sudo iw dev "${WIFI_IFACE}" set power_save off || echo "    网卡不支持"
    echo -n "    当前: "; iw dev "${WIFI_IFACE}" get power_save || true
  else
    echo "    iw 未安装，改用 nmcli"
    # 连接名可能带空格，必须整行读取
    nmcli -t -g NAME,TYPE connection show \
      | awk -F: '$2=="802-11-wireless"{print $1}' \
      | while IFS= read -r c; do
          sudo nmcli connection modify "${c}" wifi.powersave 2 \
            && echo "    已设置连接 ${c}"
        done
  fi

  # NetworkManager 重连后会恢复省电，写默认值一并关掉
  TMP2=$(mktemp)
  printf '[connection]\nwifi.powersave = 2\n' > "${TMP2}"
  sudo install -m 644 "${TMP2}" /etc/NetworkManager/conf.d/wifi-powersave-off.conf
  rm -f "${TMP2}"
fi

echo "[tune] 完成"
