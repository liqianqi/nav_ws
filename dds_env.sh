# 让 ROS 2 能跨 WiFi 跨机通信的 DDS 环境。用 source 引入，两边都要用。
#
#   source dds_env.sh                  # NX 上的节点
#   SUPER_CLIENT=1 source dds_env.sh   # PC 上的 rviz2 / ros2 CLI
#
# DS_IP 指向跑 discovery server 的机器（默认 NX），PEER_IP 用来反查本机
# 该用哪张网卡（默认取 DS_IP）。
#
# 踩过的三个坑，结论都写在这里：
#  1. 默认 FastDDS 靠组播发现。WiFi 组播按最低速率(6Mbit/s)转发且不重传，
#     NX 上跑满 Nav2 时 PC 侧一个话题都发现不到；只跑一个 talker 时能看到
#     话题名却拿不到类型，是典型的 endpoint 发现数据丢失。
#  2. 改用 CycloneDDS 关组播走单播，talker 通了，但完整 Nav2 栈有 20 多个
#     参与者，单播要逐个端口扫，40~60 秒只发现 19/53 个话题，/map 始终收不到。
#  3. 换 Discovery Server 后，抓包显示 PC 一直发、NX 收到 41 个包却回 0 个：
#     两台机器都有多张网卡（PC 上有 clash 的 TUN 和 docker0，NX 上有雷达
#     专用的 192.168.1.5），FastDDS 把所有地址都广告出去，对端挑中了不可达
#     的那个来回复。所以必须用 interfaceWhiteList 把网卡钉死。
#     白名单要配 useBuiltinTransports=false，那会挡掉组播 —— 但 Discovery
#     Server 本来就不需要组播，两者正好互补。

DS_IP="${DS_IP:-192.168.110.100}"
DS_PORT="${DS_PORT:-11811}"
_PEER="${PEER_IP:-${DS_IP}}"

# 顺着到对端的路由反查本机地址，避开 clash 的 TUN、docker0 和雷达网段
_LOCAL_IP=$(ip -o -4 route get "${_PEER}" 2>/dev/null | grep -oP 'src \K[0-9.]+')

if [ -z "${_LOCAL_IP}" ]; then
  echo "[dds] 查不到到 ${_PEER} 的路由，两台机器在同一网段吗？" >&2
else
  _CONF="/tmp/fastdds_${_LOCAL_IP}.xml"
  cat > "${_CONF}" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>udp_lan</transport_id>
        <type>UDPv4</type>
        <!-- 只在这张网卡上收发，也只把这个地址广告给对端。
             127.0.0.1 必须一起放进来：同一台机器上的节点走 loopback，
             只留网卡地址会把本机通信也掐断（实测话题数从 53 掉到 2）。 -->
        <interfaceWhiteList>
          <address>${_LOCAL_IP}</address>
          <address>127.0.0.1</address>
        </interfaceWhiteList>
      </transport_descriptor>
      <transport_descriptor>
        <transport_id>shm_local</transport_id>
        <type>SHM</type>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="lan_only" is_default_profile="true">
      <rtps>
        <!-- 本机节点之间走共享内存，跨机才走 UDP -->
        <userTransports>
          <transport_id>shm_local</transport_id>
          <transport_id>udp_lan</transport_id>
        </userTransports>
        <!-- 必须关掉内建传输，否则默认 UDP 传输会绕过白名单 -->
        <useBuiltinTransports>false</useBuiltinTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
EOF

  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE="${_CONF}"
  export ROS_DISCOVERY_SERVER="${DS_IP}:${DS_PORT}"
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  export ROS_LOCALHOST_ONLY=0
  unset CYCLONEDDS_URI 2>/dev/null || true

  # 普通节点只发现自己用得到的对端；rviz2 和 ros2 CLI 要列出全部话题，
  # 得以 SUPER_CLIENT 身份拿到服务器上的完整发现信息
  if [ "${SUPER_CLIENT:-0}" = "1" ]; then
    export ROS_SUPER_CLIENT=True
    echo "[dds] ${_LOCAL_IP} -> discovery server ${DS_IP}:${DS_PORT} (super client)"
  else
    unset ROS_SUPER_CLIENT 2>/dev/null || true
    echo "[dds] ${_LOCAL_IP} -> discovery server ${DS_IP}:${DS_PORT}"
  fi
fi

unset _PEER _LOCAL_IP _CONF
