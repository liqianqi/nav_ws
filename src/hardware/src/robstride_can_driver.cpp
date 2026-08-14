/**
 * @file robstride_can_driver.cpp
 * @brief Robstride RS02 电机 CAN 驱动实现(单 socket + 异步接收)
 */

#include "hardware/robstride_can_driver.hpp"

#include <cerrno>
#include <cstring>
#include <iostream>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

namespace hardware
{

namespace
{
// 参数索引(来自 RS02 协议)
constexpr uint16_t PARAM_RUN_MODE = 0x7005;    // 运行模式
constexpr uint16_t PARAM_SPD_REF = 0x700A;     // 速度指令

// 通信类型
constexpr uint8_t COMM_MOTOR_ENABLE = 3;
constexpr uint8_t COMM_MOTOR_STOP = 4;
constexpr uint8_t COMM_SET_PARAM = 0x12;       // 单参数写入(18)
constexpr uint8_t COMM_FEEDBACK = 2;           // 电机反馈
constexpr uint8_t COMM_MOTION_CONTROL = 1;     // 运控模式指令

// 16 位无符号 ↔ 浮点线性映射
uint16_t floatToUint16(double x, double x_min, double x_max)
{
  if (x > x_max) x = x_max;
  if (x < x_min) x = x_min;
  double span = x_max - x_min;
  uint32_t raw = static_cast<uint32_t>((x - x_min) * 65535.0 / span + 0.5);
  return static_cast<uint16_t>(raw > 65535 ? 65535 : raw);
}

double uint16ToFloat(uint16_t x_int, double x_min, double x_max)
{
  double span = x_max - x_min;
  return static_cast<double>(x_int) * span / 65535.0 + x_min;
}
}  // namespace

RobstrideCanDriver::RobstrideCanDriver(std::string can_interface, uint8_t host_can_id)
: can_interface_(std::move(can_interface)), host_can_id_(host_can_id)
{
}

RobstrideCanDriver::~RobstrideCanDriver()
{
  receive_running_ = false;
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
  close();
}

bool RobstrideCanDriver::init()
{
  if (!setupSocket()) {
    return false;
  }

  // 启动接收线程
  receive_running_ = true;
  receive_thread_ = std::thread(&RobstrideCanDriver::receiveThreadFunc, this);

  std::cout << "[RobstrideCanDriver] 已在 " << can_interface_ << " 初始化" << std::endl;
  return true;
}

bool RobstrideCanDriver::setupSocket()
{
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    std::cerr << "[RobstrideCanDriver] socket 创建失败: " << std::strerror(errno) << std::endl;
    return false;
  }

  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    std::cerr << "[RobstrideCanDriver] 获取接口 " << can_interface_
              << " 索引失败: " << std::strerror(errno) << std::endl;
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(socket_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    std::cerr << "[RobstrideCanDriver] bind 失败: " << std::strerror(errno) << std::endl;
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // 接收超时
  struct timeval tv{};
  tv.tv_usec = 100000;  // 100ms
  setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  // 发送超时:防止 bus-off 时 write 永久阻塞
  struct timeval stv{};
  stv.tv_usec = 10000;  // 10ms
  setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, &stv, sizeof(stv));

  // 发送缓冲区
  int sndbuf = 8 * 1024;
  setsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

  // 过滤器:只收通信类型 2(反馈帧)
  struct can_filter rfilter;
  rfilter.can_id = (static_cast<uint32_t>(COMM_FEEDBACK) << 24) | CAN_EFF_FLAG;
  rfilter.can_mask = (0x1Fu << 24) | CAN_EFF_FLAG;
  setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

  return true;
}

bool RobstrideCanDriver::reconnect()
{
  // 关旧 socket
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }

  // 软件重置 CAN 接口:down → up
  // CAN 收发器芯片在频繁断连后可能卡死(能发不能收),ip link down/up
  // 能 reset CAN 控制器状态,相当于"软拔插",不需要物理拔插 canable
  static int reset_counter = 0;
  reset_counter++;
  std::cout << "[RobstrideCanDriver] 重置 CAN 接口 (第 " << reset_counter << " 次)..." << std::endl;
  std::string down_cmd = "ip link set " + can_interface_ + " down 2>/dev/null";
  std::string up_cmd = "ip link set " + can_interface_ +
    " up type can bitrate 1000000 sample-point 0.750 2>/dev/null";
  system(down_cmd.c_str());
  usleep(100000);  // 100ms 等 down 完成
  system(up_cmd.c_str());
  usleep(200000);  // 200ms 等 up 完成

  // 重新创建 socket
  for (int attempt = 0; attempt < 15; ++attempt) {
    if (setupSocket()) {
      consecutive_fail_count_.store(0, std::memory_order_relaxed);
      std::cout << "[RobstrideCanDriver] CAN 重连成功 (reset + socket 重建)" << std::endl;
      return true;
    }
    usleep(200000);  // 200ms
  }

  std::cerr << "[RobstrideCanDriver] CAN 重连失败(ip reset 后 3 秒内 socket 仍创建失败)" << std::endl;
  return false;
}

void RobstrideCanDriver::close()
{
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

uint32_t RobstrideCanDriver::buildCanId(uint8_t comm_type, uint16_t data_area2, uint8_t target_id)
{
  uint32_t id = 0;
  id |= (static_cast<uint32_t>(comm_type) & 0x1F) << 24;
  id |= (static_cast<uint32_t>(data_area2) & 0xFFFF) << 8;
  id |= target_id & 0xFF;
  return id | CAN_EFF_FLAG;
}

bool RobstrideCanDriver::sendFrame(const can_frame & frame)
{
  std::lock_guard<std::mutex> lock(send_mutex_);

  constexpr int max_retries = 2;
  constexpr int retry_delay_us = 50;
  for (int retry = 0; retry <= max_retries; ++retry) {
    if (socket_fd_ < 0) break;  // socket 已断
    ssize_t n = ::write(socket_fd_, &frame, sizeof(frame));
    if (n == sizeof(frame)) {
      consecutive_fail_count_.store(0, std::memory_order_relaxed);
      return true;
    }
    if (errno == ENOBUFS || errno == EAGAIN || errno == EWOULDBLOCK) {
      send_retry_count_.fetch_add(1, std::memory_order_relaxed);
      if (retry < max_retries) {
        struct timespec ts{};
        ts.tv_nsec = retry_delay_us * 1000L;
        clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, nullptr);
      }
      continue;
    }
    // ENODEV / 其他错误:socket 失效,累加失败计数
    send_fail_count_.fetch_add(1, std::memory_order_relaxed);
    uint32_t fails = consecutive_fail_count_.fetch_add(1, std::memory_order_relaxed) + 1;

    // 连续失败超过阈值 → 自动重连(USB dongle 断开重连后自动恢复)
    if (fails >= CAN_FAIL_THRESHOLD) {
      static auto last_reconnect = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      // 限制重连频率:至少间隔 2 秒,避免疯狂重连
      if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_reconnect).count() > 2000) {
        last_reconnect = now;
        std::cerr << "[RobstrideCanDriver] CAN 断开,尝试重连..." << std::endl;
        if (reconnect()) {
          // 重连成功,重试发这一帧
          n = ::write(socket_fd_, &frame, sizeof(frame));
          if (n == sizeof(frame)) {
            return true;
          }
        }
      }
    }
    return false;
  }
  send_fail_count_.fetch_add(1, std::memory_order_relaxed);
  consecutive_fail_count_.fetch_add(1, std::memory_order_relaxed);
  return false;
}

bool RobstrideCanDriver::enableMotor(uint8_t motor_id)
{
  can_frame frame{};
  frame.can_id = buildCanId(COMM_MOTOR_ENABLE, host_can_id_, motor_id);
  frame.can_dlc = 8;
  return sendFrame(frame);
}

bool RobstrideCanDriver::disableMotor(uint8_t motor_id, bool clear_fault)
{
  can_frame frame{};
  frame.can_id = buildCanId(COMM_MOTOR_STOP, host_can_id_, motor_id);
  frame.can_dlc = 8;
  if (clear_fault) {
    frame.data[0] = 1;
  }
  return sendFrame(frame);
}

bool RobstrideCanDriver::writeParameter(uint8_t motor_id, uint16_t index, float value)
{
  can_frame frame{};
  frame.can_id = buildCanId(COMM_SET_PARAM, host_can_id_, motor_id);
  frame.can_dlc = 8;
  // index 低字节在前
  frame.data[0] = index & 0xFF;
  frame.data[1] = (index >> 8) & 0xFF;
  // 浮点值小端序
  uint32_t raw;
  std::memcpy(&raw, &value, sizeof(float));
  frame.data[4] = raw & 0xFF;
  frame.data[5] = (raw >> 8) & 0xFF;
  frame.data[6] = (raw >> 16) & 0xFF;
  frame.data[7] = (raw >> 24) & 0xFF;
  return sendFrame(frame);
}

bool RobstrideCanDriver::setRunMode(uint8_t motor_id, uint8_t mode)
{
  // run_mode(0x7005)是 uint8 参数,不是 float。
  // 之前用 float 编码(mode=2 → 0x40000000)导致 data[4]=0x00(运控模式),
  // 电机一直停在运控模式,速度指令被忽略。
  return writeParameterUint8(motor_id, PARAM_RUN_MODE, mode);
}

bool RobstrideCanDriver::writeParameterUint8(uint8_t motor_id, uint16_t index, uint8_t value)
{
  // 通信类型 18,uint8 版本。值只占 data[4] 一个字节。
  can_frame frame{};
  frame.can_id = buildCanId(COMM_SET_PARAM, host_can_id_, motor_id);
  frame.can_dlc = 8;
  frame.data[0] = index & 0xFF;
  frame.data[1] = (index >> 8) & 0xFF;
  frame.data[4] = value;  // uint8 值放在 data[4]
  return sendFrame(frame);
}

bool RobstrideCanDriver::setVelocityCommand(uint8_t motor_id, float velocity)
{
  return writeParameter(motor_id, PARAM_SPD_REF, velocity);
}

bool RobstrideCanDriver::sendMotionControl(
  uint8_t motor_id, double position, double velocity,
  double kp, double kd, double torque)
{
  // 通信类型 1:运控模式电机控制指令(RS02 说明书第 33 页)
  // 29 位 ID: bit28-24=1(类型), bit23-8=力矩(16bit映射), bit7-0=motor_id
  // 8 字节数据: pos(2) + vel(2) + kp(2) + kd(2),全部 uint16 映射,高字节在前
  uint16_t torque_raw = floatToUint16(torque, RS02Params::T_MIN, RS02Params::T_MAX);
  uint16_t pos_raw = floatToUint16(position, RS02Params::P_MIN, RS02Params::P_MAX);
  uint16_t vel_raw = floatToUint16(velocity, RS02Params::V_MIN, RS02Params::V_MAX);
  uint16_t kp_raw = floatToUint16(kp, 0.0, 500.0);
  uint16_t kd_raw = floatToUint16(kd, 0.0, 5.0);

  can_frame frame{};
  frame.can_id = buildCanId(COMM_MOTION_CONTROL, torque_raw, motor_id);
  frame.can_dlc = 8;
  frame.data[0] = (pos_raw >> 8) & 0xFF;
  frame.data[1] = pos_raw & 0xFF;
  frame.data[2] = (vel_raw >> 8) & 0xFF;
  frame.data[3] = vel_raw & 0xFF;
  frame.data[4] = (kp_raw >> 8) & 0xFF;
  frame.data[5] = kp_raw & 0xFF;
  frame.data[6] = (kd_raw >> 8) & 0xFF;
  frame.data[7] = kd_raw & 0xFF;
  return sendFrame(frame);
}

void RobstrideCanDriver::receiveThreadFunc()
{
  can_frame frame{};
  while (receive_running_.load(std::memory_order_relaxed)) {
    // poll 10ms 超时,便于响应 stop 信号
    struct pollfd pfd;
    pfd.fd = socket_fd_;
    pfd.events = POLLIN;
    int ret = poll(&pfd, 1, 10);
    if (ret <= 0) continue;

    ssize_t n = ::read(socket_fd_, &frame, sizeof(frame));
    if (n != sizeof(frame)) continue;
    if (!(frame.can_id & CAN_EFF_FLAG)) continue;

    parseFeedback(frame);
  }
}

void RobstrideCanDriver::parseFeedback(const can_frame & frame)
{
  // 协议(通信类型 2 反馈帧 ID 结构,见 RS02 说明书第 33 页):
  //   bit28-24: 通信类型 0x02
  //   bit23-22: 模式状态
  //   bit21-16: 故障信息
  //   bit15-8:  当前电机 CAN_ID
  //   bit7-0:   主机 CAN_ID
  uint32_t can_id = frame.can_id & CAN_EFF_MASK;
  uint8_t comm_type = (can_id >> 24) & 0x1F;
  if (comm_type != COMM_FEEDBACK) return;

  uint8_t motor_id = (can_id >> 8) & 0xFF;
  if (motor_id == 0 || motor_id >= MAX_MOTOR_ID) return;

  // 数据区(高字节在前)
  uint16_t pos_raw = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
  uint16_t vel_raw = (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3];
  uint16_t torque_raw = (static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5];
  uint16_t temp_raw = (static_cast<uint16_t>(frame.data[6]) << 8) | frame.data[7];

  std::lock_guard<std::mutex> lock(feedback_mutex_);
  MotorFeedback & fb = feedbacks_[motor_id];
  fb.position = uint16ToFloat(pos_raw, RS02Params::P_MIN, RS02Params::P_MAX);
  fb.velocity = uint16ToFloat(vel_raw, RS02Params::V_MIN, RS02Params::V_MAX);
  fb.torque = uint16ToFloat(torque_raw, RS02Params::T_MIN, RS02Params::T_MAX);
  fb.temperature = static_cast<double>(temp_raw) / 10.0;
  fb.mode_state = (can_id >> 22) & 0x03;
  fb.fault_code = (can_id >> 16) & 0x3F;
  fb.is_valid = true;
  fb.last_update = std::chrono::steady_clock::now();
}

MotorFeedback RobstrideCanDriver::getFeedback(uint8_t motor_id)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  if (motor_id < MAX_MOTOR_ID) {
    return feedbacks_[motor_id];
  }
  return MotorFeedback{};
}

}  // namespace hardware
