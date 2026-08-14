/**
 * @file hardware.cpp
 * @brief 全向底盘 ROS2 Control 硬件接口实现(单 socket 共享驱动)
 *
 * 相比旧版本(每电机一个 socket + 同步阻塞 receive):
 *   - 单 RobstrideCanDriver 服务所有电机
 *   - write 只发 CAN 帧,不等应答(不阻塞控制循环)
 *   - 反馈由独立接收线程异步更新到缓存,read 只读缓存
 *   - 速度模式 + 电流/加速度在 on_activate 一次性配置,运动中只发 0x700A
 */

#include "hardware/hardware.hpp"

#include <chrono>
#include <cmath>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hardware
{

// 运控模式相关常量
constexpr uint16_t PARAM_LIMIT_CUR = 0x7018;
constexpr uint8_t RUN_MODE_MOTION = 0;            // 运控模式(RS02 最可靠的控制通道)
constexpr float DEFAULT_CURRENT_LIMIT = 23.0f;    // A

// 运控模式速度控制的 PD 参数(全向轮:kp=0 纯速度跟踪,kd 提供阻尼)
constexpr double MOTION_KP = 0.0;     // 不跟踪位置(全向轮不需要位置控制)
constexpr double MOTION_KD = 2.0;     // 阻尼,值越大 CAN 断开时电机停得越快
constexpr double MOTION_TORQUE = 0.0; // 前馈力矩,底盘不需要

OmniHardwareInterface::OmniHardwareInterface() = default;
OmniHardwareInterface::~OmniHardwareInterface() = default;

hardware_interface::CallbackReturn OmniHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("OmniHardwareInterface"), "Initializing...");

  hw_actuator_data_.resize(info_.joints.size());
  for (uint i = 0; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];
    // 从 URDF 读取参数(兼容旧 URDF 的 channel / can_id / motor_type)
    std::string channel = joint.parameters.count("channel")
                            ? joint.parameters.at("channel") : "can0";
    int can_id = std::stoi(joint.parameters.at("can_id"));
    hw_actuator_data_[i].joint_name = joint.name;
    hw_actuator_data_[i].can_id = can_id;
    hw_actuator_data_[i].channel = channel;
  }

  // 所有电机共用一个 channel(URDF 里四个关节的 channel 应该相同)
  if (!hw_actuator_data_.empty()) {
    can_channel_ = hw_actuator_data_.front().channel;
  }

  RCLCPP_INFO(rclcpp::get_logger("OmniHardwareInterface"),
              "Initialized with %zu joints on channel %s",
              hw_actuator_data_.size(), can_channel_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> OmniHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (uint i = 0; i < hw_actuator_data_.size(); ++i) {
    interfaces.emplace_back(hw_actuator_data_[i].joint_name,
                            hardware_interface::HW_IF_POSITION,
                            &hw_actuator_data_[i].state_pos);
    interfaces.emplace_back(hw_actuator_data_[i].joint_name,
                            hardware_interface::HW_IF_VELOCITY,
                            &hw_actuator_data_[i].state_vel);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> OmniHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (uint i = 0; i < hw_actuator_data_.size(); ++i) {
    interfaces.emplace_back(hw_actuator_data_[i].joint_name,
                            hardware_interface::HW_IF_VELOCITY,
                            &hw_actuator_data_[i].target_vel);
  }
  return interfaces;
}

hardware_interface::CallbackReturn OmniHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("OmniHardwareInterface"), "Configuring...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OmniHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("OmniHardwareInterface"), "Activating...");

  // 创建并初始化驱动(单 socket + 接收线程)
  driver_ = std::make_unique<RobstrideCanDriver>(can_channel_, 0xFF);
  if (!driver_->init()) {
    RCLCPP_ERROR(rclcpp::get_logger("OmniHardwareInterface"),
                 "CAN driver init failed on %s", can_channel_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 按协议(RS02 说明书第41页)的速度模式激活流程:
  //   1. 失能状态下清故障
  //   2. 失能状态下设 run_mode = 2(速度模式)—— 协议要求模式切换在失能状态
  //   3. 使能(通信类型3)
  //   4. 使能后设 limit_cur / acc_rad(运行参数必须在使能后设才生效)
  //   5. 验证反馈
  // 之前的顺序(设模式→设参数→使能)导致参数在失能状态下被电机忽略,
  // 表现为"使能了但使不上劲"。
  for (const auto & data : hw_actuator_data_) {
    const uint8_t id = static_cast<uint8_t>(data.can_id);
    bool ok = false;
    constexpr int kMaxAttempts = 3;
    for (int attempt = 1; attempt <= kMaxAttempts && !ok; ++attempt) {
      // (1) 清故障:失能 + data[0]=1
      driver_->disableMotor(id, true);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));

      // (2) 设运控模式(mode=0,RS02 默认模式,最可靠)
      // 之前用速度模式(mode=2)但电机不执行速度指令,改用运控模式+sendMotionControl
      driver_->setRunMode(id, RUN_MODE_MOTION);
      std::this_thread::sleep_for(std::chrono::milliseconds(30));

      // (3) 使能
      driver_->enableMotor(id);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));

      // (4) 设电流限制(运控模式也需要,保护电机)
      driver_->writeParameter(id, PARAM_LIMIT_CUR, DEFAULT_CURRENT_LIMIT);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));

      // (5) 发一条零速运控指令,让电机进入"软保持"状态
      driver_->sendMotionControl(id, 0.0, 0.0, MOTION_KP, MOTION_KD, MOTION_TORQUE);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));

      // (5) 验证反馈
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      MotorFeedback fb = driver_->getFeedback(id);
      if (fb.is_valid) {
        ok = true;
        // mode_state=2 表示 Motor 模式(真正运行),0=Reset 还没使能
        RCLCPP_INFO(rclcpp::get_logger("OmniHardwareInterface"),
                    "Motor %d enabled (attempt %d): vel=%.2f temp=%.0f°C mode=%d fault=0x%x",
                    id, attempt, fb.velocity, fb.temperature, fb.mode_state, fb.fault_code);
      } else {
        RCLCPP_WARN(rclcpp::get_logger("OmniHardwareInterface"),
                    "Motor %d enable attempt %d/%d failed (no feedback)",
                    id, attempt, kMaxAttempts);
      }
    }
    if (!ok) {
      RCLCPP_ERROR(rclcpp::get_logger("OmniHardwareInterface"),
                   "Motor %d enable failed after %d attempts", id, kMaxAttempts);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // 等待首帧反馈到达
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  RCLCPP_INFO(rclcpp::get_logger("OmniHardwareInterface"), "Hardware activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OmniHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("OmniHardwareInterface"), "Deactivating...");
  if (driver_) {
    for (const auto & data : hw_actuator_data_) {
      driver_->disableMotor(static_cast<uint8_t>(data.can_id), false);
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OmniHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  driver_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OmniHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_deactivate(previous_state);
}

hardware_interface::CallbackReturn OmniHardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type OmniHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 读反馈缓存(接收线程已异步更新),纯内存操作,不阻塞
  for (auto & data : hw_actuator_data_) {
    MotorFeedback fb = driver_->getFeedback(static_cast<uint8_t>(data.can_id));
    if (fb.is_valid) {
      data.state_pos = fb.position;
      data.state_vel = fb.velocity;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OmniHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // CAN 不健康时,sendFrame 内部会自动重连。
  // 这里不返回 ERROR(那会让 ros2_control 直接退出,重连逻辑没机会执行)。
  // 而是:CAN 不健康时跳过发送(让 sendFrame 的重连逻辑在下一帧恢复),
  // 同时把 target_vel 清零(安全:CAN 断了不再加速)。
  // 只有 CAN 断开超过 30 秒(reconnect 在 sendFrame 里 5 秒一轮,6 轮都失败)
  // 才认为彻底断了,返回 ERROR。
  if (driver_ && !driver_->isCanHealthy()) {
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    static auto first_fail_time = std::chrono::steady_clock::now();
    static bool timer_started = false;

    if (!timer_started) {
      first_fail_time = std::chrono::steady_clock::now();
      timer_started = true;
    }

    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::steady_clock::now() - first_fail_time).count();

    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("OmniHardwareInterface"), steady_clock, 1000,
      "CAN 不健康,正在自动重连... (已等待 %ld 秒)", elapsed);

    // 超过 30 秒还没恢复,才真正放弃
    if (elapsed > 30) {
      RCLCPP_ERROR(rclcpp::get_logger("OmniHardwareInterface"),
                   "CAN 重连超过 30 秒仍失败,停止 ros2_control");
      return hardware_interface::return_type::ERROR;
    }

    // 否则继续尝试发送(sendFrame 里会触发 reconnect)
    // 用零速度发送(安全)
    for (const auto & data : hw_actuator_data_) {
      driver_->sendMotionControl(static_cast<uint8_t>(data.can_id),
                                 0.0, 0.0, MOTION_KP, MOTION_KD, MOTION_TORQUE);
    }
    return hardware_interface::return_type::OK;
  }

  // CAN 健康,正常发送
  static bool timer_reset = false;
  if (!timer_reset) {
    timer_reset = true;
  }

  for (const auto & data : hw_actuator_data_) {
    double cmd = data.target_vel;
    if (!std::isfinite(cmd)) {
      cmd = 0.0;
    }
    driver_->sendMotionControl(static_cast<uint8_t>(data.can_id),
                               0.0, cmd, MOTION_KP, MOTION_KD, MOTION_TORQUE);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hardware::OmniHardwareInterface, hardware_interface::SystemInterface)
