#include "hardware/hardware.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <cstring>
#include <string>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace hardware
{

OmniHardwareInterface::OmniHardwareInterface() = default;
OmniHardwareInterface::~OmniHardwareInterface() = default;

/**
 * @brief 硬件接口初始化回调函数
 *          URDF信息由变量info_传入
 *          根据URDF中配置的串口、波特率、CAN_ID、MST
 * ID、电机类型等参数，创建Motor_Control实例，并将其存储在motor_controls_中。
 *
 * @param info 硬件信息
 */
hardware_interface::CallbackReturn OmniHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // 先调用父类的初始化函数，如果父类初始化失败，则返回错误
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  // 打印初始化日志信息
  RCLCPP_INFO(rclcpp::get_logger("OmniHardwareInterface"), "Initializing Damiao Hardware Interface...");

  // 根据URDF中关节信息初始化运行时数据存储空间,默认一个电机一个关节
  hw_actuator_data_.resize(info_.joints.size());
  motor_controls_.resize(info_.joints.size());

  // --- STAGE 1: 根据URDF信息将关节按串口分组 ---

  for (uint i = 0; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];

    // --- 从urdf关节提取参数 ---
    std::string channel = joint.parameters.at("channel");
    int can_id = std::stoi(joint.parameters.at("can_id"));
    int motor_type_str = std::stoi(joint.parameters.at("motor_type"));

    // --- 把参数储存进达妙电机参数结构体 ---
    RsPackageData data;
    data.can_id = can_id;
    data.motorType = motor_type_str;
    data.joint_name = joint.name;

    try {
      // 注意：按关节顺序 i 存放，不能用 can_id 当下标（can_id 从 1 开始会越界）
      motor_controls_[i] = std::make_unique<RobStrideMotor>(
        channel, 0xFF, static_cast<uint8_t>(can_id), motor_type_str);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("OmniHardwareInterface"), "Failed to create Motor for ID '%d': %s", can_id,
        e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }


    // 每个关节电机数据写入对应hw_actuator_data_，hw_actuator_data_是通讯中用来存储和转移数据的最终归处
    hw_actuator_data_[i] = data;
  }

  // 打印初始化成功的日志信息
  RCLCPP_INFO(rclcpp::get_logger("OmniHardwareInterface"), "Hardware Interface initialized successfully.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> OmniHardwareInterface::export_state_interfaces()
{
  // 为每个关节添加位置、速度和力矩状态接口
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        hw_actuator_data_[i].joint_name, hardware_interface::HW_IF_POSITION,
        &hw_actuator_data_[i].state_pos));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        hw_actuator_data_[i].joint_name, hardware_interface::HW_IF_VELOCITY,
        &hw_actuator_data_[i].state_vel));

  }
  return state_interfaces;  // 返回状态接口向量
}

/**
 * @brief 导出命令接口
 *          对于每个关节，创建位置、速度、KP、KD和前馈力矩命令接口。
 *          这些接口允许控制器设置关节的目标状态。
 * @return 命令接口向量
 */
std::vector<hardware_interface::CommandInterface> OmniHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    // 为每个关节添加位置、速度、KP、KD和前馈力矩命令接口
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        hw_actuator_data_[i].joint_name, "position", &hw_actuator_data_[i].target_pos));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        hw_actuator_data_[i].joint_name, "velocity", &hw_actuator_data_[i].target_vel));
  }
  return command_interfaces;  // 返回命令接口向量
}

/**
 * @brief 激活硬件回调函数
 *          调用每个驱动器的enable函数，使其开始工作。
 * @param previous_state (自动传入)之前的生命周期状态
 */
 hardware_interface::CallbackReturn OmniHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("OmniHardwareInterface"), "Activating hardware...");  // 打印激活开始日志信息
  for (auto const &driver : motor_controls_)
  {
    // 先清故障: 电机内部的保护状态不掉电不清除,且不体现在状态帧故障位里,
    // 表现为"正常应答但拒绝执行速度指令"(实测右前轮就是这么趴窝的)
    try {
      driver->Disenable_Motor(1);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        rclcpp::get_logger("OmniHardwareInterface"), "Clear fault for motor %d failed: %s",
        driver->motor_id, e.what());
    }

    // 应答帧偶尔会超时丢失(0.1s 窗口),重试几次再判失败
    constexpr int kMaxRetries = 3;
    bool enabled = false;
    for (int attempt = 1; attempt <= kMaxRetries && !enabled; ++attempt) {
      try {
        driver->enable_motor();  // 使能每个驱动器
        enabled = true;
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          rclcpp::get_logger("OmniHardwareInterface"),
          "Enable motor %d failed (attempt %d/%d): %s",
          driver->motor_id, attempt, kMaxRetries, e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
    if (!enabled) {
      RCLCPP_ERROR(
        rclcpp::get_logger("OmniHardwareInterface"), "Enable motor %d failed after %d attempts",
        driver->motor_id, kMaxRetries);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(
    rclcpp::get_logger("OmniHardwareInterface"),
    "Hardware activated successfully.");               // 打印激活成功日志信息
  return hardware_interface::CallbackReturn::SUCCESS;  // 返回成功
}

/**
 * @brief 停用硬件回调函数
 *          调用每个驱动器的disable函数，使其停止工作。
 * @param previous_state (自动传入)之前的生命周期状态
 */
hardware_interface::CallbackReturn OmniHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("OmniHardwareInterface"), "Deactivating hardware...");  // 打印停用开始日志信息
  for (auto const &driver : motor_controls_)
  {
    try {
      driver->Disenable_Motor(0);  // 失能每个驱动器
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        rclcpp::get_logger("OmniHardwareInterface"), "Disable motor %d failed: %s",
        driver->motor_id, e.what());
    }
  }
  RCLCPP_INFO(
    rclcpp::get_logger("OmniHardwareInterface"),
    "Hardware deactivated successfully.");             // 打印停用成功日志信息
  return hardware_interface::CallbackReturn::SUCCESS;  // 返回成功
}

/**
 * @brief 读取硬件状态回调函数
 *          调用每个驱动器的read函数，读取硬件状态。
 *          将最新的状态从map同步到vector中。
 * @param time (自动传入)当前时间
 * @param period (自动传入)周期时间
 */
hardware_interface::return_type OmniHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 电机状态在每次发送指令后的应答帧中更新，这里只做同步
  for (uint i = 0; i < hw_actuator_data_.size(); ++i)
  {
    hw_actuator_data_[i].state_pos = motor_controls_[i]->position_;
    hw_actuator_data_[i].state_vel = motor_controls_[i]->velocity_;
  }

  return hardware_interface::return_type::OK;  // 返回OK表示读取成功
}

/**
 * @brief 写入硬件命令回调函数
 *          调用每个驱动器的write函数，将命令发送到硬件。
 *          将最新的命令从vector同步到map中。
 * @param time (自动传入)当前时间
 * @param period (自动传入)周期时间
 */
 hardware_interface::return_type OmniHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 把控制器写入的目标速度下发到每个电机
  for (uint i = 0; i < hw_actuator_data_.size(); i++)
  {
    double cmd = hw_actuator_data_[i].target_vel;
    if (!std::isfinite(cmd)) {
      cmd = 0.0;  // 控制器尚未下发指令时可能是 NaN，一律按停转处理
    }
    try {
      motor_controls_[i]->send_velocity_mode_command(static_cast<float>(cmd));
    } catch (const std::exception & e) {
      static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("OmniHardwareInterface"), steady_clock, 1000,
        "Motor %d write failed: %s", motor_controls_[i]->motor_id, e.what());
    }
  }
  return hardware_interface::return_type::OK;  // 返回OK表示写入成功
}

/**
 * @brief 配置硬件回调函数
 *          打印配置开始日志信息，并调用每个驱动器的configure函数，配置硬件。
 * @param previous_state (自动传入)之前的生命周期状态
 */
 hardware_interface::CallbackReturn OmniHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto cmd = export_command_interfaces();
  auto sta = export_state_interfaces();
  RCLCPP_INFO(
    rclcpp::get_logger("OmniHardwareInterface"), "==== HW provide %zu CMD  %zu STATE interfaces",
    cmd.size(), sta.size());

  for (const auto & ci : cmd)
  {
    const std::string & n = ci.get_name();
    std::ostringstream oss;
    for (unsigned char c : n)
      oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(c) << ' ';
    RCLCPP_INFO(
      rclcpp::get_logger("OmniHardwareInterface"), "HW CMD  '%s'  hex=[%s]", n.c_str(), oss.str().c_str());
  }

  for (const auto & si : sta)
  {
    const std::string & n = si.get_name();
    std::ostringstream oss;
    for (unsigned char c : n)
      oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(c) << ' ';
    RCLCPP_INFO(
      rclcpp::get_logger("OmniHardwareInterface"), "HW STATE '%s'  hex=[%s]", n.c_str(), oss.str().c_str());
  }
  RCLCPP_INFO(
    rclcpp::get_logger("OmniHardwareInterface"), "Configuring hardware...");  // 打印配置开始日志信息

  RCLCPP_INFO(
    rclcpp::get_logger("OmniHardwareInterface"),
    "Hardware configured successfully.");              // 打印配置成功日志信息
  return hardware_interface::CallbackReturn::SUCCESS;  // 返回成功
}

/**
 * @brief 清理硬件回调函数
 *          打印清理开始日志信息，并调用每个驱动器的clear函数，清理硬件。
 * @param previous_state (自动传入)之前的生命周期状态
 */
 hardware_interface::CallbackReturn OmniHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("OmniHardwareInterface"), "Cleaning up hardware...");  // 打印清理开始日志信息

  motor_controls_.clear();  // 清理所有电机控制实例
  RCLCPP_INFO(
    rclcpp::get_logger("OmniHardwareInterface"),
    "Hardware cleaned up successfully.");              // 打印清理成功日志信息
  return hardware_interface::CallbackReturn::SUCCESS;  // 返回成功
}

/**
 * @brief 关机回调函数：停用所有电机
 * @param previous_state (自动传入)之前的生命周期状态
 */
hardware_interface::CallbackReturn OmniHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_deactivate(previous_state);
}

/**
 * @brief 错误处理回调函数
 *          打印错误日志信息，并调用停用硬件函数。
 * @param previous_state (自动传入)之前的生命周期状态
 */
 hardware_interface::CallbackReturn OmniHardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_FATAL(
    rclcpp::get_logger("OmniHardwareInterface"),
    "Hardware has encountered an error. Deactivating...");  // 打印错误日志信息并停用硬件

  on_deactivate(rclcpp_lifecycle::State());            // 调用停用硬件函数
  return hardware_interface::CallbackReturn::SUCCESS;  // 返回成功
}


}  // namespace hardware


// 使用pluginlib导出该硬件接口，使其可以在ROS 2 Control中被加载
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hardware::OmniHardwareInterface, hardware_interface::SystemInterface)
