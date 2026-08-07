/**
 * @file hardware.hpp
 * @brief 全向轮 ROS2 Control 硬件接口
 */

#ifndef EL_A3_HARDWARE__EL_A3_HARDWARE_HPP_
#define EL_A3_HARDWARE__EL_A3_HARDWARE_HPP_

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "hardware/motor_cfg.h"

namespace hardware
{

struct RsPackageData
{
  std::string joint_name;
  int motorType = 0;
  int can_id = 0;
  // 必须零初始化: 激活瞬间控制器还没下发指令, 未初始化的值会直接发给电机
  double target_pos = 0.0, target_vel = 0.0;
  double state_pos = 0.0, state_vel = 0.0;
};

/**
  * @brief EL-A3 的 ROS2 Control 硬件接口
  */
class OmniHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OmniHardwareInterface)

  OmniHardwareInterface();
  ~OmniHardwareInterface() override;

  // SystemInterface 接口
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // CAN 驱动
  std::vector<std::unique_ptr<RobStrideMotor>> motor_controls_;

  std::vector<RsPackageData> hw_actuator_data_;

};

}  // namespace hardware

#endif  // EL_A3_HARDWARE__EL_A3_HARDWARE_HPP_
