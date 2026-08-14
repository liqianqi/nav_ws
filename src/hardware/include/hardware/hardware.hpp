/**
 * @file hardware.hpp
 * @brief 全向轮 ROS2 Control 硬件接口
 */

#ifndef HARDWARE__HARDWARE_HPP_
#define HARDWARE__HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware/robstride_can_driver.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace hardware
{

struct RsPackageData
{
  std::string joint_name;
  int can_id = 0;
  std::string channel;
  // 零初始化:激活瞬间控制器还没下发指令,未初始化的值会直接发给电机
  double target_vel = 0.0;
  double state_pos = 0.0;
  double state_vel = 0.0;
};

/**
 * @brief 全向底盘 ROS2 Control 硬件接口
 *        所有电机共享一个 RobstrideCanDriver
 */
class OmniHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OmniHardwareInterface)

  OmniHardwareInterface();
  ~OmniHardwareInterface() override;

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
  std::unique_ptr<RobstrideCanDriver> driver_;
  std::string can_channel_{"can0"};

  std::vector<RsPackageData> hw_actuator_data_;
};

}  // namespace hardware

#endif  // HARDWARE__HARDWARE_HPP_
