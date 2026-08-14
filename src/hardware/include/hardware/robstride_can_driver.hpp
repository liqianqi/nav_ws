/**
 * @file robstride_can_driver.hpp
 * @brief Robstride RS02 电机 CAN 通信驱动
 */

#ifndef HARDWARE__ROBSTRIDE_CAN_DRIVER_HPP_
#define HARDWARE__ROBSTRIDE_CAN_DRIVER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace hardware
{

struct RS02Params
{
  static constexpr double P_MIN = -12.57;
  static constexpr double P_MAX = 12.57;
  static constexpr double V_MIN = -33.0;
  static constexpr double V_MAX = 33.0;
  static constexpr double T_MIN = -17.0;
  static constexpr double T_MAX = 17.0;
};

struct MotorFeedback
{
  double position = 0.0;      // rad
  double velocity = 0.0;      // rad/s
  double torque = 0.0;        // Nm
  double temperature = 0.0;   // °C
  uint8_t mode_state = 0;     // 0:Reset 1:Cali 2:Motor
  uint8_t fault_code = 0;
  bool is_valid = false;
  std::chrono::steady_clock::time_point last_update;
};

/**
 * @brief 单 socket 共享驱动,管理 CAN 总线上所有 RS02 电机
 *
 * 线程安全:send_mutex_ 保护发送,feedback_mutex_ 保护反馈缓存。
 * 接收线程独立运行,write() 不阻塞。
 */
class RobstrideCanDriver
{
public:
  explicit RobstrideCanDriver(std::string can_interface, uint8_t host_can_id = 0xFD);
  ~RobstrideCanDriver();

  // 初始化 socket + 启动接收线程
  bool init();
  void close();

  // 电机控制(都不阻塞,返回是否成功发出 CAN 帧)
  bool enableMotor(uint8_t motor_id);
  bool disableMotor(uint8_t motor_id, bool clear_fault = false);
  bool setRunMode(uint8_t motor_id, uint8_t mode);            // 通信类型 18,写 0x7005(uint8)
  bool writeParameter(uint8_t motor_id, uint16_t index, float value);  // 通信类型 18
  bool writeParameterUint8(uint8_t motor_id, uint16_t index, uint8_t value);  // 通信类型 18, uint8 参数
  bool setVelocityCommand(uint8_t motor_id, float velocity);  // 通信类型 18,写 0x700A(速度模式)

  // 运控模式指令(通信类型 1)—— RS02 最可靠的控制通道
  // 底盘用 kp=0 + kd + velocity 实现速度控制(等效纯速度模式,但更可靠)
  bool sendMotionControl(uint8_t motor_id, double position, double velocity,
                         double kp, double kd, double torque = 0.0);

  // 读反馈(读缓存,不阻塞;缓存由接收线程持续更新)
  MotorFeedback getFeedback(uint8_t motor_id);

  // 诊断
  bool isConnected() const { return socket_fd_ >= 0; }
  uint64_t getSendRetryCount() const { return send_retry_count_.load(std::memory_order_relaxed); }
  uint64_t getSendFailCount() const { return send_fail_count_.load(std::memory_order_relaxed); }

  // CAN 通信健康状态:write 连续失败超过阈值时返回 false。
  // hardware_interface::write() 据此触发重连或停止
  bool isCanHealthy() const
  {
    return consecutive_fail_count_.load(std::memory_order_relaxed) < CAN_FAIL_THRESHOLD;
  }

private:
  // 创建+配置+绑定 socket(从 init() 抽出,供 reconnect 复用)
  bool setupSocket();

  // CAN 断线重连:关旧 socket → 等 dongle 重新枚举 → 开新 socket
  // 返回 true 表示重连成功(可以继续发数据)
  bool reconnect();
  // 协议 ID 构造(29 位扩展 ID)
  //   bit28-24: 通信类型
  //   bit23-8:  数据区 2(主机 CAN_ID / 力矩等)
  //   bit7-0:   目标电机 CAN_ID
  uint32_t buildCanId(uint8_t comm_type, uint16_t data_area2, uint8_t target_id);

  bool sendFrame(const can_frame & frame);
  void receiveThreadFunc();
  void parseFeedback(const can_frame & frame);

  std::string can_interface_;
  uint8_t host_can_id_;
  int socket_fd_ = -1;

  std::mutex send_mutex_;
  std::mutex feedback_mutex_;

  std::atomic<bool> receive_running_{false};
  std::thread receive_thread_;

  // 反馈缓存,按 motor_id 索引(0 不用,1-127 有效)
  static constexpr size_t MAX_MOTOR_ID = 128;
  std::vector<MotorFeedback> feedbacks_{MAX_MOTOR_ID};

  std::atomic<uint64_t> send_retry_count_{0};
  std::atomic<uint64_t> send_fail_count_{0};

  // CAN 写入连续失败计数。成功一帧就清零。
  // 超过阈值(CAN_FAIL_THRESHOLD)就判定 CAN 断了,触发紧急停止。
  static constexpr uint32_t CAN_FAIL_THRESHOLD = 20;  // 约 0.4 秒(50Hz × 20)
  std::atomic<uint32_t> consecutive_fail_count_{0};
};

}  // namespace hardware

#endif  // HARDWARE__ROBSTRIDE_CAN_DRIVER_HPP_
