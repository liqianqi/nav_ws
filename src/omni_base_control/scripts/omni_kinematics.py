#!/usr/bin/env python3
"""X 形四全向轮运动学：/cmd_vel -> 四轮转速指令。

轮子布局(俯视,X 轴朝前):
    左前(1)   右前(2)
    左后(3)   右后(4)

每个轮的驱动方向沿 45° 对角线。对轮距为 2L x 2W、轮半径 R 的 X 形布局：
    w_fl = dir_fl * √2/2 * (vx - vy - (L+W)·wz) / R
    w_fr = dir_fr * √2/2 * (vx + vy + (L+W)·wz) / R
    w_rl = dir_rl * √2/2 * (vx + vy - (L+W)·wz) / R
    w_rr = dir_rr * √2/2 * (vx - vy + (L+W)·wz) / R
混控符号按 base_link 系(x 前 y 左 z 上, REP-103)推导。
motor_directions 与 CAN ID 映射(1=右后 2=左后 3=左前 4=右前)配套,
为 2026-08-06 实车标定结果。标定依据：前后直行时前后排对顶较劲而平移/旋转
正常 => 某排一对 CAN ID 左右接反；试换后排后 w/s 整体反向 => 实为前排接反,
故只交换前排 ID(3/4),dirs 保持不变。

安全：超过 cmd_timeout 没收到新的 cmd_vel 就发零速。
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class OmniKinematics(Node):

    def __init__(self):
        super().__init__('omni_kinematics')
        self.declare_parameter('wheel_radius', 0.075)      # m, 6寸全向轮 ow150
        self.declare_parameter('half_length', 0.178)       # m, 轮心到中心纵向距离 L
        self.declare_parameter('half_width', 0.178)        # m, 轮心到中心横向距离 W
        self.declare_parameter('motor_directions', [-1.0, 1.0, -1.0, 1.0])
        self.declare_parameter('max_wheel_speed', 20.0)    # rad/s, 电机上限 44
        self.declare_parameter('cmd_timeout', 0.5)         # s
        self.declare_parameter('publish_rate', 30.0)       # Hz

        self.radius = self.get_parameter('wheel_radius').value
        lw = (self.get_parameter('half_length').value +
              self.get_parameter('half_width').value)
        self.lw = lw
        self.dirs = list(self.get_parameter('motor_directions').value)
        self.max_speed = self.get_parameter('max_wheel_speed').value
        self.timeout = self.get_parameter('cmd_timeout').value

        self.cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.stopped = True

        self.pub = self.create_publisher(
            Float64MultiArray, '/wheel_velocity_controller/commands', 10)
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        period = 1.0 / self.get_parameter('publish_rate').value
        self.timer = self.create_timer(period, self.on_timer)
        self.get_logger().info(
            f'omni_kinematics 就绪: R={self.radius} L+W={lw} dirs={self.dirs}')

    def on_cmd(self, msg):
        self.cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def mix(self, vx, vy, wz):
        c = math.sqrt(2.0) / 2.0 / self.radius
        wheels = [
            c * (vx - vy - self.lw * wz),   # 左前
            c * (vx + vy + self.lw * wz),   # 右前
            c * (vx + vy - self.lw * wz),   # 左后
            c * (vx - vy + self.lw * wz),   # 右后
        ]
        # 超限时整体等比缩小,保持运动方向不变
        peak = max(abs(w) for w in wheels)
        if peak > self.max_speed:
            wheels = [w * self.max_speed / peak for w in wheels]
        return [w * d for w, d in zip(wheels, self.dirs)]

    def on_timer(self):
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.timeout:
            vx = vy = wz = 0.0
        else:
            vx = self.cmd.linear.x
            vy = self.cmd.linear.y
            wz = self.cmd.angular.z

        is_zero = (vx == 0.0 and vy == 0.0 and wz == 0.0)
        if is_zero and self.stopped:
            return  # 已停车则不再重复发零速
        msg = Float64MultiArray()
        msg.data = self.mix(vx, vy, wz)
        self.pub.publish(msg)
        self.stopped = is_zero


def main():
    rclpy.init()
    node = OmniKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前尽力发一次零速
        try:
            stop = Float64MultiArray()
            stop.data = [0.0, 0.0, 0.0, 0.0]
            node.pub.publish(stop)
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
