#!/usr/bin/env python3
"""全向轮键盘遥控：按住才动，松开立刻停。

    w/s : 前进 / 后退   (+x / -x)
    a/d : 左平移 / 右平移 (+y / -y)
    q/e : 左旋 / 右旋   (+yaw / -yaw)
    +/- : 加减速
    Ctrl+C 退出（退出前发零速）

实现：优先用 evdev 直接读内核键盘事件（/dev/input），能拿到精确的
按下/松开事件，松开立即停，还支持组合键（如 w+q 边走边转）。
需要用户在 input 组（本机已满足）。注意 evdev 是全局监听，
切到别的窗口打字时也会触发，测试完记得退出。

evdev 不可用时回退为终端模式（依赖键盘自动重复，松开约 0.5s 停）。

用法:
    ros2 run nav_bringup teleop_keyboard.py
    ros2 run nav_bringup teleop_keyboard.py --ros-args -p linear_speed:=0.3
"""

import select
import sys
import termios
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

try:
    import evdev
    from evdev import ecodes
    HAS_EVDEV = True
except ImportError:
    HAS_EVDEV = False

# key -> (x, y, yaw) 方向系数
KEY_MAP = {
    'w': (1.0, 0.0, 0.0),
    's': (-1.0, 0.0, 0.0),
    'a': (0.0, 1.0, 0.0),
    'd': (0.0, -1.0, 0.0),
    'q': (0.0, 0.0, 1.0),
    'e': (0.0, 0.0, -1.0),
}

HELP = """--- 全向轮键盘遥控 ---
  w/s: 前进/后退    a/d: 左移/右移    q/e: 左旋/右旋
  +/-: 加减速    Ctrl+C: 退出
"""


class TeleopKeyboard(Node):

    def __init__(self):
        super().__init__('teleop_keyboard')
        self.declare_parameter('linear_speed', 0.2)    # m/s
        self.declare_parameter('angular_speed', 0.5)   # rad/s
        self.declare_parameter('key_timeout', 0.5)     # s，仅终端回退模式使用
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.lin = self.get_parameter('linear_speed').value
        self.ang = self.get_parameter('angular_speed').value

    # ---------- 公共 ----------

    def publish(self, direction, moving_flag):
        """direction=(x,y,yaw)系数; moving_flag=[bool] 记录是否在动，用于只发一次零速"""
        is_zero = direction == (0.0, 0.0, 0.0)
        if is_zero and not moving_flag[0]:
            return
        msg = Twist()
        msg.linear.x = direction[0] * self.lin
        msg.linear.y = direction[1] * self.lin
        msg.angular.z = direction[2] * self.ang
        self.pub.publish(msg)
        moving_flag[0] = not is_zero

    def adjust_speed(self, factor):
        self.lin *= factor
        self.ang *= factor
        print(f'\r速度: {self.lin:.2f} m/s, {self.ang:.2f} rad/s   ', end='', flush=True)

    # ---------- evdev 模式：真实按下/松开事件 ----------

    @staticmethod
    def find_keyboards():
        keyboards = []
        for path in evdev.list_devices():
            try:
                dev = evdev.InputDevice(path)
                keys = dev.capabilities().get(ecodes.EV_KEY, [])
                if ecodes.KEY_W in keys and ecodes.KEY_SPACE in keys:
                    keyboards.append(dev)
            except (PermissionError, OSError):
                continue
        return keyboards

    def run_evdev(self):
        code_map = {
            ecodes.KEY_W: 'w', ecodes.KEY_S: 's',
            ecodes.KEY_A: 'a', ecodes.KEY_D: 'd',
            ecodes.KEY_Q: 'q', ecodes.KEY_E: 'e',
        }
        keyboards = self.find_keyboards()
        if not keyboards:
            return False
        print(HELP)
        print(f'[evdev 模式] 松开立即停，支持组合键。键盘: '
              f'{", ".join(k.name for k in keyboards)}')
        print(f'当前速度: 线速度 {self.lin:.2f} m/s, 角速度 {self.ang:.2f} rad/s')

        pressed = set()
        moving = [False]
        fd_map = {k.fd: k for k in keyboards}
        # 屏蔽终端回显，避免 wasd 打进 shell
        old_attr = None
        if sys.stdin.isatty():
            old_attr = termios.tcgetattr(sys.stdin)
            quiet = termios.tcgetattr(sys.stdin)
            quiet[3] &= ~(termios.ICANON | termios.ECHO)
            termios.tcsetattr(sys.stdin, termios.TCSANOW, quiet)
        try:
            while rclpy.ok():
                r, _, _ = select.select(list(fd_map) + [sys.stdin.fileno()], [], [], 0.05)
                for fd in r:
                    if fd == sys.stdin.fileno():
                        ch = sys.stdin.read(1)
                        if ch in ('+', '='):
                            self.adjust_speed(1.1)
                        elif ch in ('-', '_'):
                            self.adjust_speed(0.9)
                        continue
                    for event in fd_map[fd].read():
                        if event.type != ecodes.EV_KEY or event.code not in code_map:
                            continue
                        key = code_map[event.code]
                        if event.value == 1:      # 按下
                            pressed.add(key)
                        elif event.value == 0:    # 松开
                            pressed.discard(key)
                        # value==2 是自动重复，忽略

                direction = tuple(
                    sum(KEY_MAP[k][i] for k in pressed) for i in range(3))
                # 同轴对键同按(如 w+s)相消为 0；限幅到 [-1,1]
                direction = tuple(max(-1.0, min(1.0, v)) for v in direction)
                self.publish(direction, moving)
        finally:
            if old_attr is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
            self.pub.publish(Twist())
            print('\n已退出，已发送零速指令。')
        return True

    # ---------- 终端回退模式：依赖键盘自动重复 ----------

    def run_termios(self):
        import tty
        timeout = self.get_parameter('key_timeout').value
        print(HELP)
        print('[终端模式] 松开后约 %.1fs 停(evdev 不可用)。空格: 立即停止' % timeout)
        print(f'当前速度: 线速度 {self.lin:.2f} m/s, 角速度 {self.ang:.2f} rad/s')

        direction = (0.0, 0.0, 0.0)
        last_key_time = 0.0
        moving = [False]
        old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while rclpy.ok():
                while select.select([sys.stdin], [], [], 0.02)[0]:
                    key = sys.stdin.read(1)
                    if key in KEY_MAP:
                        direction = KEY_MAP[key]
                        last_key_time = time.monotonic()
                    elif key == ' ':
                        direction = (0.0, 0.0, 0.0)
                        last_key_time = 0.0
                    elif key in ('+', '='):
                        self.adjust_speed(1.1)
                    elif key in ('-', '_'):
                        self.adjust_speed(0.9)
                if time.monotonic() - last_key_time > timeout:
                    direction = (0.0, 0.0, 0.0)
                self.publish(direction, moving)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
            self.pub.publish(Twist())
            print('\n已退出，已发送零速指令。')

    def run(self):
        if HAS_EVDEV and self.run_evdev():
            return
        if not HAS_EVDEV:
            print('提示: 未安装 python evdev(pip install evdev)，退回终端模式')
        else:
            print('提示: 没有可读的键盘设备(需在 input 组)，退回终端模式')
        self.run_termios()


def main():
    rclpy.init()
    node = TeleopKeyboard()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
