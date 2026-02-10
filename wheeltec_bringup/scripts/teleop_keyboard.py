#!/usr/bin/env python3
"""
自定义键盘遥控节点
按键说明：
  W - 前进
  S - 后退
  A - 左移（麦轮平移）
  D - 右移（麦轮平移）
  J - 左转
  K - 右转
  空格 - 停止
  Q - 退出
"""

import sys
import termios
import tty
import select
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# 按键映射
KEY_BINDINGS = {
    'w': (1.0, 0.0, 0.0),   # 前进: (vx, vy, vz)
    's': (-1.0, 0.0, 0.0),  # 后退
    'a': (0.0, 1.0, 0.0),   # 左移
    'd': (0.0, -1.0, 0.0),  # 右移
    'j': (0.0, 0.0, 1.0),   # 左转
    'k': (0.0, 0.0, -1.0),  # 右转
    ' ': (0.0, 0.0, 0.0),   # 停止
}

HELP_MSG = """
---------------------------
自定义键盘遥控
---------------------------
移动控制:
   W     - 前进
   S     - 后退
   A     - 左移
   D     - 右移

旋转控制:
   J     - 左转
   K     - 右转

其他:
   空格  - 停止
   Q     - 退出

当前速度设置:
  线速度: {linear_speed:.2f} m/s
  角速度: {angular_speed:.2f} rad/s
  发送频率: {rate:.0f} Hz
---------------------------
"""


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        
        # 声明参数
        self.declare_parameter('linear_speed', 0.03)
        self.declare_parameter('angular_speed', 0.05)
        self.declare_parameter('publish_rate', 50.0)  # 发送频率 Hz
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # 创建发布者
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 当前速度
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0
        
        # 运行标志
        self.running = True
        
        self.get_logger().info('键盘遥控节点已启动')
    
    def get_key(self, timeout=0.1):
        """获取键盘输入（非阻塞）"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
    
    def publish_loop(self):
        """持续发送速度指令的循环"""
        rate = 1.0 / self.publish_rate
        while self.running:
            twist = Twist()
            twist.linear.x = self.current_vx
            twist.linear.y = self.current_vy
            twist.angular.z = self.current_vz
            self.publisher.publish(twist)
            
            import time
            time.sleep(rate)
    
    def run(self):
        """主循环"""
        print(HELP_MSG.format(
            linear_speed=self.linear_speed,
            angular_speed=self.angular_speed,
            rate=self.publish_rate
        ))
        
        # 启动发布线程
        pub_thread = threading.Thread(target=self.publish_loop)
        pub_thread.daemon = True
        pub_thread.start()
        
        try:
            while self.running:
                key = self.get_key().lower()
                
                if key == 'q' or key == '\x03':  # q 或 Ctrl+C
                    break
                
                if key in KEY_BINDINGS:
                    vx, vy, vz = KEY_BINDINGS[key]
                    self.current_vx = vx * self.linear_speed
                    self.current_vy = vy * self.linear_speed
                    self.current_vz = vz * self.angular_speed
                    
                    # 打印当前速度
                    if vx != 0 or vy != 0 or vz != 0:
                        print(f'\r速度: vx={self.current_vx:.3f}, vy={self.current_vy:.3f}, vz={self.current_vz:.3f}    ', end='')
                    else:
                        print(f'\r停止                                        ', end='')
                else:
                    # 松开按键时速度归零
                    if self.current_vx != 0 or self.current_vy != 0 or self.current_vz != 0:
                        self.current_vx = 0.0
                        self.current_vy = 0.0
                        self.current_vz = 0.0
                        print(f'\r停止                                        ', end='')
                
        except Exception as e:
            self.get_logger().error(f'错误: {e}')
        finally:
            # 停止机器人
            self.running = False
            self.current_vx = 0.0
            self.current_vy = 0.0
            self.current_vz = 0.0
            
            twist = Twist()
            self.publisher.publish(twist)
            print('\n键盘遥控已退出')


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
