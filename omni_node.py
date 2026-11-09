#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial


class OmniControllerNode(Node):

  def __init__(self):
    super().__init__('omni_controller_node')

    # パラメータ設定（必要に応じてポート名を変更してください）
    self.declare_parameter('port', '/dev/ttyACM0')
    self.declare_parameter('baudrate', 115200)
    self.declare_parameter('robot_radius', 0.15)  # ロボット中心から車輪までの距離[m]

    port = self.get_parameter('port').get_parameter_value().string_value
    baudrate = (
        self.get_parameter('baudrate').get_parameter_value().integer_value
    )
    self.L = (
        self.get_parameter('robot_radius').get_parameter_value().double_value
    )

    # シリアル通信の初期化（Teensyとの接続）
    try:
      self.ser = serial.Serial(port, baudrate, timeout=0.1)
      self.get_logger().info(f'Connected to Teensy on {port}')
    except Exception as e:
      self.get_logger().error(f'Failed to connect to serial port: {e}')
      self.ser = None

    # サブスクライバーの作成（/cmd_vel トピックを購読）
    self.subscription = self.create_subscription(
        Twist, '/cmd_vel', self.cmd_vel_callback, 10
    )
    self.get_logger().info('Omni Controller Node has been started.')

  def cmd_vel_callback(self, msg):
    # Twistメッセージから速度ベクトルを抽出
    vx = msg.linear.x
    vy = msg.linear.y
    omega = msg.angular.z

    # 3輪オムニホイールの逆運動学計算
    # 設置角度: 0°, 120°, 240°
    theta = [0.0, math.radians(120), math.radians(240)]

    v1 = vx * math.cos(theta[0]) + vy * math.sin(theta[0]) + self.L * omega
    v2 = vx * math.cos(theta[1]) + vy * math.sin(theta[1]) + self.L * omega
    v3 = vx * math.cos(theta[2]) + vy * math.sin(theta[2]) + self.L * omega

    # Teensyへ送信するコマンド形式に変換（例: "p1,p2,p3\n"）
    # ※必要に応じてスケーリングや整数値への変換を行ってください
    command_str = f'{v1:.2f},{v2:.2f},{v3:.2f}\n'

    if self.ser and self.ser.is_open:
      self.ser.write(command_str.encode('utf-8'))
      self.get_logger().debug(f'Sent to Teensy: {command_str.strip()}')
    else:
      self.get_logger().warn(
          f'Serial not open. Computed motors: {command_str.strip()}'
      )

  def destroy_node(self):
    if self.ser and self.ser.is_open:
      self.ser.write(b'0.0,0.0,0.0\n')
      self.ser.close()
    super().destroy_node()


def main(args=None):
  rclpy.init(args=args)
  node = OmniControllerNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()