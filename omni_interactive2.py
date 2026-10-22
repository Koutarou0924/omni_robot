import math
import time
import serial

PORT = "/dev/ttyACM0"  # お環境に合わせて変更してください
BAUD_RATE = 115200

try:
  ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
  time.sleep(2)
  print(f"Connected to Teensy on {PORT}")
except Exception as e:
  print(f"Failed to connect: {e}")
  exit()


def omni_inverse_kinematics(vx, vy, omega):
  """ロボット全体の速度 (vx, vy, omega) から

  3輪それぞれの目標速度を計算し、PWM値(-255〜255)に変換する
  """
  # 車輪の設置角度（ラジアン変換）
  # 1輪目: 正面 (90°), 2輪目: 右後 (210°), 3輪目: 左後 (330°)
  theta = [math.radians(90), math.radians(210), math.radians(330)]

  # 車体中心から車輪までの係数（機体のサイズに合わせて調整）
  L = 1

  # 各車輪の速度を計算
  v1 = vx * math.cos(theta[0]) + vy * math.sin(theta[0]) + L * omega
  v2 = vx * math.cos(theta[1]) + vy * math.sin(theta[1]) + L * omega
  v3 = vx * math.cos(theta[2]) + vy * math.sin(theta[2]) + L * omega

  # 最大値で正規化してPWMの範囲(-255〜255)にスケーリング
  max_v = max(abs(v1), abs(v2), abs(v3), 1.0)
  scale = 100.0  # 基本の最高PWMスピード（調整可能）

  p1 = int((v1 / max_v) * scale)
  p2 = int((v2 / max_v) * scale)
  p3 = int((v3 / max_v) * scale)

  return p1, p2, p3


def send_robot_motion(vx, vy, omega):
  p1, p2, p3 = omni_inverse_kinematics(vx, vy, omega)
  command = f"{p1},{p2},{p3}\n"
  ser.write(command.encode("utf-8"))
  time.sleep(0.05)

  while ser.in_waiting > 0:
    print(f"  Teensy -> {ser.readline().decode('utf-8').strip()}")


if __name__ == "__main__":
  try:
    print("=== 逆運動学によるオムニホイール制御テスト ===")
    print("Commands:")
    print("  w: 前進, s: 後退, a: 左平行移動, d: 右平行移動")
    print("  q: 左回転, e: 右回転, x: 停止, exit: 終了")

    while True:
      key = input("Move (w/s/a/d/q/e/x) > ").strip().lower()

      if key == "exit":
        break
      elif key == "w":
        print("-> 前進")
        send_robot_motion(vx=1.0, vy=0.0, omega=0.0)
      elif key == "s":
        print("-> 後退")
        send_robot_motion(vx=-1.0, vy=0.0, omega=0.0)
      elif key == "a":
        print("-> 左平行移動")
        send_robot_motion(vx=0.0, vy=1.0, omega=0.0)
      elif key == "d":
        print("-> 右平行移動")
        send_robot_motion(vx=0.0, vy=-1.0, omega=0.0)
      elif key == "q":
        print("-> 左回転")
        send_robot_motion(vx=0.0, vy=0.0, omega=1.0)
      elif key == "e":
        print("-> 右回転")
        send_robot_motion(vx=0.0, vy=0.0, omega=-1.0)
      elif key == "x":
        print("-> 停止")
        send_robot_motion(vx=0.0, vy=0.0, omega=0.0)

  except KeyboardInterrupt:
    pass
  finally:
    send_robot_motion(0, 0, 0)
    ser.close()
    print("終了しました。")