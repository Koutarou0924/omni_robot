import math
import time
import serial

PORT = "/dev/ttyACM0"  # ラズパイのTeensy接続ポート
BAUD_RATE = 115200

try:
  ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
  time.sleep(2)
  print(f"Connected to Teensy on {PORT}")
except Exception as e:
  print(f"Failed to connect: {e}")
  exit()


def omni_inverse_kinematics(vx, vy, omega):
  """ユーザー様ご指定の式をベースにした逆運動学関数"""
  # 車輪の設置角度（必要に応じてラジアンの値を調整してください）
  # 例: 0°, 120°, 240° 配置の場合など
  theta = [math.radians(90), math.radians(210), math.radians(330)]
  L = 0.5  # 機体サイズの係数

  # ご指定の計算式を適用
  v1 = vx * math.cos(theta[0]) + vy * math.sin(theta[0]) + L * omega
  v2 = vx * math.cos(theta[1]) + vy * math.sin(theta[1]) + L * omega
  v3 = vx * math.cos(theta[2]) + vy * math.sin(theta[2]) + L * omega

  # 最大値で規格化しつつ、全体のスピード倍率（スケール）を調整
  max_v = max(abs(v1), abs(v2), abs(v3), 1.0)
  scale = 100.0  # 床の状況や電圧に合わせて調整

  p1 = int((v1 / max_v) * scale)
  p2 = int((v2 / max_v) * scale)
  p3 = int((v3 / max_v) * scale)

  return p1, p2, p3


if __name__ == "__main__":
  try:
    print("=== 速度・方向 自由入力モード（カスタム逆運動学） ===")
    print("【入力方法】")
    print("  vx, vy, omega をカンマ区切りで入力してください (例: 1.0, 0.0, 0.5)")
    print("  ・停止: 0,0,0 と入力")
    print("  ・終了: q と入力\n")

    while True:
      user_input = input("Velocity (vx,vy,omega) > ").strip()

      if user_input.lower() in ["q", "quit"]:
        ser.write(b"0,0,0\n")
        print("終了します。")
        break

      if not user_input:
        continue

      try:
        parts = user_input.split(",")
        if len(parts) != 3:
          print("  [エラー] 'vx,vy,omega' の形式で3つの値を入力してください。")
          continue

        vx = float(parts[0])
        vy = float(parts[1])
        omega = float(parts[2])

        # 逆運動学で各モーターのPWM値に変換
        p1, p2, p3 = omni_inverse_kinematics(vx, vy, omega)

        # Teensyへ送信
        command = f"{p1},{p2},{p3}\n"
        ser.write(command.encode("utf-8"))
        time.sleep(0.05)

        print(
            f"  Sent -> vx:{vx}, vy:{vy}, omega:{omega} => PWM:({p1}, {p2},"
            f" {p3})"
        )

        while ser.in_waiting > 0:
          line = ser.readline().decode("utf-8").strip()
          print(f"    Teensy -> {line}")

      except ValueError:
        print("  [エラー] 有効な数値を入力してください。")

  except KeyboardInterrupt:
    pass
  finally:
    ser.write(b"0,0,0\n")
    ser.close()
    print("モーターを停止して終了しました。")