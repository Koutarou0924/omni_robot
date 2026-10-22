import math
import time
import serial

PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

try:
  ser = serial.Serial(PORT, BAUD_RATE, timeout=0.1)
  time.sleep(2)
  print(f"Connected to Teensy on {PORT}")
except Exception as e:
  print(f"Failed to connect: {e}")
  exit()


def omni_inverse_kinematics(vx, vy, omega):
  """ご指定の式をベースにした逆運動学計算"""
  theta = [math.radians(90), math.radians(210), math.radians(330)]
  L = 0.5  # 機体サイズの係数

  v1 = vx * math.cos(theta[0]) + vy * math.sin(theta[0]) + L * omega
  v2 = vx * math.cos(theta[1]) + vy * math.sin(theta[1]) + L * omega
  v3 = vx * math.cos(theta[2]) + vy * math.sin(theta[2]) + L * omega

  max_v = max(abs(v1), abs(v2), abs(v3), 1.0)
  scale = 100.0  # PWMスケール（床の状況に合わせて調整）

  p1 = int((v1 / max_v) * scale)
  p2 = int((v2 / max_v) * scale)
  p3 = int((v3 / max_v) * scale)

  return p1, p2, p3


def send_motion(vx, vy, omega):
  p1, p2, p3 = omni_inverse_kinematics(vx, vy, omega)
  command = f"{p1},{p2},{p3}\n"
  ser.write(command.encode("utf-8"))
  time.sleep(0.02)
  read_serial_response()


def get_encoders():
  """Teensyからエンコーダのパルス数を取得する"""
  ser.write(b"ENC\n")
  time.sleep(0.02)
  start_time = time.time()
  while time.time() - start_time < 0.2:
    if ser.in_waiting > 0:
      line = ser.readline().decode("utf-8").strip()
      if line.startswith("ENC_DATA:"):
        parts = line.replace("ENC_DATA:", "").split(",")
        if len(parts) == 3:
          return int(parts[0]), int(parts[1]), int(parts[2])
    time.sleep(0.01)
  return None


def read_serial_response():
  while ser.in_waiting > 0:
    line = ser.readline().decode("utf-8").strip()
    if line:
      print(f"    Teensy -> {line}")


if __name__ == "__main__":
  try:
    print("\n=== 3輪オムニホイール 統合コントロールシステム ===")
    print("【操作方法の選び方】")
    print("  ・簡単キー入力: w(前), s(後), a(左), d(右), z(左回), c(右回), x(停止)")
    print("  ・速度数値入力: vx,vy,omega をカンマ区切りで入力 (例: 1.0, 0.0, 0.5)")
    print("  ・エンコーダ確認: 'enc' と入力")
    print("  ・終了: 'q' または 'quit'\n")

    while True:
      user_input = input("Command > ").strip().lower()

      if user_input in ["q", "quit"]:
        ser.write(b"0,0,0\n")
        print("モーターを停止して終了します。")
        break

      if user_input == "enc":
        encs = get_encoders()
        if encs:
          print(f"  [Encoder] M1: {encs[0]} | M2: {encs[1]} | M3: {encs[2]}")
        else:
          print("  [Encoder] 応答がありませんでした。")
        continue

      if not user_input:
        continue

      # キー入力ショートカットの処理
      if user_input == "w":
        print("-> 前進")
        send_motion(1.0, 0.0, 0.0)
      elif user_input == "s":
        print("-> 後退")
        send_motion(-1.0, 0.0, 0.0)
      elif user_input == "a":
        print("-> 左平行移動")
        send_motion(0.0, -1.0, 0.0)
      elif user_input == "d":
        print("-> 右平行移動")
        send_motion(0.0, 1.0, 0.0)
      elif user_input == "z":
        print("-> 左回転")
        send_motion(0.0, 0.0, 1.0)
      elif user_input == "c":
        print("-> 右回転")
        send_motion(0.0, 0.0, -1.0)
      elif user_input == "x":
        print("-> 停止")
        send_motion(0.0, 0.0, 0.0)
      else:
        # カンマ区切りの数値入力 (vx, vy, omega) の処理を試みる
        try:
          parts = user_input.split(",")
          if len(parts) == 3:
            vx = float(parts[0])
            vy = float(parts[1])
            omega = float(parts[2])
            print(f"-> 速度指定 (vx:{vx}, vy:{vy}, omega:{omega})")
            send_motion(vx, vy, omega)
          else:
            print(
                "  [エラー] 無効なコマンドか、形式が違います ('vx,vy,omega'"
                " または 'enc')"
            )
        except ValueError:
          print("  [エラー] 数値の変換に失敗しました。")

      # 操作の直後にエンコーダの値も自動で一回取得して表示してみる
      encs = get_encoders()
      if encs:
        print(f"  (Enc -> M1:{encs[0]}, M2:{encs[1]}, M3:{encs[2]})")

  except KeyboardInterrupt:
    pass
  finally:
    ser.write(b"0,0,0\n")
    ser.close()
    print("\n安全に終了しました。")