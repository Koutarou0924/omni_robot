import time
import serial

PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

try:
  ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
  time.sleep(2)
  print(f"Connected to Teensy on {PORT}")
except Exception as e:
  print(f"Failed to connect: {e}")
  exit()


def get_encoders():
  """Teensyにエンコーダ値の取得を要求し、パルス数を取得する"""
  ser.write(b"ENC\n")
  time.sleep(0.02)

  while ser.in_waiting > 0:
    line = ser.readline().decode("utf-8").strip()
    if line.startswith("ENC_DATA:"):
      # "ENC_DATA:100,-50,200" のような文字列を分解
      data_str = line.replace("ENC_DATA:", "")
      parts = data_str.split(",")
      if len(parts) == 3:
        return int(parts[0]), int(parts[1]), int(parts[2])
  return None


if __name__ == "__main__":
  try:
    print("=== エンコーダ値の取得テスト ===")
    print("タイヤを手で回すか、モーターを動かして値の変化を確認します。")
    print("Ctrl+C で終了します。\n")

    while True:
      # エンコーダの現在値を取得して表示
      enc_values = get_encoders()
      if enc_values:
        e1, e2, e3 = enc_values
        print(f"Encoder Counts -> M1: {e1} | M2: {e2} | M3: {e3}")

      time.sleep(0.2)  

  except KeyboardInterrupt:
    print("\n終了します。")
  finally:
    ser.write(b"0,0,0\n")
    ser.close()