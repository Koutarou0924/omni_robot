import time
import serial

# お使いの環境に合わせてポートを変更してください（Windowsなら 'COM5' など）
PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

try:
  ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
  time.sleep(2)
  print(f"Connected to Teensy on {PORT}")
except Exception as e:
  print(f"Failed to connect: {e}")
  exit()

print("\n=== 3輪オムニホイール ターミナル操作モード ===")
print("【入力のしかた】")
print("  ・個別操作: モーター番号,PWM値  (例: 1,120 または 2,-100)")
print("  ・一括操作: M1値,M2値,M3値    (例: 100,-100,50)")
print("  ・終了する: q または quit と入力")
print("=========================================\n")

try:
  while True:
    # ターミナルからの入力を待つ
    cmd = input("Command > ").strip()

    if cmd.lower() in ["q", "quit"]:
      # 終了時は安全のため全停止コマンドを送る
      ser.write(b"0,0,0\n")
      print("モーターを停止して終了します。")
      break

    if not cmd:
      continue

    # Teensyへ送信
    ser.write((cmd + "\n").encode("utf-8"))
    time.sleep(0.05)

    # Teensyからの返答があれば表示
    while ser.in_waiting > 0:
      line = ser.readline().decode("utf-8").strip()
      print(f"  Teensy -> {line}")

except KeyboardInterrupt:
  # Ctrl+Cで強制終了されたときも安全に停止
  ser.write(b"0,0,0\n")
  print("\n安全に停止しました。")
finally:
  ser.close()