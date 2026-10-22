import time
import serial

PORT = "COM5"  # お使いの環境に合わせて変更してください (Macなら '/dev/tty.usbmodem...')
BAUD_RATE = 115200

try:
  ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
  time.sleep(2)
  print(f"Connected to Teensy on {PORT}")
except Exception as e:
  print(f"Failed to connect: {e}")
  exit()


def send_batch(p1, p2, p3):
  """3輪同時にPWM値を送信 (例: 100, -100, 50)"""
  command = f"{p1},{p2},{p3}\n"
  ser.write(command.encode("utf-8"))
  time.sleep(0.05)
  while ser.in_waiting > 0:
    print(f"Teensy -> {ser.readline().decode('utf-8').strip()}")


def send_individual(motor_id, pwm):
  """特定の1輪だけを操作 (例: motor_id=1, pwm=120)"""
  command = f"{motor_id},{pwm}\n"
  ser.write(command.encode("utf-8"))
  time.sleep(0.05)
  while ser.in_waiting > 0:
    print(f"Teensy -> {ser.readline().decode('utf-8').strip()}")


if __name__ == "__main__":
  try:
    print("=== 3輪オムニホイール 個別・一括テスト開始 ===")

    # 1. 個別操作のテスト
    print("\n[Test 1] モーター1だけを正転 (PWM: 100)")
    send_individual(1, 100)
    time.sleep(1.5)

    print("[Test 1] モーター1を停止")
    send_individual(1, 0)
    time.sleep(1)

    # 2. 一括操作のテスト（3輪同時に違う速度・方向を指定）
    print("\n[Test 2] 3輪一括指示 (M1:100, M2:-100, M3:50)")
    send_batch(100, -100, 50)
    time.sleep(2)

    print("[Test 2] 全輪停止")
    send_batch(0, 0, 0)

    print("\n=== テスト完了 ===")

  except KeyboardInterrupt:
    print("\n中断されました。全輪を停止します...")
    send_batch(0, 0, 0)
    ser.close()