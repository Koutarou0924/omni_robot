import math
import time
import serial

PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

# --- 相対位置制御用のパラメータ（実機に合わせて調整してください） ---
WHEEL_RADIUS = 0.05   # タイヤ半径 [m] (例: 3cm)
PPR = 1200            # 1回転あたりのエンコーダパルス数（ギア比含む）
DIST_PER_PULSE = (2.0 * math.pi * WHEEL_RADIUS) / PPR

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
  L = 0.3  # 機体サイズの係数

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
  global ser
  p1, p2, p3 = omni_inverse_kinematics(vx, vy, omega)
  command = f"{p1},{p2},{p3}\n"

  try:
    ser.write(command.encode("utf-8"))
    time.sleep(0.02)
    read_serial_response()
  except (serial.SerialException, OSError) as e:
    print(f"\n[警告] 通信が切断されました: {e}")
    print("再接続を試みています...")
    reconnect_serial()


def get_encoders():
  global ser
  try:
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
  except (serial.SerialException, OSError):
    print("\n[警告] エンコーダ取得時に通信エラーが発生しました。")
    reconnect_serial()
  return None


def move_straight_relative(target_distance_m, speed=1):
    """
    指定した距離（メートル）だけ直進する相対位置制御（改善版）
    """
    target_pulses = int(target_distance_m / DIST_PER_PULSE)
    
    encs_start = get_encoders()
    if not encs_start:
        print("  [エラー] スタート時のエンコーダ取得に失敗しました。")
        return
        
    start_p1, start_p2, start_p3 = encs_start
    Kp = 0.0015       # ゲインを少しマイルドに調整
    MIN_SPEED = 1  # 最低速度（これ以下ではモーターがうなる場合、直前でカットする）
    
    print(f"-> 相対移動開始: {target_distance_m} m (目標パルス差: {target_pulses})")
    
    while True:
        encs_current = get_encoders()
        if not encs_current:
            continue
            
        current_p1, current_p2, current_p3 = encs_current
        
        # --- このロボットの実際の動きに合わせた差分計算 ---
        # スタート時からの変化量を求める
        d1 = current_p1 - start_p1
        d2 = current_p2 - start_p2  # 減る方向（マイナス）
        d3 = current_p3 - start_p3  # 増える方向（プラス）
        
        # 前進したとき、d2はマイナス、d3はプラスになるため、
        # 「どれだけ進んだか」のトータル量は、マイナスを反転させて足し合わせる
        # （例: M2の減少分とM3の増加分を合算して、全体の移動量とする）
        forward_progress = (abs(d2) + abs(d3)) / 2.0 
        
        error = target_pulses - forward_progress
        
        # 【重要】目標に十分近づいたら（例: 5パルス以内）、そこで即座に終了して停止
        if abs(error) <= 5:
            print("  [到達] 目標パルスに到達しました。")
            break
            
        # P制御による速度指令計算
        v_cmd = error * Kp
        
        # 速度の最大・最小制限
        # ※目標付近（誤差が小さいとき）は、最低速度に張り付かせず、そのまま小さな値にして自然に減速させる
        if abs(v_cmd) < MIN_SPEED:
            # ただし、遠く離れているのに動かない現象を防ぐため、誤差がある程度ある時だけ最低速度を適用
            if abs(error) > 15:
                v_cmd = MIN_SPEED if v_cmd > 0 else -MIN_SPEED
            else:
                # 目標間近では極低速を許容（あるいはそのまま出力）
                pass
                
        v_cmd = max(min(v_cmd, speed), -speed)
        
        # 直進指令を送信
        send_motion(v_cmd, 0.0, 0.0)
        time.sleep(0.03)
        
    # 完全に停止する指令を送る
    send_motion(0.0, 0.0, 0.0)
    print("  [完了] 停止しました。")


def reconnect_serial():
  """シリアルポートの再接続を行う関数"""
  global ser
  try:
    ser.close()
  except:
    pass

  while True:
    try:
      time.sleep(1)
      ser = serial.Serial(PORT, BAUD_RATE, timeout=0.1)
      time.sleep(2)  # Teensyの再起動待ち
      print("[復旧成功] Teensyとの再接続に成功しました！\n")
      break
    except:
      print(".", end="", flush=True)


def read_serial_response():
  while ser.in_waiting > 0:
    line = ser.readline().decode("utf-8").strip()
    if line:
      print(f"    Teensy -> {line}")

def main():
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

# --- （既存の if文の下あたりに追加） ---
      
      # 例: "m,0.5" と入力されたら 0.5メートル前進
      # 例: "m,-0.3" と入力されたら 0.3メートル後退
      if user_input.startswith("m,"):
        try:
          parts = user_input.split(",")
          distance = float(parts[1])
          move_straight_relative(distance)
        except (IndexError, ValueError):
          print("  [エラー] 形式が違います。例: m,0.5 （0.5メートル前進）")
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

if __name__ == "__main__":
  main()
