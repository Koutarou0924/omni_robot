import math
import time

class OmniPositionController:
    def __init__(self, serial_handler):
        self.serial = serial_handler
        
        # --- ハードウェア固有のパラメータ（要調整） ---
        self.WHEEL_RADIUS = 0.03  # タイヤ半径 [m] (例: 3cm)
        self.PPR = 1200           # 1回転あたりのエンコーダパルス数（ギア比考慮後）
        self.DIST_PER_PULSE = (2.0 * math.pi * self.WHEEL_RADIUS) / self.PPR
        
        # 車体中心から車輪までの距離 L [m]
        self.L = 0.15 

    def get_encoder_pulses(self):
        """Teensyから 'ENC' コマンドで現在のパルス数を取得する例"""
        self.serial.write(b"ENC\n")
        response = self.serial.readline().decode('utf-8').strip()
        # 応答例: "120, -45, 300"
        try:
            p1, p2, p3 = map(int, response.split(','))
            return p1, p2, p3
        except ValueError:
            return 0, 0, 0

    def move_straight_relative(self, target_distance_m, speed=0.3):
        """
        指定した距離（メートル）だけ直進する相対位置制御
        target_distance_m: 正なら前進、負なら後退
        """
        # 目標距離に必要な車輪の移動パルス数を計算
        # 3輪オムニで直進する場合、M1, M2, M3の目標パルス比率は運動学から決まる
        # 簡単のため、代表的な車輪の移動量換算を使用
        target_pulses = int(target_distance_m / self.DIST_PER_PULSE)
        
        # スタート時のエンコーダ値を取得
        start_p1, start_p2, start_p3 = self.get_encoder_pulses()
        
        # 簡易P制御ゲイン
        Kp = 0.002
        
        print(f"Starting relative move: {target_distance_m} m (Target pulses: {target_pulses})")
        
        while True:
            current_p1, current_p2, current_p3 = self.get_encoder_pulses()
            
            # 各車輪の進捗（経過パルス数）
            diff_p1 = current_p1 - start_p1
            
            # 平均の進捗で誤差を計算
            error = target_pulses - diff_p1
            
            if abs(error) < 5:  # 目標に十分近づいたら終了
                break
                
            # 速度指令を計算（P制御 + 最小速度の担保）
            v_cmd = error * Kp
            v_cmd = max(min(v_cmd, speed), -speed)  # 最大・最小速度の制限
            
            # 直進用の速度指令 (vx = v_cmd, vy = 0, omega = 0) をTeensyへ送信
            # 以前の通信プロトコル形式にあわせる (例: "vx,vy,omega\n")
            command_str = f"{v_cmd:.2f},0.0,0.0\n"
            self.serial.write(command_str.encode('utf-8'))
            
            time.sleep(0.05)
            
        # 停止指令
        self.serial.write(b"0.0,0.0,0.0\n")
        print("Reached target position!")