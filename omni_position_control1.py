import math
import time

class OmniPositionController:
    def __init__(self, serial_handler):
        self.serial = serial_handler
        
        # --- ハードウェア固有のパラメータ（要調整） ---
        self.WHEEL_RADIUS = 0.05  # タイヤ半径 [m] (例: 3cm)
        self.PPR = 1200           # 1回転あたりのエンコーダパルス数（ギア比考慮後）
        self.DIST_PER_PULSE = (2.0 * math.pi * self.WHEEL_RADIUS) / self.PPR
        
        # 車体中心から車輪までの距離 L [m]
        self.L = 0.2 

    def get_encoder_pulses(self):
        """Teensyからのデータを受信し、ACKなどを無視してENC_DATAだけを確実に取得する関数"""
        # 送信前のゴミデータをクリア
        self.serial.reset_input_buffer()
        self.serial.write(b"ENC\n")
        
        # 最大5回まで読み込みを試行し、目的のデータを探す
        for _ in range(5):
            response = self.serial.readline()
            response_str = response.decode('utf-8', errors='ignore').strip()
            
            # ENC_DATAから始まっていればパースする
            if response_str.startswith("ENC_DATA:"):
                clean_str = response_str.replace("ENC_DATA:", "")
                try:
                    p1, p2, p3 = map(int, clean_str.split(','))
                    return p1, p2, p3
                except ValueError:
                    pass
            elif response_str:
                # ACK_PWMなどのメッセージはここでは読み飛ばす
                # print(f"（スキップしたメッセージ: {response_str}）")
                pass
                
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
        Kp = 0.02
        
        print(f"Starting relative move: {target_distance_m} m (Target pulses: {target_pulses})")
        
        while True:
            current_p1, current_p2, current_p3 = self.get_encoder_pulses()
            
            # 経過パルス数
            diff_p1 = current_p1 - start_p1
            error = target_pulses - diff_p1
            
            # 【対策1】目標に十分近づいたらループを抜けて即座に停止
            if abs(error) < 5:  # 5パルス以内なら完了
                break
                
            # 速度指令を計算（P制御）
            v_cmd = error * Kp
            
            # 【対策2】不感帯対策：出力が小さすぎるときは、最低限回る速度（下限値）を確保する
            # 例として最小速度を 0.15 に設定
            MIN_SPEED = 3
            MAX_SPEED = speed
            
            if abs(v_cmd) < MIN_SPEED:
                # 符号（プラス・マイナス）を維持したまま、最小速度に引き上げる
                v_cmd = MIN_SPEED if v_cmd > 0 else -MIN_SPEED
                
            # 最大速度の制限
            v_cmd = max(min(v_cmd, MAX_SPEED), -MAX_SPEED)
            
            # Teensyへ速度指令を送信
            command_str = f"{v_cmd:.2f},0.0,0.0\n"
            self.serial.write(command_str.encode('utf-8'))
            
            time.sleep(0.05)
            
        # 完全に停止する指令を送る
        self.serial.write(b"0.0,0.0,0.0\n")
        print("Reached target position!")
