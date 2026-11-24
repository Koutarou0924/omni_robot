import serial
import time
import omni_position_control1

if __name__ == "__main__":
    # 1. Teensyとのシリアル接続を開く
    # (ラズパイの場合は '/dev/ttyACM0' など、Windowsの場合は 'COM3' など環境に合わせて変更)
    PORT = '/dev/ttyACM0' 
    BAUD = 115200
    
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2) # 接続の安定化待ち
        print("Teensy connected successfully!")
        
        # 2. コントローラーの初期化
        robot = omni_position_control1.OmniPositionController(ser)
        
        print(robot.get_encoder_pulses())
        ser.write(b"0.3,0.0,0.0\n")

        # 3. 相対位置制御の実行
        print("Moving forward 0.5 meters...")
        robot.move_straight_relative(target_distance_m=0.5, speed=0.3)
        
        print("Task finished!")

    except Exception as e:
        print(f"Error: {e}")
    finally:
        ser.close()
