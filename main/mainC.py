"""
test_GPSrun_new.py
===================
GPS誘導走行テストコード（新版）
 
元の test_GPSrun.py (RPi.GPIO直叩き版) の構成・挙動をそのまま踏襲しつつ、
NEW2026 リポジトリの test/ 以下 (GPS.py, HC-SR04.py, runtest.py) と
矛盾しないよう、以下の点を書き換えています。
 
【リポジトリとの整合性を取るために変更した点】
  1. モーター制御: RPi.GPIO の直接叩きから、runtest.py と同じ
     gpiozero (Motor / PWMOutputDevice / OutputDevice) + LGPIOFactory に統一。
     PWMA/AIN1/AIN2/PWMB/BIN1/BIN2 のBCM番号も runtest.py の
     「BOARD→BCM変換」に合わせて 13/5/6/18/23/24 に変更し、
     runtest.py にある MOTOR_STBY(BCM17) の ON/OFF 制御も追加。
     (元コードの 18/8/25/10/9/11 という番号はリポジトリのハード構成と
      矛盾するため採用していません)
  2. GPSスレッド: GPS.py の rungps() と同じ方式に統一。
     - readline() が空バイト列を返すケース (sentence[0] で IndexError に
       なる元コードの潜在バグ) を回避
     - decode は GPS.py と同じ "ascii", errors="replace"
     - MicropyGPS(9, "dd") はそのまま(dd modeでは latitude/longitude が
       [10進度, 半球] のリストで返るため、元コードの gps.latitude[0] の
       使い方自体は正しいので変更していません)
  3. CSVヘッダー: 元コードはヘッダー17列とデータ行17列の「並び」が
     ずれるバグ(2列目がAccXのはずがPhaseの値が入る等)があったため、
     ヘッダーとデータの列順を一致させています。
 
【追加】最終ゴール判定へのHC-SR04超音波距離センサの統合
  GPS誘導で目標地点付近(distance<5.0)まで来た後、HC-SR04.py と同じ
  ピン配置(TRIG=BCM8, ECHO=BCM7)・計算式でオブジェクトを探索する
  最終フェーズ(sonar_final_phase)を追加しました。
    1. 機体を左右に振りながら前方をスキャン
    2. 前方にオブジェクトを検出したらその方向へ前進
    3. HC-SR04の距離がゴール判定用の閾値を下回ったらゴール確定
  この最終フェーズで使う閾値・スイング時間は、他の定数とは別の
  「ゴール判定(HC-SR04)用の閾値」セクションにまとめています。
 
【今回追加】一時停止機能
  走行開始(start)から PAUSE_START_SEC 秒経過した時点で、
  PAUSE_DURATION_SEC 秒間だけモーターを停止します。
  停止判定は moveMotor_thread 内で行うため、フェーズに関係なく効き、
  direction の値は書き換えません(停止時間が終われば元の動作に戻ります)。
  設定値は「一時停止設定」セクションにまとめています。
 
BNO055 (9軸センサ) 部分はリポジトリの test/ フォルダには実装が
含まれていない(main/main.py 側で使う想定)ため、元コードのまま
`import BNO055` を維持しています。実行環境にこのモジュールが
必要です。
"""
 
import csv
import datetime
import math
import threading
import time
from pathlib import Path
 
import serial
 
import BNO055
from micropyGPS import MicropyGPS
 
from gpiozero import (
    Motor,
    PWMOutputDevice,
    OutputDevice,
    DigitalOutputDevice,
    DigitalInputDevice,
    Device,
)
from gpiozero.pins.lgpio import LGPIOFactory
 
Device.pin_factory = LGPIOFactory()
 
# ===========================================================================
# 定数　上書きしない
# ===========================================================================
MAG_CONST = 8.9  # 地磁気補正用の偏角
CALIBRATION_MILLITIME = 20 * 1000
TARGET_LAT = 38.26052
TARGET_LNG = 140.8544151
DATA_SAMPLING_RATE = 0.00001
EARTH_RADIUS = 6378136.59
 
# --- GPSモジュール設定 (GPS.py と同じ) ---
GPS_PORT = "/dev/serial0"
GPS_BAUDRATE = 9600
GPS_LOCAL_OFFSET = 9  # JST(+9h)
 
# --- モーターピン(BCM) : runtest.py と同じ配線 ---
PWMA = 13   # 右タイヤ用PWM
AIN1 = 5
AIN2 = 6
PWMB = 18   # 左タイヤ用PWM
BIN1 = 23
BIN2 = 24
STBY = 17
 
SPEED_MAX = 1.0  # gpiozeroは0.0〜1.0でデューティ比を指定する
 
# ===========================================================================
# 一時停止設定　※走行開始から一定時間後に、一定時間だけ停止する
# ===========================================================================
PAUSE_START_SEC = 60.0     # 開始から何s時点で停止するか
PAUSE_DURATION_SEC = 10.0  # 何s間停止するか(0以下で一時停止機能を無効化)
 
# ===========================================================================
# ゴール判定(HC-SR04)用の閾値　※他の定数とは分けて管理する
# ===========================================================================
SONAR_TRIG_PIN = 8   # HC-SR04.py と同じ TRIG(BCM8)
SONAR_ECHO_PIN = 7   # HC-SR04.py と同じ ECHO(BCM7)
 
GOAL_SONAR_THRESHOLD_CM = 15.0   # この距離未満になったらゴール確定
OBJECT_DETECT_RANGE_CM = 150.0   # この距離未満なら「前方にオブジェクトあり」と判定
 
SWING_STEP_SEC = 0.4     # 左右に振る際、片側あたりの動作時間[s]
SONAR_POLL_INTERVAL_SEC = 0.05  # HC-SR04を読む間隔[s]
 
# ===========================================================================
# 変数
# ===========================================================================
start = 0.0
end = 0.0
acc = [0.0, 0.0, 0.0]
gyro = [0.0, 0.0, 0.0]
mag = [0.0, 0.0, 0.0]
lat = 0.0  # GPSセンサーから取得
lng = 0.0
distance = 0.0
angle = 0.0
azimuth = 0.0
direction = 0.0
phase = 0
gps_detect = 0
 
bmx = BNO055.BNO055()
nowTime = datetime.datetime.now()
fileName = Path("log") / ("testlog_" + nowTime.strftime("%Y-%m%d-%H%M%S") + ".csv")
 
# ===========================================================================
# モーター (gpiozero, runtest.py と同じ構成)
# ===========================================================================
pwm_a = PWMOutputDevice(PWMA)
pwm_b = PWMOutputDevice(PWMB)
motor_a = Motor(forward=AIN1, backward=AIN2)
motor_b = Motor(forward=BIN1, backward=BIN2)
stby = OutputDevice(STBY)
 
 
def motors_stop():
    pwm_a.value = 0
    pwm_b.value = 0
    motor_a.stop()
    motor_b.stop()
    stby.off()
 
 
def motors_drive(pwm_a_ratio, pwm_b_ratio):
    """左右のタイヤを両方前進方向で駆動する(比率違いで旋回/超信地旋回を表現)"""
    stby.on()
    motor_a.forward()
    motor_b.forward()
    pwm_a.value = pwm_a_ratio
    pwm_b.value = pwm_b_ratio
 
 
def motor_left_only():
    """右モーターのみ停止・左モーターのみ前進(超信地に近い左旋回)"""
    stby.on()
    motor_a.stop()
    motor_b.forward()
    pwm_a.value = 0
    pwm_b.value = SPEED_MAX
 
 
def motor_right_only():
    """左モーターのみ停止・右モーターのみ前進(超信地に近い右旋回)"""
    stby.on()
    motor_a.forward()
    motor_b.stop()
    pwm_a.value = SPEED_MAX
    pwm_b.value = 0
 
 
# ===========================================================================
# HC-SR04 (超音波距離センサ) : test/HC-SR04.py の実装をそのまま踏襲
# ===========================================================================
class HCSR04:
    """HC-SR04.py と同じ計算式・ピン配置の超音波距離センサ制御クラス"""
 
    SOUND_SPEED = 343.0    # [m/s] 20℃ 空気中の音速
    TRIGGER_PULSE = 10e-6  # [s]   TRIGパルス幅
    SETTLE_TIME = 0.01     # [s]   TRIG送出前の安定待機
    ECHO_TIMEOUT = 0.03    # [s]   ECHO待機タイムアウト
 
    def __init__(self, pin_trig=SONAR_TRIG_PIN, pin_echo=SONAR_ECHO_PIN):
        self._trig = DigitalOutputDevice(pin_trig, initial_value=False)
        self._echo = DigitalInputDevice(pin_echo)
 
    def get_distance_m(self):
        self._trig.off()
        time.sleep(self.SETTLE_TIME)
 
        self._trig.on()
        time.sleep(self.TRIGGER_PULSE)
        self._trig.off()
 
        timeout_start = time.time()
        while not self._echo.value:
            if time.time() - timeout_start > self.ECHO_TIMEOUT:
                return None
        pulse_start = time.time()
 
        while self._echo.value:
            if time.time() - pulse_start > self.ECHO_TIMEOUT:
                return None
        pulse_end = time.time()
 
        duration = pulse_end - pulse_start
        return duration * self.SOUND_SPEED / 2.0
 
    def close(self):
        self._trig.off()
        self._trig.close()
        self._echo.close()
 
 
def sonar_final_phase():
    """
    GPS誘導でゴールエリア(distance<5.0)に入った後の最終ゴール判定。
 
    機体を左右に振ってHC-SR04で前方をスキャンし、
    オブジェクトを検出したらその方向へ前進、
    距離が GOAL_SONAR_THRESHOLD_CM を下回ったらゴールとする。
    """
    global direction
 
    sensor = HCSR04()
    try:
        print("phase5 : HC-SR04 final approach")
        while True:
            dist_m = sensor.get_distance_m()
            dist_cm = dist_m * 100.0 if dist_m is not None else None
 
            # 閾値未満になったらゴール確定
            if dist_cm is not None and dist_cm < GOAL_SONAR_THRESHOLD_CM:
                direction = 360.0
                motors_stop()
                print(f"goal! (HC-SR04 dist={dist_cm:.1f}cm)")
                return
 
            # 前方にオブジェクトを検出済みならそのまま前進を続ける
            if dist_cm is not None and dist_cm < OBJECT_DETECT_RANGE_CM:
                direction = -360.0  # 前進
                time.sleep(SONAR_POLL_INTERVAL_SEC)
                continue
 
            # 見つからない場合は左右に振ってスキャンする
            direction = 500.0  # 左に振る
            time.sleep(SWING_STEP_SEC)
            direction = 360.0
            dist_m = sensor.get_distance_m()
            dist_cm = dist_m * 100.0 if dist_m is not None else None
            if dist_cm is not None and dist_cm < OBJECT_DETECT_RANGE_CM:
                continue  # 次のループ先頭でオブジェクト方向に前進する
 
            direction = 600.0  # 右に振る
            time.sleep(SWING_STEP_SEC)
            direction = 360.0
            time.sleep(SONAR_POLL_INTERVAL_SEC)
    finally:
        sensor.close()
 
 
# ===========================================================================
# メイン
# ===========================================================================
def main():
    global direction
 
    Setup()
 
    while True:
        try:
            print("phase3 : GPS start")
            if distance < 5.0:
                # GPS誘導ではここでゴール確定としていたが、
                # 最終的なゴール判定はHC-SR04による判定に置き換える
                sonar_final_phase()
                break
        except KeyboardInterrupt:
            direction = 360.0
            motors_stop()
            break
 
 
def currentMilliTime():
    return round(time.time() * 1000)
 
 
def Setup():
    bmx.setUp()
    fileName.parent.mkdir(parents=True, exist_ok=True)
    with open(fileName, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "Time",
                "Phase",
                "AccX",
                "AccY",
                "AccZ",
                "GyroX",
                "GyroY",
                "GyroZ",
                "MagX",
                "MagY",
                "MagZ",
                "LAT",
                "LNG",
                "Distance",
                "Azimuth",
                "Angle",
                "Direction",
            ]
        )
 
    getThread = threading.Thread(target=moveMotor_thread, args=())
    getThread.daemon = True
    getThread.start()
 
    dataThread = threading.Thread(target=setData_thread, args=())
    dataThread.daemon = True
    dataThread.start()
 
    gpsThread = threading.Thread(target=GPS_thread, args=())
    gpsThread.daemon = True
    gpsThread.start()
 
    print("Setup OK")
 
 
def getBmxData():  # BMXデータ取得
    global acc, gyro, mag
    acc = bmx.getAcc()
    gyro = bmx.getGyro()
    mag = bmx.getMag()
 
 
def calcdistance():  # 目標地点までの距離計算
    global distance
    dx = (math.pi / 180) * EARTH_RADIUS * (TARGET_LNG - lng)
    dy = (math.pi / 180) * EARTH_RADIUS * (TARGET_LAT - lat)
    distance = math.sqrt(dx * dx + dy * dy)
 
 
def calcAngle():  # 目標地点への角度計算 : north=0 east=90 west=-90
    global angle
    dx = (math.pi / 180) * EARTH_RADIUS * (TARGET_LNG - lng)
    dy = (math.pi / 180) * EARTH_RADIUS * (TARGET_LAT - lat)
    angle = 90 - math.degrees(math.atan2(dy, dx))
    angle %= 360.0
 
 
def calcAzimuth():  # 機体の方位角計算
    global azimuth
    azimuth = 90 - math.degrees(math.atan2(mag[1], mag[0]))
    azimuth *= -1
    azimuth %= 360.0
 
 
def set_direction():  # -180<direction<180 : direction>0で右旋回
    global direction
    direction = azimuth - angle
    direction %= 360.0
    if direction > 180:
        direction -= 360
    if abs(direction) < 5.0:
        direction = -360
 
 
def GPS_thread():  # GPSモジュールを読み、緯度経度を更新する (GPS.pyと同方式)
    global lat, lng, gps_detect
 
    try:
        s = serial.Serial(GPS_PORT, GPS_BAUDRATE, timeout=5)
    except serial.SerialException as e:
        print(f"[ERROR] シリアルポートを開けません: {e}")
        return
 
    s.readline()  # 最初の1行は中途半端なデータのことがあるので捨てる
    gps = MicropyGPS(GPS_LOCAL_OFFSET, "dd")
 
    while True:
        raw = s.readline()
        if not raw:
            continue
 
        if s.in_waiting > 64:  # バッファ削除
            s.reset_input_buffer()
 
        sentence = raw.decode("ascii", errors="replace")
        if not sentence.startswith("$"):
            continue
 
        for ch in sentence:  # 1文字ずつmicropyGPSに渡す
            gps.update(ch)
 
        lat = gps.latitude[0]
        lng = gps.longitude[0]
 
        if lat != 0.0:
            gps_detect = 1
        else:
            gps_detect = 0
            print("None GNSS value")
 
 
def setData_thread():
    global end
    while True:
        getBmxData()
        calcAngle()
        calcAzimuth()
        set_direction()
        calcdistance()
        end = time.time()
 
        print(f"lat:{lat}")
        print(f"lng:{lng}")
        print(f"azimuth:{azimuth}")
        print(f"angle:{angle}")
 
        with open(fileName, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    round(end - start, 3),
                    round(phase, 1),
                    acc[0],
                    acc[1],
                    acc[2],
                    gyro[0],
                    gyro[1],
                    gyro[2],
                    mag[0],
                    mag[1],
                    mag[2],
                    lat,
                    lng,
                    distance,
                    azimuth,
                    angle,
                    direction,
                ]
            )
        time.sleep(DATA_SAMPLING_RATE)
 
 
def is_pause_time():
    """走行開始からの経過時間が一時停止区間に入っているか"""
    if PAUSE_DURATION_SEC <= 0:
        return False
    elapsed = time.time() - start
    return PAUSE_START_SEC <= elapsed < PAUSE_START_SEC + PAUSE_DURATION_SEC
 
 
def moveMotor_thread():
    while True:
        if is_pause_time():  # 一時停止区間:direction に関係なく停止
            motors_stop()
            time.sleep(0.05)
            continue
 
        if direction == 360.0:  # 停止
            motors_stop()
        elif direction == 500.0:  # 左折(超信地に近い)
            motor_left_only()
        elif direction == 600.0:  # 右折(超信地に近い)
            motor_right_only()
        elif direction == -360.0:  # 前進
            motors_drive(SPEED_MAX, SPEED_MAX)
        elif direction == -400.0:  # 左に回頭
            motors_drive(0.6, SPEED_MAX)
        elif 0.0 < direction <= 180.0:  # 左に緩旋回
            motors_drive(0.2, SPEED_MAX)
        elif -180.0 <= direction < 0.0:  # 右に緩旋回
            motors_drive(SPEED_MAX, 0.2)
 
 
if __name__ == "__main__":
    try:
        start = time.time()
        main()
    finally:
        motors_stop()
        pwm_a.close()
        pwm_b.close()
        motor_a.close()
        motor_b.close()
        stby.close()
 
