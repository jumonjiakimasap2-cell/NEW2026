"""
main.py  --  GPS誘導走行 (NEW2026 ハードウェア構成版)

元プログラム (robot-project/main.py) の動きを、NEW2026 リポジトリの
ハード構成 (差動2輪 TB6612FNG / BNO055 / GT-502MGG-N + micropyGPS /
gpiozero + LGPIOFactory) の上で再現したもの。

【動きの流れ (元プログラムと同じ)】
  1. スイッチが押されるまで待つ
  2. GPS が Fix するまで待つ
  3. 目的地に着くまで次を繰り返す
       a. 目的地まで ARRIVAL_RADIUS_M 以内なら「到着」して停止
       b. BNO055 の方位と目的地方位のズレが HEADING_TOLERANCE_DEG を超えていたら
          「短く旋回 → 止まる → 揺れが収まるまで待つ → 測り直す」を繰り返し、
          向きを合わせる (最大 MAX_ALIGN_ATTEMPTS 回)
       c. ズレが許容内なら MOVE_BURST_SECONDS 秒だけ前進して止まる
  4. 走行中にもう一度スイッチを押すと緊急停止

【元プログラムとの対応】
  方向転換モーター (おもりの反動で回転) → 左右タイヤの片輪駆動パルスによる旋回
  前進モーター                          → 左右タイヤ同時前進
  スイッチ (GPIO17)                     → BCM26 に変更 (BCM17 は STBY と衝突するため)

【使い方】
  python3 main.py          # 通常走行
  python3 main.py check    # 旋回方向・BNO055 の向きの確認 (走行前に1回やる)

BNO055.py / micropyGPS.py は同じフォルダに置くこと (リポジトリの main/ と同じ)。
"""

import csv
import datetime
import math
import sys
import threading
import time
from pathlib import Path

import serial

import BNO055
from micropyGPS import MicropyGPS

from gpiozero import Button, Motor, PWMOutputDevice, OutputDevice, Device
from gpiozero.pins.lgpio import LGPIOFactory

Device.pin_factory = LGPIOFactory()

# ===========================================================================
# 定数 (NEW2026 の配線・環境。走行前に確認するもの)
# ===========================================================================
TARGET_LAT = 38.26052
TARGET_LNG = 140.8544151
EARTH_RADIUS = 6378136.59

# 地磁気の偏角 [度]。BNO055(NDOF)の方位は磁北基準、GPSから求める方位は真北基準。
# 元プログラムは補正なし。仙台付近は約 8〜9° 西偏なので、補正するなら -8.9 前後
# (真方位 = 磁方位 + 偏角、西偏は負)。まずは 0.0 で元の動きを再現。
MAG_DECLINATION_DEG = 0.0

# --- GPSモジュール (GPS.py と同じ) ---
GPS_PORT = "/dev/serial0"
GPS_BAUDRATE = 9600
GPS_LOCAL_OFFSET = 9  # JST(+9h)

# --- モーターピン (BCM) : runtest.py と同じ配線 ---
PWMA = 13
AIN1 = 5
AIN2 = 6
PWMB = 18
BIN1 = 23
BIN2 = 24
STBY = 17

# A/B のどちらが右タイヤか。リポジトリ内で食い違いがある:
#   main/main.py : PWMA = 右タイヤ (この設定 True)
#   test/runtest.py の left()/right() : A を減速させて「left」としている (= A が左)
# 実機で `python3 main.py check` を実行し、右旋回で方位が増えなければ False にする。
MOTOR_A_IS_RIGHT = True

# --- スイッチ (BCM)。片足をこのピン、もう片足を GND に接続 ---
# TRIG=BCM8 / ECHO=BCM7 (HC-SR04) は今回未使用だが、他のピンと衝突しない番号にしてある。
SWITCH_PIN = 26

# ===========================================================================
# 走行パラメータ (元プログラムの config.py と同じ値)
# ===========================================================================
ARRIVAL_RADIUS_M = 3.0        # この距離以内で到着扱い
MOVE_BURST_SECONDS = 3.0      # 1回の前進時間
HEADING_TOLERANCE_DEG = 8.0   # この角度以内なら「向きは合っている」
TURN_PULSE_SECONDS = 0.3      # 1回の旋回パルス時間
TURN_SETTLE_SECONDS = 0.5     # パルス後、揺れが収まるまでの待ち時間
MAX_ALIGN_ATTEMPTS = 20       # 向き合わせパルスの上限回数
DRIVE_SPEED = 0.6             # 前進の速さ 0.0〜1.0 (元の MOTOR_SPEED)
TURN_SPEED = 0.6              # 旋回パルスの速さ 0.0〜1.0

DATA_SAMPLING_RATE = 0.05     # センサ読み取り・CSV記録の周期 [s]

# ===========================================================================
# フェーズ番号 (CSV の Phase 列)
# ===========================================================================
PHASE_WAIT_SWITCH = 0
PHASE_WAIT_GPS = 1
PHASE_ALIGN = 2
PHASE_DRIVE = 3
PHASE_GOAL = 4

# ===========================================================================
# 共有状態
# ===========================================================================
_lock = threading.Lock()
start = 0.0
phase = PHASE_WAIT_SWITCH
lat = 0.0
lng = 0.0
gps_detect = 0
gps_error = None
azimuth = 0.0  # 機体の方位 (BNO055, 0=北, 90=東)

start_event = threading.Event()
stop_event = threading.Event()

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

if MOTOR_A_IS_RIGHT:
    _right_tire, _left_tire = (pwm_a, motor_a), (pwm_b, motor_b)
else:
    _right_tire, _left_tire = (pwm_b, motor_b), (pwm_a, motor_a)


def _run_tire(tire, ratio):
    pwm, motor = tire
    if ratio > 0:
        motor.forward()
        pwm.value = ratio
    else:
        pwm.value = 0
        motor.stop()


def motors_stop():
    pwm_a.value = 0
    pwm_b.value = 0
    motor_a.stop()
    motor_b.stop()
    stby.off()


def motors_drive(right_ratio, left_ratio):
    """左右のタイヤを前進方向で駆動する。0 なら、そのタイヤは停止。"""
    stby.on()
    _run_tire(_right_tire, right_ratio)
    _run_tire(_left_tire, left_ratio)


def drive_forward(duration_sec):
    """指定秒数だけ前進して止まる (停止スイッチで中断できる)。"""
    motors_drive(DRIVE_SPEED, DRIVE_SPEED)
    stop_event.wait(duration_sec)
    motors_stop()


def turn_pulse(direction_deg):
    """
    direction_deg > 0 なら右へ、< 0 なら左へ、TURN_PULSE_SECONDS だけ旋回して止まる。
    片輪だけ駆動して曲がる (右へ曲がる = 左タイヤだけ前進)。
    """
    if direction_deg > 0:
        motors_drive(0, TURN_SPEED)
    else:
        motors_drive(TURN_SPEED, 0)
    stop_event.wait(TURN_PULSE_SECONDS)
    motors_stop()


# ===========================================================================
# スイッチ (1回目=走行開始 / 走行中にもう一度=緊急停止)
# ===========================================================================
_button = Button(SWITCH_PIN, bounce_time=0.05)


def _on_pressed():
    if not start_event.is_set():
        print("[スイッチ] 走行開始")
        start_event.set()
    else:
        print("[スイッチ] 緊急停止")
        stop_event.set()


_button.when_pressed = _on_pressed

# ===========================================================================
# 計算 (距離・方位)  ハード不要の純粋関数
# ===========================================================================


def calc_distance(lat1, lng1, lat2=TARGET_LAT, lng2=TARGET_LNG):
    """2点間の距離 [m] (haversine)。"""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lng2 - lng1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS * math.asin(math.sqrt(a))


def calc_angle(lat1, lng1, lat2=TARGET_LAT, lng2=TARGET_LNG):
    """現在地から見た目的地の方位 [度] (0=北, 90=東, 180=南, 270=西)。"""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lng2 - lng1)
    x = math.sin(dl) * math.cos(p2)
    y = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def bearing_diff(heading, target_bearing):
    """目的方位とのズレ [-180, 180]。プラス=右に曲がる、マイナス=左に曲がる。"""
    return (target_bearing - heading + 180.0) % 360.0 - 180.0


def get_azimuth():
    with _lock:
        return azimuth


def get_navigation():
    """
    最新の (距離[m], 目的方位[度], 機体方位[度], ズレ[度]) を返す。
    GPS が Fix していなければ None。
    """
    with _lock:
        fix, la, ln, az = gps_detect, lat, lng, azimuth
    if not fix:
        return None
    dist = calc_distance(la, ln)
    ang = calc_angle(la, ln)
    return dist, ang, az, bearing_diff(az, ang)


def set_phase(p):
    global phase
    phase = p


# ===========================================================================
# スレッド
# ===========================================================================


def GPS_thread():
    """GPSモジュールを読み、緯度経度を更新する (GPS.py と同方式)。"""
    global lat, lng, gps_detect, gps_error

    try:
        s = serial.Serial(GPS_PORT, GPS_BAUDRATE, timeout=5)
    except serial.SerialException as e:
        gps_error = str(e)
        print(f"[ERROR] シリアルポートを開けません: {e}")
        return

    s.readline()  # 最初の1行は中途半端なことがあるので捨てる
    gps = MicropyGPS(GPS_LOCAL_OFFSET, "dd")

    while True:
        raw = s.readline()
        if not raw:
            continue

        if s.in_waiting > 64:  # バッファが溜まったら捨てる
            s.reset_input_buffer()

        sentence = raw.decode("ascii", errors="replace")
        if not sentence.startswith("$"):
            continue

        for ch in sentence:  # 1文字ずつ micropyGPS に渡す
            gps.update(ch)

        la, ns = gps.latitude
        ln, ew = gps.longitude
        if ns == "S":
            la = -la
        if ew == "W":
            ln = -ln

        # GGA の Fix品質 (fix_stat) が 0 なら未Fix (元プログラムの gps_qual と同じ判定)
        fixed = gps.fix_stat > 0 and la != 0.0 and ln != 0.0
        with _lock:
            gps_detect = 1 if fixed else 0
            if fixed:
                lat, lng = la, ln


def setData_thread():
    """BNO055 を読んで方位を更新し、CSV に記録する。"""
    global azimuth

    with open(fileName, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "Time", "Phase",
                "AccX", "AccY", "AccZ",
                "GyroX", "GyroY", "GyroZ",
                "MagX", "MagY", "MagZ",
                "LAT", "LNG", "Distance", "Azimuth", "Angle", "Direction",
            ]
        )

        while True:
            try:
                acc = list(bmx.getAcc())
                gyro = list(bmx.getGyro())
                mag = list(bmx.getMag())
                heading = bmx.getEuler()[0]
            except Exception as e:  # センサ一時エラーでスレッドを止めない
                print(f"[WARN] BNO055読み取り失敗: {e}")
                time.sleep(DATA_SAMPLING_RATE)
                continue

            with _lock:
                azimuth = (heading + MAG_DECLINATION_DEG) % 360.0

            nav = get_navigation()
            with _lock:
                la, ln = lat, lng
            if nav is None:
                dist = ang = direction = ""
            else:
                dist, ang, _, direction = nav

            writer.writerow(
                [
                    round(time.time() - start, 3), phase,
                    *acc, *gyro, *mag,
                    la, ln, dist, azimuth, ang, direction,
                ]
            )
            f.flush()
            time.sleep(DATA_SAMPLING_RATE)


# ===========================================================================
# 走行ロジック
# ===========================================================================


def wait_for_gps_fix():
    print("GPSのFixを待っています(屋外で空が見える場所へ)...")
    set_phase(PHASE_WAIT_GPS)
    while True:
        if stop_event.is_set():
            return False
        if gps_error:
            print("[ERROR] GPSが使えないため終了します")
            return False
        with _lock:
            if gps_detect:
                break
        stop_event.wait(1.0)
    print("GPS Fix取得完了")
    return True


def align_to_bearing(target_bearing):
    """
    BNO055 の値を見ながら、目的方位を向くまで旋回パルスを繰り返す。
    (パルス → 停止 → 揺れが収まるのを待つ → 測り直す)
    """
    set_phase(PHASE_ALIGN)
    for _ in range(MAX_ALIGN_ATTEMPTS):
        if stop_event.is_set():
            motors_stop()
            return

        diff = bearing_diff(get_azimuth(), target_bearing)
        if abs(diff) <= HEADING_TOLERANCE_DEG:
            return

        turn_pulse(diff)
        stop_event.wait(TURN_SETTLE_SECONDS)

    print("警告: 規定回数のパルスで向きを合わせきれませんでした")


def run_navigation():
    while True:
        if stop_event.is_set():
            print("緊急停止が押されました")
            return

        nav = get_navigation()
        if nav is None:
            print("GPSを見失いました。停止して待機します。")
            motors_stop()
            stop_event.wait(1.0)
            continue

        distance, target_bearing, heading, diff = nav
        print(f"目的地まで残り {distance:.1f} m")
        if distance <= ARRIVAL_RADIUS_M:
            set_phase(PHASE_GOAL)
            print("到着しました!")
            return

        if abs(diff) > HEADING_TOLERANCE_DEG:
            print(f"向きを補正中(ズレ {diff:+.1f}度)")
            align_to_bearing(target_bearing)
        else:
            set_phase(PHASE_DRIVE)
            drive_forward(MOVE_BURST_SECONDS)


def Setup():
    if not bmx.setUp():
        raise RuntimeError("BNO055の初期化に失敗しました (I2C配線・アドレス0x28を確認)")
    print("BNO055 キャリブレーション(sys,gyro,acc,mag; 3が最良):", bmx.getCalibrationStatus())

    fileName.parent.mkdir(parents=True, exist_ok=True)

    threading.Thread(target=GPS_thread, daemon=True).start()
    threading.Thread(target=setData_thread, daemon=True).start()
    time.sleep(0.5)  # 最初のセンサ値が入るまで少し待つ
    print("Setup OK")


def check_turn():
    """走行前の確認: 右パルスで方位が増え、左パルスで減れば配線・向きはOK。"""
    for name, d in (("右", +1), ("左", -1)):
        before = get_azimuth()
        turn_pulse(d)
        time.sleep(TURN_SETTLE_SECONDS)
        after = get_azimuth()
        change = bearing_diff(before, after)
        print(f"{name}パルス: 方位 {before:.1f} → {after:.1f}  (変化 {change:+.1f}度)")
    print("右で増加・左で減少ならOK。逆なら MOTOR_A_IS_RIGHT を反転、"
          "動かなければBNO055の取り付け向きを確認。")


# ===========================================================================
# メイン
# ===========================================================================


def main():
    global start
    start = time.time()

    Setup()

    if len(sys.argv) > 1 and sys.argv[1] == "check":
        check_turn()
        return

    print("スイッチが押されるのを待っています...")
    set_phase(PHASE_WAIT_SWITCH)
    start_event.wait()

    if not wait_for_gps_fix():
        return

    run_navigation()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n中断しました")
    finally:
        motors_stop()
        pwm_a.close()
        pwm_b.close()
        motor_a.close()
        motor_b.close()
        stby.close()
        _button.close()
