"""
test_gps.py
===========
GPS の数値のみを取得してリアルタイム表示・CSV保存するプログラム

micropyGPS で NMEA センテンスを解析し、以下の値をログ表示する。

使い方:
    python3 test_gps.py
    Ctrl+C で終了。終了時に CSV ログを保存する。

ログ列:
    Time[s] | Fix | Lat | Lng | Alt[m]
            | Speed[kt] | Speed[km/h] | Course[deg]
            | Sats | HDOP | GPS_Time
"""
import sys
import time
import csv
import datetime
import threading
from pathlib import Path

# --- シリアル (GPS 用) ---
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

# --- センサーモジュールのパスを追加 ----------------------------------------
SCRIPT_DIR = Path(__file__).resolve().parent          # .../NSE2026/test/
SENSOR_DIR = SCRIPT_DIR.parent / "sensor"             # .../NSE2026/sensor/
if str(SENSOR_DIR) not in sys.path:
    sys.path.insert(0, str(SENSOR_DIR))

from micropyGPS import MicropyGPS

# ===========================================================================
# 設定
# ===========================================================================

PRINT_INTERVAL = 0.5          # 表示・記録周期 [s]
GPS_PORT       = "/dev/serial0"
GPS_BAUDRATE   = 9600
LOG_DIR        = SCRIPT_DIR.parent / "logs"

# ===========================================================================
# 共有データ
# ===========================================================================

gps_data = {
    "fix":        False,     # 測位が有効か
    "lat":        0.0,       # [deg] 北緯 +, 南緯 -
    "lng":        0.0,       # [deg] 東経 +, 西経 -
    "alt":        0.0,       # [m]
    "speed_kt":   0.0,       # [knots]
    "speed_kmh":  0.0,       # [km/h]
    "course":     0.0,       # [deg]
    "sats":       0,
    "hdop":       0.0,
    "gps_time":   "--:--:--",
}
gps_lock = threading.Lock()
stop_event = threading.Event()

# ===========================================================================
# GPS スレッド
# ===========================================================================

def gps_thread_func(gps_obj: MicropyGPS, port: str, baudrate: int):
    """GPS NMEA センテンスをバックグラウンドで読み続け、共有データを更新する。"""
    if not SERIAL_AVAILABLE:
        print("[GPS] pyserial が見つかりません。GPS は無効です。")
        return

    try:
        with serial.Serial(port, baudrate, timeout=10) as ser:
            print(f"[GPS] Serial open: {port} @ {baudrate} bps")

            # 最初の1行目は中途半端なデータである可能性があるため読み飛ばす
            ser.readline()

            while not stop_event.is_set():
                try:
                    sentence = ser.readline().decode("utf-8", errors="ignore")
                    if sentence == "" or sentence[0] != "$":
                        continue

                    for char in sentence:
                        gps_obj.update(char)

                    # --- 緯度経度 ('dd' 形式: [deg, hemisphere]) ---
                    lat_raw = gps_obj.latitude
                    lng_raw = gps_obj.longitude
                    lat = lat_raw[0] * (-1 if lat_raw[1] == "S" else 1)
                    lng = lng_raw[0] * (-1 if lng_raw[1] == "W" else 1)

                    # --- 時刻 ---
                    h, m, s = gps_obj.timestamp
                    gps_time = f"{int(h):02d}:{int(m):02d}:{int(s):02d}"

                    with gps_lock:
                        gps_data["fix"]       = bool(gps_obj.valid)
                        gps_data["lat"]       = lat
                        gps_data["lng"]       = lng
                        gps_data["alt"]       = gps_obj.altitude
                        gps_data["speed_kt"]  = gps_obj.speed[0]
                        gps_data["speed_kmh"] = gps_obj.speed[2]
                        gps_data["course"]    = gps_obj.course
                        gps_data["sats"]      = gps_obj.satellites_in_use
                        gps_data["hdop"]      = gps_obj.hdop
                        gps_data["gps_time"]  = gps_time

                except Exception as e:
                    print(f"[GPS] 読み取りエラー: {e}")
    except serial.SerialException as e:
        print(f"[GPS] ポートを開けません ({port}): {e}")
        print("[GPS] GPS データは 0.0 で表示されます。")

# ===========================================================================
# 表示フォーマット
# ===========================================================================

HEADER_FMT = (
    "{:>8}  {:>3}  {:>11}  {:>12}  {:>8}  "
    "{:>10}  {:>11}  {:>11}  {:>4}  {:>5}  {:>8}"
)

HEADER = HEADER_FMT.format(
    "Time[s]", "Fix", "Lat", "Lng", "Alt[m]",
    "Speed[kt]", "Speed[km/h]", "Course[deg]", "Sats", "HDOP", "GPS_Time",
)

DATA_FMT = (
    "{:>8.3f}  {:>3}  {:>11.6f}  {:>12.6f}  {:>8.2f}  "
    "{:>10.3f}  {:>11.3f}  {:>11.2f}  {:>4d}  {:>5.2f}  {:>8}"
)

# ===========================================================================
# メイン
# ===========================================================================

def main():
    # --- ログ保存先 ---
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts_str   = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = LOG_DIR / f"gps_{ts_str}.csv"

    print("=" * 90)
    print("  GPS 測定プログラム  (test_gps.py)")
    print("=" * 90)

    # ---- GPS 初期化 ----
    gps_obj = MicropyGPS(local_offset=9, location_formatting="dd")  # JST +9h
    gps_thread = threading.Thread(
        target=gps_thread_func,
        args=(gps_obj, GPS_PORT, GPS_BAUDRATE),
        daemon=True,
    )
    gps_thread.start()
    time.sleep(0.3)

    # ---- コンソールヘッダー ----
    print()
    separator = "-" * len(HEADER)
    print(separator)
    print(HEADER)
    print(separator)

    # ---- CSV ヘッダー ----
    csv_header = [
        "Time_s", "Fix", "Lat", "Lng", "Alt_m",
        "GPS_Speed_kts", "GPS_Speed_kmh", "Course_deg",
        "GPS_Sats", "HDOP", "GPS_Time",
    ]
    log_rows   = []
    start_time = time.time()

    try:
        while True:
            loop_start = time.time()

            # 共有データをスナップショットとして取得
            with gps_lock:
                d = dict(gps_data)

            elapsed = loop_start - start_time

            # --- コンソール表示 ---
            print(DATA_FMT.format(
                elapsed,
                "OK" if d["fix"] else "NG",
                d["lat"], d["lng"], d["alt"],
                d["speed_kt"], d["speed_kmh"], d["course"],
                int(d["sats"]), d["hdop"], d["gps_time"],
            ))

            # --- ログ蓄積 ---
            log_rows.append([
                round(elapsed, 3),
                int(d["fix"]),
                round(d["lat"], 6), round(d["lng"], 6), round(d["alt"], 2),
                round(d["speed_kt"], 3), round(d["speed_kmh"], 3),
                round(d["course"], 2),
                int(d["sats"]), round(d["hdop"], 2), d["gps_time"],
            ])

            # --- 待機 ---
            wait = PRINT_INTERVAL - (time.time() - loop_start)
            if wait > 0:
                time.sleep(wait)

    except KeyboardInterrupt:
        print("\n\n[INFO] Ctrl+C を受信しました。終了処理中...")

    finally:
        stop_event.set()

        # ---- CSV 書き出し ----
        print(f"\n[INFO] ログを保存中: {log_path}")
        try:
            with open(log_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(csv_header)
                writer.writerows(log_rows)
            print(f"[INFO] {len(log_rows)} 行を保存しました → {log_path}")
        except Exception as e:
            print(f"[ERROR] CSV 保存失敗: {e}")

# ===========================================================================
if __name__ == "__main__":
    main()
