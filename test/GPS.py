"""
GPS.py
======
GT-502MGG-N GPS モジュール  緯度経度 + 全情報 表示プログラム

NSE2026/sensor/GPS.py

対象ハードウェア:
    GPS モジュール : GT-502MGG-N (秋月電子 G117980)
                    UART 出力 9600 bps / NMEA 0183
    接続           : Raspberry Pi Zero 2W の UART
                    TX(GPS) → RX(ラズパイ) BCM15 / /dev/serial0

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
【前回エラーの原因と対処】

原因 1: シリアルポートが正しく開けていない可能性
    → 起動直後にポートを開けたか・データが来ているかを逐一 print で確認する
      デバッグモード (DEBUG=True) を追加。

原因 2: clean_sentences が増えない
    → micropyGPS.update() は 1 文字ずつ渡す必要がある。
      readline() で取得した文字列を for ループで 1 文字ずつ渡す必要がある。
      NICS2026/GPS_test.py の rungps() も同じ方式。
      前回コードは for x in sentence: gps.update(x) と書いていたが、
      decode('utf-8') が失敗していた可能性がある (errors='replace' で回避)。

原因 3: while True で sleep なし → CPU 100% + print が出力されない
    → メインループに time.sleep(1.0) を入れる。
      ただし clean_sentences > 20 の条件が満たされない間は何も出ないため、
      待機中も状態を表示する進捗表示を追加。

原因 4: /dev/serial0 が使えない設定になっている場合
    → raspi-config で UART を有効化する必要がある。
      起動時に設定確認メッセージを表示する。
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

使い方:
    python3 GPS.py           # 通常起動
    python3 GPS.py debug     # デバッグモード (生 NMEA センテンスも表示)

表示例:
    [OK] lat: 38.260520°N   lon: 140.854415°E
         速度: 0.23kt (0.43km/h)  方位: 275.0°  高度: 42.3m
         Fix: 3D  衛星: 8機  HDOP: 1.20  時刻(JST): 09:15:30

UART 有効化 (未設定の場合):
    sudo raspi-config → Interface Options → Serial Port
    "login shell over serial?" → No
    "serial port hardware enabled?" → Yes
    再起動後 /dev/serial0 が使えるようになる。
"""

import sys
import threading
import time
from pathlib import Path

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("[ERROR] pyserial が見つかりません。")
    print("        pip install pyserial --break-system-packages")
    sys.exit(1)

# --- micropyGPS パス解決 ---
_THIS_DIR = Path(__file__).resolve().parent   # NSE2026/sensor/
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

_CURRENT_DIR = Path(__file__).resolve().parent
_SENSOR_DIR = _CURRENT_DIR.parent / "sensor"
if str(_SENSOR_DIR) not in sys.path:
    sys.path.insert(0, str(_SENSOR_DIR))
    
from micropyGPS import MicropyGPS

# ===========================================================================
# 設定
# ===========================================================================

PORT         = "/dev/serial0"   # Raspberry Pi Zero 2W の UART
BAUDRATE     = 9600             # GT-502MGG-N デフォルト
LOCAL_OFFSET = 9                # JST (+9h)
WAIT_COUNT   = 20               # clean_sentences がこの数を超えたら表示開始
                                # (NICS2026/GPS_test.py と同じ閾値)

# コマンドライン引数 "debug" でデバッグモード ON
DEBUG = len(sys.argv) > 1 and sys.argv[1].lower() == "debug"

# ===========================================================================
# 共有状態
# ===========================================================================

gps       = MicropyGPS(LOCAL_OFFSET, 'dd')   # 'dd' = 十進度表示
gps_lock  = threading.Lock()
gps_error = None   # スレッド内で発生したエラーを格納

# ===========================================================================
# GPS 受信スレッド
# (NICS2026/GPS_test.py の rungps() を基に、エラー処理・デバッグ表示を強化)
# ===========================================================================

def rungps():
    global gps_error

    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=5)
        print(f"[GPS] ポートを開きました: {PORT} @ {BAUDRATE} bps")
    except serial.SerialException as e:
        gps_error = str(e)
        print(f"\n[ERROR] シリアルポートを開けません: {e}")
        print("        以下を確認してください:")
        print("        1. sudo raspi-config → Interface Options → Serial Port")
        print("           'login shell?' → No  /  'hardware enabled?' → Yes  → 再起動")
        print("        2. /dev/serial0 の存在確認: ls -la /dev/serial*")
        print("        3. ユーザーが dialout グループに所属しているか:")
        print("           sudo usermod -aG dialout $USER  → ログアウト・再ログイン")
        return

    # 先頭の不完全な行を捨てる (NICS2026/GPS_test.py と同じ)
    ser.readline()

    print("[GPS] NMEA センテンス受信中...")

    while True:
        try:
            # タイムアウト内にデータが来なければ空バイト列が返る
            raw = ser.readline()

            if not raw:
                if DEBUG:
                    print("[GPS] タイムアウト: データなし")
                continue

            # デコード (errors='replace' で文字化けを回避)
            sentence = raw.decode('ascii', errors='replace')

            if DEBUG:
                print(f"[RAW] {sentence.strip()}")

            # '$' で始まらない行は捨てる (NICS2026/GPS_test.py と同じ)
            if not sentence.startswith('$'):
                continue

            # 1 文字ずつ micropyGPS に渡す (これが正しい使い方)
            with gps_lock:
                for char in sentence:
                    gps.update(char)

        except Exception as e:
            if DEBUG:
                print(f"[GPS] 読み取りエラー: {e}")
            time.sleep(0.1)

# ===========================================================================
# 表示関数
# ===========================================================================

def print_gps_info():
    """
    緯度・経度を中心に GPS 全情報をコンソールに表示する。
    'dd' モードでは gps.latitude / longitude が float で返る。
    """
    with gps_lock:
        # --- 位置 ---
        lat = gps.latitude   # 'dd' モード → float [度]
        lng = gps.longitude

        # --- 時刻 (JST 補正済み) ---
        h  = gps.timestamp[0]
        m  = gps.timestamp[1]
        s  = gps.timestamp[2]
        # MicropyGPS は local_offset 適用後に 24 を超える場合がある
        if h >= 24:
            h -= 24

        # --- 日付 ---
        d  = gps.date[0]
        mo = gps.date[1]
        y  = gps.date[2]

        # --- 速度・方位・高度 ---
        spd_kt  = gps.speed[0]
        spd_kmh = gps.speed[2]
        course  = gps.course
        alt     = gps.altitude
        geoid   = gps.geoid_height

        # --- Fix ---
        fix_type = {1: "NO_FIX", 2: "2D", 3: "3D"}.get(gps.fix_type, "?")
        sats_use  = gps.satellites_in_use
        sats_view = gps.satellites_in_view
        hdop = gps.hdop
        pdop = gps.pdop
        vdop = gps.vdop

        # --- 統計 ---
        clean  = gps.clean_sentences
        crc_ng = gps.crc_fails
        valid  = gps.valid

    # --- 表示 ---
    sep = "─" * 65
    print(sep)

    # 完全に数値(floatかint)であり、かつ初期値の0.0ではない場合のみ位置を表示する
    if isinstance(lat, (int, float)) and isinstance(lng, (int, float)) and lat != 0.0 and lng != 0.0:
        # 十進度 → 度分秒に変換して両方表示
        lat_deg = int(abs(lat))
        lat_min = (abs(lat) - lat_deg) * 60
        lng_deg = int(abs(lng))
        lng_min = (abs(lng) - lng_deg) * 60
        lat_hemi = "N" if lat >= 0 else "S"
        lng_hemi = "E" if lng >= 0 else "W"

        print(f"  【位置】")
        print(f"    緯度 : {lat:>12.6f}°{lat_hemi}   "
              f"({lat_deg}° {lat_min:.4f}′ {lat_hemi})")
        print(f"    経度 : {lng:>12.6f}°{lng_hemi}   "
              f"({lng_deg}° {lng_min:.4f}′ {lng_hemi})")
    else:
        print(f"  【位置】Fix 未取得 (衛星を捕捉中...)")
        print(f"    緯度 :      ---")
        print(f"    経度 :      ---")

    print(f"  【時刻】 {y:04d}-{mo:02d}-{d:02d}  {h:02d}:{m:02d}:{int(s):02d} JST")
    print(f"  【速度】 {spd_kt:.3f} kt  ({spd_kmh:.3f} km/h)   方位: {course:.1f}°")
    print(f"  【高度】 {alt:.2f} m   ジオイド高: {geoid:.2f} m")
    print(f"  【Fix】  {fix_type}   衛星: {sats_use} 機使用 / {sats_view} 機視野")
    print(f"  【精度】 HDOP={hdop:.2f}  PDOP={pdop:.2f}  VDOP={vdop:.2f}")
    print(f"  【統計】 正常センテンス={clean}  CRC失敗={crc_ng}")

# ===========================================================================
# メイン
# ===========================================================================

def main():
    print("=" * 65)
    print("  GPS.py  GT-502MGG-N  緯度経度 リアルタイム表示")
    print(f"  Port: {PORT}  Baudrate: {BAUDRATE}  JST+{LOCAL_OFFSET}")
    if DEBUG:
        print("  *** DEBUG モード: 生 NMEA センテンスも表示します ***")
    print("=" * 65)
    print()
    print("  UART が使えない場合は以下を実行して再起動してください:")
    print("    sudo raspi-config")
    print("    → Interface Options → Serial Port")
    print("    → 'login shell?' = No  /  'hardware enabled?' = Yes")
    print()

    # --- GPS スレッド起動 ---
    t = threading.Thread(target=rungps, daemon=True)
    t.start()

    # スレッドが起動するまで少し待つ
    time.sleep(1.0)

    # --- エラーチェック ---
    if gps_error:
        print(f"[ERROR] GPS スレッドがエラーで終了しました。終了します。")
        sys.exit(1)

    # --- clean_sentences が溜まるまで待機 ---
    print(f"[INFO] GPS データ受信待機中 (clean_sentences > {WAIT_COUNT} になるまで)...")
    print(f"       (屋外か窓際に設置し、衛星を捕捉するまで数分かかる場合があります)")
    print()

    wait_start = time.time()
    while True:
        with gps_lock:
            cnt   = gps.clean_sentences
            valid = gps.valid
            sats  = gps.satellites_in_use

        elapsed = time.time() - wait_start

        if gps_error:
            print(f"\n[ERROR] GPS スレッドが停止しました。終了します。")
            sys.exit(1)

        if cnt > WAIT_COUNT:
            print(f"\r[INFO] 受信開始！ clean_sentences={cnt}  衛星={sats}  {elapsed:.0f}s経過")
            print()
            break

        # 進捗表示 (カウントが進まないときに何が起きているか分かるように)
        print(f"\r  受信中... clean_sentences={cnt:3d} / {WAIT_COUNT}  "
              f"衛星={sats}  {elapsed:.0f}s経過    ",
              end="", flush=True)

        time.sleep(0.5)

    # --- メイン表示ループ ---
    try:
        while True:
            print_gps_info()
            time.sleep(1.0)   # 1 秒ごとに更新

    except KeyboardInterrupt:
        print("\n\n[INFO] 終了します。")


# ===========================================================================
if __name__ == "__main__":
    main()
