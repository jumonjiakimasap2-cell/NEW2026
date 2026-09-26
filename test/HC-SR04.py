"""
HC-SR04.py
==========
超音波距離センサ HC-SR04 ドライバ & リアルタイム距離表示

NSE2026/sensor/HC-SR04.py

ピン (BCM):
    TRIG = 8   (出力)
    ECHO = 7   (入力)

gpiozero + LGPIOFactory を使用。
    ← test_run.py / fall.py / test_GPSrun.py / test_onoff.py と同じバックエンド。
    ※ NICS2026/HC-SR04.py は RPi.GPIO だが、
       NSE2026 全体は gpiozero + lgpio に統一しているため移植する。

距離計算 (NICS2026/HC-SR04.py と同じ原理):
    超音波パルスの往復時間 × 音速 / 2
    distance_cm = duration_s × 34300 / 2
    distance_m  = duration_s × 343.0  / 2

測定可能範囲: 約 2 cm 〜 400 cm
タイムアウト: ECHO が一定時間内に応答しない場合は None を返す

スタンドアロン実行:
    python3 HC-SR04.py
    → 前方オブジェクトまでの距離をリアルタイムで 1 行表示し続ける。
    Ctrl+C で終了。

他モジュールからのインポート:
    from HC_SR04 import HCSR04
    sensor = HCSR04()
    dist_m = sensor.get_distance_m()   # None = 測定失敗
    sensor.close()
"""

import sys
import time
import datetime
from pathlib import Path

# --- gpiozero (NSE2026 全体と同じバックエンド) ---
from gpiozero import DigitalOutputDevice, DigitalInputDevice, Device
from gpiozero.pins.lgpio import LGPIOFactory

# ===========================================================================
# 設定
# ===========================================================================

PIN_TRIG = 8        # BCM8  (TRIG)
PIN_ECHO = 7        # BCM7  (ECHO)

SOUND_SPEED   = 343.0    # [m/s]  20°C 空気中の音速
TRIGGER_PULSE = 10e-6    # [s]    TRIG パルス幅 10 µs (HC-SR04 仕様)
SETTLE_TIME   = 0.01     # [s]    TRIG 送出前の安定待機 (NICS2026 と同じ 10 ms)
ECHO_TIMEOUT  = 0.03     # [s]    ECHO 待機タイムアウト (≈ 5 m 相当)

LOOP_INTERVAL = 0.1      # [s]    表示ループ周期 (10 Hz)
DISPLAY_UNIT  = "cm"     # "cm" または "m"  表示単位

# ===========================================================================
# HC-SR04 クラス
# ===========================================================================

class HCSR04:
    """
    HC-SR04 超音波距離センサ制御クラス。

    gpiozero の DigitalOutputDevice / DigitalInputDevice を使い、
    NICS2026/HC-SR04.py の get_distance() ロジックを再実装。
    """

    def __init__(self,
                 pin_trig: int = PIN_TRIG,
                 pin_echo: int = PIN_ECHO):
        Device.pin_factory = LGPIOFactory()
        self._trig = DigitalOutputDevice(pin_trig, initial_value=False)
        self._echo = DigitalInputDevice(pin_echo)
        self._pin_trig = pin_trig
        self._pin_echo = pin_echo

    # ── 距離取得 ──────────────────────────────────────────────────────────

    def get_distance_m(self) -> float | None:
        """
        前方オブジェクトまでの距離を [m] で返す。
        測定失敗（タイムアウト）の場合は None を返す。

        計算式 (NICS2026/HC-SR04.py と同じ原理):
            distance = duration × 音速 / 2
        """
        # 安定待機 (NICS2026: GPIO.output(TRIG, False) + sleep(0.01))
        self._trig.off()
        time.sleep(SETTLE_TIME)

        # TRIG パルス送出 (10 µs)
        self._trig.on()
        time.sleep(TRIGGER_PULSE)
        self._trig.off()

        # ECHO 立ち上がり待機 (NICS2026: while GPIO.input(ECHO)==0)
        timeout_start = time.time()
        while not self._echo.value:
            if time.time() - timeout_start > ECHO_TIMEOUT:
                return None   # タイムアウト (オブジェクトなし or 範囲外)
        pulse_start = time.time()

        # ECHO 立ち下がり待機 (NICS2026: while GPIO.input(ECHO)==1)
        while self._echo.value:
            if time.time() - pulse_start > ECHO_TIMEOUT:
                return None   # タイムアウト (エコー消失)
        pulse_end = time.time()

        duration = pulse_end - pulse_start
        return duration * SOUND_SPEED / 2.0

    def get_distance_cm(self) -> float | None:
        """距離を [cm] で返す。測定失敗時は None。"""
        dist_m = self.get_distance_m()
        return dist_m * 100.0 if dist_m is not None else None

    # ── リソース解放 ──────────────────────────────────────────────────────

    def close(self):
        """GPIO リソースを解放する。"""
        self._trig.off()
        self._trig.close()
        self._echo.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def __repr__(self):
        return (f"HCSR04(TRIG=BCM{self._pin_trig}, "
                f"ECHO=BCM{self._pin_echo})")

# ===========================================================================
# 表示フォーマット
# ===========================================================================

_HEADER = (
    "{:>10}  {:>12}  {:>10}  {:>10}  {:>14}"
)
HEADER = _HEADER.format(
    "Time", "Dist[cm]", "Dist[m]", "Status", "Bar"
)
SEPARATOR = "-" * len(HEADER)

# 距離バー表示 (最大 400 cm)
_BAR_MAX_CM = 400.0
_BAR_WIDTH  = 20

def _distance_bar(dist_cm: float) -> str:
    """距離を ASCII バーグラフで表現する。"""
    ratio  = min(dist_cm / _BAR_MAX_CM, 1.0)
    filled = int(ratio * _BAR_WIDTH)
    return "[" + "█" * filled + "░" * (_BAR_WIDTH - filled) + "]"


def format_row(dist_m: float | None) -> str:
    """測定値を 1 行の文字列にフォーマットして返す。"""
    now_str = datetime.datetime.now().strftime("%H:%M:%S.%f")[:11]

    if dist_m is None:
        return _HEADER.format(
            now_str, "---", "---", "TIMEOUT", "[ out of range  ]"
        )

    dist_cm = dist_m * 100.0

    if dist_cm < 2.0:
        status = "TOO_CLOSE"
    elif dist_cm > 400.0:
        status = "TOO_FAR"
    else:
        status = "OK"

    bar = _distance_bar(dist_cm)

    return "{:>10}  {:>12.2f}  {:>10.4f}  {:>10}  {:>14}".format(
        now_str,
        dist_cm,
        dist_m,
        status,
        bar,
    )

# ===========================================================================
# スタンドアロン実行
# ===========================================================================

def main():
    print("=" * len(HEADER))
    print("  HC-SR04.py  超音波距離センサ リアルタイム表示")
    print(f"  TRIG=BCM{PIN_TRIG}  ECHO=BCM{PIN_ECHO}  "
          f"更新: {1/LOOP_INTERVAL:.0f} Hz  (Ctrl+C で終了)")
    print("=" * len(HEADER))

    sensor     = HCSR04(pin_trig=PIN_TRIG, pin_echo=PIN_ECHO)
    line_count = 0

    try:
        while True:
            loop_start = time.time()

            # ヘッダー再表示 (30 行おき)
            if line_count % 30 == 0:
                print(SEPARATOR)
                print(HEADER)
                print(SEPARATOR)

            dist_m = sensor.get_distance_m()
            print(format_row(dist_m))
            line_count += 1

            # ループ待機
            wait = LOOP_INTERVAL - (time.time() - loop_start)
            if wait > 0:
                time.sleep(wait)

    except KeyboardInterrupt:
        print(f"\n[INFO] 終了  ({line_count} 回測定)")

    finally:
        sensor.close()
        print("[INFO] GPIO リソースを解放しました。")


# ===========================================================================
if __name__ == "__main__":
    main()
