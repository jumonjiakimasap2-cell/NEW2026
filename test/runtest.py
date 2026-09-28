from gpiozero import Motor, PWMOutputDevice, OutputDevice  # OutputDevice を追加
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device
import time
import sys

# Use lgpio backend (works on Pi Zero 2 W with modern OS)
Device.pin_factory = LGPIOFactory()

# ===== Pin configuration (BCM numbering) =====
# Converted from BOARD to BCM:
#   BOARD 33 -> BCM 13 (PWMA)
#   BOARD 29 -> BCM 5  (AIN1)
#   BOARD 31 -> BCM 6  (AIN2)
#   BOARD 18 -> BCM 18 (PWMB)
#   BOARD 12 -> BCM 23 (BIN1)
#   BOARD 16 -> BCM 24 (BIN2)
pwm_a = PWMOutputDevice(18)  # PWMA
pwm_b = PWMOutputDevice(13)  # PWMB
motor_a = Motor(forward=23, backward=24)   # AIN1, AIN2
motor_b = Motor(forward=5, backward=6)  # BIN1, BIN2
stby = OutputDevice(17)                  # MOTOR_STBY (追加)

speed = 0.8  # 0.0〜1.0（80%相当）

def stop():
    pwm_a.value = 0
    pwm_b.value = 0
    motor_a.stop()
    motor_b.stop()
    stby.off()  # モーター停止後にSTBYをLOWにする (追加)

def forward():
    stby.on()  # モーター動作直前にSTBYをHIGHにする (追加)
    pwm_a.value = speed
    pwm_b.value = speed
    motor_a.forward()
    motor_b.forward()

def backward():
    stby.on()  # モーター動作直前にSTBYをHIGHにする (追加)
    pwm_a.value = speed
    pwm_b.value = speed
    motor_a.backward()
    motor_b.backward()

def left():
    stby.on()  # モーター動作直前にSTBYをHIGHにする (追加)
    pwm_a.value = speed * 0.5
    pwm_b.value = speed
    motor_a.forward()
    motor_b.forward()

def right():
    stby.on()  # モーター動作直前にSTBYをHIGHにする (追加)
    pwm_a.value = speed
    pwm_b.value = speed * 0.5
    motor_a.forward()
    motor_b.forward()

# ===== キー入力（1文字取得）=====
def get_key():
    try:
        import termios, tty
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ch.lower()
    except:
        return input().strip().lower()


# W/A/S/Dキー操作ループ
try:
    print("操作開始：W/A/S/D で移動、Space で停止、Q で終了")
    while True:
        cmd = get_key()
        if cmd == 'w':   forward()
        elif cmd == 's': backward()
        elif cmd == 'a': left()
        elif cmd == 'd': right()
        elif cmd == ' ': stop()
        elif cmd == 'q': break
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\n強制終了")
finally:
    stop()
    pwm_a.close(); pwm_b.close()
    motor_a.close(); motor_b.close()
    stby.close()  # STBYピンのリソース解放 (追加)
