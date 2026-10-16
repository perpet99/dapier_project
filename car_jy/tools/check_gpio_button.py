#!/usr/bin/env python3
"""
AMR Raspberry Pi GPIO Button Diagnostic & Scanner Tool
라즈베리파이 물리 버튼 및 GPIO 핀 실시간 진단 / 자동 감지 도구

기능:
1. 지정한 핀(기본: BCM 18)의 실시간 전압 상태(HIGH:1, LOW:0)를 0.05초 주기로 출력
2. 만약 핀을 다른 곳에 꽂았더라도, 40핀 헤더 전체 GPIO(2~27)를 동시 감시하여
   "어느 핀에서 신호가 들어오는지" 자동으로 찾아냄!
"""

import sys
import time
import os

print("\n" + "="*60)
print("   🔍 라즈베리파이 버튼 & GPIO 실시간 진단기")
print("="*60)

# GPIO 라이브러리 로드
gpio_mode = None
try:
    from gpiozero import Button, Device
    gpio_mode = "gpiozero"
    print("✓ gpiozero 라이브러리 사용")
except ImportError:
    try:
        import RPi.GPIO as GPIO
        gpio_mode = "RPi.GPIO"
        print("✓ RPi.GPIO 라이브러리 사용")
    except ImportError:
        try:
            import gpiod
            gpio_mode = "gpiod"
            print("✓ gpiod 라이브러리 사용")
        except ImportError:
            print("\n❌ [오류] GPIO 라이브러리가 하나도 설치되어 있지 않습니다!")
            print("  아래 명령어로 라이브러리를 설치한 후 다시 실행해주세요:")
            print("    pip install gpiozero --break-system-packages")
            print("    또는: sudo apt install -y python3-gpiozero")
            sys.exit(1)

# 감시할 주요 GPIO 핀 목록 (BCM 번호 및 물리 핀 매핑)
PIN_MAP = {
    18: "물리 12번 (기본 권장 핀)",
    17: "물리 11번",
    27: "물리 13번",
    22: "물리 15번",
    23: "물리 16번",
    24: "물리 18번 (★물리 18번에 잘못 꽂았을 때)",
    25: "물리 22번",
    4:  "물리 7번",
    5:  "물리 29번",
    6:  "물리 31번",
    12: "물리 32번",
    13: "물리 33번",
    16: "물리 36번",
    19: "물리 35번",
    20: "물리 38번",
    21: "물리 40번",
}

print(f"\n📡 [감시 시작] 주요 GPIO 핀 16개를 실시간 스캔 중입니다...")
print("👉 버튼을 꾹 누르고 있어보세요! (또는 점퍼선 두 가닥을 직접 맞대보세요)")
print("   종료: Ctrl + C\n")

if gpio_mode == "gpiozero":
    buttons = {}
    for pin in PIN_MAP:
        try:
            buttons[pin] = Button(pin, pull_up=True, bounce_time=0.03)
        except Exception:
            pass

    try:
        last_print = 0
        while True:
            pressed_pins = []
            for pin, btn in buttons.items():
                if btn.is_pressed:
                    pressed_pins.append(pin)

            now = time.time()
            if pressed_pins:
                for p in pressed_pins:
                    desc = PIN_MAP.get(p, f"BCM {p}")
                    print(f"🎉 [버튼 신호 감지!!] 🔘 BCM {p}번 핀 ({desc}) 이 눌렸습니다! (LOW: 0)")
                time.sleep(0.3)
            elif now - last_print > 1.5:
                # 18번 핀 상태 주기적 보고
                b18 = buttons.get(18)
                val_str = "눌림 (LOW 0)" if (b18 and b18.is_pressed) else "열림/평상시 (HIGH 1)"
                print(f"  [모니터링 중...] BCM 18번(물리 12번) 상태: {val_str} | 신호 대기 중...")
                last_print = now

            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n진단 종료.")

elif gpio_mode == "RPi.GPIO":
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    active_pins = []
    for pin in PIN_MAP:
        try:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            active_pins.append(pin)
        except Exception:
            pass

    try:
        last_print = 0
        while True:
            pressed_pins = []
            for pin in active_pins:
                if GPIO.input(pin) == 0:  # Active Low
                    pressed_pins.append(pin)

            now = time.time()
            if pressed_pins:
                for p in pressed_pins:
                    desc = PIN_MAP.get(p, f"BCM {p}")
                    print(f"🎉 [버튼 신호 감지!!] 🔘 BCM {p}번 핀 ({desc}) 이 눌렸습니다! (LOW: 0)")
                time.sleep(0.3)
            elif now - last_print > 1.5:
                v18 = GPIO.input(18) if 18 in active_pins else -1
                val_str = "눌림 (LOW 0)" if v18 == 0 else "열림/평상시 (HIGH 1)"
                print(f"  [모니터링 중...] BCM 18번(물리 12번) 상태: {val_str} | 신호 대기 중...")
                last_print = now

            time.sleep(0.02)
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("\n진단 종료.")

elif gpio_mode == "gpiod":
    import gpiod
    chip_name = 'gpiochip4' if os.path.exists('/dev/gpiochip4') else 'gpiochip0'
    chip = gpiod.Chip(chip_name)
    lines = {}
    req_flags = getattr(gpiod, 'LINE_REQ_FLAG_BIAS_PULL_UP', 0)
    for pin in PIN_MAP:
        try:
            line = chip.get_line(pin)
            line.request(consumer="diag", type=gpiod.LINE_REQ_DIR_IN, flags=req_flags)
            lines[pin] = line
        except Exception:
            pass

    try:
        last_print = 0
        while True:
            pressed_pins = []
            for pin, line in lines.items():
                if line.get_value() == 0:
                    pressed_pins.append(pin)

            now = time.time()
            if pressed_pins:
                for p in pressed_pins:
                    desc = PIN_MAP.get(p, f"BCM {p}")
                    print(f"🎉 [버튼 신호 감지!!] 🔘 BCM {p}번 핀 ({desc}) 이 눌렸습니다! (LOW: 0)")
                time.sleep(0.3)
            elif now - last_print > 1.5:
                l18 = lines.get(18)
                v18 = l18.get_value() if l18 else -1
                val_str = "눌림 (LOW 0)" if v18 == 0 else "열림/평상시 (HIGH 1)"
                print(f"  [모니터링 중...] BCM 18번(물리 12번) 상태: {val_str} | 신호 대기 중...")
                last_print = now

            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n진단 종료.")
