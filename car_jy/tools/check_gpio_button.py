#!/usr/bin/env python3
"""
AMR Raspberry Pi GPIO Button Diagnostic & Scanner Tool (v2)
- 초기 상시 접지 핀(모터 드라이버/로봇 하드웨어가 쓰는 핀) 자동 필터링
- 버튼을 누를 때 1 -> 0 으로 '순간 변화(Transition)'하는 핀만 정확히 감지
"""

import sys
import time
import os

print("\n" + "="*65)
print("   🔍 라즈베리파이 버튼 & GPIO 실시간 진단기 (v2)")
print("="*65)

# GPIO 라이브러리 로드
gpio_mode = None
try:
    from gpiozero import Button
    gpio_mode = "gpiozero"
    print("✓ gpiozero 라이브러리 로드 성공")
except ImportError:
    try:
        import RPi.GPIO as GPIO
        gpio_mode = "RPi.GPIO"
        print("✓ RPi.GPIO 라이브러리 로드 성공")
    except ImportError:
        try:
            import gpiod
            gpio_mode = "gpiod"
            print("✓ gpiod 라이브러리 로드 성공")
        except ImportError:
            print("\n❌ [오류] GPIO 라이브러리가 없습니다.")
            sys.exit(1)

# 감시할 주요 GPIO 핀 목록
PIN_MAP = {
    18: "물리 12번 (★기본 추천 핀)",
    17: "물리 11번",
    27: "물리 13번",
    22: "물리 15번",
    23: "물리 16번",
    24: "물리 18번",
    25: "물리 22번",
    4:  "물리 7번",
    5:  "물리 29번",
    6:  "물리 31번",
    12: "물리 32번",
    13: "물리 33번",
    16: "물리 36번",
    19: "물리 35번 (로봇 보드 내부 사용 핀)",
    20: "물리 38번 (로봇 보드 내부 사용 핀)",
    21: "물리 40번",
}

print("⚙️ 초기 핀 상태 측정 및 로봇 하드웨어 기사용 핀 제외 중...")

if gpio_mode == "gpiozero":
    buttons = {}
    for pin in PIN_MAP:
        try:
            buttons[pin] = Button(pin, pull_up=True, bounce_time=0.03)
        except Exception:
            pass

    time.sleep(0.3)
    # 초기 기사용 핀(상시 0인 핀) 자동 제외
    always_grounded = set()
    for pin, btn in buttons.items():
        if btn.is_pressed:
            always_grounded.add(pin)

    if always_grounded:
        print(f"ℹ️ [알림] BCM {sorted(list(always_grounded))}번 핀은 로봇 보드가 이미 사용 중(GND 연결)이므로 제외합니다.")

    valid_pins = {p: b for p, b in buttons.items() if p not in always_grounded}

    print(f"\n📡 [실시간 스캔 시작] 유효 핀 {len(valid_pins)}개 감시 중!")
    print("👉 지금 버튼을 '딸깍' 눌렀다 떼보세요!")
    print("   종료: Ctrl + C\n")

    last_states = {p: valid_pins[p].is_pressed for p in valid_pins}
    last_heartbeat = time.time()

    try:
        while True:
            for p, btn in valid_pins.items():
                curr = btn.is_pressed
                prev = last_states[p]
                if curr and not prev:
                    desc = PIN_MAP.get(p, f"BCM {p}")
                    print(f"\n🎉 [★버튼 클릭 감지★] 🔘 BCM {p}번 핀 ({desc}) 이 눌렸습니다!!")
                elif not curr and prev:
                    desc = PIN_MAP.get(p, f"BCM {p}")
                    print(f"⚪ [버튼 뗌] BCM {p}번 핀 ({desc}) 에서 손을 뗐습니다.")
                last_states[p] = curr

            now = time.time()
            if now - last_heartbeat > 2.0:
                b18 = valid_pins.get(18)
                st18 = "눌림(0)" if (b18 and b18.is_pressed) else "대기중(1, 정상)"
                print(f"  [모니터링 정상 작동 중...] BCM 18번(물리 12번) 상태: {st18}")
                last_heartbeat = now

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

    time.sleep(0.3)
    always_grounded = {p for p in active_pins if GPIO.input(p) == 0}
    if always_grounded:
        print(f"ℹ️ [알림] BCM {sorted(list(always_grounded))}번 핀은 로봇 보드가 이미 사용 중이므로 제외합니다.")

    valid_pins = [p for p in active_pins if p not in always_grounded]
    print(f"\n📡 [실시간 스캔 시작] 유효 핀 {len(valid_pins)}개 감시 중!")
    print("👉 지금 버튼을 '딸깍' 눌렀다 떼보세요!\n")

    last_states = {p: GPIO.input(p) for p in valid_pins}
    last_heartbeat = time.time()

    try:
        while True:
            for p in valid_pins:
                val = GPIO.input(p)
                prev = last_states[p]
                if val == 0 and prev == 1:
                    desc = PIN_MAP.get(p, f"BCM {p}")
                    print(f"\n🎉 [★버튼 클릭 감지★] 🔘 BCM {p}번 핀 ({desc}) 이 눌렸습니다!!")
                elif val == 1 and prev == 0:
                    desc = PIN_MAP.get(p, f"BCM {p}")
                    print(f"⚪ [버튼 뗌] BCM {p}번 핀 ({desc}) 에서 손을 뗐습니다.")
                last_states[p] = val

            now = time.time()
            if now - last_heartbeat > 2.0:
                v18 = GPIO.input(18) if 18 in valid_pins else -1
                st18 = "눌림(0)" if v18 == 0 else "대기중(1, 정상)"
                print(f"  [모니터링 정상 작동 중...] BCM 18번(물리 12번) 상태: {st18}")
                last_heartbeat = now

            time.sleep(0.02)
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("\n진단 종료.")
