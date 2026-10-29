#!/usr/bin/env python3
"""
ESP32 시리얼 포트 탐색 및 데이터 수신 진단 툴 (check_esp32.py)
-----------------------------------------------------------
1. 시스템에 연결된 모든 USB 시리얼 포트(/dev/ttyUSB*, /dev/ttyACM*)를 탐색
2. 각 포트의 장치 이름, 칩셋 제조사(CP2102, CH340 등) 확인
3. 각 포트를 115200 bps로 열어서 ESP32가 보내는 데이터($odom, 센서값 등) 실시간 모니터링
"""

import sys
import time
import glob
import serial
import serial.tools.list_ports


def main():
    print("=" * 65)
    print("        ESP32 시리얼 포트 정밀 진단 툴")
    print("=" * 65)

    # 1. 시리얼 장치 목록 상세 조회
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("❌ 시스템에서 발견된 시리얼 장치가 전혀 없습니다!")
        print("   -> ESP32의 USB 케이블이 제대로 꽂혀 있는지 확인하세요.")
        return

    print("🔍 발견된 시리얼 포트 목록:")
    for p in ports:
        print(f"   • [{p.device}] {p.description} (제조사: {p.manufacturer or 'N/A'}, VID:PID={p.hwid})")

    print("\n" + "-" * 65)
    print("📡 각 포트별 ESP32 데이터 수신 테스트 (115200 bps)...")
    print("-" * 65)

    esp32_candidate = None

    for p in ports:
        dev = p.device
        print(f"\n👉 [{dev}] 연결 시도 중...", end="", flush=True)

        try:
            ser = serial.Serial(dev, 115200, timeout=1.5)
            print(" 연결 성공! 데이터 읽는 중...")
        except Exception as e:
            print(f" ❌ 포트 열기 실패 ({e})")
            print("    (다른 프로그램에서 이미 사용 중이거나 권한 문제일 수 있습니다)")
            continue

        # 3초 동안 들어오는 데이터 모니터링
        start = time.time()
        received_lines = []
        while time.time() - start < 2.5:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    received_lines.append(line)
                    if len(received_lines) >= 3:
                        break
            except Exception:
                pass
            time.sleep(0.05)

        ser.close()

        if received_lines:
            print(f"   ✅ 데이터 수신 확인! ({len(received_lines)}줄)")
            for l in received_lines[:3]:
                print(f"      📥 {l}")
            esp32_candidate = dev
        else:
            print("   ⚠️ 포트는 열렸으나 2.5초 동안 수신된 텍스트 데이터가 없습니다.")

    print("\n" + "=" * 65)
    if esp32_candidate:
        print(f"🎉 ESP32가 정상적으로 통신 중인 포트는 👉 [{esp32_candidate}] 입니다!")
        print(f"\n💡 full_bringup 실행 시 포트를 아래와 같이 지정해 보세요:")
        print(f"   ros2 launch amr_bringup full_bringup.launch.py esp32_port:={esp32_candidate}")
    else:
        print("💡 진단 요약 및 해결 방법:")
        print("   1. 다른 터미널에서 full_bringup 이나 esp32_bridge 노드가 켜져 있다면 먼저 종료해 주세요.")
        print("   2. ESP32 보드의 EN/RST 버튼을 한 번 눌러 재부팅해 보세요.")
        print("   3. `ls -l /dev/serial/by-id/` 명령어로 고정 장치명을 확인해 보세요.")
    print("=" * 65)


if __name__ == '__main__':
    main()
