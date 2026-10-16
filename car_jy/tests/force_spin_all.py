#!/usr/bin/env python3
"""
강제 모터 회전 점검 스크립트 (force_spin_all.py)
------------------------------------------------
1. 리눅스 cdc_acm 특성인 DTR/RTS 신호를 강제 활성화 (TX 데이터 누락 방지)
2. 모터 ID에 구애받지 않는 전체 브로드캐스트 ID (254, 0xFE) 사용
3. EEPROM 언락 -> 휠 모드 설정 -> 2초간 회전 -> 안전 정지
"""
import sys
import time
import glob
import serial


def calc_chk(buf):
    return (~(sum(buf) & 0xFF)) & 0xFF


def send_packet(ser, sid, reg, data):
    length = len(data) + 3
    body = [sid, length, 0x03, reg] + list(data)
    chk = calc_chk(body)
    packet = bytearray([0xFF, 0xFF] + body + [chk])
    ser.write(packet)
    ser.flush()


def main():
    print("=" * 60)
    print("      STS3215 모터 강제 회전 테스트 (DTR/RTS 보정 + 브로드캐스트)")
    print("=" * 60)

    # 포트 결정
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 1000000

    print(f"🔌 포트: {port}")
    print(f"📡 속도: {baud:,} bps")

    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as e:
        print(f"❌ 포트 열기 실패: {e}")
        print("💡 팁: 'sudo chmod 666 /dev/ttyACM*' 를 먼저 실행하세요.")
        return

    # ★ 리눅스 cdc_acm 필수: DTR/RTS 핀을 켜야 실제 TX 선으로 데이터가 출력됨!
    ser.dtr = True
    ser.rts = True
    time.sleep(0.1)

    print("\n[1단계] 모든 모터 EEPROM 잠금 해제 & 휠 모드(Wheel Mode) 강제 설정...")
    # 브로드캐스트 ID 254 (0xFE)
    send_packet(ser, 254, 0x37, [0x00])  # Unlock
    time.sleep(0.03)
    send_packet(ser, 254, 0x21, [0x01])  # Wheel Mode
    time.sleep(0.03)
    send_packet(ser, 254, 0x37, [0x01])  # Lock
    time.sleep(0.05)

    # 개별 ID 1, 2, 3, 4번에도 개별적으로 한 번 더 전송
    for sid in [1, 2, 3, 4]:
        send_packet(ser, sid, 0x37, [0x00])
        time.sleep(0.01)
        send_packet(ser, sid, 0x21, [0x01])
        time.sleep(0.01)
        send_packet(ser, sid, 0x37, [0x01])
        time.sleep(0.01)

    print("\n[2단계] 🚀 모터 회전 명령 전송! (2초간 주행)")
    # Feetech STS3215 1000 속도 (Little-Endian: 0xE8, 0x03)
    # 1) 브로드캐스트로 전송
    send_packet(ser, 254, 0x2E, [0xE8, 0x03])
    time.sleep(0.02)
    # 2) ID 1~4 개별로도 전송
    for sid in [1, 2, 3, 4]:
        send_packet(ser, sid, 0x2E, [0xE8, 0x03])
        time.sleep(0.01)

    print("   👉 지금 바퀴가 돌고 있어야 합니다!")
    for i in range(2, 0, -1):
        print(f"   ⏱️  정지까지 {i}초...")
        time.sleep(1.0)

    print("\n[3단계] 🛑 모터 정지 명령 전송")
    send_packet(ser, 254, 0x2E, [0x00, 0x00])
    for sid in [1, 2, 3, 4]:
        send_packet(ser, sid, 0x2E, [0x00, 0x00])
        time.sleep(0.01)

    ser.close()
    print("=" * 60)
    print("테스트 완료! 모터가 돌았는지 확인해 주세요.")
    print("=" * 60)


if __name__ == '__main__':
    main()
