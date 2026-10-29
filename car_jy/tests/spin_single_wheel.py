#!/usr/bin/env python3
"""
STS3215 한쪽 바퀴 단독 회전 테스트 툴
- 좌측 바퀴만 회전 (ID 1)
- 우측 바퀴만 회전 (ID 2)
- 사용법:
    python3 spin_single_wheel.py left       # 좌측 바퀴 5초 회전
    python3 spin_single_wheel.py right      # 우측 바퀴 5초 회전
    python3 spin_single_wheel.py left 2000 10  # 좌측 바퀴 속도 2000으로 10초간 회전
    python3 spin_single_wheel.py            # 대화형 메뉴로 선택
"""
import glob
import struct
import sys
import time
import serial


def calc_checksum(buf):
    return (~(sum(buf) & 0xFF)) & 0xFF


def send_write_packet(ser, servo_id, reg, data_bytes):
    length = len(data_bytes) + 3
    body = [servo_id, length, 0x03, reg] + list(data_bytes)
    chk = calc_checksum(body)
    packet = bytearray([0xFF, 0xFF] + body + [chk])
    ser.write(packet)
    ser.flush()


def find_default_port():
    ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    for p in ports:
        if 'ACM' in p:
            return p
    return ports[0] if ports else '/dev/ttyACM0'


def spin_wheel(port, target_id, speed, duration):
    baud = 1000000
    wheel_name = "좌측 바퀴 (ID 1)" if target_id == 1 else "우측 바퀴 (ID 2)"
    other_id = 2 if target_id == 1 else 1

    print("=" * 60)
    print(f"  [STS3215] {wheel_name} 단독 회전 시작")
    print(f"  - 포트: {port}")
    print(f"  - 회전 속도: {speed} step/s")
    print(f"  - 회전 시간: {duration}초")
    print("=" * 60)

    try:
        ser = serial.Serial(port, baud, timeout=0.05)
    except Exception as e:
        print(f"❌ 포트 열기 실패: {e}")
        print("팁: sudo chmod 666 /dev/ttyACM0 권한을 확인하세요.")
        return

    try:
        # 1. 양쪽 모터 모드 설정 및 상대 바퀴 완전 정지
        for sid in [1, 2]:
            send_write_packet(ser, sid, 0x28, [0x01])  # Torque Enable
            time.sleep(0.02)
            send_write_packet(ser, sid, 0x21, [0x01])  # Wheel Mode
            time.sleep(0.02)
            send_write_packet(ser, sid, 0x2E, struct.pack('<h', 0))  # 초기 정지
            time.sleep(0.02)

        # 2. 대상 바퀴만 회전 시작
        # 좌측(ID 1)은 양수(+), 우측(ID 2)은 마주보므로 전진 시 음수(-)
        applied_speed = speed if target_id == 1 else -speed
        print(f"\n🚀 {wheel_name} 회전 구동 중...")
        send_write_packet(ser, target_id, 0x2E, struct.pack('<h', applied_speed))

        # 카운트다운
        for sec in range(int(duration), 0, -1):
            print(f"   ⏱️  남은 시간: {sec}초...")
            time.sleep(1)

        # 3. 안전 정지
        print(f"\n🛑 {wheel_name} 안전 정지 완료!")
        send_write_packet(ser, target_id, 0x2E, struct.pack('<h', 0))
        time.sleep(0.05)

    finally:
        ser.close()
        print("=== 테스트 종료 ===\n")


def main():
    port = find_default_port()
    target_side = None
    speed = 2000
    duration = 5

    # 인자 파싱
    if len(sys.argv) > 1:
        arg = sys.argv[1].lower()
        if arg in ['left', 'l', '1']:
            target_side = 1
        elif arg in ['right', 'r', '2']:
            target_side = 2
        elif arg.startswith('/dev/'):
            port = arg
            if len(sys.argv) > 2:
                sub = sys.argv[2].lower()
                target_side = 1 if sub in ['left', 'l', '1'] else 2

    if len(sys.argv) > 2 and target_side is not None:
        try:
            speed = int(sys.argv[2])
        except ValueError:
            pass

    if len(sys.argv) > 3:
        try:
            duration = float(sys.argv[3])
        except ValueError:
            pass

    # 인자가 없으면 대화형 메뉴
    if target_side is None:
        print("=" * 50)
        print("   STS3215 한쪽 바퀴 단독 회전 테스트")
        print(f"   (기본 포트: {port})")
        print("=" * 50)
        print("  1. 좌측 바퀴만 회전 (ID 1)")
        print("  2. 우측 바퀴만 회전 (ID 2)")
        print("  3. 좌측 3초 돌리고 ➔ 우측 3초 돌리기 (교차 테스트)")
        print("=" * 50)
        choice = input("선택할 번호를 입력하세요 (1, 2, 3): ").strip()

        if choice == '1':
            spin_wheel(port, target_id=1, speed=2000, duration=5)
        elif choice == '2':
            spin_wheel(port, target_id=2, speed=2000, duration=5)
        elif choice == '3':
            print("\n[STEP 1] 좌측 바퀴 먼저 회전...")
            spin_wheel(port, target_id=1, speed=2000, duration=3)
            time.sleep(1)
            print("[STEP 2] 우측 바퀴 이어서 회전...")
            spin_wheel(port, target_id=2, speed=2000, duration=3)
        else:
            print("올바른 번호를 선택하지 않았습니다. 종료합니다.")
    else:
        spin_wheel(port, target_id=target_side, speed=speed, duration=duration)


if __name__ == '__main__':
    main()
