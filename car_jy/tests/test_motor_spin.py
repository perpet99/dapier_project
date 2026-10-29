#!/usr/bin/env python3
import serial
import struct
import time
import sys

# 포트 지정 (기본값 /dev/ttyACM0)
port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
baud = 1000000

print(f"==========================================")
print(f"  STS3215 서보 모터 단독 회전 테스트")
print(f"  - 포트: {port}")
print(f"  - 통신 속도: {baud} bps")
print(f"==========================================")

def calc_checksum(buf):
    return (~(sum(buf) & 0xFF)) & 0xFF

def write_reg(ser, servo_id, reg, data):
    length = len(data) + 3
    inst = 0x03  # WRITE_DATA
    body = [servo_id, length, inst, reg] + list(data)
    chk = calc_checksum(body)
    packet = bytearray([0xFF, 0xFF] + body + [chk])
    ser.write(packet)
    ser.flush()

try:
    ser = serial.Serial(port, baud, timeout=0.1)
    print("✓ 서보 포트 열기 성공!")
except Exception as e:
    print(f"❌ 포트 열기 실패: {e}")
    print("팁: sudo chmod 666 /dev/ttyACM0 권한을 확인하세요.")
    sys.exit(1)

try:
    # 0. 토크 활성화 (0x28 -> 1)
    print("\n[1/4] 서보 ID 1, ID 2 토크 활성화(Torque Enable)...")
    write_reg(ser, 1, 0x28, [0x01])
    time.sleep(0.05)
    write_reg(ser, 2, 0x28, [0x01])
    time.sleep(0.05)

    # 1. 서보 1, 2 모드를 Wheel Mode (0x21 -> 1)로 전환
    print("[2/4] 서보 ID 1, ID 2 모드를 Wheel Mode(연속 회전)로 설정...")
    write_reg(ser, 1, 0x21, [0x01])
    time.sleep(0.05)
    write_reg(ser, 2, 0x21, [0x01])
    time.sleep(0.05)

    # 2. 5초간 회전 시작 (속도 = 2000 step/s)
    speed = 2500
    print(f"[3/4] 5초간 모터 회전 구동 중 (속도: {speed} step/s)...")
    # 좌측(ID 1): +speed, 우측(ID 2): -speed (반대방향 마주보고 대칭 회전)
    write_reg(ser, 1, 0x2E, struct.pack('<h', speed))
    time.sleep(0.02)
    write_reg(ser, 2, 0x2E, struct.pack('<h', -speed))

    for i in range(5, 0, -1):
        print(f"   - 정지까지 {i}초...")
        time.sleep(1)

    # 3. 바퀴 정지 (속도 = 0)
    print("\n[4/4] 모터 안전 정지!")
    write_reg(ser, 1, 0x2E, struct.pack('<h', 0))
    time.sleep(0.02)
    write_reg(ser, 2, 0x2E, struct.pack('<h', 0))

finally:
    ser.close()
    print("\n=== 모터 단독 테스트 완료 ===")
