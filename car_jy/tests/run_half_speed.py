#!/usr/bin/env python3
"""
STS3215 좌측 바퀴 절반 속도 구동 테스트 (Left = Right / 2)
- 우측 바퀴(ID 2): 2000 step/s
- 좌측 바퀴(ID 1): 1000 step/s (우측의 50% 절반 속도)
- 5초간 회전 후 안전 정지
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


def encode_speed(speed_val):
    val = int(speed_val)
    mag = min(abs(val), 32767)
    return struct.pack('<H', (1 << 15) | mag if val < 0 else mag)


def find_default_port():
    ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    for p in ports:
        if 'ACM' in p:
            return p
    return ports[0] if ports else '/dev/ttyACM0'


def main():
    port = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1].startswith('/dev/') else find_default_port()
    baud = 1000000

    # 속도 설정: 우측 2000, 좌측은 절반인 1000
    right_speed = 2000
    left_speed = int(right_speed * 0.5)  # 1000 step/s
    duration = 5.0

    fast_speed = 2000
    slow_speed = 1000

    print("=" * 65)
    print("  [STS3215] 좌/우 바퀴 번갈아 고속 주행 테스트 (총 6초)")
    print(f"  - 시리얼 포트: {port} @ {baud} bps")
    print(f"  - 1단계 (처음 3초): 좌측 바퀴 더 빠름 (좌={fast_speed}, 우={slow_speed})")
    print(f"  - 2단계 (다음 3초): 우측 바퀴 더 빠름 (좌={slow_speed}, 우={fast_speed})")
    print("=" * 65)

    try:
        ser = serial.Serial(port, baud, timeout=0.05)
    except Exception as e:
        print(f"❌ 포트 열기 실패: {e}")
        print("팁: sudo chmod 666 /dev/ttyACM0 권한을 확인하세요.")
        sys.exit(1)

    try:
        # 1. 토크 활성화 및 휠 모드 설정
        for sid in [1, 2]:
            send_write_packet(ser, sid, 0x28, [0x01])  # Torque Enable
            time.sleep(0.02)
            send_write_packet(ser, sid, 0x21, [0x01])  # Wheel Mode
            time.sleep(0.02)

        # 2. [1단계: 3초간] 좌측 바퀴가 더 빠르게 주행
        print("\n🚀 [1단계 시작] ◀ 좌측 바퀴 더 빠르게 구동 중 (좌=2000, 우=1000)...")
        send_write_packet(ser, 1, 0x2E, encode_speed(-fast_speed))
        time.sleep(0.02)
        send_write_packet(ser, 2, 0x2E, encode_speed(slow_speed))

        start = time.time()
        while time.time() - start < 3.0:
            rem = 3.0 - (time.time() - start)
            print(f"   ⏱️  1단계 남은 시간: {max(0.0, rem):.1f}초...", end='\r')
            time.sleep(0.1)

        # 3. [2단계: 3초간] 우측 바퀴가 더 빠르게 주행
        print("\n\n🚀 [2단계 시작] ▶ 우측 바퀴 더 빠르게 구동 중 (좌=1000, 우=2000)...")
        send_write_packet(ser, 1, 0x2E, encode_speed(-slow_speed))
        time.sleep(0.02)
        send_write_packet(ser, 2, 0x2E, encode_speed(fast_speed))

        start = time.time()
        while time.time() - start < 3.0:
            rem = 3.0 - (time.time() - start)
            print(f"   ⏱️  2단계 남은 시간: {max(0.0, rem):.1f}초...", end='\r')
            time.sleep(0.1)

        # 4. 안전 정지
        print("\n\n🛑 총 6초 주행 완료. 모터 안전 정지!")
        send_write_packet(ser, 1, 0x2E, struct.pack('<h', 0))
        time.sleep(0.02)
        send_write_packet(ser, 2, 0x2E, struct.pack('<h', 0))
        time.sleep(0.02)
        send_write_packet(ser, 2, 0x2E, struct.pack('<h', 0))

    except KeyboardInterrupt:
        print("\n⚠️ 사용자 중단. 모터 즉시 정지!")
        send_write_packet(ser, 1, 0x2E, struct.pack('<h', 0))
        time.sleep(0.02)
        send_write_packet(ser, 2, 0x2E, struct.pack('<h', 0))
    finally:
        ser.close()
        print("=== 테스트 완료 ===")


if __name__ == '__main__':
    main()
