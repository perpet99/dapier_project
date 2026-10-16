#!/usr/bin/env python3
"""
STS3215 '진짜' 물리적 회전수(RPM) 정밀 측정 툴
- 모터가 주장하는 속도 레지스터(0x38) 대신,
  모터 내부의 12비트 절대 위치 엔코더(0x36 Present Position)의 회전수를 실시간 카운트!
- 3초 동안 실제로 모터 축이 몇 바퀴(Revolutions)를 돌았는지 정확히 세어서 실제 RPM 출력
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


def read_position(ser, servo_id):
    # READ_DATA (0x02), 0x38 (SMS_STS_PRESENT_POSITION_L/H), 2 bytes
    body = [servo_id, 4, 0x02, 0x38, 2]
    chk = calc_checksum(body)
    packet = bytearray([0xFF, 0xFF] + body + [chk])
    ser.reset_input_buffer()
    ser.write(packet)
    ser.flush()

    start = time.time()
    res = bytearray()
    while time.time() - start < 0.03:
        if ser.in_waiting > 0:
            res.extend(ser.read(ser.in_waiting))
            if len(res) >= 8 and res[0] == 0xFF and res[1] == 0xFF and res[2] == servo_id:
                raw_pos = struct.unpack('<H', res[5:7])[0]
                return raw_pos
        time.sleep(0.001)
    return None


def encode_speed(speed_val):
    val = int(speed_val)
    mag = min(abs(val), 32767)
    return struct.pack('<H', (1 << 15) | mag if val < 0 else mag)


def main():
    ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    port = ports[0] if ports else '/dev/ttyACM0'
    baud = 1000000

    print("=" * 65)
    print("      STS3215 '진짜' 물리 바퀴 회전수(Real RPM) 정밀 측정기")
    print(f"      - 시리얼 포트: {port} @ {baud} bps")
    print("=" * 65)
    print("⚠️  바퀴를 바닥에서 살짝 들어 공중에 띄우고 테스트하세요.\n")

    try:
        ser = serial.Serial(port, baud, timeout=0.05)
    except Exception as e:
        print(f"❌ 포트 열기 실패: {e}")
        sys.exit(1)

    try:
        for sid in [1, 2]:
            send_write_packet(ser, sid, 0x28, [0x01])  # Torque Enable
            time.sleep(0.02)
            send_write_packet(ser, sid, 0x21, [0x01])  # Wheel Mode
            time.sleep(0.02)

        test_speed = 2000
        print(f"[*] 양쪽 모터에 똑같이 목표 속도 {test_speed} 인가!")
        print("[*] 4초 동안 실제 엔코더 바퀴 수(Turns)를 직접 카운트합니다...\n")

        # 전진 방향 인가
        send_write_packet(ser, 1, 0x2E, encode_speed(-test_speed))
        time.sleep(0.02)
        send_write_packet(ser, 2, 0x2E, encode_speed(test_speed))
        time.sleep(0.3)

        # 4초간 실제 위치 추적 및 바퀴 수 계산
        last_pos = {1: read_position(ser, 1), 2: read_position(ser, 2)}
        total_delta = {1: 0, 2: 0}

        start_time = time.time()
        last_print = 0
        while time.time() - start_time < 4.0:
            for sid in [1, 2]:
                pos = read_position(ser, sid)
                if pos is not None:
                    if last_pos[sid] is not None:
                        diff = pos - last_pos[sid]
                        # 12비트(4096) 롤오버 처리
                        if diff > 2048:
                            diff -= 4096
                        elif diff < -2048:
                            diff += 4096
                        total_delta[sid] += abs(diff)
                    last_pos[sid] = pos
                time.sleep(0.005)

            if time.time() - last_print > 0.2:
                last_print = time.time()
                t1 = total_delta[1] / 4096.0
                t2 = total_delta[2] / 4096.0
                rem = 4.0 - (time.time() - start_time)
                print(f"   ⏱️ {rem:.1f}초 남음... 좌측 회전수: {t1:.2f}바퀴 | 우측 회전수: {t2:.2f}바퀴", end='\r')

        # 정지
        send_write_packet(ser, 1, 0x2E, struct.pack('<h', 0))
        time.sleep(0.02)
        send_write_packet(ser, 2, 0x2E, struct.pack('<h', 0))

        elapsed = time.time() - start_time
        turns_left = total_delta[1] / 4096.0
        turns_right = total_delta[2] / 4096.0
        real_rpm_left = (turns_left / elapsed) * 60.0
        real_rpm_right = (turns_right / elapsed) * 60.0

        print("-" * 65)
        print(f"📊 [4초간 실제 바퀴 회전 결과]")
        print(f"   • 좌측 바퀴 (ID 1): 총 {turns_left:.2f} 바퀴 회전 ➔ 실제 속도: {real_rpm_left:.1f} RPM")
        print(f"   • 우측 바퀴 (ID 2): 총 {turns_right:.2f} 바퀴 회전 ➔ 실제 속도: {real_rpm_right:.1f} RPM")
        print("-" * 65)

        if real_rpm_right > 0:
            ratio = real_rpm_left / real_rpm_right
            print(f"🔍 실제 회전 속도 비율 (좌/우): {ratio:.2f}배")
            if ratio > 1.2:
                print(f"👉 실제로 좌측 바퀴가 우측보다 {((ratio - 1)*100):.1f}% 더 많이 돌았습니다!")
                print(f"💡 완벽한 직진을 위한 scale_left 추천값: {1.0 / ratio:.3f}")
            elif ratio < 0.8:
                print(f"👉 실제로 우측 바퀴가 좌측보다 {((1 - ratio)*100):.1f}% 더 많이 돌았습니다!")
            else:
                print("✅ 양쪽 바퀴가 실제로 거의 같은 바퀴 수를 돌았습니다!")
                print("   ➔ 만약 공중에서는 같은데 바닥에서만 휜다면 타이어 헛돔/마찰 문제입니다.")
        print("=" * 65)

    except KeyboardInterrupt:
        send_write_packet(ser, 1, 0x2E, struct.pack('<h', 0))
        send_write_packet(ser, 2, 0x2E, struct.pack('<h', 0))
    finally:
        ser.close()


if __name__ == '__main__':
    main()
