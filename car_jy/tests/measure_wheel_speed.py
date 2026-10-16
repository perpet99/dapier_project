#!/usr/bin/env python3
"""
STS3215 좌/우 바퀴 실시간 엔코더 속도 측정 및 밸런스 캘리브레이션 툴
- 내부 12비트 고정밀 엔코더(0x38 Speed Register)로부터 실제 회전 속도를 실시간 측정
- 좌/우 바퀴의 실제 속도 차이(%) 및 오차 비율을 실시간 계산
- 직진 보정을 위한 최적의 scale_left / scale_right 보정값을 자동으로 추천
"""
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


def read_speed(ser, servo_id):
    # READ_DATA (0x02), 0x38 (Present Speed), Length: 2 bytes
    body = [servo_id, 4, 0x02, 0x38, 2]
    chk = calc_checksum(body)
    packet = bytearray([0xFF, 0xFF] + body + [chk])
    ser.reset_input_buffer()
    ser.write(packet)
    ser.flush()

    # 응답 패킷: [0xFF, 0xFF, ID, Length, Error, LowByte, HighByte, Chksum] -> 8 bytes
    start = time.time()
    res = bytearray()
    while time.time() - start < 0.04:
        if ser.in_waiting > 0:
            res.extend(ser.read(ser.in_waiting))
            if len(res) >= 8 and res[0] == 0xFF and res[1] == 0xFF and res[2] == servo_id:
                raw_speed = struct.unpack('<h', res[5:7])[0]
                return abs(raw_speed)
        time.sleep(0.002)
    return None


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    baud = 1000000
    test_speed = int(sys.argv[2]) if len(sys.argv) > 2 else 2000

    print("=" * 65)
    print("      STS3215 좌/우 바퀴 실시간 속도 측정 & 캘리브레이션 툴")
    print(f"      - 포트: {port} @ {baud} bps")
    print(f"      - 목표 테스트 속도: {test_speed} step/s (약 {test_speed * 60 / 4096:.1f} RPM)")
    print("=" * 65)

    try:
        ser = serial.Serial(port, baud, timeout=0.05)
    except Exception as e:
        print(f"❌ 포트 열기 실패: {e}")
        print("팁: sudo chmod 666 /dev/ttyACM0 또는 포트명을 확인하세요.")
        sys.exit(1)

    try:
        # 1. 토크 활성화 및 Wheel Mode 설정
        print("\n[*] 모터 토크 활성화 및 연속 회전 모드 설정 중...")
        for sid in [1, 2]:
            send_write_packet(ser, sid, 0x28, [0x01])  # Torque Enable
            time.sleep(0.02)
            send_write_packet(ser, sid, 0x21, [0x01])  # Wheel Mode
            time.sleep(0.02)

        # 2. 양쪽 모터에 동일한 테스트 속도 인가
        print(f"[*] 양쪽 바퀴에 동일한 속도({test_speed}) 인가 시작 (10초간 측정)...")
        print("    [바퀴를 공중에 띄우거나 안전한 곳에 두고 측정하세요]\n")

        # 좌측(ID 1): +test_speed, 우측(ID 2): -test_speed (동일 전진 회전)
        send_write_packet(ser, 1, 0x2E, struct.pack('<h', test_speed))
        time.sleep(0.02)
        send_write_packet(ser, 2, 0x2E, struct.pack('<h', -test_speed))
        time.sleep(0.5)  # 속도 안정화 대기

        print(f"{'시간(초)':^8} | {'좌측 바퀴(ID 1)':^16} | {'우측 바퀴(ID 2)':^16} | {'속도 차이 / 상태':^20}")
        print("-" * 68)

        left_samples = []
        right_samples = []

        start_time = time.time()
        while time.time() - start_time < 10.0:
            elapsed = time.time() - start_time

            spd_left = read_speed(ser, 1)
            time.sleep(0.01)
            spd_right = read_speed(ser, 2)
            time.sleep(0.01)

            if spd_left is not None and spd_right is not None:
                left_samples.append(spd_left)
                right_samples.append(spd_right)

                diff = spd_left - spd_right
                if abs(diff) < 30:
                    status = "✅ 완벽 균형"
                elif diff > 0:
                    pct = (diff / spd_right) * 100 if spd_right > 0 else 0
                    status = f"◀ 좌측 +{pct:.1f}% 빠름"
                else:
                    pct = (abs(diff) / spd_left) * 100 if spd_left > 0 else 0
                    status = f"▶ 우측 +{pct:.1f}% 빠름"

                left_rpm = spd_left * 60.0 / 4096.0
                right_rpm = spd_right * 60.0 / 4096.0

                print(f"{elapsed:^7.1f}s | {spd_left:>5} ({left_rpm:>4.1f} RPM) | {spd_right:>5} ({right_rpm:>4.1f} RPM) | {status:<20}")

            time.sleep(0.2)

        # 3. 모터 정지
        print("-" * 68)
        print("[*] 측정 완료. 모터 안전 정지 중...")
        send_write_packet(ser, 1, 0x2E, struct.pack('<h', 0))
        time.sleep(0.02)
        send_write_packet(ser, 2, 0x2E, struct.pack('<h', 0))

        # 4. 종합 분석 및 최적 보정값 산출
        if left_samples and right_samples:
            avg_left = sum(left_samples) / len(left_samples)
            avg_right = sum(right_samples) / len(right_samples)

            print("\n" + "=" * 65)
            print("                📊 종합 분석 및 추천 보정값")
            print("=" * 65)
            print(f"  • 좌측 바퀴(ID 1) 평균 실제 속도: {avg_left:.1f} step/s ({avg_left * 60 / 4096:.1f} RPM)")
            print(f"  • 우측 바퀴(ID 2) 평균 실제 속도: {avg_right:.1f} step/s ({avg_right * 60 / 4096:.1f} RPM)")

            diff_val = avg_left - avg_right
            if abs(diff_val) < 20:
                print("\n  🎉 좌우 바퀴의 속도가 거의 완벽하게 일치합니다! (보정 불필요)")
            elif diff_val > 0:
                # 좌측이 더 빠름 -> scale_left를 줄여야 함
                recommended_scale_left = avg_right / avg_left
                print(f"\n  ⚠️ 좌측 바퀴가 우측보다 약 {((avg_left / avg_right) - 1.0) * 100:.1f}% 더 빠릅니다.")
                print(f"  💡 추천 보정값:")
                print(f"     scale_left := {recommended_scale_left:.3f}")
                print(f"     scale_right := 1.000")
            else:
                # 우측이 더 빠름 -> scale_right를 줄여야 함
                recommended_scale_right = avg_left / avg_right
                print(f"\n  ⚠️ 우측 바퀴가 좌측보다 약 {((avg_right / avg_left) - 1.0) * 100:.1f}% 더 빠릅니다.")
                print(f"  💡 추천 보정값:")
                print(f"     scale_left := 1.000")
                print(f"     scale_right := {recommended_scale_right:.3f}")
            print("=" * 65)

    finally:
        ser.close()


if __name__ == '__main__':
    main()
