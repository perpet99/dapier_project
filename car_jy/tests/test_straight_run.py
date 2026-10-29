#!/usr/bin/env python3
import glob
import struct
import sys
import time
import serial


def calc_checksum(buf):
    return (~(sum(buf) & 0xFF)) & 0xFF


def send_packet(ser, servo_id, inst, reg_or_params):
    length = len(reg_or_params) + 2
    body = [servo_id, length, inst] + list(reg_or_params)
    chk = calc_checksum(body)
    packet = bytearray([0xFF, 0xFF] + body + [chk])
    ser.write(packet)
    ser.flush()


def set_wheel_mode(ser, servo_id):
    # EEPROM Unlock (0x37 = 0) -> Mode = 1 (Wheel Mode) -> Lock (0x37 = 1)
    send_packet(ser, servo_id, 0x03, [0x37, 0x00])
    time.sleep(0.01)
    send_packet(ser, servo_id, 0x03, [0x21, 0x01])
    time.sleep(0.01)
    send_packet(ser, servo_id, 0x03, [0x37, 0x01])
    time.sleep(0.01)


def send_sync_speeds(ser, speeds_dict):
    """Feetech STS3215 Sync Write (0x83) - 모든 바퀴 동시 속도 전송"""
    reg_addr = 0x2E
    data_len = 2
    num_servos = len(speeds_dict)
    length = (data_len + 1) * num_servos + 4
    instruction = 0x83

    body = [0xFE, length, instruction, reg_addr, data_len]
    for sid, speed_val in speeds_dict.items():
        val = int(speed_val)
        mag = min(abs(val), 32767)
        if val < 0:
            encoded_val = (1 << 15) | mag
        else:
            encoded_val = mag
        spd_l = encoded_val & 0xFF
        spd_h = (encoded_val >> 8) & 0xFF
        body.extend([sid, spd_l, spd_h])

    chk = calc_checksum(body)
    packet = bytearray([0xFF, 0xFF] + body + [chk])
    ser.write(packet)
    ser.flush()


def stop_robot(ser):
    speeds = {1: 0, 2: 0, 3: 0, 4: 0}
    send_sync_speeds(ser, speeds)


def find_default_port():
    import os
    if os.path.exists('/dev/servo_port'):
        return '/dev/servo_port'
    ports = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
    for p in ports:
        if 'ttyACM0' in p or 'ttyUSB0' in p:
            return p
    return ports[0] if ports else '/dev/ttyACM0'


def run_straight(ser, base_speed=1500, duration=1.0, scale_left=1.0, scale_right=1.0):
    # 실제 물리적 전진 회전: 좌측 바퀴 = -음수, 우측 바퀴 = +양수
    spd_left = int(-base_speed * scale_left)
    spd_right = int(base_speed * scale_right)

    speeds = {
        1: spd_left,   # 좌측 앞
        2: spd_right,  # 우측 앞
        3: spd_left,   # 좌측 뒤
        4: spd_right   # 우측 뒤
    }

    print(f"\n🚀 [직진 출발] 속도: 좌측={spd_left} (scale_left: {scale_left:.3f}), 우측={spd_right}")
    print(f"⏱️  {duration}초 동안 전진합니다...")

    send_sync_speeds(ser, speeds)

    start = time.time()
    while time.time() - start < duration:
        remaining = duration - (time.time() - start)
        print(f"   남은 시간: {max(0.0, remaining):.1f}초...", end='\r')
        time.sleep(0.05)

    print("\n🛑 정지 완료!")
    stop_robot(ser)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1].startswith('/dev/') else find_default_port()
    baud = 1000000

    print("=" * 65)
    print("   STS3215 4륜 동시 구동 칼직진 튜닝 툴 (텔레옵 정면 기준)")
    print(f"   - 시리얼 포트: {port} @ {baud} bps")
    print("=" * 65)
    print("⚠️  로봇을 바닥(앞으로 2m 이상 여유 공간이 있는 곳)에 똑바로 놓으세요.")
    print("------------------------------------------------------------")

    try:
        ser = serial.Serial(port, baud, timeout=0.05)
    except Exception as e:
        print(f"❌ 포트 열기 실패: {e}")
        sys.exit(1)

    try:
        # 4개 바퀴 Wheel Mode 설정
        for sid in [1, 2, 3, 4]:
            set_wheel_mode(ser, sid)
        stop_robot(ser)

        current_scale_left = 1.0
        speed = 1500
        duration = 1.0

        while True:
            print("\n" + "-" * 50)
            print(f"현재 설정: scale_left = {current_scale_left:.3f} | 주행시간 = {duration}초")
            print("1. 직진 테스트 실행 (출발!)")
            print("2. 오른쪽으로 휘어짐 ➔ 좌측 바퀴 속도 3% 줄이기 (scale_left -0.03)")
            print("3. 오른쪽으로 심하게 휘어짐 ➔ 좌측 바퀴 속도 6% 줄이기 (scale_left -0.06)")
            print("4. 왼쪽으로 휘어짐 ➔ 좌측 바퀴 속도 3% 늘리기 (scale_left +0.03)")
            print("5. 직접 scale_left 수치 입력")
            print("q. 종료 (현재 추천값 확인)")
            print("-" * 50)

            cmd = input("선택하세요 (1/2/3/4/5/q): ").strip().lower()

            if cmd == '1':
                run_straight(ser, base_speed=speed, duration=duration, scale_left=current_scale_left)
            elif cmd == '2':
                current_scale_left -= 0.03
                print(f"👉 scale_left를 {current_scale_left:.3f}로 조정했습니다.")
                run_straight(ser, base_speed=speed, duration=duration, scale_left=current_scale_left)
            elif cmd == '3':
                current_scale_left -= 0.06
                print(f"👉 scale_left를 {current_scale_left:.3f}로 조정했습니다.")
                run_straight(ser, base_speed=speed, duration=duration, scale_left=current_scale_left)
            elif cmd == '4':
                current_scale_left += 0.03
                print(f"👉 scale_left를 {current_scale_left:.3f}로 조정했습니다.")
                run_straight(ser, base_speed=speed, duration=duration, scale_left=current_scale_left)
            elif cmd == '5':
                val = input("새로운 scale_left 값을 입력하세요 (예: 1.04): ").strip()
                try:
                    current_scale_left = float(val)
                    print(f"👉 scale_left = {current_scale_left:.3f}")
                except ValueError:
                    print("잘못된 숫자입니다.")
            elif cmd == 'q':
                break

        print("\n" + "=" * 60)
        print("🎉 [최종 튜닝 결과]")
        print(f"   최적 scale_left 값: {current_scale_left:.3f}")
        print("   실제 내비게이션 실행 시 아래처럼 적용하시면 됩니다:")
        print(f"   ros2 launch amr_bringup full_bringup.launch.py scale_left:={current_scale_left:.3f}")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\n⚠️ 사용자 중단. 모터 정지!")
        stop_robot(ser)
    finally:
        stop_robot(ser)
        ser.close()


if __name__ == '__main__':
    main()
