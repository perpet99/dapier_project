#!/usr/bin/env python3
"""
4개 바퀴(4WD Skid-Steer) 선택형 개별/전체 회전 점검 툴 (test_4wheel_spin.py)
----------------------------------------------------------------------
- 원하는 바퀴 번호(1: 좌앞, 2: 우앞, 3: 좌뒤, 4: 우뒤)를 선택하여 단독 회전 테스트
- 4바퀴 동시 전진/후진 테스트 지원
- 실제 물리적 전진 방향(좌측: -속도, 우측: +속도) 적용
"""

import sys
import time
import glob
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


def stop_all(ser, ids=[1, 2, 3, 4]):
    speeds = {sid: 0 for sid in ids}
    send_sync_speeds(ser, speeds)


def find_servo_port():
    import os
    if os.path.exists('/dev/servo_port'):
        return '/dev/servo_port'
    ports = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
    for p in ports:
        if 'ttyACM0' in p or 'ttyUSB0' in p:
            return p
    return ports[0] if ports else None


def spin_single(ser, sid, name, target_speed, duration=2.0):
    print(f"\n👉 [{name} (ID {sid})] {duration}초간 회전 시작 (속도: {target_speed})...")
    # 선택된 바퀴만 회전, 나머지는 0 정지
    speeds = {1: 0, 2: 0, 3: 0, 4: 0}
    speeds[sid] = target_speed
    send_sync_speeds(ser, speeds)
    time.sleep(duration)
    stop_all(ser)
    print(f"🛑 [{name} (ID {sid})] 정지 완료")


def main():
    port = find_servo_port()
    if not port:
        print("❌ 시리얼 포트를 찾을 수 없습니다!")
        return

    print("=" * 65)
    print("   AMR 4개 바퀴 (4WD) 선택형 개별/전체 회전 점검 툴")
    print(f"   - 시리얼 포트: {port} @ 1,000,000 bps")
    print("=" * 65)

    try:
        ser = serial.Serial(port, 1000000, timeout=0.05)
    except Exception as e:
        print(f"❌ 포트 열기 실패: {e}")
        return

    # 전진 회전 방향: 좌측 = -속도, 우측 = +속도
    id_lf = 1  # 왼쪽 앞
    id_rf = 2  # 오른쪽 앞
    id_lr = 3  # 왼쪽 뒤
    id_rr = 4  # 오른쪽 뒤
    all_ids = [id_lf, id_rf, id_lr, id_rr]

    # 4개 바퀴 휠 모드 활성화
    for sid in all_ids:
        set_wheel_mode(ser, sid)
    stop_all(ser)

    base_speed = 1000  # 약 25% 회전 속도

    try:
        while True:
            print("\n" + "=" * 55)
            print("  [테스트할 바퀴를 번호로 선택하세요]")
            print("=" * 55)
            print("  1. 왼쪽 앞바퀴   (ID 1)  전진 회전")
            print("  2. 오른쪽 앞바퀴 (ID 2)  전진 회전")
            print("  3. 왼쪽 뒷바퀴   (ID 3)  전진 회전")
            print("  4. 오른쪽 뒷바퀴 (ID 4)  전진 회전")
            print("-" * 55)
            print("  5. 4바퀴 동시 직진 전진 (2초)")
            print("  6. 4바퀴 동시 후진     (2초)")
            print("  7. 1~4번 바퀴 차례대로 1.5초씩 자동 순차 테스트")
            print("  q. 종료")
            print("=" * 55)

            choice = input("선택 번호 입력: ").strip().lower()

            if choice == '1':
                spin_single(ser, id_lf, "왼쪽 앞바퀴", -base_speed, duration=2.0)
            elif choice == '2':
                spin_single(ser, id_rf, "오른쪽 앞바퀴", base_speed, duration=2.0)
            elif choice == '3':
                spin_single(ser, id_lr, "왼쪽 뒷바퀴", -base_speed, duration=2.0)
            elif choice == '4':
                spin_single(ser, id_rr, "오른쪽 뒷바퀴", base_speed, duration=2.0)
            elif choice == '5':
                print(f"\n🚀 [4바퀴 동시 전진] 2초간 앞으로 직진합니다...")
                send_sync_speeds(ser, {
                    id_lf: -base_speed,
                    id_rf: base_speed,
                    id_lr: -base_speed,
                    id_rr: base_speed
                })
                time.sleep(2.0)
                stop_all(ser)
                print("🛑 정지 완료!")
            elif choice == '6':
                print(f"\n🚀 [4바퀴 동시 후진] 2초간 뒤로 이동합니다...")
                send_sync_speeds(ser, {
                    id_lf: base_speed,
                    id_rf: -base_speed,
                    id_lr: base_speed,
                    id_rr: -base_speed
                })
                time.sleep(2.0)
                stop_all(ser)
                print("🛑 정지 완료!")
            elif choice == '7':
                print("\n🔄 [1~4번 바퀴 순차 테스트 시작]")
                spin_single(ser, id_lf, "1번 왼쪽 앞바퀴", -base_speed, duration=1.5)
                time.sleep(0.5)
                spin_single(ser, id_rf, "2번 오른쪽 앞바퀴", base_speed, duration=1.5)
                time.sleep(0.5)
                spin_single(ser, id_lr, "3번 왼쪽 뒷바퀴", -base_speed, duration=1.5)
                time.sleep(0.5)
                spin_single(ser, id_rr, "4번 오른쪽 뒷바퀴", base_speed, duration=1.5)
                time.sleep(0.5)
                print("🎉 4개 바퀴 순차 테스트 완료!")
            elif choice == 'q':
                print("종료합니다.")
                break
            else:
                print("❌ 올바른 번호를 입력하세요.")

    except KeyboardInterrupt:
        print("\n🛑 사용자 중단. 모터 정지!")
    finally:
        stop_all(ser)
        ser.close()


if __name__ == '__main__':
    main()
