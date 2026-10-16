#!/usr/bin/env python3
"""
연결된 서보 모터 식별 및 물리적 바퀴 회전 확인 툴 (identify_wheel.py)
------------------------------------------------------------------
1. 현재 버스에서 응답하는 모든 ID를 찾습니다 (1~20번까지 확장 탐색).
2. 발견된 ID를 1초간 살짝 회전시켜, 로봇의 "어떤 실제 바퀴"가 돌아가는지 눈으로 직접 확인합니다.
3. 115200 bps 및 1000000 bps 양쪽 속도를 모두 자동 검사합니다.
"""

import os
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


def read_response(ser, timeout=0.05):
    start = time.time()
    res = bytearray()
    while time.time() - start < timeout:
        if ser.in_waiting > 0:
            res.extend(ser.read(ser.in_waiting))
            if len(res) >= 6 and res[0] == 0xFF and res[1] == 0xFF:
                pkg_len = res[3]
                if len(res) >= pkg_len + 4:
                    return bytes(res)
        time.sleep(0.002)
    return bytes(res)


def ping_servo(ser, servo_id):
    try:
        ser.reset_input_buffer()
    except Exception:
        pass
    try:
        send_packet(ser, servo_id, 0x01, [])
        res = read_response(ser, timeout=0.04)
        return len(res) >= 6 and res[2] == servo_id
    except Exception:
        return False


def spin_motor(ser, servo_id, duration=1.2, speed=600):
    # 휠 모드 설정
    send_packet(ser, servo_id, 0x03, [0x37, 0x00])
    time.sleep(0.01)
    send_packet(ser, servo_id, 0x03, [0x21, 0x01])
    time.sleep(0.01)
    send_packet(ser, servo_id, 0x03, [0x37, 0x01])
    time.sleep(0.01)

    # 정회전
    val = int(speed)
    low_b = val & 0xFF
    high_b = (val >> 8) & 0xFF
    send_packet(ser, servo_id, 0x03, [0x2E, low_b, high_b])
    time.sleep(duration)

    # 정지
    send_packet(ser, servo_id, 0x03, [0x2E, 0x00, 0x00])
    time.sleep(0.1)


def find_servo_port():
    if os.path.exists('/dev/servo_port'):
        return '/dev/servo_port'
    ports = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
    for p in ports:
        if 'ttyACM0' in p or 'ttyUSB0' in p:
            return p
    return ports[0] if ports else None


def main():
    print("=" * 65)
    print("      STS3215 서보 모터 식별 & 실제 바퀴 확인 툴")
    print("=" * 65)

    port = sys.argv[1] if len(sys.argv) > 1 else find_servo_port()
    if not port:
        print("❌ 시리얼 포트를 찾을 수 없습니다!")
        return

    print(f"🔌 포트: {port}")

    # 두 가지 Baud Rate 모두 검사 (1000000, 115200)
    for baud in [1000000, 115200]:
        print(f"\n📡 통신 속도 {baud:,} bps 로 스캔 중...")
        try:
            ser = serial.Serial(port, baud, timeout=0.03)
        except Exception as e:
            print(f"   포트 열기 실패: {e}")
            continue

        found = []
        for sid in range(1, 21):  # 1번부터 20번까지 넓게 탐색
            if ping_servo(ser, sid):
                found.append(sid)
                print(f"   🎯 [발견!] 서보 ID: {sid} 번 (응답 속도: {baud:,} bps)")

        if not found:
            print(f"   -> {baud:,} bps 에서는 응답하는 모터가 없습니다.")
            ser.close()
            continue

        print(f"\n총 {len(found)}개 모터가 발견되었습니다: {found}")

        for sid in found:
            try:
                ans = input(f"\n👉 [ID {sid}번 모터]를 1초간 살짝 돌려보시겠습니까? (엔터: 회전, n: 건너뛰기): ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if ans.lower() != 'n':
                print(f"   🔄 ID {sid}번 모터 1초간 회전 중... (어느 바퀴가 도는지 보세요!)")
                spin_motor(ser, sid, duration=1.2, speed=700)
                print(f"   🛑 ID {sid}번 모터 정지 완료.")

        ser.close()
        break


if __name__ == '__main__':
    main()
