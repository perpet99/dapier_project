#!/usr/bin/env python3
"""
STS3215 서보 통신(Ping) 및 전압 점검 도구 (check_servo_ping.py)
- 외부 서드파티 라이브러리 없이 순수 pyserial로 직접 패킷 전송
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


def read_response(ser, timeout=0.06):
    start = time.time()
    res = bytearray()
    while time.time() - start < timeout:
        if ser.in_waiting > 0:
            res.extend(ser.read(ser.in_waiting))
            if len(res) >= 6 and res[0] == 0xFF and res[1] == 0xFF:
                pkg_len = res[3]
                if len(res) >= pkg_len + 4:
                    return bytes(res)
        time.sleep(0.005)
    return bytes(res)


def find_port():
    if len(sys.argv) > 1:
        return sys.argv[1]
    ports = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
    for p in ports:
        if 'ttyACM0' in p or 'ttyUSB0' in p:
            return p
    return ports[0] if ports else '/dev/ttyACM0'


def main():
    port = find_port()
    print("=" * 55)
    print(f" 🔍 STS3215 서보 통신(Ping) & 12V 전압 정밀 점검")
    print(f" 대상 포트: {port} (Baudrate: 1,000,000 bps)")
    print("=" * 55)

    try:
        ser = serial.Serial(port, 1000000, timeout=0.05)
        print(f"[✔] 시리얼 포트 ({port}) 열기 성공!\n")
    except Exception as e:
        print(f"[✖] 시리얼 포트 열기 실패: {e}")
        print(" -> USB 케이블이 꽂혀 있는지, 다른 프로세스가 포트를 쓰고 있는지 확인하세요.")
        print(" -> 포트 해제: 'pkill -f python3'")
        sys.exit(1)

    print(">> 서보 ID 1 ~ 20 번 Ping 및 전압 확인 시작...")
    found_servos = []

    for sid in range(1, 21):
        # 1. Ping (0x01)
        send_packet(ser, sid, 0x01, [])
        resp = read_response(ser, timeout=0.03)

        if len(resp) >= 6 and resp[0] == 0xFF and resp[1] == 0xFF and resp[2] == sid:
            # 2. 전압(0x3E, 1바이트) 읽기 (Read: 0x02)
            send_packet(ser, sid, 0x02, [0x3E, 1])
            v_resp = read_response(ser, timeout=0.03)
            volt_str = "확인 불가"
            if len(v_resp) >= 7 and v_resp[2] == sid:
                volt_val = v_resp[5] / 10.0
                volt_str = f"{volt_val:.1f}V"

            print(f"  [✔] Servo ID {sid:2d}: 응답 정상! (공급 전압: {volt_str})")
            found_servos.append(sid)
        time.sleep(0.01)

    ser.close()

    print("\n" + "=" * 55)
    if found_servos:
        print(f" 🎉 [결과] 정상 응답 서보 ID: {found_servos}")
        print(" -> 서보 통신 및 하드웨어 전원이 정상입니다.")
        print(" 👉 바퀴 회전 테스트: python3 tests/test_4wheel_spin.py")
    else:
        print(" ❌ [결과] 응답하는 서보가 하나도 없습니다!")
        print(" 💡 필수 체크리스트:")
        print("  1. 서보 보드에 12V 배터리 전원(스위치 ON)이 켜져 있나요?")
        print("     (배터리가 꺼져 있으면 통신 칩에 전원이 안 들어갑니다!)")
        print("  2. 모터와 보드 사이의 흰색 3핀 케이블이 헐겁지 않나요?")
        print("  3. 배터리 잔량이 부족하여 저전압 차단이 걸렸을 수 있습니다.")
    print("=" * 55)


if __name__ == '__main__':
    main()
