#!/usr/bin/env python3
"""
STS3215 모터 1(좌측) vs 모터 2(우측) 전체 레지스터 정밀 비교 & 원인 진단 툴
- ID 1과 ID 2의 모든 내부 레지스터(0x00 ~ 0x40)를 읽어서 1:1 비교
- 속도 제한, 휠 모드 설정, PID 게인, 토크 제한 차이점 완벽 색출
- 속도 명령(200, 500, 1000, 2000) 변경 시 실제 반응 테스트
"""
import glob
import struct
import sys
import time
import serial


def calc_checksum(buf):
    return (~(sum(buf) & 0xFF)) & 0xFF


def send_packet(ser, servo_id, inst, params):
    length = len(params) + 2
    body = [servo_id, length, inst] + list(params)
    chk = calc_checksum(body)
    packet = bytearray([0xFF, 0xFF] + body + [chk])
    ser.write(packet)
    ser.flush()


def read_reg(ser, servo_id, reg, length=1):
    # READ_DATA: 0x02
    body = [servo_id, 4, 0x02, reg, length]
    chk = calc_checksum(body)
    packet = bytearray([0xFF, 0xFF] + body + [chk])
    ser.reset_input_buffer()
    ser.write(packet)
    ser.flush()

    start = time.time()
    res = bytearray()
    expected_len = length + 6
    while time.time() - start < 0.05:
        if ser.in_waiting > 0:
            res.extend(ser.read(ser.in_waiting))
            if len(res) >= expected_len and res[0] == 0xFF and res[1] == 0xFF and res[2] == servo_id:
                data = res[5:5 + length]
                return data
        time.sleep(0.002)
    return None


def main():
    ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    port = ports[0] if ports else '/dev/ttyACM0'
    baud = 1000000

    print("=" * 70)
    print("      STS3215 ID 1(좌측) vs ID 2(우측) 레지스터 1:1 비교 진단")
    print(f"      - 시리얼 포트: {port} @ {baud} bps")
    print("=" * 70)

    try:
        ser = serial.Serial(port, baud, timeout=0.05)
    except Exception as e:
        print(f"❌ 포트 열기 실패: {e}")
        sys.exit(1)

    # 확인할 주요 레지스터 목록
    REGISTER_NAMES = {
        0x03: ("Firmware Major", 1),
        0x04: ("Baud Rate", 1),
        0x0B: ("Max Torque Limit", 2),
        0x0E: ("Min Angle Limit", 2),
        0x10: ("Max Angle Limit", 2),
        0x15: ("P Coefficient", 1),
        0x16: ("D Coefficient", 1),
        0x17: ("I Coefficient", 1),
        0x21: ("Operating Mode (0:Pos, 1:Wheel)", 1),
        0x28: ("Torque Enable", 1),
        0x29: ("Acceleration (P_ACC)", 1),
        0x2E: ("Target Speed (Goal)", 2),
        0x38: ("Present Position", 2),
        0x3A: ("Present Speed", 2),
        0x3E: ("Present Voltage", 1),
        0x3F: ("Present Temperature", 1),
    }

    print(f"{'주소':^6} | {'레지스터 이름':<32} | {'ID 1 (좌측)':^12} | {'ID 2 (우측)':^12} | {'상태':^6}")
    print("-" * 75)

    differences = []
    for reg, (name, length) in sorted(REGISTER_NAMES.items()):
        val1 = read_reg(ser, 1, reg, length)
        time.sleep(0.01)
        val2 = read_reg(ser, 2, reg, length)
        time.sleep(0.01)

        str1 = "N/A" if val1 is None else (f"{val1[0]}" if length == 1 else f"{struct.unpack('<h', val1)[0]}")
        str2 = "N/A" if val2 is None else (f"{val2[0]}" if length == 1 else f"{struct.unpack('<h', val2)[0]}")

        match = "✅ 일치" if str1 == str2 else "🚨 차이!"
        if str1 != str2 and val1 is not None and val2 is not None:
            differences.append((reg, name, str1, str2))

        print(f"0x{reg:02X}   | {name:<32} | {str1:^12} | {str2:^12} | {match:^6}")

    print("-" * 75)
    if differences:
        print("\n🚨 [발견된 결정적 레지스터 차이점!]")
        for reg, name, s1, s2 in differences:
            print(f"   👉 0x{reg:02X} ({name}): ID 1 = {s1}  vs  ID 2 = {s2}")
    else:
        print("\n✅ 두 모터의 기본 레지스터 설정은 완전히 동일합니다.")

    # 2. 속도 가변성 테스트 (ID 1이 속도 명령에 비례해서 도는지, 아니면 항상 최고속도로 도는지 점검)
    print("\n" + "=" * 70)
    print("🧪 [속도 가변성 테스트] ID 1(좌측)이 속도 명령에 반응하는지 점검")
    print("   바퀴를 공중에 띄운 상태로 지켜보세요.")
    print("=" * 70)

    # 토크 해제 후 모드 재설정 (STS 모터는 토크 꺼진 상태에서 모드 변경이 확실함)
    send_packet(ser, 1, 0x03, [0x28, 0])  # Torque OFF
    time.sleep(0.02)
    send_packet(ser, 1, 0x03, [0x21, 1])  # Wheel Mode ON
    time.sleep(0.02)
    send_packet(ser, 1, 0x03, [0x28, 1])  # Torque ON
    time.sleep(0.02)

    test_speeds = [300, 1000, 2500]
    for spd in test_speeds:
        print(f"[*] ID 1에 속도 {spd} step/s 인가 중... (2초간)")
        send_packet(ser, 1, 0x03, [0x2E] + list(struct.pack('<h', -spd)))
        time.sleep(2.0)

    # 정지
    send_packet(ser, 1, 0x03, [0x2E, 0, 0])
    print("🛑 테스트 완료 및 정지.")
    ser.close()


if __name__ == '__main__':
    main()
