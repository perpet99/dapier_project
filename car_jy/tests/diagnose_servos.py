#!/usr/bin/env python3
"""
STS3215 서보 모터 종합 진단 & 강제 회전 툴
- 모든 시리얼 포트 (/dev/ttyACM*, /dev/ttyUSB*) 자동 탐색
- 모든 Baud Rate (1000000, 115200, 500000) 테스트
- 서보 PING 및 전압/온도 읽기
- 브로드캐스트 ID (0xFE)로 모든 모터 강제 회전 시도
"""
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


def read_response(ser, timeout=0.1):
    start = time.time()
    res = bytearray()
    while time.time() - start < timeout:
        if ser.in_waiting > 0:
            res.extend(ser.read(ser.in_waiting))
            if len(res) >= 6 and res[0] == 0xFF and res[1] == 0xFF:
                # 패킷 길이 확인
                pkg_len = res[3]
                if len(res) >= pkg_len + 4:
                    return bytes(res)
        time.sleep(0.005)
    return bytes(res)


def main():
    print("==================================================")
    print("      STS3215 서보 모터 종합 정밀 진단 툴")
    print("==================================================")

    # 1. 포트 목록 찾기
    ports = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
    if not ports:
        print("❌ 연결된 USB/ACM 시리얼 장치를 찾을 수 없습니다!")
        print("   -> USB 케이블이 제대로 연결되었는지 확인하세요.")
        return

    print(f"발견된 시리얼 포트: {ports}\n")

    bauds = [1000000, 115200, 500000]
    found_any = False

    for port in ports:
        for baud in bauds:
            try:
                ser = serial.Serial(port, baud, timeout=0.05)
            except Exception as e:
                continue

            print(f"[*] 테스트 중: 포트 {port} @ {baud} bps...")

            # ID 1 ~ 10 Ping 테스트
            detected_ids = []
            for sid in range(1, 10):
                # PING instruction: 0x01
                send_packet(ser, sid, 0x01, [])
                resp = read_response(ser, timeout=0.04)
                if len(resp) >= 6 and resp[0] == 0xFF and resp[1] == 0xFF and resp[2] == sid:
                    detected_ids.append(sid)

            if detected_ids:
                found_any = True
                print(f"  🎉 [발견!] {port} @ {baud} bps 에서 서보 ID {detected_ids} 응답 확인!")

                for sid in detected_ids:
                    # 전압(0x3E) 및 온도(0x3F) 읽기 (READ_DATA: 0x02, Reg: 0x3E, Len: 2)
                    send_packet(ser, sid, 0x02, [0x3E, 2])
                    resp = read_response(ser, timeout=0.05)
                    voltage_str = "알 수 없음"
                    if len(resp) >= 8:
                        raw_v = resp[5]
                        voltage_str = f"{raw_v / 10.0:.1f} V (온도: {resp[6]}°C)"
                    print(f"     - 서보 ID {sid}: 공급 전압 = {voltage_str}")

                # 모터 3초간 강제 회전 시도
                print(f"  👉 발견된 서보 {detected_ids} 3초간 회전 시도...")
                for sid in detected_ids:
                    # Torque Enable (0x28 -> 1)
                    send_packet(ser, sid, 0x03, [0x28, 1])
                    time.sleep(0.02)
                    # Wheel Mode (0x21 -> 1)
                    send_packet(ser, sid, 0x03, [0x21, 1])
                    time.sleep(0.02)
                    # Speed = 2500 step/s
                    speed_bytes = struct.pack('<h', 2500)
                    send_packet(ser, sid, 0x03, [0x2E] + list(speed_bytes))
                    time.sleep(0.02)

                time.sleep(3)

                # 정지
                for sid in detected_ids:
                    send_packet(ser, sid, 0x03, [0x2E, 0, 0])
                    time.sleep(0.02)
                print("  ✓ 테스트 완료 및 정지.\n")

            ser.close()

    if not found_any:
        print("\n==================================================")
        print("⚠️ 모든 포트/Baud에서 서보 응답(Ping)이 오지 않았습니다.")
        print("--------------------------------------------------")
        print("점검 리스트:")
        print("1. [외부 배터리]: 서보 드라이버 보드에 7.4V~12V 전원선이 연결되어 있고 스위치가 켜져 있나요?")
        print("2. [TX/RX 신호선]: 모터 드라이버와 서보 사이의 3핀(GND, VCC, Data) 케이블이 끝까지 꽂혀 있나요?")
        print("3. [브로드캐스트 강제 회전]: 아래 명령어로 전원/모터 강제 회전을 시도해 보세요:")
        print("   python3 -c \"import serial, struct, time; s=serial.Serial('/dev/ttyACM0', 1000000); s.write(bytearray([0xFF,0xFF,0xFE,0x04,0x03,0x28,0x01,0xD5])); s.write(bytearray([0xFF,0xFF,0xFE,0x04,0x03,0x21,0x01,0xDC])); s.write(bytearray([0xFF,0xFF,0xFE,0x05,0x03,0x2E,0xC4,0x09,0x22])); time.sleep(3); s.write(bytearray([0xFF,0xFF,0xFE,0x05,0x03,0x2E,0x00,0x00,0xCF]))\"")
        print("==================================================")


if __name__ == '__main__':
    main()
