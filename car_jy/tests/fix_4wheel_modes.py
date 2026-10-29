#!/usr/bin/env python3
"""
4개 서보 모터(ID 1, 2, 3, 4) EEPROM 영구 휠 모드(Wheel Mode) 강제 복구 & 각도 제한 해제 스크립트
(fix_4wheel_modes.py)
-----------------------------------------------------------------------------------------
- 모든 모터의 각도 제한(Min/Max Angle Limit)을 해제하고
- EEPROM에 영구 Wheel Mode(0x21=1)를 기록하여 각도 리미트 걸림/중간 정지 현상을 완전 박멸합니다.
"""

import sys
import time
import glob
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


def find_servo_port():
    import os
    if os.path.exists('/dev/servo_port'):
        return '/dev/servo_port'
    ports = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
    for p in ports:
        if 'ttyACM0' in p or 'ttyUSB0' in p:
            return p
    return ports[0] if ports else None


def fix_servo(ser, sid):
    print(f"[*] 서보 ID {sid} 휠 모드 영구 복구 중...")
    
    # 1. EEPROM Unlock (0x37 = 0)
    send_packet(ser, sid, 0x03, [0x37, 0x00])
    time.sleep(0.02)
    
    # 2. 토크 끄기 (모드 변경 시 필수) (0x28 = 0)
    send_packet(ser, sid, 0x03, [0x28, 0x00])
    time.sleep(0.02)
    
    # 3. Min Angle Limit = 0, Max Angle Limit = 0 (연속 회전 허용)
    send_packet(ser, sid, 0x03, [0x09, 0x00, 0x00])
    time.sleep(0.02)
    send_packet(ser, sid, 0x03, [0x0B, 0x00, 0x00])
    time.sleep(0.02)

    # 4. Mode = 1 (Wheel Mode, 360도 무한 연속 회전)
    send_packet(ser, sid, 0x03, [0x21, 0x01])
    time.sleep(0.02)

    # 5. 토크 켜기 (0x28 = 1)
    send_packet(ser, sid, 0x03, [0x28, 0x01])
    time.sleep(0.02)

    # 6. EEPROM Lock (0x37 = 1) -> EEPROM 영구 저장
    send_packet(ser, sid, 0x03, [0x37, 0x01])
    time.sleep(0.02)
    
    # 7. 속도 0으로 정지
    send_packet(ser, sid, 0x03, [0x2E, 0x00, 0x00])
    time.sleep(0.02)
    print(f"    ✅ 서보 ID {sid} 복구 완료!")


def main():
    print("=" * 65)
    print("  Feetech STS3215 4개 바퀴 EEPROM 영구 휠 모드 복구 마법사")
    print("=" * 65)

    port = find_servo_port()
    if not port:
        print("❌ 시리얼 포트를 찾을 수 없습니다!")
        return

    print(f"🔌 포트 연결: {port}")
    try:
        ser = serial.Serial(port, 1000000, timeout=0.05)
    except Exception as e:
        print(f"❌ 포트 열기 실패: {e}")
        return

    try:
        for sid in [1, 2, 3, 4]:
            fix_servo(ser, sid)
            
        print("\n" + "=" * 65)
        print("🎉 4개 모터(ID 1, 2, 3, 4) 모두 무한 회전 휠 모드로 영구 복구 완료!")
        print("   이제 각도 리미트나 멈춤 없이 무제한으로 회전합니다.")
        print("=" * 65)
    finally:
        ser.close()


if __name__ == '__main__':
    main()
