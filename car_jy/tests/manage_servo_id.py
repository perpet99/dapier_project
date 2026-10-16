#!/usr/bin/env python3
"""
STS3215 서보 모터 ID 확인, 변경 및 휠 모드 설정 툴 (manage_servo_id.py)
------------------------------------------------------------------
1. 현재 연결된 모든 서보 모터의 ID 및 모드(Wheel/Position) 자동 스캔
2. 서보 ID 변경 (EEPROM 언락 -> ID 변경 -> 영구 저장)
3. 4개 바퀴 모두 무한 연속 회전(Wheel Mode)으로 일괄 설정
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


def read_response(ser, timeout=0.08):
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
        res = read_response(ser, timeout=0.06)
        return len(res) >= 6 and res[2] == servo_id
    except Exception:
        return False


def read_register(ser, servo_id, reg, length=1):
    try:
        ser.reset_input_buffer()
    except Exception:
        pass
    try:
        send_packet(ser, servo_id, 0x02, [reg, length])
        res = read_response(ser, timeout=0.06)
        if len(res) >= 5 + length and res[2] == servo_id:
            return list(res[5:5 + length])
    except Exception:
        pass
    return None


def write_register(ser, servo_id, reg, data_bytes):
    send_packet(ser, servo_id, 0x03, [reg] + list(data_bytes))
    time.sleep(0.02)


def unlock_eeprom(ser, servo_id):
    # Register 0x37 (Lock) = 0 (Unlock)
    write_register(ser, servo_id, 0x37, [0x00])


def lock_eeprom(ser, servo_id):
    # Register 0x37 (Lock) = 1 (Lock)
    write_register(ser, servo_id, 0x37, [0x01])


def set_servo_mode(ser, servo_id, mode=1):
    """0: Position Mode (0~360도), 1: Wheel Mode (연속 회전)"""
    unlock_eeprom(ser, servo_id)
    write_register(ser, servo_id, 0x21, [mode])
    lock_eeprom(ser, servo_id)
    mode_name = "연속 회전(Wheel Mode)" if mode == 1 else "각도 제어(Position Mode)"
    print(f"   ⚙️  서보 ID {servo_id} -> {mode_name} 설정 완료!")


def change_servo_id(ser, old_id, new_id):
    """서보 ID 변경 및 영구 저장"""
    print(f"\n🔄 서보 ID 변경 시도: [{old_id}] ➔ [{new_id}] ...")
    if not ping_servo(ser, old_id):
        print(f"❌ 서보 ID {old_id}로부터 응답이 없습니다! 번호를 확인하세요.")
        return False

    unlock_eeprom(ser, old_id)
    write_register(ser, old_id, 0x05, [new_id])
    lock_eeprom(ser, new_id)
    time.sleep(0.1)

    # 새로운 ID로 확인
    if ping_servo(ser, new_id):
        print(f"🎉 성공! 서보 ID가 [{new_id}] 번으로 성공적으로 변경 및 영구 저장되었습니다!")
        # 휠 모드도 자동 설정
        set_servo_mode(ser, new_id, mode=1)
        return True
    else:
        print(f"⚠️ 새 ID {new_id}로 응답이 오지 않습니다. 전원을 껐다 켜서 확인해 보세요.")
        return False


def scan_servos(ser, max_id=10):
    print(f"\n🔍 서보 모터 스캔 중 (ID 1 ~ {max_id})...")
    found_servos = []
    for sid in range(1, max_id + 1):
        if ping_servo(ser, sid):
            mode_data = read_register(ser, sid, 0x21, 1)
            volt_data = read_register(ser, sid, 0x3E, 1)
            mode_str = "알 수 없음"
            if mode_data:
                mode_str = "연속 회전(Wheel Mode)" if mode_data[0] == 1 else "각도 제어(Position Mode)"
            volt_str = f"{volt_data[0] / 10.0:.1f}V" if volt_data else "N/A"

            print(f"   ✅ [발견] 서보 ID: {sid:<2} | 모드: {mode_str:<20} | 전압: {volt_str}")
            found_servos.append(sid)
        time.sleep(0.01)

    if not found_servos:
        print("❌ 발견된 서보 모터가 없습니다! 전원선 및 직렬 연결을 확인해 주세요.")
    else:
        print(f"\n총 {len(found_servos)}개의 서보 모터가 발견되었습니다: {found_servos}")
    return found_servos


def find_servo_port():
    import os
    if os.path.exists('/dev/servo_port'):
        return '/dev/servo_port'
    ports = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
    for p in ports:
        if 'ttyACM0' in p or 'ttyUSB0' in p:
            return p
    return ports[0] if ports else None


def main():
    print("=" * 65)
    print("      Feetech STS3215 서보 모터 ID 관리 & 4바퀴 설정 도구")
    print("=" * 65)

    # 포트 인자 확인 (--port /dev/xxx)
    port = None
    if '--port' in sys.argv:
        p_idx = sys.argv.index('--port')
        if p_idx + 1 < len(sys.argv):
            port = sys.argv[p_idx + 1]
    
    if not port:
        port = find_servo_port()

    if not port:
        print("❌ 연결된 시리얼 포트를 찾을 수 없습니다!")
        return

    print(f"🔌 포트: {port} (Baud Rate: 1,000,000 bps)")

    try:
        ser = serial.Serial(port, 1000000, timeout=0.05)
    except Exception as e:
        print(f"❌ 포트 열기 실패: {e}")
        print("💡 팁: 혹시 다른 터미널에서 full_bringup 이나 sts_controller 가 켜져 있다면 먼저 종료해 주세요!")
        return

    # CLI 인자 처리
    # 사용법 1: python3 manage_servo_id.py (스캔 및 메뉴)
    # 사용법 2: python3 manage_servo_id.py change <old_id> <new_id>
    # 사용법 3: python3 manage_servo_id.py wheel <id>
    # 사용법 4: python3 manage_servo_id.py wheel-all
    if len(sys.argv) >= 4 and sys.argv[1] == 'change':
        old_id = int(sys.argv[2])
        new_id = int(sys.argv[3])
        change_servo_id(ser, old_id, new_id)
        ser.close()
        return

    if len(sys.argv) >= 3 and sys.argv[1] == 'wheel':
        sid = int(sys.argv[2])
        set_servo_mode(ser, sid, mode=1)
        ser.close()
        return

    # 기본: 스캔 후 대화형 메뉴
    found = scan_servos(ser, max_id=10)

    print("\n" + "=" * 65)
    print("  [메뉴 선택]")
    print("  1. 특정 서보 모터 ID 변경 (예: 1번 ➔ 3번)")
    print("  2. 발견된 모든 서보를 무한 연속 회전(Wheel Mode)으로 일괄 설정")
    print("  3. 다시 스캔하기")
    print("  4. 종료")
    print("=" * 65)

    try:
        choice = input("선택 번호를 입력하세요 (1~4): ").strip()
    except (EOFError, KeyboardInterrupt):
        ser.close()
        return

    if choice == '1':
        try:
            old_id = int(input("현재 바꿀 서보의 기존 ID (예: 1): "))
            new_id = int(input("새로 지정할 ID 번호 (예: 3 또는 4): "))
            change_servo_id(ser, old_id, new_id)
        except ValueError:
            print("❌ 올바른 숫자를 입력해 주세요.")
    elif choice == '2':
        print("\n⚙️  모든 발견된 서보를 연속 회전(Wheel Mode)으로 설정합니다...")
        for sid in found:
            set_servo_mode(ser, sid, mode=1)
        print("🎉 모든 모터가 바퀴 회전 모드로 설정되었습니다!")
    elif choice == '3':
        scan_servos(ser, max_id=10)

    ser.close()


if __name__ == '__main__':
    main()
