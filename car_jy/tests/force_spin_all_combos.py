#!/usr/bin/env python3
"""
만능 모터 회전 스캔 도구 (force_spin_all_combos.py)
------------------------------------------------
- 포트: /dev/ttyACM0, /dev/ttyUSB0 자동 순회
- 통신속도: 1,000,000 bps, 500,000 bps, 115,200 bps 자동 순회
- DTR/RTS 보정 적용
- 브로드캐스트(254) + 개별 ID(1~4) 모두 회전 명령 송신
- 바퀴가 도는 순간 어떤 포트/속도가 정답인지 바로 찾아냅니다!
"""
import glob
import time
import serial


def calc_chk(buf):
    return (~(sum(buf) & 0xFF)) & 0xFF


def send_packet(ser, sid, reg, data):
    length = len(data) + 3
    body = [sid, length, 0x03, reg] + list(data)
    chk = calc_chk(body)
    packet = bytearray([0xFF, 0xFF] + body + [chk])
    ser.write(packet)
    ser.flush()


def test_combo(port, baud):
    print("\n" + "=" * 65)
    print(f"👉 [테스트 조합] 포트: {port}  |  통신속도: {baud:,} bps")
    print("=" * 65)

    try:
        ser = serial.Serial(port, baud, timeout=0.05)
    except PermissionError:
        print(f"   ❌ 권한 오류: 'sudo chmod 666 {port}' 가 필요합니다.")
        return
    except Exception as e:
        print(f"   ⚠️ 포트 열기 실패: {e}")
        return

    # DTR / RTS 활성화
    ser.dtr = True
    ser.rts = True
    time.sleep(0.05)

    # 1. 휠 모드 설정 (브로드캐스트 254 및 1~4번)
    send_packet(ser, 254, 0x37, [0x00])  # Unlock
    time.sleep(0.01)
    send_packet(ser, 254, 0x21, [0x01])  # Wheel Mode
    time.sleep(0.01)
    send_packet(ser, 254, 0x37, [0x01])  # Lock
    time.sleep(0.02)

    for sid in [1, 2, 3, 4]:
        send_packet(ser, sid, 0x37, [0x00])
        time.sleep(0.005)
        send_packet(ser, sid, 0x21, [0x01])
        time.sleep(0.005)
        send_packet(ser, sid, 0x37, [0x01])

    # 2. 1.5초간 회전 명령 (속도 1000)
    print(f"   🚀 [{baud:,} bps] 회전 신호 전송 중! (바퀴를 보세요...)")
    for _ in range(15):
        send_packet(ser, 254, 0x2E, [0xE8, 0x03])
        for sid in [1, 2, 3, 4]:
            send_packet(ser, sid, 0x2E, [0xE8, 0x03])
        time.sleep(0.1)

    # 3. 정지
    send_packet(ser, 254, 0x2E, [0x00, 0x00])
    for sid in [1, 2, 3, 4]:
        send_packet(ser, sid, 0x2E, [0x00, 0x00])
    ser.close()
    time.sleep(0.3)


def main():
    print("*" * 65)
    print("      STS3215 만능 모터 회전 조합 탐색기")
    print("      (모든 포트와 통신속도를 순서대로 돌려봅니다)")
    print("*" * 65)

    ports = sorted(list(set(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))))
    if not ports:
        print("❌ 발견된 시리얼 포트가 없습니다! USB를 확인하세요.")
        return

    bauds = [1000000, 500000, 115200]

    for p in ports:
        for b in bauds:
            test_combo(p, b)

    print("\n" + "*" * 65)
    print("모든 포트와 속도 조합 테스트가 끝났습니다.")
    print("혹시 중간에 바퀴가 조금이라도 돈 순간이 있었나요?")
    print("*" * 65)


if __name__ == '__main__':
    main()
