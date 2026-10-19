#!/usr/bin/env python3
"""
터미널 원클릭 모터 진단 도구 (check_motors.py)
- 포트 자동 탐색 (/dev/ttyACM*, /dev/ttyUSB*)
- 1,000,000 bps & 115,200 bps 동시 검사
- ID 1~4 서보 Ping 및 전압(배터리 상태) 측정
"""
import sys
import glob
import time
import serial

def calc_chk(buf):
    return (~(sum(buf) & 0xFF)) & 0xFF

def ping(ser, sid):
    try:
        ser.reset_input_buffer()
        body = [sid, 2, 1]
        ser.write(bytearray([0xFF, 0xFF] + body + [calc_chk(body)]))
        ser.flush()
        t0 = time.time()
        res = bytearray()
        while time.time() - t0 < 0.04:
            if ser.in_waiting:
                res.extend(ser.read(ser.in_waiting))
                if len(res) >= 6 and res[0] == 0xFF and res[1] == 0xFF and res[2] == sid:
                    return True
            time.sleep(0.002)
    except Exception:
        pass
    return False

def read_voltage(ser, sid):
    try:
        ser.reset_input_buffer()
        body = [sid, 4, 2, 0x3E, 1]  # READ Present Voltage (0x3E)
        ser.write(bytearray([0xFF, 0xFF] + body + [calc_chk(body)]))
        ser.flush()
        t0 = time.time()
        res = bytearray()
        while time.time() - t0 < 0.04:
            if ser.in_waiting:
                res.extend(ser.read(ser.in_waiting))
                if len(res) >= 7 and res[0] == 0xFF and res[1] == 0xFF and res[2] == sid:
                    return res[5] / 10.0
            time.sleep(0.002)
    except Exception:
        pass
    return None

def check_all():
    print("=" * 60)
    print(" 🔍 STS3215 모터 연결 상태 종합 진단")
    print("=" * 60)
    
    ports = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
    if not ports:
        print("❌ 발견된 시리얼 장치가 없습니다! USB 케이블을 꽂아주세요.")
        return

    any_found = False
    for port in ports:
        print(f"\n[포트 점검: {port}]")
        for baud in [1000000, 115200]:
            try:
                ser = serial.Serial(port, baud, timeout=0.03)
            except PermissionError:
                print(f" ❌ 권한 거부! 'sudo chmod 666 {port}' 를 먼저 실행하세요.")
                break
            except Exception as e:
                print(f" ⚠️ 포트 열기 실패 ({baud} bps): {e}")
                continue

            found_ids = []
            voltages = {}
            for sid in range(1, 9):  # 1~8번 탐색
                if ping(ser, sid):
                    found_ids.append(sid)
                    v = read_voltage(ser, sid)
                    voltages[sid] = v

            ser.close()

            if found_ids:
                any_found = True
                print(f" ✅ 모터 발견! 통신속도: {baud:,} bps")
                for sid in found_ids:
                    v_str = f"{voltages[sid]:.1f}V" if voltages[sid] is not None else "측정불가"
                    print(f"    - 서보 ID {sid}: 정상 응답 (현재 전압: {v_str})")
                break
            else:
                print(f" ℹ️  {baud:,} bps: 응답 없음")

    print("\n" + "=" * 60)
    if any_found:
        print(" 🎉 모터가 정상적으로 연결되어 통신하고 있습니다!")
        print(" 💡 [바퀴 회전 테스트]: python3 check_motors.py --spin")
    else:
        print(" ❌ 어떤 포트에서도 모터 응답이 없습니다.")
        print(" 💡 필수 체크포인트:")
        print("    1. 로봇 배터리 전원 스위치가 켜져 있나요? (모터는 12V 배터리 필수)")
        print("    2. 드라이버 보드와 서보 사이의 3핀 흰색 케이블이 꽂혀 있나요?")
        print("    3. 다른 ROS 노드가 켜져 있다면 종료하세요: 'fuser -k /dev/ttyACM0'")
    print("=" * 60)


def spin_test(port='/dev/ttyACM0', baud=1000000, duration=1.5):
    """ROS 노드 없이 시리얼 포트로 STS3215 모터 1.5초간 직접 회전 테스트"""
    ports = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
    if not ports:
        print("❌ 발견된 시리얼 장치가 없습니다.")
        return
    target_port = ports[0]
    print("=" * 60)
    print(f" 🚗 [모터 직접 회전 테스트] {target_port} ({duration}초간 천천히 전진)")
    print("=" * 60)

    try:
        ser = serial.Serial(target_port, baud, timeout=0.04)
    except Exception as e:
        print(f" ❌ 포트 열기 실패: {e}")
        print("    (혹시 다른 노드가 포트를 잡고 있다면 'fuser -k /dev/ttyACM0' 실행)")
        return

    # 1. 서보 1~4를 Wheel Mode (무한회전 모드)로 설정
    for sid in [1, 2, 3, 4]:
        for reg, val in [(0x37, [0x00]), (0x21, [0x01]), (0x37, [0x01])]:
            body = [sid, 3 + len(val), 3, reg] + val
            ser.write(bytearray([0xFF, 0xFF] + body + [calc_chk(body)]))
            time.sleep(0.01)

    # 2. 바퀴 회전 속도 전송 (좌/우 방향 반전 고려: ID 1,3 = 전진, ID 2,4 = 역방향 전진)
    # 0x2E Goal Speed: 800 step/s
    speed_mag = 800
    sp_fwd = [speed_mag & 0xFF, (speed_mag >> 8) & 0xFF]
    sp_rev = [speed_mag & 0xFF, ((speed_mag >> 8) | 0x80) & 0xFF]

    sync_packet = [
        0xFE, 16, 0x83, 0x2E, 2,
        1, sp_fwd[0], sp_fwd[1],
        2, sp_rev[0], sp_rev[1],
        3, sp_fwd[0], sp_fwd[1],
        4, sp_rev[0], sp_rev[1]
    ]

    print(" 🚀 바퀴가 회전합니다...")
    t0 = time.time()
    while time.time() - t0 < duration:
        ser.write(bytearray([0xFF, 0xFF] + sync_packet + [calc_chk(sync_packet)]))
        time.sleep(0.05)

    # 3. 정지 (속도 0)
    stop_packet = [
        0xFE, 16, 0x83, 0x2E, 2,
        1, 0, 0, 2, 0, 0, 3, 0, 0, 4, 0, 0
    ]
    ser.write(bytearray([0xFF, 0xFF] + stop_packet + [calc_chk(stop_packet)]))
    time.sleep(0.05)
    ser.close()
    print(" ✅ 회전 테스트 완료! 모터가 정상적으로 정지했습니다.\n")


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] in ['--spin', '-s']:
        spin_test()
    else:
        check_all()
