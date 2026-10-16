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
    else:
        print(" ❌ 어떤 포트에서도 모터 응답이 없습니다.")
        print(" 💡 필수 체크포인트:")
        print("    1. 로봇 배터리 전원 스위치가 켜져 있나요? (모터는 12V 배터리 필수)")
        print("    2. 드라이버 보드와 서보 사이의 3핀 흰색 케이블이 꽂혀 있나요?")
        print("    3. 다른 ROS 노드가 켜져 있다면 종료하세요: 'fuser -k /dev/ttyACM0'")
    print("=" * 60)

if __name__ == '__main__':
    check_all()
