#!/usr/bin/env python3
"""
LD14 / LD14P 라이다 정밀 신호 및 하드웨어 진단 툴 (test_lidar.py)
------------------------------------------------------------
1. /dev/ttyUSB0 포트를 사용하는 다른 프로세스(fuser) 감지
2. DTR / RTS 신호 제어 (슬립 모드 해제)
3. 230400, 115200, 256000 bps 전수 조사 및 원시 바이트 수신 체크
"""

import os
import sys
import time
import glob
import subprocess
import serial


def find_lidar_port():
    matches = glob.glob('/dev/serial/by-id/*CP210*') + glob.glob('/dev/serial/by-id/*Silicon_Labs*')
    if matches:
        return os.path.realpath(matches[0])
    for p in ['/dev/lidar_port', '/dev/ttyUSB0', '/dev/ttyUSB1']:
        if os.path.exists(p):
            return os.path.realpath(p)
    return None


def check_competing_processes(port):
    """포트를 다른 프로세스가 점유하고 있는지 확인"""
    try:
        res = subprocess.run(['fuser', port], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        pids = res.stdout.strip().split()
        if pids:
            print(f"⚠️ 경고: 현재 다음 프로세스가 포트({port})를 잡고 있습니다: PID {pids}")
            print("   -> 'kill -9 <PID>' 로 종료하거나 다른 터미널의 노드를 꺼주세요.")
            return False
    except Exception:
        pass
    return True


def test_lidar_stream(port):
    print("=" * 65)
    print("        LDROBOT 라이다 정밀 통신 & 하드웨어 진단")
    print("=" * 65)
    print(f"🔌 검사 대상 포트: {port}")

    check_competing_processes(port)

    # 1. 속도별 테스트 목록
    bauds = [230400, 115200, 256000]

    for baud in bauds:
        print(f"\n📡 [속도 {baud:,} bps] 테스트 (DTR/RTS 신호 제어)...")
        try:
            ser = serial.Serial()
            ser.port = port
            ser.baudrate = baud
            ser.timeout = 1.0
            ser.open()

            # DTR/RTS 신호 토글 (일부 어댑터 보드의 절전 모드 해제)
            ser.dtr = False
            ser.rts = False
            time.sleep(0.1)
            ser.dtr = True
            ser.rts = True
            time.sleep(0.1)
            ser.reset_input_buffer()

        except Exception as e:
            print(f"❌ 포트 열기 실패: {e}")
            continue

        start = time.time()
        raw_bytes = bytearray()
        header_count = 0

        while time.time() - start < 2.0:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                raw_bytes.extend(chunk)
                for i in range(len(chunk) - 1):
                    if chunk[i] == 0x54 and chunk[i + 1] == 0x2C:
                        header_count += 1
            time.sleep(0.02)

        ser.close()

        if header_count > 5:
            print(f"   🎉 대성공! {baud:,} bps 에서 유효한 라이다 패킷(0x54 0x2C)이 {header_count}개 감지되었습니다!")
            return True, baud
        elif len(raw_bytes) > 0:
            print(f"   ⚠️ 데이터는 들어오고 있습니다 ({len(raw_bytes)} bytes 수신), 하지만 헤더가 일치하지 않습니다.")
        else:
            print(f"   ❌ 2초 동안 수신된 데이터: 0 bytes (RX 신호 없음)")

    return False, None


def main():
    port = find_lidar_port()
    if not port:
        print("❌ 라이다 USB 장치를 찾을 수 없습니다!")
        return

    success, baud = test_lidar_stream(port)

    print("\n" + "=" * 65)
    if success:
        print(f"✅ 라이다 통신 정상 확인! 최적 속도: {baud:,} bps")
    else:
        print("🔍 [0바이트 수신 원인 & 하드웨어 해결 가이드]")
        print("1. 라이다 모터는 전원(5V, GND)만 들어가면 컴퓨터 연결과 상관없이 무조건 돕니다.")
        print("2. 그런데 데이터가 0바이트라는 것은 '데이터 신호선(TX)'이 끊겼거나 덜 꽂힌 것입니다!")
        print("   👉 라이다와 파란색/검은색 USB 어댑터 기판 사이에 꽂혀있는")
        print("      '작은 4핀(또는 3핀) 하얀색 커넥터'를 뺐다가 꽉 다시 꽂아보세요!")
        print("   👉 핀 하나가 살짝 뒤로 밀려나와 접촉이 안 될 때 딱 이 증상이 발생합니다.")
    print("=" * 65)


if __name__ == '__main__':
    main()
