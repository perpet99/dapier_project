"""
car2.ino(ST3215 4륜 자동차) 시리얼 테스트 스크립트.

필요 패키지: pyserial
    pip install pyserial

사용법:
    python car2_test.py --port /dev/ttyUSB0            # 전체 자동 테스트
    python car2_test.py --port COM5 --test wheels      # 바퀴별 방향 점검만
    python car2_test.py --port /dev/ttyUSB0 --keyboard # 키보드(WASD) 조종

!! 첫 테스트는 반드시 차체를 들어 바퀴가 공중에 뜬 상태에서 진행 !!

테스트 항목
    ping    : 4개 서보 응답 확인
    stat    : 전압/온도/위치 등 상태 읽기
    wheels  : 바퀴 1개씩 "전진 방향"으로 회전 -> 눈으로 방향 확인 (반대면 Car.cpp INVERT_DEFAULT 수정)
    drive   : 전진 / 후진 / 좌회전 / 우회전 / 정지
    timeout : 명령을 끊었을 때 자동 정지(TIMEOUT,STOP) 확인
"""

import argparse
import sys
import time

import serial

TEST_SPEED = 800  # step/s (최대 3400)
MOVE_PERIOD = 0.1  # 이동 중 명령 재전송 주기(초), car2.ino의 CMD_TIMEOUT_MS(500ms)보다 짧게


class Car:
    def __init__(self, port, baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        time.sleep(2.0)  # ESP32 USB 연결 시 자동 리셋 -> 부팅 대기
        self.read_lines(1.0)

    def send(self, cmd):
        self.ser.write((cmd + "\n").encode("utf-8"))

    def read_lines(self, duration=0.3):
        lines = []
        end = time.time() + duration
        while time.time() < end:
            line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                print("  <-", line)
                lines.append(line)
        return lines

    def query(self, cmd, duration=0.3):
        print("  ->", cmd)
        self.send(cmd)
        return self.read_lines(duration)

    def hold(self, cmd, seconds):
        """seconds 동안 cmd를 주기적으로 재전송 (타임아웃 자동 정지 방지)."""
        print("  ->", cmd, f"({seconds:.1f}s)")
        end = time.time() + seconds
        while time.time() < end:
            self.send(cmd)
            time.sleep(MOVE_PERIOD)
        self.ser.reset_input_buffer()

    def stop(self):
        self.query("STOP", 0.2)

    def close(self):
        self.send("STOP")
        self.ser.close()


def test_ping(car):
    print("\n[PING] 4개 서보 응답 확인")
    lines = car.query("PING", 0.5)
    ok = sum(1 for l in lines if l.startswith("PING,") and l.endswith(",OK"))
    print(f"  결과: {ok}/4 응답")
    return ok == 4


def test_stat(car):
    print("\n[STAT] 상태 읽기 (V=전압, T=온도)")
    lines = car.query("STAT", 0.5)
    stats = [l for l in lines if l.startswith("STAT,")]
    bad = [l for l in stats if "NO_RESPONSE" in l]
    for l in stats:
        for field in l.split(","):
            if field.startswith("V=") and float(field[2:]) < 7.0:
                print("  경고: 전압이 낮음 (7V 미만) ->", l)
    return len(stats) == 4 and not bad


def test_wheels(car):
    print("\n[WHEELS] 바퀴별 방향 점검 - 각 바퀴가 '차량 전진 방향'으로 도는지 확인")
    names = ["FL(앞왼쪽)", "FR(앞오른쪽)", "RL(뒤왼쪽)", "RR(뒤오른쪽)"]
    for i, name in enumerate(names):
        speeds = [0, 0, 0, 0]
        speeds[i] = TEST_SPEED
        print(f"  {name} 회전")
        car.hold("W," + ",".join(map(str, speeds)), 1.5)
        car.stop()
        time.sleep(0.3)
    return True


def test_drive(car):
    print("\n[DRIVE] 기본 주행")
    steps = [
        ("전진", f"V,{TEST_SPEED},0"),
        ("후진", f"V,{-TEST_SPEED},0"),
        ("좌회전(제자리)", f"V,0,{TEST_SPEED}"),
        ("우회전(제자리)", f"V,0,{-TEST_SPEED}"),
        ("좌/우 개별(D)", f"D,{TEST_SPEED // 2},{TEST_SPEED}"),
    ]
    for name, cmd in steps:
        print(f"  {name}")
        car.hold(cmd, 1.5)
        car.stop()
        time.sleep(0.5)
    return True


def test_timeout(car):
    print("\n[TIMEOUT] 명령 1회만 보내고 대기 -> 0.5초 후 자동 정지해야 함")
    car.query(f"V,{TEST_SPEED},0", 0.1)
    lines = car.read_lines(1.0)
    ok = any(l == "TIMEOUT,STOP" for l in lines)
    print("  결과:", "OK" if ok else "FAIL (자동 정지 메시지 없음)")
    return ok


def keyboard_mode(car):
    """터미널에서 w/a/s/d + Enter로 0.5초씩 이동, x=정지, q=종료."""
    print("\n[KEYBOARD] w=전진 s=후진 a=좌 d=우 x=정지 +/-=속도 q=종료 (입력 후 Enter)")
    speed = TEST_SPEED
    cmds = {"w": (1, 0), "s": (-1, 0), "a": (0, 1), "d": (0, -1)}
    while True:
        key = input(f"[speed={speed}] > ").strip().lower()
        if key == "q":
            break
        if key == "x":
            car.stop()
        elif key == "+":
            speed = min(speed + 200, 3400)
        elif key == "-":
            speed = max(speed - 200, 200)
        elif key in cmds:
            lin, ang = cmds[key]
            car.hold(f"V,{lin * speed},{ang * speed}", 0.5)
            car.stop()


TESTS = {
    "ping": test_ping,
    "stat": test_stat,
    "wheels": test_wheels,
    "drive": test_drive,
    "timeout": test_timeout,
}


def main():
    parser = argparse.ArgumentParser(description="car2 ST3215 자동차 테스트")
    parser.add_argument("--port", required=True, help="예: /dev/ttyUSB0, COM5")
    parser.add_argument("--test", choices=list(TESTS) + ["all"], default="all")
    parser.add_argument("--keyboard", action="store_true", help="키보드 조종 모드")
    args = parser.parse_args()

    car = Car(args.port)
    # Ctrl+C 시에도 반드시 정지 명령 전송
    try:
        if args.keyboard:
            keyboard_mode(car)
            return
        names = list(TESTS) if args.test == "all" else [args.test]
        results = {}
        for name in names:
            results[name] = TESTS[name](car)
            if name == "ping" and not results[name]:
                print("\n서보 응답 불량 -> 이후 테스트 중단 (배선/전원/ID 확인)")
                break
        print("\n=== 결과 ===")
        for name, ok in results.items():
            print(f"  {name:8s} {'PASS' if ok else 'FAIL'}")
        sys.exit(0 if all(results.values()) else 1)
    except KeyboardInterrupt:
        print("\n중단")
    finally:
        car.close()


if __name__ == "__main__":
    main()
