"""
car2 키보드 방향키 조종 테스트 (Linux 터미널 / SSH, curses 사용).

사용법:
    python car2_arrow.py --port /dev/ttyS0
    python car2_arrow.py --port /dev/ttyS0 --speed 1200

조작:
    ↑ 전진   ↓ 후진   ← 좌회전(제자리)   → 우회전(제자리)
    키를 누르고 있는 동안만 움직이고, 떼면 정지
    Space 정지   +/- 속도 조절   q 종료

!! 첫 테스트는 반드시 차체를 들어 바퀴가 공중에 뜬 상태에서 진행 !!

참고: 터미널은 "키를 뗐다"는 이벤트를 주지 않으므로 키 자동 반복이 끊기는 것으로 뗌을 판단한다.
      처음 누른 뒤 자동 반복이 시작되기까지(보통 0.25~0.5초)는 FIRST_RELEASE 만큼 기다린다.
"""

import argparse
import curses
import locale
import os
import time

from car2_test import MOVE_PERIOD, Car

MAX_SPEED = 3400
MIN_SPEED = 200
SPEED_STEP = 200
FIRST_RELEASE = 0.6  # 첫 입력 후 자동 반복 시작 전까지 유지 시간(초)
REPEAT_RELEASE = 0.15  # 자동 반복 중 이 시간 동안 입력이 없으면 뗀 것으로 판단(초)

# 키 -> (이름, linear 부호, angular 부호), angular 양수 = 좌회전
ACTIONS = {
    curses.KEY_UP: ("전진", 1, 0),
    curses.KEY_DOWN: ("후진", -1, 0),
    curses.KEY_LEFT: ("좌회전", 0, 1),
    curses.KEY_RIGHT: ("우회전", 0, -1),
}


def draw(stdscr, speed, action, last_cmd):
    # 중요한 정보(상태/속도/명령)를 위에 두고, 터미널이 작으면 들어가는 줄까지만 그린다
    lines = [
        f"상태: {action[0] if action else '정지'}   속도: {speed}   명령: {last_cmd}",
        "↑전진 ↓후진 ←좌회전 →우회전 (누르는 동안 이동)",
        "Space 정지  +/- 속도  q 종료",
    ]
    height, width = stdscr.getmaxyx()
    stdscr.erase()
    for row, text in enumerate(lines[:height]):
        try:
            stdscr.addnstr(row, 0, text, width - 1)
        except curses.error:
            pass  # 한글(2칸 문자) 때문에 폭을 넘는 경우 등은 표시만 생략
    stdscr.refresh()


def run(stdscr, car, speed):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)

    action = None  # 현재 동작 (ACTIONS 값)
    repeats = 0  # 같은 키 자동 반복 횟수 (0이면 아직 첫 입력)
    last_key = 0.0
    last_send = 0.0
    moving = False
    last_cmd = "-"
    draw(stdscr, speed, action, last_cmd)

    while True:
        now = time.time()
        changed = False

        key = stdscr.getch()
        while key != -1:
            changed = True
            if key in ACTIONS:
                if ACTIONS[key] is action:
                    repeats += 1
                else:
                    action, repeats = ACTIONS[key], 0
                last_key = now
            elif key == ord(" "):
                action = None
            elif key in (ord("+"), ord("=")):
                speed = min(speed + SPEED_STEP, MAX_SPEED)
            elif key in (ord("-"), ord("_")):
                speed = max(speed - SPEED_STEP, MIN_SPEED)
            elif key in (ord("q"), ord("Q"), 27):
                return
            key = stdscr.getch()

        # 키를 뗐는지 판단
        if action and now - last_key > (REPEAT_RELEASE if repeats else FIRST_RELEASE):
            action = None
            changed = True

        # 이동 중에는 MOVE_PERIOD마다 재전송(car2.ino 500ms 타임아웃 방지), 멈출 때는 STOP 1회
        if action and (not moving or changed or now - last_send >= MOVE_PERIOD):
            _, lin, ang = action
            last_cmd = f"V,{lin * speed},{ang * speed}"
            car.send(last_cmd)
            last_send, moving = now, True
        elif not action and moving:
            last_cmd = "STOP"
            car.send(last_cmd)
            moving = False
            changed = True

        car.ser.reset_input_buffer()  # ACK 응답은 버림
        if changed:
            draw(stdscr, speed, action, last_cmd)
        time.sleep(0.01)


def main():
    parser = argparse.ArgumentParser(description="car2 방향키 조종 테스트")
    parser.add_argument("--port", required=True, help="예: /dev/ttyS0, /dev/ttyUSB0")
    parser.add_argument("--speed", type=int, default=800, help=f"시작 속도 (step/s, {MIN_SPEED}~{MAX_SPEED})")
    args = parser.parse_args()

    locale.setlocale(locale.LC_ALL, "")  # curses 한글 출력
    os.environ.setdefault("ESCDELAY", "25")  # Esc 키 반응 지연 줄이기

    car = Car(args.port)
    try:
        curses.wrapper(run, car, max(MIN_SPEED, min(args.speed, MAX_SPEED)))
    except KeyboardInterrupt:
        pass
    finally:
        car.close()  # 종료 시 반드시 STOP
        print("종료 (STOP 전송)")


if __name__ == "__main__":
    main()
