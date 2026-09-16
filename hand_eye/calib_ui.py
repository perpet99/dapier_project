"""
calib_ui.py - SO-101 캘리브레이션을 눈으로 보며 조절하는 GUI.

콘솔판(calibrate_arm.py)은 값을 확인하며 고치기가 어려웠다. 여기서는 관절 위치가
실시간으로 보이고, min/max 를 끌어서 맞추고, 결과 TCP 가 즉시 갱신된다.

화면 구성:
  - 관절마다 가로 막대 = 0~4095 카운트 전체 범위
      밝은 구간 = [min, max] (test4.py 의 하드 클램프 범위)
      세로 노란 선 = 현재 위치
  - 실시간 TCP(FK) 를 위에 표시. 기준자세를 잡으면 바로 맞는지 알 수 있다.

조작:
  마우스   min/max 손잡이를 끌어서 조절
           [min<] / [>max] 현재 위치를 그 한계로 지정
           [+/-]  부호 뒤집기
           관절 이름을 클릭하면 선택 (오프셋 미세조정 대상)
  키보드   z 영점 잡기(현재 자세를 IK 0도로)   [ ] 선택 관절 오프셋 -/+ 0.5도
           s 저장    r 되돌리기    q 종료

안전:
  토크를 처음부터 끝까지 끈다. 목표위치를 쓰지 않으므로 팔은 스스로 움직이지 않는다.
  팔은 중력으로 처지니 손으로 받쳐야 한다.

사용:
  python calib_ui.py [COM18]
"""

from __future__ import annotations

import copy
import math
import sys
import time

import cv2
import numpy as np

from test4 import (ARM_CALIB_PATH, CENTER_COUNT, COUNTS_PER_DEG, JOINT_ORDER,
                   L0_BASE_H, L1_UPPER, L2_FORE, L3_WRIST, L_SHOULDER_R, LEROBOT_CALIB,
                   SERVO_IDS, STS_ADDR, ArmCalibration, FeetechBus,
                   find_so101_port, load_lerobot_calibration, so101_fk)

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass


W, H = 1000, 566
ROW_Y0, ROW_H = 104, 64
TRK_X0, TRK_X1 = 150, 620
COUNT_MAX = 4095
HANDLE_GRAB = 9          # 손잡이를 잡았다고 볼 픽셀 거리
MIN_GAP = 40             # min/max 사이 최소 간격 [counts]
POLL_HZ = 20

# 모든 관절이 0도일 때의 TCP. 영자세는 링크가 일직선인 자세가 아니라
# 가동범위 한가운데 자세다 (test4.py 의 A1_ZERO/A2_ZERO/A3_ZERO 참고).
ZERO_TCP = so101_fk({n: 0.0 for n in JOINT_ORDER})

BG = (26, 26, 28)
FG = (232, 232, 236)
DIM = (128, 128, 136)
ACC = (60, 168, 235)      # 현재 위치 (BGR: 주황)
OK_C = (110, 210, 120)
BAD_C = (70, 70, 235)
SEL_C = (200, 130, 235)
TRACK_BG = (52, 52, 58)
TRACK_IN = (140, 104, 78)


def c2d(c: float) -> float:
    return (c - CENTER_COUNT) / COUNTS_PER_DEG


def x_of(count: float) -> int:
    return int(TRK_X0 + (count / COUNT_MAX) * (TRK_X1 - TRK_X0))


def count_of(x: float) -> int:
    v = (x - TRK_X0) / (TRK_X1 - TRK_X0) * COUNT_MAX
    return int(round(min(COUNT_MAX, max(0, v))))


def text(img, s, xy, scale=0.44, col=FG, thick=1):
    cv2.putText(img, s, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, col, thick, cv2.LINE_AA)


class Button:
    def __init__(self, x, y, w, h, label, action, col=None):
        self.rect = (x, y, w, h)
        self.label = label
        self.action = action
        self.col = col

    def hit(self, x, y) -> bool:
        rx, ry, rw, rh = self.rect
        return rx <= x <= rx + rw and ry <= y <= ry + rh

    def draw(self, img, scale=0.4):
        x, y, w, h = self.rect
        cv2.rectangle(img, (x, y), (x + w, y + h), self.col or (62, 62, 70), -1)
        cv2.rectangle(img, (x, y), (x + w, y + h), (96, 96, 104), 1)
        (tw, th), _ = cv2.getTextSize(self.label, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
        text(img, self.label, (x + (w - tw) // 2, y + (h + th) // 2), scale)


class CalibUI:
    def __init__(self, port: str):
        self.bus = FeetechBus(port)
        missing = [n for n, sid in SERVO_IDS.items() if not self.bus.ping(sid)]
        if missing:
            raise RuntimeError(f"응답 없는 서보: {missing}")
        for sid in SERVO_IDS.values():           # 안전: 토크 OFF 보장
            self.bus.write_u8(sid, STS_ADDR["torque_enable"], 0)
        print(f"[ui] {port} 연결, 전 관절 토크 OFF")

        self.ranges, self.mapping = self._load()
        self.saved = (copy.deepcopy(self.ranges), copy.deepcopy(self.mapping))

        self.live = {n: CENTER_COUNT for n in SERVO_IDS}
        self.rows = list(SERVO_IDS.keys())
        self.sel = JOINT_ORDER[0]
        self.drag = None                          # (joint, 'min'|'max')
        self.msg = "기준자세를 잡고 z 를 누르면 영점이 잡힙니다"
        self.msg_col = FG
        self._last_poll = 0.0
        self.buttons: list[Button] = []

    # --- 상태 ------------------------------------------------------------
    def _load(self):
        cal = ArmCalibration.load(ARM_CALIB_PATH)
        if cal is not None:
            print(f"[ui] {ARM_CALIB_PATH} 로드")
            m = {n: dict(cal.mapping[n]) for n in JOINT_ORDER if n in cal.mapping}
            r = {n: dict(cal.ranges[n]) for n in cal.ranges}
        else:
            try:
                r = {n: dict(v) for n, v in load_lerobot_calibration(LEROBOT_CALIB).items()}
                print(f"[ui] lerobot 리밋 로드")
            except Exception:
                r = {}
                print("[ui] 기존 리밋 없음 - 전 범위로 시작")
            m = {}
        for n, sid in SERVO_IDS.items():
            r.setdefault(n, {"id": sid, "drive_mode": 0, "homing_offset": 0,
                             "range_min": 0, "range_max": COUNT_MAX})
        for n in JOINT_ORDER:
            m.setdefault(n, {"sign": 1.0, "offset_deg": 0.0})
        return r, m

    def poll(self):
        now = time.time()
        if now - self._last_poll < 1.0 / POLL_HZ:
            return
        self._last_poll = now
        for n, sid in SERVO_IDS.items():
            v = self.bus.read_u16(sid, STS_ADDR["present_position"])
            if v is not None:
                self.live[n] = v

    def ik_deg(self, n: str) -> float:
        m = self.mapping[n]
        return (c2d(self.live[n]) - m["offset_deg"]) / m["sign"]

    def tcp(self) -> np.ndarray:
        return so101_fk({n: math.radians(self.ik_deg(n)) for n in JOINT_ORDER})

    # --- 조작 ------------------------------------------------------------
    def capture_zero(self):
        for n in JOINT_ORDER:
            self.mapping[n]["offset_deg"] = c2d(self.live[n])
        self.msg = "영점 기록됨. 위의 TCP 가 (313, 0, 56)mm 에 가까운지 확인하세요"
        self.msg_col = OK_C

    def save(self):
        bad = [n for n in SERVO_IDS
               if self.ranges[n]["range_max"] - self.ranges[n]["range_min"] < MIN_GAP]
        if bad:
            self.msg, self.msg_col = f"범위가 너무 좁습니다: {', '.join(bad)}", BAD_C
            return
        for n, sid in SERVO_IDS.items():          # homing_offset 을 서보에서 갱신
            v = self.bus.read_u16(sid, STS_ADDR["homing_offset"])
            if v is not None:
                self.ranges[n]["homing_offset"] = -(v & 0x7FF) if (v & 0x800) else v
            self.ranges[n]["id"] = sid
        ArmCalibration(self.mapping, self.ranges).save(ARM_CALIB_PATH)
        self.saved = (copy.deepcopy(self.ranges), copy.deepcopy(self.mapping))
        self.msg, self.msg_col = f"{ARM_CALIB_PATH} 저장됨", OK_C

    def revert(self):
        self.ranges, self.mapping = copy.deepcopy(self.saved[0]), copy.deepcopy(self.saved[1])
        self.msg, self.msg_col = "마지막 저장 상태로 되돌림", FG

    def dirty(self) -> bool:
        return (self.ranges, self.mapping) != self.saved

    def set_limit(self, n: str, which: str):
        cur = self.live[n]
        r = self.ranges[n]
        if which == "min":
            r["range_min"] = min(cur, r["range_max"] - MIN_GAP)
        else:
            r["range_max"] = max(cur, r["range_min"] + MIN_GAP)
        self.msg, self.msg_col = f"{n} {which} = {cur}", FG

    def toggle_sign(self, n: str):
        self.mapping[n]["sign"] *= -1
        self.msg, self.msg_col = f"{n} 부호 {self.mapping[n]['sign']:+.0f}", FG

    def nudge(self, d: float):
        self.mapping[self.sel]["offset_deg"] += d
        self.msg = f"{self.sel} 오프셋 {self.mapping[self.sel]['offset_deg']:+.2f}deg"
        self.msg_col = FG

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            for b in self.buttons:
                if b.hit(x, y):
                    b.action()
                    return
            for i, n in enumerate(self.rows):
                ry = ROW_Y0 + i * ROW_H
                if not (ry <= y <= ry + ROW_H):
                    continue
                if x < TRK_X0:
                    if n in JOINT_ORDER:
                        self.sel = n
                    return
                r = self.ranges[n]
                for key in ("min", "max"):
                    if abs(x - x_of(r[f"range_{key}"])) <= HANDLE_GRAB:
                        self.drag = (n, key)
                        return
        elif event == cv2.EVENT_MOUSEMOVE and self.drag:
            n, key = self.drag
            r = self.ranges[n]
            v = count_of(x)
            if key == "min":
                r["range_min"] = min(v, r["range_max"] - MIN_GAP)
            else:
                r["range_max"] = max(v, r["range_min"] + MIN_GAP)
        elif event == cv2.EVENT_LBUTTONUP:
            self.drag = None

    # --- 그리기 ----------------------------------------------------------
    def draw(self) -> np.ndarray:
        img = np.full((H, W, 3), BG, np.uint8)
        self.buttons = []

        p = self.tcp()
        err = float(np.linalg.norm(p - ZERO_TCP))
        near_zero = all(abs(self.ik_deg(n)) < 12 for n in JOINT_ORDER)

        text(img, "SO-101 캘리브레이션", (14, 30), 0.68, FG, 2)
        text(img, f"TCP (FK):  x={p[0]*1000:+7.1f}   y={p[1]*1000:+7.1f}   z={p[2]*1000:+7.1f} mm",
             (14, 58), 0.52, FG)
        if near_zero:
            col = OK_C if err < 0.02 else BAD_C
            text(img, f"기준자세 목표 (313.3, 0.0, 56.3)mm  ->  오차 {err*1000:5.1f}mm",
                 (470, 58), 0.46, col)
        # 현재 위치가 리밋 밖이면 토크를 켜는 순간 그 관절이 범위 안으로 끌려간다.
        outside = [n for n in SERVO_IDS
                   if not (self.ranges[n]["range_min"] <= self.live[n]
                           <= self.ranges[n]["range_max"])]
        if outside:
            text(img, f"주의: 현재 위치가 리밋 밖 - {', '.join(outside)}"
                      "  (토크 ON 시 끌려감)", (14, 78), 0.44, BAD_C)

        text(img, f"{'ID':<3} {'관절':<14}", (14, 92), 0.42, DIM)
        text(img, "min / 현재 / max  [counts]", (TRK_X0, 92), 0.42, DIM)
        text(img, "IK각    부호", (800, 92), 0.42, DIM)

        for i, n in enumerate(self.rows):
            y = ROW_Y0 + i * ROW_H
            cy = y + ROW_H // 2
            r = self.ranges[n]
            cur = self.live[n]
            is_ik = n in JOINT_ORDER

            if is_ik and n == self.sel:
                cv2.rectangle(img, (6, y + 3), (W - 6, y + ROW_H - 3), (44, 38, 52), -1)
            text(img, f"{r['id']}", (14, cy + 5), 0.44, DIM)
            text(img, n, (36, cy + 5), 0.46, FG if is_ik else DIM)

            cv2.line(img, (TRK_X0, cy), (TRK_X1, cy), TRACK_BG, 9)
            xa, xb = x_of(r["range_min"]), x_of(r["range_max"])
            cv2.line(img, (xa, cy), (xb, cy), TRACK_IN, 9)
            for xx in (xa, xb):
                cv2.rectangle(img, (xx - 3, cy - 11), (xx + 3, cy + 11), FG, -1)

            xc = x_of(cur)
            inside = r["range_min"] <= cur <= r["range_max"]
            cv2.line(img, (xc, cy - 15), (xc, cy + 15), ACC if inside else BAD_C, 2)

            bm = Button(632, cy - 12, 48, 24, "min<", lambda n=n: self.set_limit(n, "min"))
            bx = Button(684, cy - 12, 48, 24, ">max", lambda n=n: self.set_limit(n, "max"))
            self.buttons += [bm, bx]
            bm.draw(img, 0.36)
            bx.draw(img, 0.36)

            if is_ik:
                sg = self.mapping[n]["sign"]
                bs = Button(744, cy - 12, 34, 24, "+" if sg > 0 else "-",
                            lambda n=n: self.toggle_sign(n),
                            (46, 78, 52) if sg > 0 else (52, 46, 78))
                self.buttons.append(bs)
                bs.draw(img, 0.5)
                text(img, f"{self.ik_deg(n):+7.1f}", (790, cy + 5), 0.44, FG)
                text(img, f"off {self.mapping[n]['offset_deg']:+6.1f}", (858, cy + 5), 0.38, DIM)
            else:
                text(img, f"{c2d(cur):+7.1f}deg", (790, cy + 5), 0.42, DIM)

            text(img, f"{r['range_min']:>4} | {cur:>4} | {r['range_max']:>4}",
                 (TRK_X0 + 2, y + 15), 0.36, DIM)

        fy = ROW_Y0 + 6 * ROW_H + 8
        for x, w, lbl, act, col in [
            (14, 132, "z  영점 잡기", self.capture_zero, (96, 76, 58)),
            (154, 108, "s  저장", self.save, (48, 84, 56)),
            (270, 118, "r  되돌리기", self.revert, (48, 62, 72)),
        ]:
            b = Button(x, fy, w, 30, lbl, act, col)
            self.buttons.append(b)
            b.draw(img, 0.42)

        text(img, ("* 저장되지 않은 변경 있음" if self.dirty() else "* 저장됨"),
             (404, fy + 20), 0.42, ACC if self.dirty() else DIM)
        text(img, self.msg, (14, fy + 52), 0.44, self.msg_col)
        text(img, "선택 관절 오프셋: [ 는 -0.5도, ] 는 +0.5도    q 종료",
             (470, fy + 52), 0.40, DIM)
        return img

    # --- 루프 ------------------------------------------------------------
    def run(self):
        win = "SO-101 calibration"
        cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(win, self.on_mouse)
        print(__doc__)
        try:
            while True:
                self.poll()
                cv2.imshow(win, self.draw())
                k = cv2.waitKey(20) & 0xFF
                if k in (ord("q"), 27):
                    if self.dirty():
                        self.msg = "저장되지 않은 변경이 있습니다. s 로 저장하거나 다시 q"
                        self.msg_col = ACC
                        self.saved = (copy.deepcopy(self.ranges), copy.deepcopy(self.mapping))
                        continue
                    break
                elif k == ord("z"):
                    self.capture_zero()
                elif k == ord("s"):
                    self.save()
                elif k == ord("r"):
                    self.revert()
                elif k == ord("["):
                    self.nudge(-0.5)
                elif k == ord("]"):
                    self.nudge(+0.5)
        finally:
            cv2.destroyAllWindows()
            for sid in SERVO_IDS.values():
                try:
                    self.bus.write_u8(sid, STS_ADDR["torque_enable"], 0)
                except Exception:
                    pass
            self.bus.close()
            print("[ui] 토크 OFF, 종료")


def main() -> None:
    port = sys.argv[1] if len(sys.argv) > 1 else (find_so101_port() or "COM18")
    CalibUI(port).run()


if __name__ == "__main__":
    main()
