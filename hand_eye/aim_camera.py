"""
aim_camera.py - 카메라를 로봇 작업공간에 맞추도록 도와주는 실시간 뷰어.

왜 필요한가:
  hand-eye 캘리브레이션이 두 번 실패했는데 원인이 알고리즘이 아니라 설치였다.
  실측 결과 팔은 화면 왼쪽 가장자리 23%(u=3~146 / 640) 안에만 들어오고,
  거리가 53~56cm 라 Astra 최소측정거리(약 60cm) 미만이어서 팔 자리의 depth 가
  통째로 무효였다. 카메라를 제대로 겨누고 물리면 두 문제가 같이 풀린다.

목표:
  - 로봇 그리퍼가 화면 중앙 근처에 오도록
  - 작업면까지 거리가 70~85cm 가 되도록 (여유 있게 최소거리 위)
  - 중앙 영역의 depth 유효율이 높도록

화면:
  왼쪽 IR, 오른쪽 depth. 가운데 사각형이 목표 영역이고, 그 안의 통계와
  판정을 위에 띄운다. 전부 초록이 되면 캘리브레이션을 다시 돌리면 된다.

키:
  q 종료      s 현재 프레임 저장

사용:
  python aim_camera.py
"""

from __future__ import annotations

import sys

import cv2
import numpy as np

from test4 import colorize_depth, open_depth_source

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass


GOOD_MIN, GOOD_MAX = 0.68, 0.88     # 작업면까지 목표 거리 [m]
ROI_FRAC = 0.45                     # 중앙 관심영역 크기 (화면 대비)
MIN_VALID = 0.80                    # 관심영역 depth 유효율 목표

OK_C, BAD_C, WARN_C = (90, 220, 90), (60, 60, 235), (60, 200, 240)


def verdict(median: float, valid: float) -> tuple[str, tuple[int, int, int], str]:
    if valid < MIN_VALID:
        return ("depth 구멍이 많음", BAD_C,
                "카메라를 더 멀리 물리거나 각도를 낮추세요")
    if median < GOOD_MIN:
        return (f"너무 가까움 ({median*100:.0f}cm)", BAD_C,
                f"{(GOOD_MIN-median)*100:.0f}cm 이상 뒤로/위로 옮기세요")
    if median > GOOD_MAX:
        return (f"너무 멂 ({median*100:.0f}cm)", WARN_C,
                f"{(median-GOOD_MAX)*100:.0f}cm 정도 가까이 오세요")
    return (f"거리 양호 ({median*100:.0f}cm)", OK_C,
            "그리퍼가 중앙 사각형 안에 오도록 겨누세요")


def main() -> None:
    print(__doc__)
    src = open_depth_source()
    h, w = src.intr.height, src.intr.width
    rw, rh = int(w * ROI_FRAC), int(h * ROI_FRAC)
    x0, y0 = (w - rw) // 2, (h - rh) // 2

    win = "aim camera - q:quit  s:save"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    try:
        while True:
            ir, depth = src.read()
            roi = depth[y0:y0 + rh, x0:x0 + rw]
            v = roi[roi > 0]
            valid = v.size / roi.size
            med = float(np.median(v)) if v.size else 0.0
            msg, col, hint = verdict(med, valid)

            dvis = colorize_depth(depth)[0]
            left = ir.copy()
            # 유효하지 않은 depth 를 IR 위에 붉게 덮어 어디가 비었는지 보여준다
            left[depth <= 0] = (0, 0, 90)

            for img in (left, dvis):
                cv2.rectangle(img, (x0, y0), (x0 + rw, y0 + rh), col, 2)
                cv2.drawMarker(img, (w // 2, h // 2), col, cv2.MARKER_CROSS, 22, 1)

            out = np.hstack([left, dvis])
            cv2.rectangle(out, (0, 0), (out.shape[1], 62), (32, 32, 32), -1)
            cv2.putText(out, msg, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2, cv2.LINE_AA)
            cv2.putText(out, f"ROI 유효율 {valid*100:.0f}% (목표 {MIN_VALID*100:.0f}%+)   "
                             f"목표거리 {GOOD_MIN*100:.0f}-{GOOD_MAX*100:.0f}cm",
                        (10, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1, cv2.LINE_AA)
            cv2.putText(out, hint, (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (180, 180, 180), 1, cv2.LINE_AA)
            cv2.putText(out, "IR (붉은색 = depth 없음)", (10, h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(out, "depth", (w + 10, h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

            cv2.imshow(win, out)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord('q'), 27):
                break
            if k == ord('s'):
                cv2.imwrite("aim_snapshot.png", out)
                print("저장: aim_snapshot.png")
    finally:
        cv2.destroyAllWindows()
        src.close()


if __name__ == "__main__":
    main()
