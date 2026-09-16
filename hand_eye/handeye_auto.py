"""
handeye_auto.py - 카메라<->로봇 hand-eye 변환을 자동으로 구한다.

원리:
  로봇을 여러 자세로 보내고, 각 자세에서 그리퍼의 카메라 3D 좌표를 측정한 뒤,
  그 좌표와 FK 로 계산한 로봇 좌표를 짝지어 Kabsch 로 강체변환을 푼다.

그리퍼를 어떻게 찾는가:
  팔은 가만히 둔 채 그리퍼만 여닫고 두 프레임을 뺀다. 그리퍼만 움직였으므로
  변한 화소가 곧 그리퍼다. 마커를 붙이거나 사람이 클릭할 필요가 없다.
  IR 은 depth 와 같은 센서라 픽셀이 1:1 대응하므로 IR 로 찾은 위치의 depth 를
  그대로 쓸 수 있다.

왜 이게 부호 검증도 겸하는가:
  강체변환은 직선을 직선으로, 등간격을 등간격으로 보낸다. 캘리브레이션의 부호나
  오프셋이 틀렸으면 로봇 좌표와 카메라 좌표가 강체변환으로 이어지지 않고,
  잔차(RMS)가 크게 나온다. 즉 RMS 가 작다는 것 자체가 기구학이 맞다는 증거다.

안전:
  - 모든 자세는 plan_pose(안전박스 + IK)를 통과한 것만 사용
  - 속도/가속 제한, 큰 이동은 분할
  - 매 이동 후 서보 부하를 확인해 충돌이 의심되면 중단
  - 어떤 경로로 끝나든 토크를 끈다

사용:
  python handeye_auto.py [COM18]
"""

from __future__ import annotations

import math
import sys
import time

import cv2
import numpy as np

from test4 import (COUNTS_PER_DEG, HANDEYE_PATH, JOINT_ORDER, SERVO_IDS, STS_ADDR,
                   TOPDOWN_PITCH, ArmCalibration, FeetechSO101Arm, HandEye, IKError,
                   deproject, open_depth_source, plan_pose, sample_depth,
                   solve_rigid_transform)

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass


# 그리퍼를 찾을 때 쓰는 파라미터
FLUTTER_SETTLE = 1.5      # 그리퍼 여닫은 뒤 안정화 시간 [s]
FRAMES_PER_GRAB = 12      # 중앙값 누적 프레임 수 (노이즈 억제)
IR_PCTL = 99.5            # IR 변화량 상위 몇 % 를 그리퍼 후보로 볼지
MIN_BLOB_PX = 40          # 이보다 작은 덩어리는 노이즈로 간주
DEPTH_WIN = 21            # 그리퍼 중심 주변에서 depth 를 모을 창 크기
MIN_DEPTH_PX = 12         # 이보다 유효 depth 화소가 적으면 그 자세는 버린다

LOAD_LIMIT = 700          # 서보 부하 경보 임계 (STS3215 raw)


def calib_poses() -> list[tuple[float, float, float]]:
    """안전 작업영역 안에 고르게 퍼진 자세들. 3D 로 잘 퍼져야 Kabsch 가 안정적이다."""
    out = []
    for r in (0.16, 0.20, 0.23):
        for ang in (-25.0, 0.0, 25.0):
            for z in (0.04, 0.09):
                a = math.radians(ang)
                out.append((r * math.cos(a), r * math.sin(a), z))
    return [p for p in out if _reachable(p)]


def _reachable(p) -> bool:
    try:
        plan_pose(*p)
        return True
    except IKError:
        return False


def grab_median(src, k: int = FRAMES_PER_GRAB):
    """여러 프레임의 중앙값. IR 과 depth 를 함께 돌려준다."""
    irs, ds = [], []
    for _ in range(k):
        c, d = src.read()
        irs.append(c[:, :, 0].astype(np.float32))
        ds.append(d)
    return np.median(irs, 0), np.median(ds, 0)


def locate_gripper(src, arm) -> tuple[tuple[int, int], np.ndarray] | tuple[None, str]:
    """그리퍼만 여닫아 그 위치를 찾는다. 성공하면 ((u,v), depth맵)."""
    arm.set_gripper(0.0, settle=FLUTTER_SETTLE)
    ir0, d0 = grab_median(src)
    arm.set_gripper(1.0, settle=FLUTTER_SETTLE)
    ir1, _d1 = grab_median(src)
    arm.set_gripper(0.0, settle=FLUTTER_SETTLE)

    diff = cv2.GaussianBlur(np.abs(ir0 - ir1), (9, 9), 0)
    mask = (diff > np.percentile(diff, IR_PCTL)).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

    n, lab, stats, cent = cv2.connectedComponentsWithStats(mask, 8)
    if n <= 1:
        return None, "IR 변화 없음"
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[k, cv2.CC_STAT_AREA] < MIN_BLOB_PX:
        return None, f"덩어리가 너무 작음({stats[k, cv2.CC_STAT_AREA]}px)"
    u, v = int(round(cent[k][0])), int(round(cent[k][1]))
    return (u, v), d0


def check_load(arm) -> tuple[bool, str]:
    worst, who = 0, ""
    for n, sid in SERVO_IDS.items():
        raw = arm.bus.read_u16(sid, 60)
        mag = (raw & 0x3FF) if raw is not None else 0
        if mag > worst:
            worst, who = mag, n
    return (worst < LOAD_LIMIT), f"{who} 부하 {worst}"


def main() -> None:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM18"
    print(__doc__)

    cal = ArmCalibration.load()
    if cal is None:
        print("arm_calib.json 이 없다. calibrate_arm.py 를 먼저 실행할 것.")
        return

    poses = calib_poses()
    print(f"[plan] 도달 가능한 캘리브레이션 자세 {len(poses)}개")
    if len(poses) < 5:
        print("자세가 너무 적다. 작업영역 설정을 확인할 것.")
        return

    src = open_depth_source()
    arm = FeetechSO101Arm(port, cal, speed=400, accel=18)
    arm.connect()

    # 현재 위치에서 가까운 것부터 도는 순서로 재정렬한다.
    # 총 이동거리가 줄면 그만큼 부딪힐 기회도 줄고 시간도 짧아진다.
    here = arm.get_tcp()
    remaining, ordered = list(poses), []
    while remaining:
        k = min(range(len(remaining)), key=lambda j: math.dist(here, remaining[j]))
        here = remaining.pop(k)
        ordered.append(here)
    poses = ordered
    print(f"[plan] 현재 TCP 기준 최단순회로 재정렬 "
          f"(첫 자세까지 {math.dist(arm.get_tcp(), poses[0])*100:.1f}cm)")

    cam_pts, base_pts, used = [], [], []
    try:
        for i, p in enumerate(poses, 1):
            print(f"\n[{i}/{len(poses)}] 목표 TCP ({p[0]*100:+.1f}, {p[1]*100:+.1f}, {p[2]*100:+.1f}) cm")
            try:
                arm.move_to(*p, pitch=TOPDOWN_PITCH, roll=0.0, settle=1.2)
            except IKError as e:
                print(f"    건너뜀: {e}")
                continue

            ok, note = check_load(arm)
            if not ok:
                print(f"    부하 이상({note}) - 충돌 의심. 중단한다.")
                break

            tcp = arm.get_tcp()          # FK 실측 (명령값이 아니라 실제 도달값)
            err = math.dist(tcp, p)
            print(f"    실제 TCP ({tcp[0]*100:+.1f}, {tcp[1]*100:+.1f}, {tcp[2]*100:+.1f}) cm "
                  f"(목표와 {err*1000:.1f}mm 차이)")

            res, payload = locate_gripper(src, arm)
            if res is None:
                print(f"    그리퍼 검출 실패: {payload}")
                continue
            (u, v), depth = res, payload

            z = sample_depth(depth, u, v, patch=DEPTH_WIN)
            win = depth[max(0, v - DEPTH_WIN // 2):v + DEPTH_WIN // 2 + 1,
                        max(0, u - DEPTH_WIN // 2):u + DEPTH_WIN // 2 + 1]
            nvalid = int((win > 0).sum())
            print(f"    그리퍼 화소 ({u},{v})  유효depth {nvalid}/{win.size}  d={z*100:.1f}cm")

            if z <= 0 or nvalid < MIN_DEPTH_PX:
                print("    depth 부족 - 이 자세는 버린다")
                continue

            cam_pts.append(deproject(src.intr, u, v, z))
            base_pts.append(np.asarray(tcp, dtype=np.float64))
            used.append((u, v, z))

        # ---- 결과 ----
        print("\n" + "=" * 60)
        print(f"수집: {len(cam_pts)} / {len(poses)} 자세")
        if len(cam_pts) < 4:
            print("대응점이 4개 미만이라 hand-eye 를 풀 수 없다.")
            print("원인: 그리퍼 위치의 depth 가 유효하지 않음.")
            print("대책: 카메라를 작업대에서 더 멀리(70~80cm) 올려 설치할 것.")
            return

        A, B = np.array(cam_pts), np.array(base_pts)
        T, rms = solve_rigid_transform(A, B)
        print(f"전체 RMS 잔차: {rms*1000:.1f} mm")

        # leave-one-out: 한 점을 빼고 풀어서 그 점을 예측 -> 과적합 여부 확인
        if len(A) >= 5:
            errs = []
            for i in range(len(A)):
                m = np.arange(len(A)) != i
                Ti, _ = solve_rigid_transform(A[m], B[m])
                pred = Ti[:3, :3] @ A[i] + Ti[:3, 3]
                errs.append(np.linalg.norm(pred - B[i]))
            print(f"leave-one-out 예측오차: 평균 {np.mean(errs)*1000:.1f}mm, "
                  f"최대 {np.max(errs)*1000:.1f}mm")

        print()
        if rms < 0.010:
            verdict, save = "양호 - 픽앤플레이스에 쓸 수 있다", True
        elif rms < 0.020:
            verdict, save = "보통 - 작은 물체는 놓칠 수 있다", True
        else:
            verdict, save = "불량 - 저장하지 않는다", False
        print(f"판정: {verdict}")

        if save:
            HandEye(T).save(HANDEYE_PATH)
            print(f"\n{HANDEYE_PATH} 저장 완료. test4.py 에서 바로 쓴다.")
        else:
            print("\n원인 후보:")
            print("  - 카메라가 작업대에 너무 가까움(Astra 최소 측정거리 약 60cm)")
            print("  - arm_calib.json 의 부호/오프셋 오류 (기준자세가 부정확했을 때)")
            print("  - 링크 길이(L0~L3)가 실제 조립본과 다름")

    except KeyboardInterrupt:
        print("\n사용자 중단")
    finally:
        arm.disconnect()
        src.close()


if __name__ == "__main__":
    main()
