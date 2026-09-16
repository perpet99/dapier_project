"""
handeye_pnp.py - depth 없이 2D-3D 대응으로 hand-eye 를 푼다 (Astra Pro 용).

왜 PnP 인가:
  이 셋업에서 로봇팔은 depth 에 잡히지 않는다. 팔 표면이 IR 에서 포화되고
  거리가 최소측정거리(~60cm) 부근이라, 팔이 있는 자리는 깊이값이 아니라
  '구멍'(무효)으로 나온다. 그래서 3D-3D(Kabsch)로는 풀 수 없다.

  그러나 팔은 IR 영상에는 또렷하게 보인다. 즉 그리퍼의 '2D 화소'는 얻을 수 있고
  '로봇 3D 좌표'는 FK 로 정확히 안다. 이 2D-3D 대응이면 PnP 로 카메라 자세를
  풀 수 있다. 그리퍼의 depth 는 한 번도 필요하지 않다.

  픽앤플레이스에는 지장이 없다. 집을 물체는 테이블 위에 있고 그 depth 는 유효하다
  (실측 62%, 50~77cm). 필요한 건 '물체 깊이'이지 '팔 깊이'가 아니다.

그리퍼를 어떻게 찾는가 (2단계):
  1) 팔을 치운 배경 depth 와 비교해 '새로 생긴 구멍'을 찾는다. 테이블은 depth 가
     유효하므로 새 구멍 = 팔의 실루엣이다. 전역 IR 노이즈에 흔들리지 않는다.
  2) 그 실루엣 안에서만 그리퍼를 여닫아 IR 차분을 본다. 탐색 범위를 팔로 좁혔기
     때문에, 앞서 전체 화면에서 실패했던 노이즈 문제가 사라진다.

사용:
  python handeye_pnp.py [COM18]
"""

from __future__ import annotations

import math
import sys

import cv2
import numpy as np

from test4 import (HANDEYE_PATH, SERVO_IDS, TOPDOWN_PITCH, ArmCalibration,
                   FeetechSO101Arm, HandEye, IKError, open_depth_source, plan_pose)

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass


PARK_POSE = (0.13, -0.02, 0.16)   # 배경 촬영용: 팔을 최대한 접어 치운 자세
FRAMES = 9                        # 중앙값 누적 프레임 수
FLUTTER_SETTLE = 1.2
MIN_SILHOUETTE_PX = 150           # 팔 실루엣으로 인정할 최소 면적
DILATE = 25                       # 실루엣을 넓혀 그리퍼 주변까지 포함


def calib_poses() -> list[tuple[float, float, float]]:
    out = []
    for r in (0.15, 0.19, 0.23):
        for ang in (-22.0, 0.0, 22.0):
            for z in (0.04, 0.10):
                a = math.radians(ang)
                out.append((r * math.cos(a), r * math.sin(a), z))
    keep = []
    for p in out:
        try:
            plan_pose(*p)
            keep.append(p)
        except IKError:
            pass
    return keep


def grab(src, k: int = FRAMES):
    irs, ds = [], []
    for _ in range(k):
        c, d = src.read()
        irs.append(c[:, :, 0].astype(np.float32))
        ds.append(d)
    return np.median(irs, 0), np.median(ds, 0)


def arm_silhouette(bg_depth: np.ndarray, now_depth: np.ndarray) -> np.ndarray | None:
    """배경에선 유효했는데 지금은 무효인 영역 = 팔이 가린 자리."""
    hole = (bg_depth > 0) & (now_depth <= 0)
    m = cv2.morphologyEx(hole.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, 8)
    if n <= 1:
        return None
    k = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    if st[k, cv2.CC_STAT_AREA] < MIN_SILHOUETTE_PX:
        return None
    return lab == k


def find_gripper(src, arm, sil: np.ndarray) -> tuple[int, int] | None:
    """실루엣 안에서 그리퍼만 여닫아 위치를 특정한다."""
    region = cv2.dilate(sil.astype(np.uint8), np.ones((DILATE, DILATE), np.uint8)).astype(bool)

    arm.set_gripper(0.0, settle=FLUTTER_SETTLE)
    ir0, _ = grab(src)
    arm.set_gripper(1.0, settle=FLUTTER_SETTLE)
    ir1, _ = grab(src)
    arm.set_gripper(0.0, settle=FLUTTER_SETTLE)

    diff = cv2.GaussianBlur(np.abs(ir0 - ir1), (7, 7), 0)
    diff[~region] = 0                      # 팔 밖은 아예 보지 않는다
    if diff.max() < 5:
        return None

    thr = max(8.0, float(np.percentile(diff[region], 97)))
    m = cv2.morphologyEx((diff > thr).astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, st, ct = cv2.connectedComponentsWithStats(m, 8)
    if n <= 1:
        return None
    k = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    if st[k, cv2.CC_STAT_AREA] < 12:
        return None
    return int(round(ct[k][0])), int(round(ct[k][1]))


def main() -> None:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM18"
    print(__doc__)

    cal = ArmCalibration.load()
    if cal is None:
        print("arm_calib.json 이 없다.")
        return

    poses = calib_poses()
    print(f"[plan] 자세 {len(poses)}개")

    src = open_depth_source()
    arm = FeetechSO101Arm(port, cal, speed=400, accel=18)
    arm.connect()

    obj_pts, img_pts, rec = [], [], []
    try:
        print(f"\n[bg] 팔을 치우고 배경 depth 촬영 {PARK_POSE}")
        arm.move_to(*PARK_POSE, settle=1.5)
        _, bg = grab(src, FRAMES * 2)
        print(f"     배경 유효화소 {100*(bg>0).mean():.0f}%")

        for i, p in enumerate(poses, 1):
            try:
                arm.move_to(*p, pitch=TOPDOWN_PITCH, roll=0.0, settle=1.2)
            except (IKError, RuntimeError) as e:
                print(f"[{i}/{len(poses)}] 이동 실패: {e}")
                continue

            tcp = arm.get_tcp()
            _, now = grab(src)
            sil = arm_silhouette(bg, now)
            if sil is None:
                print(f"[{i}/{len(poses)}] TCP({tcp[0]*100:+.1f},{tcp[1]*100:+.1f},{tcp[2]*100:+.1f}) "
                      f"실루엣 검출 실패")
                continue

            g = find_gripper(src, arm, sil)
            if g is None:
                print(f"[{i}/{len(poses)}] 실루엣 {int(sil.sum())}px 이나 그리퍼 특정 실패")
                continue

            obj_pts.append(np.asarray(tcp, np.float64))
            img_pts.append(np.array(g, np.float64))
            rec.append((tcp, g, int(sil.sum())))
            print(f"[{i}/{len(poses)}] TCP({tcp[0]*100:+.1f},{tcp[1]*100:+.1f},{tcp[2]*100:+.1f})cm "
                  f"-> 화소{g}  실루엣{int(sil.sum())}px")

        print("\n" + "=" * 60)
        print(f"수집 {len(obj_pts)} / {len(poses)}")
        if len(obj_pts) < 6:
            print("PnP 를 안정적으로 풀기에 대응점이 부족하다.")
            return

        K = np.array([[src.intr.fx, 0, src.intr.cx],
                      [0, src.intr.fy, src.intr.cy],
                      [0, 0, 1]], np.float64)
        O = np.array(obj_pts, np.float64)
        I = np.array(img_pts, np.float64)

        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            O, I, K, None, flags=cv2.SOLVEPNP_ITERATIVE,
            reprojectionError=8.0, confidence=0.999, iterationsCount=5000)
        if not ok:
            print("PnP 실패")
            return
        inl = inliers.ravel() if inliers is not None else np.arange(len(O))
        print(f"RANSAC inlier {len(inl)}/{len(O)}")
        if len(inl) >= 6:
            _, rvec, tvec = cv2.solvePnP(O[inl], I[inl], K, None, rvec, tvec,
                                         useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)

        proj, _ = cv2.projectPoints(O[inl], rvec, tvec, K, None)
        err = np.linalg.norm(proj.reshape(-1, 2) - I[inl], axis=1)
        print(f"재투영 오차: 평균 {err.mean():.2f}px  최대 {err.max():.2f}px")

        R, _ = cv2.Rodrigues(rvec)
        t = tvec.ravel()
        depth_med = float(np.median((R @ O[inl].T).T[:, 2] + t[2]))
        mm_per_px = depth_med / src.intr.fx * 1000
        print(f"작업거리 {depth_med*100:.0f}cm 에서 1px = {mm_per_px:.2f}mm "
              f"-> 평균오차 약 {err.mean()*mm_per_px:.1f}mm")

        # p_cam = R p_base + t  =>  p_base = R^T (p_cam - t)
        T = np.eye(4)
        T[:3, :3] = R.T
        T[:3, 3] = -R.T @ t

        print()
        if err.mean() < 4.0:
            print("판정: 양호 - 저장한다")
            HandEye(T).save(HANDEYE_PATH)
            print(f"{HANDEYE_PATH} 저장 완료.")
        else:
            print("판정: 재투영 오차가 커서 저장하지 않는다.")
            print("  그리퍼 검출이 흔들렸거나 arm_calib.json 의 기구학이 부정확하다.")

    except KeyboardInterrupt:
        print("\n중단")
    finally:
        arm.disconnect()
        src.close()


if __name__ == "__main__":
    main()
