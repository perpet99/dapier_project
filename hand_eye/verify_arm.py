"""
verify_arm.py - arm_calib.json 을 실기에서 단계적으로 검증한다.

캘리브레이션이 끝난 직후, test4.py 로 픽앤플레이스를 돌리기 전에 이걸 먼저 돌린다.
부호가 뒤집혀 있으면 팔이 테이블로 내리꽂히므로, 큰 동작 전에 작은 동작으로
방향부터 확인하는 것이 목적이다.

단계:
  0  계산만. 하드웨어에 아무것도 쓰지 않는다. 현재 자세를 IK 좌표계로 해석하고
     HOME 까지의 이동량을 보여준다.
  1  속도/가속 제한을 걸고 토크를 켠 뒤, '현재 위치'를 그대로 명령한다.
     이동량이 0이므로 팔은 움직이지 않아야 한다. 쓰기 경로 검증.
  2  관절 하나씩 IK 기준 +6도만 움직이고 방향을 눈으로 확인한다.
     확인 후 원위치로 되돌린다.
  3  전 단계 통과 시에만 HOME 자세로 분할 이동한다.

어느 단계에서 중단하든(Ctrl+C 포함) 토크를 끄고 종료한다.

사용:
  python verify_arm.py COM18
"""

from __future__ import annotations

import math
import sys
import time

from test4 import (COUNTS_PER_DEG, CENTER_COUNT, HOME_POSE, JOINT_ORDER, SERVO_IDS,
                   STS_ADDR, ArmCalibration, FeetechBus, FeetechSO101Arm,
                   so101_fk, so101_ik)

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass


# 관절을 IK 기준 +방향으로 움직였을 때 눈에 보여야 하는 현상
EXPECT = {
    "shoulder_pan":  "베이스가 위에서 봤을 때 반시계(로봇 기준 왼쪽)로 회전",
    "shoulder_lift": "위팔이 위로 올라감",
    "elbow_flex":    "아래팔이 위로 펴짐",
    "wrist_flex":    "손목이 위로 젖혀짐",
    "wrist_roll":    "그리퍼가 반시계로 회전",
}

TEST_DELTA_DEG = 6.0


def ask(prompt: str) -> bool:
    try:
        return input(prompt).strip().lower() in ("y", "yes")
    except EOFError:
        return False


def current_ik(bus: FeetechBus, cal: ArmCalibration) -> dict[str, float]:
    """서보 위치 -> 내 IK 좌표계 각도[rad]"""
    out = {}
    for n in T_ORDER:
        raw = bus.read_u16(SERVO_IDS[n], STS_ADDR["present_position"])
        sdeg = (raw - CENTER_COUNT) / COUNTS_PER_DEG
        m = cal.mapping[n]
        out[n] = math.radians((sdeg - m["offset_deg"]) / m["sign"])
    return out


T_ORDER = JOINT_ORDER


def stage0(bus: FeetechBus, cal: ArmCalibration) -> None:
    print("\n" + "=" * 62)
    print("단계 0: 계산만 (하드웨어에 쓰지 않음)")
    print("=" * 62)

    q = current_ik(bus, cal)
    for n in T_ORDER:
        raw = bus.read_u16(SERVO_IDS[n], STS_ADDR["present_position"])
        print(f"  {n:<15} raw={raw:<5} IK={math.degrees(q[n]):+8.2f}deg")

    p = so101_fk(q)
    print(f"\n  현재 TCP(FK): x={p[0]*100:+.1f} y={p[1]*100:+.1f} z={p[2]*100:+.1f} cm "
          f"(r={math.hypot(p[0],p[1])*100:.1f}cm)")
    if p[2] < -0.05:
        print("  경고: TCP 높이가 베이스보다 5cm 이상 아래다. 캘리브레이션이 의심스럽다.")

    qh = so101_ik(*HOME_POSE)
    print("\n  HOME 까지 이동량:")
    for n in T_ORDER:
        cur = bus.read_u16(SERVO_IDS[n], STS_ADDR["present_position"])
        tgt = cal.ik_to_count(n, qh[n])
        print(f"    {n:<15} {cur:>5} -> {tgt:>5}  ({(tgt-cur)/COUNTS_PER_DEG:+7.1f}deg)")


def stage1(arm: FeetechSO101Arm) -> bool:
    print("\n" + "=" * 62)
    print("단계 1: 토크 ON + 현재 위치 유지 (움직이지 않아야 정상)")
    print("=" * 62)
    print("  팔을 손으로 받치고 있다가, 토크가 켜지면 손을 떼세요.")
    if not ask("  진행할까요? (y/N) > "):
        return False

    before = arm.read_counts()
    arm.bus.sync_write_positions({SERVO_IDS[n]: before[n] for n in T_ORDER})
    time.sleep(1.0)
    after = arm.read_counts()

    print("\n  관절별 변화:")
    worst = 0.0
    for n in T_ORDER:
        d = (after[n] - before[n]) / COUNTS_PER_DEG
        worst = max(worst, abs(d))
        print(f"    {n:<15} {before[n]:>5} -> {after[n]:>5}  ({d:+5.2f}deg)")

    print(f"\n  최대 변화 {worst:.2f}deg", end="  ")
    if worst < 3.0:
        print("-> 정상 (제자리 유지)")
        return True
    print("-> 비정상. 팔이 스스로 움직였다. 중단한다.")
    return False


def stage2(arm: FeetechSO101Arm, cal: ArmCalibration) -> bool:
    print("\n" + "=" * 62)
    print(f"단계 2: 관절별 +{TEST_DELTA_DEG:.0f}도 방향 확인")
    print("=" * 62)
    print("  각 관절을 조금씩만 움직입니다. 방향이 설명과 다르면 n 을 누르세요.")
    if not ask("  진행할까요? (y/N) > "):
        return False

    for n in T_ORDER:
        q = current_ik(arm.bus, cal)
        start = arm.read_counts()

        tgt = dict(q)
        tgt[n] = q[n] + math.radians(TEST_DELTA_DEG)
        cnt = cal.ik_to_count(n, tgt[n])
        if cnt == start[n]:
            print(f"\n  [{n}] 리밋에 걸려 움직일 수 없음 - 건너뜀")
            continue

        print(f"\n  [{n}] +{TEST_DELTA_DEG:.0f}도 -> 기대: {EXPECT[n]}")
        input("        Enter 를 누르면 움직입니다 > ")
        arm.bus.sync_write_positions({SERVO_IDS[n]: cnt})
        time.sleep(1.0)

        moved = (arm.read_counts()[n] - start[n]) / COUNTS_PER_DEG
        print(f"        실제 서보 변화: {moved:+.2f}deg")

        ok = ask(f"        '{EXPECT[n]}' 대로 움직였습니까? (y/N) > ")
        arm.bus.sync_write_positions({SERVO_IDS[n]: start[n]})   # 원위치
        time.sleep(1.0)

        if not ok:
            print(f"\n  {n} 의 방향이 반대입니다.")
            print(f"  arm_calib.json 에서 mapping.{n}.sign 을 "
                  f"{cal.mapping[n]['sign']:+.0f} -> {-cal.mapping[n]['sign']:+.0f} 로 뒤집고")
            print("  다시 이 스크립트를 실행하세요. 중단합니다.")
            return False
        print("        OK, 원위치 복귀")
    return True


def stage3(arm: FeetechSO101Arm) -> bool:
    print("\n" + "=" * 62)
    print(f"단계 3: HOME {HOME_POSE} 로 이동")
    print("=" * 62)
    print("  분할 이동으로 천천히 갑니다. 전원 차단 준비를 하세요.")
    if not ask("  진행할까요? (y/N) > "):
        return False

    arm.home()
    counts = arm.read_counts()
    q = {n: math.radians((( counts[n] - CENTER_COUNT) / COUNTS_PER_DEG
                          - arm.calib.mapping[n]["offset_deg"])
                         / arm.calib.mapping[n]["sign"]) for n in T_ORDER}
    p = so101_fk(q)
    err = math.dist(p, HOME_POSE)
    print(f"\n  도달 TCP: x={p[0]*100:+.1f} y={p[1]*100:+.1f} z={p[2]*100:+.1f} cm")
    print(f"  목표와의 오차: {err*1000:.1f}mm", end="  ")
    print("-> 양호" if err < 0.02 else "-> 오차가 큼. 링크길이/기준자세를 재확인할 것")
    return True


def main() -> None:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM18"
    print(__doc__)

    cal = ArmCalibration.load()
    if cal is None:
        print("arm_calib.json 이 없다. 먼저 calibrate_arm.py 를 실행할 것.")
        return

    bus = FeetechBus(port)
    try:
        stage0(bus, cal)
    finally:
        bus.close()

    if not ask("\n단계 1 로 넘어갈까요? (토크가 켜집니다) (y/N) > "):
        print("여기서 중단. 하드웨어는 그대로입니다.")
        return

    arm = FeetechSO101Arm(port, cal)
    arm.connect()
    try:
        if not stage1(arm):
            return
        if not stage2(arm, cal):
            return
        stage3(arm)
        print("\n전 단계 통과. 이제 test4.py 로 픽앤플레이스를 진행해도 됩니다.")
    except KeyboardInterrupt:
        print("\n사용자 중단")
    finally:
        arm.disconnect()      # 토크 OFF 보장


if __name__ == "__main__":
    main()
