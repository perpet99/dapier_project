"""
calibrate_arm.py - SO-101 팔 캘리브레이션 (서보 영점 + 관절 리밋 + 기준자세).

test4.py 는 여기서 만든 arm_calib.json 을 읽기만 한다. 파일이 없으면 test4.py 는
팔을 mock 으로 돌리므로, 실기를 쓰려면 이걸 먼저 돌려야 한다.

무엇을 정하는가:
  0) 서보 영점 (Homing Offset, 서보 EEPROM)
       관절 가동범위의 가운데가 엔코더 중앙(2047)으로 읽히게 한다. 이게 없으면
       범위가 0/4095 경계에 걸쳐 리밋이 [0, 4095] 로 측정되고, 위치 명령이 경계
       반대편으로 돌아 스토퍼에 부딪힌다.
  1) 관절별 min/max 리밋 [서보 카운트]
       test4.py 의 ik_to_count() 가 이 값으로 목표를 하드 클램프한다.
       IK 가 이상한 값을 내도 팔이 이 범위를 절대 넘지 않는다. 안전의 마지막 방어선.
  2) 관절별 부호 + 오프셋
       서보가 보고하는 0도가 test4.py IK 의 0도(위팔 수직 / 아래팔 수평)와 같다는
       보장이 없다. 그 차이를 실측한다. 추측하면 팔이 꽂힌다.

안전:
  처음부터 끝까지 토크를 끈다. 팔은 손으로 잡고 진행하며, 토크가 꺼져 있으면
  중력으로 처지니 받쳐야 한다.

사용:
  python calibrate_arm.py [포트]          전체 캘리브레이션 (포트 생략 시 자동검출)
  python calibrate_arm.py [포트] --tcp    저장된 값으로 TCP 만 확인 (아무것도 안 바꿈)
"""

from __future__ import annotations

import math
import sys
import time
from typing import Optional

from test4 import (ARM_CALIB_PATH, CENTER_COUNT, COUNTS_PER_DEG, HOME_POSE, JOINT_ORDER,
                   LEROBOT_CALIB, SERVO_IDS, SHORT_NAME, STS_ADDR, ArmCalibration,
                   FeetechBus, find_so101_port, load_lerobot_calibration, so101_fk,
                   so101_ik)

try:
    import msvcrt          # Windows 콘솔에서 논블로킹 키 입력 (없으면 시간제한으로 대체)
except ImportError:
    msvcrt = None

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass


# ---------------------------------------------------------------------------
# 설정
# ---------------------------------------------------------------------------

# 서보 가동범위의 한가운데는 우리 영자세(위팔 수직 / 아래팔 수평)가 아니다.
# 공식 URDF(so101_new_calib)의 영자세 = 범위 중앙이고, 그 자세의 링크각은
# 76.03 / 2.21 / -2.84 도다. 범위 중앙으로 영점을 잡을 때는 그 차이를 빼줘야 한다.
MIDRANGE_Q = {"shoulder_pan": 0.0,
              "shoulder_lift": math.radians(-13.9677),
              "elbow_flex": math.radians(16.1752),
              "wrist_flex": math.radians(-5.0480),
              "wrist_roll": 0.0}

COUNT_MIN, COUNT_MAX = 0, 4095
SAFETY_MARGIN_DEG = 2.0     # 실측한 끝단에서 안쪽으로 물러설 여유
MIN_SPAN_DEG = 10.0         # 이보다 좁으면 측정이 잘못된 것으로 본다
SWEEP_SECONDS = 12.0        # msvcrt 가 없을 때의 고정 시간

POSITIVE_HINT = {
    "shoulder_pan":  "베이스를 위에서 봤을 때 반시계 방향(로봇 기준 왼쪽)으로",
    "shoulder_lift": "위팔을 위로 들어올리는 방향으로",
    "elbow_flex":    "아래팔을 위로 펴올리는 방향으로",
    "wrist_flex":    "손목을 위로 젖히는 방향으로",
    "wrist_roll":    "그리퍼를 반시계 방향으로 회전",
}

# ---------------------------------------------------------------------------
# 유틸
# ---------------------------------------------------------------------------

def c2d(count: float) -> float:
    """서보 카운트 -> 각도[deg] (캘리브레이션 좌표계)"""
    return (count - CENTER_COUNT) / COUNTS_PER_DEG


def d2c(deg: float) -> int:
    return int(round(CENTER_COUNT + deg * COUNTS_PER_DEG))


def read_pos(bus: FeetechBus, sid: int, name: str = "") -> int:
    raw = bus.read_u16(sid, STS_ADDR["present_position"])
    if raw is None:
        raise RuntimeError(f"{name}(ID{sid}) 위치 읽기 실패")
    return raw


def read_servo_deg(bus: FeetechBus) -> dict[str, float]:
    return {n: c2d(read_pos(bus, sid, n)) for n, sid in SERVO_IDS.items()}


def read_homing_offset(bus: FeetechBus, sid: int) -> int:
    v = bus.read_u16(sid, STS_ADDR["homing_offset"])
    if v is None:
        return 0
    return -(v & 0x7FF) if (v & 0x800) else v


def encode_homing_offset(v: int) -> int:
    """read_homing_offset 의 역. 11번 비트가 부호인 부호-크기 표현."""
    return (0x800 | (-v & 0x7FF)) if v < 0 else (v & 0x7FF)


def set_half_turn_homing(bus: FeetechBus) -> bool:
    """0단계: 지금 자세가 CENTER_COUNT 로 읽히도록 서보 EEPROM 에 영점을 기록한다.

    서보는 present = 엔코더 - Homing_Offset (mod 4096) 을 보고한다. 영점이 없으면
    관절 가동범위가 0/4095 경계에 걸칠 수 있고, 그러면 리밋이 [0, 4095] 로 측정되고
    위치 명령이 경계 반대편으로 멀리 돌아 스토퍼에 부딪힌다. lerobot 의
    half-turn homing 과 같은 방식이다. 자세가 대충이어도 범위가 경계에서만 멀면 된다.
    """
    print("\n" + "=" * 66)
    print("0단계: 서보 영점(Homing Offset) 기록")
    print("=" * 66)
    print("  모든 관절을 각자 가동범위의 대략 가운데로 손으로 옮기세요.")
    print("  (pan 정면, 위팔은 앞뒤 끝의 중간, 팔꿈치/손목도 양 끝의 중간, 그리퍼 반쯤)")
    print("  정밀할 필요는 없습니다. 양 끝에서 30도 이상만 떨어져 있으면 됩니다.")
    print("  이 값은 서보 EEPROM 에 영구 기록되고, 이후 리밋/영점은 전부 다시 잡습니다.")
    if ask("  기록할까요? (y/N) > ", "n").lower() != "y":
        print("  건너뜀 - 기존 서보 영점을 그대로 씁니다.")
        return False
    wait_enter("  자세를 잡은 채로 Enter > ")

    new_ofs: dict[str, int] = {}
    for name, sid in SERVO_IDS.items():
        present = read_pos(bus, sid, name)
        old = read_homing_offset(bus, sid)
        ofs = (old + present - CENTER_COUNT + 2048) % 4096 - 2048
        new_ofs[name] = max(-2047, min(2047, ofs))

    for name, sid in SERVO_IDS.items():
        bus.write_u8(sid, STS_ADDR["torque_enable"], 0)
        bus.write_u8(sid, STS_ADDR["lock"], 0)
        bus.write_u16(sid, STS_ADDR["homing_offset"], encode_homing_offset(new_ofs[name]))
        bus.write_u8(sid, STS_ADDR["lock"], 1)
    time.sleep(0.2)

    ok = True
    for name, sid in SERVO_IDS.items():
        got = read_homing_offset(bus, sid)
        pos = read_pos(bus, sid, name)
        bad = got != new_ofs[name] or abs(pos - CENTER_COUNT) > 10
        ok &= not bad
        print(f"    {name:<15} offset {got:+5d}  현재 {pos:>4} "
              f"({c2d(pos):+6.1f}deg){'  <-- 기록 실패' if bad else ''}")
    if not ok:
        raise CalibAborted("영점 기록 확인 실패 - 전원/배선을 확인하고 다시 하세요")
    print("  기록 완료. 이제 리밋을 새로 훑어야 합니다.")
    return True


class CalibAborted(RuntimeError):
    """캘리브레이션을 계속할 수 없다 (입력 불가 등). 저장하지 않고 빠져나간다."""


def ask(prompt: str, default: str = "") -> str:
    try:
        s = input(prompt).strip()
    except EOFError:
        return default
    return s or default


def wait_enter(prompt: str) -> None:
    """Enter 를 기다린다. 입력을 받을 수 없는 환경이면 중단한다.

    파이프로 돌리는 등 stdin 이 없을 때 EOFError 로 죽으면 팔이 토크 OFF 인 채
    프로그램만 사라진다. 정상 경로로 빠져나가 토크 정리까지 하게 만든다.
    """
    try:
        input(prompt)
    except EOFError:
        raise CalibAborted("입력을 받을 수 없는 환경입니다")


# ---------------------------------------------------------------------------
# 리밋 측정
# ---------------------------------------------------------------------------

def sweep_joint(bus: FeetechBus, name: str, sid: int) -> tuple[int, int, bool]:
    """관절을 손으로 끝에서 끝까지 움직이는 동안 최소/최대 카운트를 기록한다.

    토크가 꺼져 있으므로 사람이 미는 대로만 움직인다. 기계적 스토퍼에 살짝
    닿을 때까지 양쪽 끝을 훑으면 된다. 세 번째 값은 0/4095 경계를 넘었는지다 -
    넘었다면 min/max 가 엔코더 양끝으로 나와 리밋으로 쓸 수 없다.
    """
    lo = hi = prev = read_pos(bus, sid, name)
    wrapped = False
    print(f"    지금부터 [{name}] 을 양쪽 끝까지 천천히 움직이세요.")
    if msvcrt is not None:
        print("    다 하면 Enter. (실시간으로 최소/최대를 기록합니다)")
    else:
        print(f"    {SWEEP_SECONDS:.0f}초 동안 기록합니다.")

    t0 = time.time()
    last = 0.0
    while True:
        p = read_pos(bus, sid, name)
        lo, hi = min(lo, p), max(hi, p)
        if abs(p - prev) > 2048 and not wrapped:    # 10ms 사이 반 바퀴 = 경계 통과
            wrapped = True
            print(f"\n    !! {prev} -> {p}: 엔코더 경계(4095/0)를 넘었습니다. "
                  "0단계(서보 영점)를 먼저 해야 합니다.")
        prev = p

        now = time.time()
        if now - last > 0.08:
            last = now
            print(f"\r      현재 {p:>4} ({c2d(p):+7.2f}deg)   "
                  f"측정범위 [{lo:>4}, {hi:>4}] = [{c2d(lo):+7.2f}, {c2d(hi):+7.2f}]deg "
                  f"폭 {c2d(hi)-c2d(lo):6.2f}deg   ", end="", flush=True)

        if msvcrt is not None:
            if msvcrt.kbhit() and msvcrt.getch() in (b"\r", b"\n"):
                break
        elif now - t0 > SWEEP_SECONDS:
            break
        time.sleep(0.01)

    print()
    return lo, hi, wrapped


def apply_margin(lo: int, hi: int) -> tuple[int, int]:
    m = SAFETY_MARGIN_DEG * COUNTS_PER_DEG
    a, b = int(round(lo + m)), int(round(hi - m))
    if b <= a:                      # 범위가 너무 좁으면 여유를 포기한다
        a, b = lo, hi
    return max(COUNT_MIN, a), min(COUNT_MAX, b)


def set_ranges(bus: FeetechBus, base: dict) -> dict:
    """관절별로 리밋을 유지/측정/직접입력 중에 고른다."""
    print("\n" + "=" * 66)
    print("1단계: 관절 min/max 리밋")
    print("=" * 66)
    print("  이 값이 test4.py 의 하드 클램프가 됩니다. 팔은 절대 이 범위를 넘지 않습니다.")
    print("  관절마다: [k]유지  [s]손으로 훑어 측정  [t]각도 직접입력\n")

    out: dict[str, dict] = {}
    for name, sid in SERVO_IDS.items():
        cur = base.get(name, {})
        lo = cur.get("range_min", COUNT_MIN)
        hi = cur.get("range_max", COUNT_MAX)
        pos = read_pos(bus, sid, name)
        print(f"[{name}] ID{sid}  현재 {pos} ({c2d(pos):+.2f}deg)")
        print(f"    기존 리밋 [{lo}, {hi}] = [{c2d(lo):+.2f}, {c2d(hi):+.2f}]deg "
              f"(폭 {c2d(hi)-c2d(lo):.1f}deg)")

        # 측정된 적 없는 리밋([0, 4095])을 Enter 한 번으로 유지하면 클램프가 아무것도
        # 막지 못한다. 그런 관절은 기본을 측정으로 둔다.
        measured = name in base
        default = "k" if measured else "s"
        while True:
            m = ask(f"    선택 [k/s/t] (기본 {default}) > ", default).lower()
            if m == "k":
                if c2d(hi) - c2d(lo) > 300:
                    print("    이 리밋은 거의 한 바퀴라 보호가 되지 않습니다. s 로 훑으세요.")
                    continue
                nlo, nhi = lo, hi
                break
            if m == "s":
                rlo, rhi, wrapped = sweep_joint(bus, name, sid)
                if wrapped:
                    raise CalibAborted(f"{name} 가 엔코더 경계를 넘음 - 0단계(서보 영점)부터 다시")
                span = c2d(rhi) - c2d(rlo)
                if span < MIN_SPAN_DEG:
                    print(f"    측정 폭이 {span:.1f}deg 로 너무 좁습니다. 다시 하세요.")
                    continue
                nlo, nhi = apply_margin(rlo, rhi)
                print(f"    실측 [{rlo}, {rhi}] -> 여유 {SAFETY_MARGIN_DEG}deg 적용 "
                      f"[{nlo}, {nhi}] = [{c2d(nlo):+.2f}, {c2d(nhi):+.2f}]deg")
                break
            if m == "t":
                s = ask("    min max [deg] 를 공백으로 구분해 입력 > ")
                try:
                    a, b = (float(v) for v in s.replace(",", " ").split())
                except ValueError:
                    print("    숫자 두 개를 입력하세요.")
                    continue
                if b <= a:
                    print("    max 가 min 보다 커야 합니다.")
                    continue
                nlo, nhi = max(COUNT_MIN, d2c(a)), min(COUNT_MAX, d2c(b))
                print(f"    -> [{nlo}, {nhi}] 카운트")
                break
            print("    k, s, t 중에 고르세요.")

        if not (nlo <= pos <= nhi):
            print(f"    주의: 현재 위치 {pos} 가 새 리밋 밖입니다. "
                  f"이 관절은 클램프 때문에 즉시 끌려갑니다.")

        out[name] = {"id": sid,
                     "drive_mode": cur.get("drive_mode", 0),
                     "homing_offset": read_homing_offset(bus, sid),
                     "range_min": int(nlo), "range_max": int(nhi)}
        print()
    return out

ZERO_POSE_DESC = """
[영자세] 모든 관절 0도 = lift 90도 / elbow 90도 / wrist 0도

  - 위팔(shoulder_lift -> elbow)  : 수직으로 곧게 선다
  - 아래팔(elbow -> wrist)        : 수평. 위팔과 직각
  - 손목+그리퍼(wrist -> TCP)     : 아래팔과 일직선, 그대로 수평
  - 팔은 정면(pan 중앙), 손목 회전(wrist_roll)은 중립

  직각자를 위팔에 대고 아래팔이 직각인지 보면 된다. 링크를 앞으로 쭉 편
  수평 자세가 아니다 - 그렇게 잡으면 관절마다 70도 넘게 어긋난다.
  이 자세의 TCP 는 pan 축에서 324.8mm 앞, 테이블에서 232.6mm 위다.
"""


def set_mapping(bus: FeetechBus, ranges: dict) -> dict:
    """관절별 부호와 영점(offset_deg)을 정한다.

    부호를 먼저 잡는다. 범위 중앙으로 영점을 계산하려면 부호가 있어야 하고,
    자세를 손으로 잡는 경우에도 부호 확인 중에 팔을 흔들게 되므로 자세는
    맨 마지막에 잡는 편이 낫다.

    영점 방식:
      [r] 가동범위의 한가운데에서 계산 - 사람이 자세를 잡지 않는다. 대신
          1단계에서 관절을 양쪽 끝까지 제대로 훑었어야 한다.
      [p] 영자세를 손으로 잡아 실측 - 범위를 다 훑지 못했을 때의 대안.
    """
    print("\n" + "=" * 66)
    print("2단계: 부호 + 영점")
    print("=" * 66)
    print(ZERO_POSE_DESC)

    while True:
        how = ask("  영점 [r]가동범위 중앙에서 계산(권장) / [p]자세를 손으로 잡아 측정 > ",
                  "r").lower()
        if how in ("r", "p"):
            break
        print("  r 또는 p 를 고르세요.")

    print("\n  먼저 관절별 '+방향'을 확인합니다. 안내 방향으로 15도 이상 움직인 뒤 Enter.\n")
    sign: dict[str, float] = {}
    for n in JOINT_ORDER:
        base = read_servo_deg(bus)[n]
        while True:
            wait_enter(f"  [{n}] {POSITIVE_HINT[n]} 움직이고 Enter > ")
            delta = read_servo_deg(bus)[n] - base
            if abs(delta) >= 5.0:
                break
            print(f"    변화가 {delta:+.1f}도로 너무 작습니다. 더 크게 움직여 주세요.")
        sign[n] = 1.0 if delta > 0 else -1.0
        print(f"    delta={delta:+7.2f}deg -> sign={sign[n]:+.0f}")

    print()
    zero: dict[str, float] = {}
    if how == "r":
        for n in JOINT_ORDER:
            rg = ranges[n]
            mid = (rg["range_min"] + rg["range_max"]) / 2.0
            # 범위 중앙은 URDF 영자세다. 우리 0도(90/90/0)와의 차이를 빼준다.
            zero[n] = c2d(mid) - sign[n] * math.degrees(MIDRANGE_Q[n])
            span = c2d(rg["range_max"]) - c2d(rg["range_min"])
            warn = "  <-- 범위가 좁다. 1단계를 다시 훑을 것" if span < 60 else ""
            print(f"    {n:<15} 중앙 {mid:7.1f}카운트 = {c2d(mid):+7.2f}deg"
                  f"  -> 영점 {zero[n]:+7.2f}deg  (폭 {span:.1f}deg){warn}")
    else:
        wait_enter("  팔을 영자세(위팔 수직 / 아래팔 수평)로 잡은 채로 Enter > ")
        zero = read_servo_deg(bus)
        print("\n  영자세 서보각:")
        for n in JOINT_ORDER:
            print(f"    {n:<15}{zero[n]:+8.2f} deg")

    return {n: {"sign": sign[n], "offset_deg": zero[n]} for n in JOINT_ORDER}


# ---------------------------------------------------------------------------
# TCP 확인
# ---------------------------------------------------------------------------
#
# 영점(offset_deg)이 맞았는지는 서보각만 봐서는 알 수 없다. 링크 길이까지
# 통과한 TCP 좌표라야 "그리퍼가 테이블에 닿아 있는데 z 가 -116mm 라고 한다"
# 같은 모순이 눈에 보인다. 그 모순을 여기서 못 잡으면 hand-eye 캘리브레이션까지
# 끌고 가서, 카메라 탓을 하며 몇 시간을 버리게 된다.

TABLE_Z_TOL = 0.010     # 테이블에 댔을 때 TCP z 가 0 에서 벗어나도 되는 폭 [m]


def read_all_counts(bus: FeetechBus) -> dict[str, int]:
    return {n: read_pos(bus, sid, n) for n, sid in SERVO_IDS.items()}


def tcp_of(calib: ArmCalibration, counts: dict[str, int]):
    """카운트 -> (IK 관절각[rad], TCP[m])"""
    q = calib.counts_to_ik(counts)
    return q, so101_fk(q)




def tcp_text(calib: ArmCalibration, counts: dict[str, int]) -> str:
    q, p = tcp_of(calib, counts)
    r = math.hypot(p[0], p[1])
    joints = "  ".join(f"{SHORT_NAME[n]} {math.degrees(q[n]):+6.1f}" for n in JOINT_ORDER)
    return (f"TCP  x{p[0]*1000:+7.1f}  y{p[1]*1000:+7.1f}  z{p[2]*1000:+7.1f} mm"
            f"   반경 {r*1000:6.1f}   |  {joints}")


def watch_tcp(bus: FeetechBus, calib: ArmCalibration, note: str = "") -> dict[str, int]:
    """Enter 를 누를 때까지 TCP 를 실시간으로 보여주고, 그 순간의 카운트를 돌려준다."""
    if note:
        print(f"  {note}")
    if msvcrt is not None:
        print("  움직여 보면서 값을 확인하세요. 다 보면 Enter.")
    else:
        print(f"  {SWEEP_SECONDS:.0f}초 동안 표시합니다.")

    counts = read_all_counts(bus)
    t0 = last = time.time()
    while True:
        counts = read_all_counts(bus)
        now = time.time()
        if now - last > 0.1:
            last = now
            print("\r  " + tcp_text(calib, counts) + "   ", end="", flush=True)
        if msvcrt is not None:
            if msvcrt.kbhit() and msvcrt.getch() in (b"\r", b"\n"):
                break
        elif now - t0 > SWEEP_SECONDS:
            break
        time.sleep(0.02)
    print()
    return counts


def table_check(bus: FeetechBus, calib: ArmCalibration) -> None:
    """그리퍼 끝을 테이블에 댄 자세에서 TCP z 가 0 인지 본다.

    팔이 테이블 위에 놓여 있다면 이건 자를 대지 않고도 쓸 수 있는 유일한
    절대 기준이다. 여기서 z 가 0 이 아니면 그만큼 기준자세가 틀린 것이고,
    그 오차는 자세마다 다르게 나타나서 나중에 어떤 보정으로도 흡수되지 않는다.
    """
    print("\n" + "=" * 66)
    print("4단계: 테이블 접촉 검사 (영점이 실제로 맞는지)")
    print("=" * 66)
    print("  그리퍼 끝을 테이블면에 살짝 대세요. 팔이 테이블 위에 놓여 있다면")
    print("  그 순간 TCP z 는 0 근처여야 합니다.")
    counts = watch_tcp(bus, calib, "그리퍼 끝을 테이블에 댄 채로 Enter")

    q, p = tcp_of(calib, counts)
    z, r = float(p[2]), math.hypot(p[0], p[1])
    print(f"\n  접촉 지점의 TCP: z = {z*1000:+.1f}mm  (반경 {r*1000:.1f}mm)")
    if abs(z) <= TABLE_Z_TOL:
        print(f"  -> 정상. 영점이 맞습니다 (허용 +-{TABLE_Z_TOL*1000:.0f}mm)")
        return

    print(f"  -> 어긋남 {z*1000:+.1f}mm. 기준자세(2단계)가 그만큼 틀렸습니다.")
    if r > 1e-3:
        deg = math.degrees(math.atan2(-z, r))
        print(f"     이 반경에서는 팔 전체가 약 {deg:+.1f}도 기울어진 것과 같습니다.")
        print(f"     (shoulder_lift offset_deg 를 {deg*(-calib.mapping['shoulder_lift']['sign']):+.1f}도 "
              f"움직이면 이 자세에서는 z 가 0 이 됩니다. 다만 한 자세만 맞춘 것이라")
        print("      다른 자세에서 또 틀어집니다 - 되도록 2단계를 다시 하세요.)")
    print("     영자세는 위팔 수직 / 아래팔 수평(위팔과 직각) / 그리퍼는 아래팔과 일직선입니다.")
    print("     링크를 쭉 편 수평 자세가 아닙니다. 직각자를 대면 눈대중보다 훨씬 정확합니다.")


# ---------------------------------------------------------------------------

def verify_home_reach(calib: ArmCalibration) -> bool:
    print("\n" + "=" * 66)
    print("5단계: 검증 (전송하지 않고 계산만)")
    print("=" * 66)
    q = so101_ik(*HOME_POSE)
    ok = True
    counts = {}
    for n in JOINT_ORDER:
        r = calib.ranges[n]
        m = calib.mapping[n]
        servo_deg = math.degrees(q[n]) * m["sign"] + m["offset_deg"]
        raw = CENTER_COUNT + servo_deg * COUNTS_PER_DEG
        clamped = calib.ik_to_count(n, q[n])
        counts[n] = clamped
        hit = "" if abs(raw - clamped) < 1 else "  <-- 리밋에 걸림"
        if hit:
            ok = False
        print(f"  {n:<15} ik={math.degrees(q[n]):+7.2f}deg  servo={servo_deg:+7.2f}deg  "
              f"count={raw:7.0f}  범위[{r['range_min']},{r['range_max']}]{hit}")

    # 왕복 검사: 목표 -> 카운트 -> 다시 TCP. 리밋에 걸리면 여기서 어긋난다.
    reached = so101_fk(calib.counts_to_ik(counts))
    want = HOME_POSE
    d = math.dist(reached, want)
    print(f"\n  HOME 목표    x{want[0]*1000:+7.1f} y{want[1]*1000:+7.1f} z{want[2]*1000:+7.1f} mm")
    print(f"  클램프 후 TCP x{reached[0]*1000:+7.1f} y{reached[1]*1000:+7.1f} "
          f"z{reached[2]*1000:+7.1f} mm   차이 {d*1000:.1f}mm")
    if d > 0.002:
        print("  -> 리밋 때문에 목표에 못 간다. 리밋이나 기준자세를 다시 볼 것")
        ok = False
    return ok


def zero_pose_tcp():
    """기준자세(모든 관절 0도)에서 나와야 하는 TCP. 화면에서 바로 대조할 기준값."""
    return so101_fk({n: 0.0 for n in JOINT_ORDER})


def monitor_tcp(port: str) -> None:
    """저장된 arm_calib.json 으로 TCP 만 실시간으로 본다. 아무것도 바꾸지 않는다.

    영점이 의심스러울 때 캘리브레이션 전체를 다시 하지 않고 확인만 하는 길.
    """
    calib = ArmCalibration.load()
    if calib is None:
        print(f"[calib] {ARM_CALIB_PATH} 가 없다 - 먼저 캘리브레이션할 것")
        return
    bus = FeetechBus(port)
    try:
        missing = [n for n, sid in SERVO_IDS.items() if not bus.ping(sid)]
        if missing:
            print(f"[calib] 응답 없는 서보: {missing}")
            return
        for sid in SERVO_IDS.values():
            bus.write_u8(sid, STS_ADDR["torque_enable"], 0)
        z = zero_pose_tcp()
        print(f"[calib] {port} 연결, 토크 OFF (손으로 움직입니다)")
        print(f"        기준자세라면 TCP 는 x{z[0]*1000:+.0f} y{z[1]*1000:+.0f} "
              f"z{z[2]*1000:+.0f}mm 여야 합니다.")
        watch_tcp(bus, calib)
        table_check(bus, calib)
    except KeyboardInterrupt:
        print("\n[calib] 중단")
    finally:
        for sid in SERVO_IDS.values():
            try:
                bus.write_u8(sid, STS_ADDR["torque_enable"], 0)
            except Exception:
                pass
        bus.close()


def run_arm_calibration(port: str) -> Optional[ArmCalibration]:
    """관절 리밋 -> 기준자세 -> TCP 확인 -> 테이블 검사 -> 저장. 실패/중단이면 None.

    전 과정 토크 OFF 다. 서보에 목표위치를 쓰지 않으므로 팔은 스스로 움직이지
    않는다. 사람이 손으로 자세를 잡는 방식이고, 팔은 중력으로 처지니 받쳐야 한다.
    """
    try:
        base = load_lerobot_calibration(LEROBOT_CALIB)
        print(f"[calib] 기존 리밋 로드: {LEROBOT_CALIB}")
    except Exception as e:
        print(f"[calib] 기존 리밋 없음 ({e.__class__.__name__}) - 전부 새로 측정해야 한다")
        base = {}

    bus = FeetechBus(port)
    try:
        missing = [n for n, sid in SERVO_IDS.items() if not bus.ping(sid)]
        if missing:
            print(f"[calib] 응답 없는 서보: {missing} - 캘리브레이션 중단")
            return None

        for sid in SERVO_IDS.values():          # 안전: 전 관절 토크 OFF 보장
            bus.write_u8(sid, STS_ADDR["torque_enable"], 0)
        print(f"[calib] {port} 연결, 전 관절 토크 OFF (팔이 손으로 움직입니다)")

        if set_half_turn_homing(bus):
            base = {}                 # 영점이 바뀌었으니 예전 리밋 카운트는 무효

        ranges = set_ranges(bus, base)
        mapping = set_mapping(bus, ranges)
        calib = ArmCalibration(mapping, ranges)

        print("\n" + "=" * 66)
        print("3단계: 실시간 TCP 확인")
        print("=" * 66)
        z = zero_pose_tcp()
        print("  이제 팔을 손으로 움직이면 그 자세의 TCP 가 실시간으로 보입니다.")
        print(f"  기준자세로 되돌리면 x{z[0]*1000:+.0f} y{z[1]*1000:+.0f} z{z[2]*1000:+.0f}mm "
              "근처여야 합니다.")
        watch_tcp(bus, calib)

        table_check(bus, calib)

        if not verify_home_reach(calib):
            print("\n[calib] 경고: HOME 자세가 리밋에 걸립니다.")
            print("        기준자세가 부정확했거나 리밋이 너무 좁습니다.")
            if ask("        그래도 저장할까요? (y/N) > ").lower() != "y":
                print("[calib] 저장하지 않고 종료")
                return None

        calib.save(ARM_CALIB_PATH)
        print(f"[calib] 완료. 이제 {ARM_CALIB_PATH} 로 실기를 제어합니다.")
        return calib
    except KeyboardInterrupt:
        print("\n[calib] 사용자 중단 - 저장하지 않음")
        return None
    except CalibAborted as e:
        print(f"[calib] 중단: {e} - 저장하지 않음")
        return None
    finally:
        for sid in SERVO_IDS.values():
            try:
                bus.write_u8(sid, STS_ADDR["torque_enable"], 0)
            except Exception:
                pass
        bus.close()



def main() -> None:
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    flags = {a for a in sys.argv[1:] if a.startswith("-")}
    port = args[0] if args else find_so101_port()
    if not port:
        print("SO-101 어댑터를 찾지 못했다. 포트를 인자로 주세요 (예: /dev/ttyACM0, COM18)")
        return

    if "--tcp" in flags:
        monitor_tcp(port)
        return

    print(__doc__)
    if run_arm_calibration(port) is not None:
        print("        다음: verify_arm.py 로 방향을 검증하세요. 전원 차단 준비 필수.")


if __name__ == "__main__":
    main()
