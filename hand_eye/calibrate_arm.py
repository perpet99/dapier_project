"""
calibrate_arm.py - SO-101 팔 캘리브레이션 CLI (관절 리밋 + 기준자세).

본체는 test4.py 에 있다(run_arm_calibration / monitor_tcp). test4.py 를 그냥
실행해도 캘리브레이션을 할지 물어보므로, 이 파일은 '팔만 다시 잡고 싶을 때' 의
지름길이다. 로직을 여기에 따로 두면 두 벌이 되어 한쪽만 고쳐지므로 두지 않는다.

무엇을 정하는가:
  1) 관절별 min/max 리밋 [서보 카운트]
       test4.py 의 ik_to_count() 가 이 값으로 목표를 하드 클램프한다.
       IK 가 이상한 값을 내도 팔이 이 범위를 절대 넘지 않는다. 안전의 마지막 방어선.
  2) 관절별 부호 + 오프셋
       서보가 보고하는 0도가 test4.py IK 의 0도(모든 링크를 앞으로 수평하게 편
       자세)와 같다는 보장이 없다. 그 차이를 실측한다. 추측하면 팔이 꽂힌다.

안전:
  처음부터 끝까지 토크를 끈다. 팔은 손으로 잡고 진행하며, 토크가 꺼져 있으면
  중력으로 처지니 받쳐야 한다.

사용:
  python calibrate_arm.py [COM18]           전체 캘리브레이션
  python calibrate_arm.py [COM18] --tcp     저장된 값으로 TCP 만 확인 (아무것도 안 바꿈)
"""

from __future__ import annotations

import sys

from test4 import find_so101_port, monitor_tcp, run_arm_calibration

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass


def main() -> None:
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    flags = {a for a in sys.argv[1:] if a.startswith("-")}
    port = args[0] if args else (find_so101_port() or "COM18")

    if "--tcp" in flags:
        monitor_tcp(port)
        return

    print(__doc__)
    if run_arm_calibration(port) is not None:
        print("        다음: verify_arm.py 로 방향을 검증하세요. 전원 차단 준비 필수.")


if __name__ == "__main__":
    main()
