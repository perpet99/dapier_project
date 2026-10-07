#!/usr/bin/env bash
# robot_arm.py 실행. 이미 떠 있는 robot_arm.py 가 있으면 먼저 종료한다.
#
#   ./run.sh              # 인자는 그대로 robot_arm.py 에 넘어간다
#
# 기존 프로세스는 Ctrl+C(SIGINT) 로 먼저 끈다. 그래야 robot_arm.py 의 finally 가 돌아
# 팔 토크 해제와 카메라 닫기를 하고 나간다. 바로 kill -9 하면 정리 없이 죽어서
# 카메라/시리얼 포트가 잡힌 채로 남을 수 있다. 안 꺼질 때만 TERM -> KILL 순으로 올린다.
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${PYTHON:-$HERE/../.venv/bin/python}"
# 실행 파일 자체가 python 이고 스크립트가 robot_arm.py 인 것만. robot_arm_win.py 나,
# 명령줄에 'python robot_arm.py' 문자열이 들어 있을 뿐인 셸(bash -c ...)은 건드리지 않는다.
PATTERN='^[^ ]*python[0-9.]*( -[^ ]+)* ([^ ]*/)?robot_arm\.py( |$)'

if [ ! -x "$PY" ]; then
    echo "[run] 파이썬을 찾을 수 없습니다: $PY  (PYTHON=/경로/python 으로 지정)" >&2
    exit 1
fi

# $1 신호를 보내고 $2 초 동안 꺼지기를 기다린다. 다 꺼졌으면 0.
stop_with() {
    local sig=$1 wait_s=$2 pids
    pids=$(pgrep -f "$PATTERN" | grep -vx "$$")
    [ -z "$pids" ] && return 0
    echo "[run] 기존 robot_arm.py 종료 ($sig): $(echo $pids)"
    kill "-$sig" $pids 2>/dev/null
    for _ in $(seq $((wait_s * 10))); do
        pgrep -f "$PATTERN" | grep -vqx "$$" || return 0
        sleep 0.1
    done
    return 1
}

stop_with INT 8 || stop_with TERM 3 || stop_with KILL 2 || {
    echo "[run] 기존 프로세스를 끄지 못했습니다" >&2
    exit 1
}

cd "$HERE"
exec "$PY" robot_arm.py "$@"
