#!/usr/bin/env bash
set -euo pipefail

# Run robot_arm.py (depth click pick & place, SO-101) on the robot (Raspberry Pi)
# from the laptop. The laptop's current robot_arm.py is copied to
# ~/hand_eye_robot/ on the Pi on every start (the Pi's git clone ~/dapier_project
# stays untouched); calibration files are copied only when the Pi has none yet,
# so calibrations made on the Pi are never overwritten (--push-calib to force).
# Runs in tmux session 'robot_arm' on the Pi (log ~/robot_arm.log), with the
# Pi's ~/dapier_project/.venv python. Web UI / REST: http://<pi>:8765/
#
# Usage: ./remote_robot_arm.sh <command> [options] [VAR=value ...]
#   start   [opts]   start and print the URL
#   stop             stop (SIGINT first: robot_arm.py turns the arm torque off and closes the camera)
#   restart [opts]   stop + start
#   status           running?, URL, camera / arm lines from the log
#   log              follow the log (Ctrl-C to quit)
#   attach           attach to the tmux session (console input for hand-eye calibration
#                    points; Ctrl-b d to detach)
#   pull-calib       copy the Pi's calibration files back to this folder
#
# start options:
#   --calib          answer "yes" to the start-up question "redo the camera-robot
#                    (hand-eye) calibration?" (default: no). Then confirm each point
#                    in the console: ./remote_robot_arm.sh attach
#   --push-calib     overwrite the Pi's calibration files with this folder's
#   VAR=value        environment for robot_arm.py, e.g. SO101_PORT=/dev/ttyUSB1
#                    PICK_API_PORT=8770 PICK_STREAM_FPS=10 DEPTH_ALIGN=0
#
# Examples:
#   ./remote_robot_arm.sh start
#   ./remote_robot_arm.sh start --calib SO101_PORT=/dev/ttyUSB1
#
# Environment: ROBOT (default user@192.168.0.31), REMOTE_DIR (default hand_eye_robot),
#              REMOTE_PY (default ~/dapier_project/.venv/bin/python)
#
# robot_arm.py opens the Astra Pro (OpenNI2 depth + UVC color) itself, so this
# refuses to start while the camera is used by ROS (remote_robot_rgbd.sh, or
# remote_car2_lidar.sh --camera) -- stop that first.

ROBOT="${ROBOT:-user@192.168.0.31}"
REMOTE_DIR="${REMOTE_DIR:-hand_eye_robot}"
REMOTE_PY="${REMOTE_PY:-~/dapier_project/.venv/bin/python}"
SESSION=robot_arm
REMOTE_LOG='~/robot_arm.log'
HERE="$(cd "$(dirname "$0")" && pwd)"
ROBOT_HOST="${ROBOT#*@}"
CALIB_FILES=(arm_calib.json handeye.json handeye_points.json color_cam.json)
# Anchored at the python interpreter: a tmux server's cmdline is its first
# session's command, so an unanchored 'robot_arm.py' would match (and kill) it.
PROC_PAT='^[^ ]*python[0-9.]* (-u )?[^ ]*robot_arm\.py'

# One ssh connection for the whole script (one password prompt).
_CTL="$(mktemp -u /tmp/remote_robot_arm_ssh.XXXXXX)"
SSH_OPTS=(-o ControlMaster=auto -o ControlPath="$_CTL" -o ControlPersist=30 -o ConnectTimeout=8)
trap 'ssh "${SSH_OPTS[@]}" -O exit "$ROBOT" 2>/dev/null || true' EXIT
remote() { ssh "${SSH_OPTS[@]}" "$ROBOT" "$@"; }
remote_tty() { ssh -t "${SSH_OPTS[@]}" "$ROBOT" "$@"; }
usage() { sed -n '4,/^$/p' "$0" | sed 's/^# \{0,1\}//'; }

CMD="${1:-}"
shift || true

api_port() {   # PICK_API_PORT=N from the args, else robot_arm.py's default
  local a
  for a in "$@"; do
    [[ "$a" == PICK_API_PORT=* ]] && { echo "${a#PICK_API_PORT=}"; return; }
  done
  echo 8765
}

print_info() {   # $1 = port
  if curl -s -m 3 -o /dev/null "http://${ROBOT_HOST}:$1/status"; then
    echo "web UI : http://${ROBOT_HOST}:$1/"
  else
    echo "web UI : not answering on http://${ROBOT_HOST}:$1/"
  fi
  remote "grep -E '^\[(source|arm|api)\]|^\[[12]/2\]|mock|합성' $REMOTE_LOG 2>/dev/null | tail -n 8 | cut -c1-160" || true
}

do_status() {
  remote "bash -s" <<EOF
if tmux has-session -t $SESSION 2>/dev/null; then echo "tmux '$SESSION': running"; else echo "tmux '$SESSION': not running"; fi
pgrep -af '$PROC_PAT' | cut -c1-120 || echo "robot_arm.py: not running"
lsusb | grep -q '1a86:' && echo "  arm USB (CH34x): connected" || echo "  arm USB (CH34x): NOT connected -> robot_arm.py runs with a mock arm"
lsusb | grep -q '2bc5:060f' && echo "  camera USB (Astra Pro): connected" || echo "  camera USB (Astra Pro): NOT connected"
EOF
  local port
  port="$(remote "pgrep -af '$PROC_PAT' >/dev/null && tr '\0' '\n' < /proc/\$(pgrep -f '$PROC_PAT' | head -1)/environ | grep '^PICK_API_PORT=' | cut -d= -f2" || true)"
  print_info "${port:-8765}"
}

do_stop() {
  remote "bash -s" <<EOF
pids=\$(pgrep -f '$PROC_PAT' || true)
if [ -z "\$pids" ] && ! tmux has-session -t $SESSION 2>/dev/null; then echo "not running"; exit 0; fi
# SIGINT -> robot_arm.py's finally: arm torque off, camera closed. Escalate only if needed.
for sig_wait in INT:8 TERM:3 KILL:2; do
  sig=\${sig_wait%:*}; w=\${sig_wait#*:}
  pids=\$(pgrep -f '$PROC_PAT' || true)
  [ -z "\$pids" ] && break
  [ \$sig != INT ] && echo "robot_arm.py did not exit, sending SIG\$sig"
  kill -\$sig \$pids 2>/dev/null || true
  for i in \$(seq \$((w * 10))); do pgrep -f '$PROC_PAT' >/dev/null || break; sleep 0.1; done
done
tmux kill-session -t $SESSION 2>/dev/null || true
echo "stopped"
EOF
}

do_start() {
  local answer=n push_calib=0 envs="" port a f
  for a in "$@"; do
    case "$a" in
      --calib)      answer=y ;;
      --push-calib) push_calib=1 ;;
      *=*)          envs+=" $(printf '%q' "$a")" ;;
      *)            echo "unknown option: $a"; usage; exit 1 ;;
    esac
  done
  port="$(api_port "$@")"

  remote "bash -s" <<EOF
if tmux has-session -t $SESSION 2>/dev/null || pgrep -f '$PROC_PAT' >/dev/null; then
  echo "already running. Use: restart / stop"; exit 1
fi
busy=0
for p in 'openni2_camera_driver' 'usb_cam_node_exe' 'component_container.*__ns:=/camera'; do
  if pgrep -f "\$p" >/dev/null; then
    echo "ERROR: the camera is in use by ROS: \$(pgrep -af "\$p" | head -1 | cut -c1-100)"; busy=1
  fi
done
if [ \$busy = 1 ]; then
  pgrep -f 'robot_rgbd.launch.py' >/dev/null && echo "  -> stop it: ./ros2_slam/scripts/remote_robot_rgbd.sh stop"
  pgrep -f 'robot_lidar.launch.py.*use_camera:=true' >/dev/null && \
    echo "  -> car2_lidar was started with --camera: ./ros2_slam/scripts/remote_car2_lidar.sh restart (without --camera)"
  exit 1
fi
lsusb | grep -q '2bc5:060f' || echo "WARNING: camera (Astra Pro) not connected -> robot_arm.py falls back to synthetic depth"
lsusb | grep -q '1a86:' || echo "WARNING: SO-101 USB adapter (CH34x, 1a86) not connected -> robot_arm.py uses a mock arm"
[ -x $REMOTE_PY ] || { echo "ERROR: $REMOTE_PY not found (REMOTE_PY=... to use another python)"; exit 1; }
mkdir -p ~/$REMOTE_DIR
EOF

  # Code: always the laptop's current version. Calibration: only if the Pi has none (or --push-calib).
  rsync -az -e "ssh ${SSH_OPTS[*]}" "${HERE}/robot_arm.py" "${ROBOT}:~/${REMOTE_DIR}/"
  local calib=()
  for f in "${CALIB_FILES[@]}"; do [ -f "${HERE}/$f" ] && calib+=("${HERE}/$f"); done
  if [ ${#calib[@]} -gt 0 ]; then
    if [ $push_calib = 1 ]; then
      rsync -az -e "ssh ${SSH_OPTS[*]}" "${calib[@]}" "${ROBOT}:~/${REMOTE_DIR}/"
      echo "calibration files pushed (overwrote the Pi's)"
    else
      rsync -az --ignore-existing -e "ssh ${SSH_OPTS[*]}" "${calib[@]}" "${ROBOT}:~/${REMOTE_DIR}/"
    fi
  fi

  # stdin: the start-up question's answer, then the tmux pane (via cat) so console
  # input still reaches robot_arm.py after './remote_robot_arm.sh attach'.
  remote "bash -s" <<EOF
rm -f $REMOTE_LOG   # an old log's lines must not count as this start
tmux new-session -d -s $SESSION "cd ~/$REMOTE_DIR && { printf '$answer\\n'; cat; } | env PYTHONUNBUFFERED=1$envs $REMOTE_PY -u robot_arm.py 2>&1 | tee $REMOTE_LOG"
for i in \$(seq 1 60); do
  curl -s -m 1 -o /dev/null http://localhost:$port/status 2>/dev/null && { echo "started (tmux '$SESSION')"; exit 0; }
  tmux has-session -t $SESSION 2>/dev/null || break
  sleep 0.5
done
echo "ERROR: robot_arm.py did not come up. Log:"; tail -n 20 $REMOTE_LOG; exit 1
EOF
  [ "$answer" = y ] && echo "hand-eye calibration mode: confirm points in the console -> ./remote_robot_arm.sh attach"
  print_info "$port"
}

do_pull_calib() {
  local f
  for f in "${CALIB_FILES[@]}"; do
    if remote "test -f ~/${REMOTE_DIR}/$f"; then
      rsync -az -e "ssh ${SSH_OPTS[*]}" "${ROBOT}:~/${REMOTE_DIR}/$f" "${HERE}/$f"
      echo "pulled $f"
    fi
  done
}

case "$CMD" in
  start)      do_start "$@" ;;
  stop)       do_stop ;;
  restart)    do_stop; do_start "$@" ;;
  status)     do_status ;;
  log)        remote_tty "tail -n 40 -f $REMOTE_LOG" ;;
  attach)     remote_tty "tmux attach -t $SESSION" ;;
  pull-calib) do_pull_calib ;;
  *)          usage; exit 1 ;;
esac
