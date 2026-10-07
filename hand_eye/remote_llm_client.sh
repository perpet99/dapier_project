#!/usr/bin/env bash
set -euo pipefail

# Run llm_client.py (Gemini Robotics-ER drives robot_arm.py's REST API) on the
# robot (Raspberry Pi) from the laptop. The laptop's current llm_client.py is
# copied to ~/hand_eye_robot/ on the Pi (next to robot_arm.py, see
# remote_robot_arm.sh) on every start. Runs in tmux session 'llm_client' on the
# Pi (log ~/llm_client.log), with the Pi's ~/dapier_project/.venv python.
# Web UI (give orders, see the view / log / robot state): http://<pi>:8770/
#
# Usage: ./remote_llm_client.sh <command> [options] [llm_client.py args...]
#   start   [args]   start and print the URL (robot_arm.py must be running)
#   stop             stop
#   restart [args]   stop + start
#   status           running?, URL, last log lines
#   log              follow the log (Ctrl-C to quit)
#   attach           attach to the tmux session (needed with --cli; Ctrl-b d to detach)
#
# start options (everything else goes to llm_client.py):
#   --push-env       copy this folder's .env (GEMINI_API_KEY) to the Pi, replacing its copy
#   "order"          first order, e.g. "주황색 통을 상자에 넣어"
#   --auto           don't ask before retrying a failed pick/place
#   --cli            orders from the terminal instead of the web UI (then: attach)
#   --api URL        robot_arm.py REST address (default http://127.0.0.1:8765 = on the Pi)
#   --port N         web UI port (default 8770)
#   --map-api URL    car2 map_web REST (mobile-base skills; default: --api's host, port 8081)
#   --no-base        no mobile-base skills (arm only)
#
# Examples:
#   ./remote_robot_arm.sh start && ./remote_llm_client.sh start
#   ./remote_llm_client.sh start --auto "책상 위 물건 하나만 집어봐"
#
# Environment: ROBOT (default user@192.168.0.31), REMOTE_DIR (default hand_eye_robot),
#              REMOTE_PY (default ~/dapier_project/.venv/bin/python), GEMINI_MODEL
#
# The API key: the Pi's ~/hand_eye_robot/.env; if missing it is taken from the
# Pi's git clone (~/dapier_project/hand_eye/.env), else from this folder's .env.
# The key is never printed. pick/place move the real arm without asking.

ROBOT="${ROBOT:-user@192.168.0.31}"
REMOTE_DIR="${REMOTE_DIR:-hand_eye_robot}"
REMOTE_PY="${REMOTE_PY:-~/dapier_project/.venv/bin/python}"
SESSION=llm_client
REMOTE_LOG='~/llm_client.log'
HERE="$(cd "$(dirname "$0")" && pwd)"
ROBOT_HOST="${ROBOT#*@}"
# Anchored at the python interpreter (an unanchored pattern matches the tmux server).
PROC_PAT='^[^ ]*python[0-9.]* (-u )?[^ ]*llm_client\.py'

# One ssh connection for the whole script (one password prompt).
_CTL="$(mktemp -u /tmp/remote_llm_client_ssh.XXXXXX)"
SSH_OPTS=(-o ControlMaster=auto -o ControlPath="$_CTL" -o ControlPersist=30 -o ConnectTimeout=8)
trap 'ssh "${SSH_OPTS[@]}" -O exit "$ROBOT" 2>/dev/null || true' EXIT
remote() { ssh "${SSH_OPTS[@]}" "$ROBOT" "$@"; }
remote_tty() { ssh -t "${SSH_OPTS[@]}" "$ROBOT" "$@"; }
usage() { sed -n '4,/^$/p' "$0" | sed 's/^# \{0,1\}//'; }

CMD="${1:-}"
shift || true

print_info() {   # $1 = web port, $2 = 1 if --cli
  if [ "${2:-0}" = 1 ]; then
    echo "terminal mode (--cli): give orders with ./remote_llm_client.sh attach"
  elif curl -s -m 3 -o /dev/null "http://${ROBOT_HOST}:$1/"; then
    echo "web UI : http://${ROBOT_HOST}:$1/"
  else
    echo "web UI : not answering on http://${ROBOT_HOST}:$1/"
  fi
  remote "grep -E '^\[(api|web|error)\]|주의|GEMINI_API_KEY' $REMOTE_LOG 2>/dev/null | tail -n 5 | cut -c1-160" || true
}

do_status() {
  remote "bash -s" <<EOF
if tmux has-session -t $SESSION 2>/dev/null; then echo "tmux '$SESSION': running"; else echo "tmux '$SESSION': not running"; fi
pgrep -af '$PROC_PAT' | cut -c1-140 || echo "llm_client.py: not running"
EOF
  local cmdline port cli=0
  cmdline="$(remote "pgrep -af '$PROC_PAT' | head -1" || true)"
  port="$(grep -oE -- '--port[ =][0-9]+' <<<"$cmdline" | grep -oE '[0-9]+$' || true)"
  grep -q -- ' --cli' <<<"$cmdline" && cli=1
  [ -n "$cmdline" ] && print_info "${port:-8770}" "$cli"
  remote "[ -f $REMOTE_LOG ] && { echo '--- $REMOTE_LOG (last lines)'; tail -n 6 $REMOTE_LOG | cut -c1-160; }" || true
}

do_stop() {
  remote "bash -s" <<EOF
pids=\$(pgrep -f '$PROC_PAT' || true)
if [ -z "\$pids" ] && ! tmux has-session -t $SESSION 2>/dev/null; then echo "not running"; exit 0; fi
for sig_wait in INT:5 TERM:3 KILL:2; do
  sig=\${sig_wait%:*}; w=\${sig_wait#*:}
  pids=\$(pgrep -f '$PROC_PAT' || true)
  [ -z "\$pids" ] && break
  [ \$sig != INT ] && echo "llm_client.py did not exit, sending SIG\$sig"
  kill -\$sig \$pids 2>/dev/null || true
  for i in \$(seq \$((w * 10))); do pgrep -f '$PROC_PAT' >/dev/null || break; sleep 0.1; done
done
tmux kill-session -t $SESSION 2>/dev/null || true
echo "stopped"
EOF
}

do_start() {
  local push_env=0 args=() port=8770 api="http://127.0.0.1:8765" cli=0 host_given=0 a prev=""
  for a in "$@"; do
    case "$a" in
      --push-env) push_env=1; continue ;;
      --cli)      cli=1 ;;
      --host|--host=*) host_given=1 ;;
      --port=*)   port="${a#--port=}" ;;
      --api=*)    api="${a#--api=}" ;;
    esac
    [ "$prev" = --port ] && port="$a"
    [ "$prev" = --api ] && api="$a"
    args+=("$a"); prev="$a"
  done
  # Reachable from the laptop, and no browser on the Pi.
  [ $cli = 0 ] && [ $host_given = 0 ] && args+=(--host 0.0.0.0)
  args+=(--no-browser)
  # Arguments travel base64-encoded (NUL-separated): a Korean order with spaces
  # must reach llm_client.py as one argument, untouched by any shell quoting.
  local b64 envline=""
  b64="$(printf '%s\0' "${args[@]}" | base64 -w0)"
  [ -n "${GEMINI_MODEL:-}" ] && envline="export GEMINI_MODEL=$(printf '%q' "$GEMINI_MODEL")"

  remote "bash -s" <<EOF
if tmux has-session -t $SESSION 2>/dev/null || pgrep -f '$PROC_PAT' >/dev/null; then
  echo "already running. Use: restart / stop"; exit 1
fi
[ -x $REMOTE_PY ] || { echo "ERROR: $REMOTE_PY not found (REMOTE_PY=... to use another python)"; exit 1; }
if ! curl -s -m 3 '$api/get_state' | grep -q '"ok": *true'; then
  echo "ERROR: robot_arm.py's REST API is not answering at $api"
  echo "  -> start it first: ./remote_robot_arm.sh start"
  exit 1
fi
mkdir -p ~/$REMOTE_DIR
EOF

  rsync -az -e "ssh ${SSH_OPTS[*]}" "${HERE}/llm_client.py" "${ROBOT}:~/${REMOTE_DIR}/"
  # API key: keep the Pi's own .env unless --push-env; never print it.
  if [ $push_env = 1 ]; then
    [ -f "${HERE}/.env" ] || { echo "ERROR: ${HERE}/.env not found"; exit 1; }
    rsync -az --chmod=F600 -e "ssh ${SSH_OPTS[*]}" "${HERE}/.env" "${ROBOT}:~/${REMOTE_DIR}/.env"
    echo ".env pushed to the Pi"
  elif ! remote "test -f ~/${REMOTE_DIR}/.env"; then
    if remote "test -f ~/dapier_project/hand_eye/.env"; then
      remote "install -m 600 ~/dapier_project/hand_eye/.env ~/${REMOTE_DIR}/.env"
      echo ".env: copied from the Pi's ~/dapier_project/hand_eye/.env"
    elif [ -f "${HERE}/.env" ]; then
      rsync -az --chmod=F600 -e "ssh ${SSH_OPTS[*]}" "${HERE}/.env" "${ROBOT}:~/${REMOTE_DIR}/.env"
      echo ".env: copied from this folder"
    else
      echo "WARNING: no .env with GEMINI_API_KEY anywhere -- llm_client.py will stop (see .env.example)"
    fi
  fi

  remote "bash -s" <<EOF
rm -f $REMOTE_LOG   # an old log's lines must not count as this start
cat > ~/$REMOTE_DIR/.run_llm_client.sh <<'RUN'
#!/bin/bash
cd ~/$REMOTE_DIR
$envline
mapfile -t -d '' ARGS < <(printf '%s' '$b64' | base64 -d)
PYTHONUNBUFFERED=1 $REMOTE_PY -u llm_client.py "\${ARGS[@]}" 2>&1 | tee $REMOTE_LOG
RUN
tmux new-session -d -s $SESSION "bash ~/$REMOTE_DIR/.run_llm_client.sh"
for i in \$(seq 1 40); do
  if [ $cli = 1 ]; then
    grep -qE '^\[api\]' $REMOTE_LOG 2>/dev/null && { echo "started (tmux '$SESSION')"; exit 0; }
  else
    curl -s -m 1 -o /dev/null http://localhost:$port/ 2>/dev/null && { echo "started (tmux '$SESSION')"; exit 0; }
  fi
  tmux has-session -t $SESSION 2>/dev/null || break
  sleep 0.5
done
echo "ERROR: llm_client.py did not come up. Log:"; tail -n 15 $REMOTE_LOG; exit 1
EOF
  print_info "$port" "$cli"
}

case "$CMD" in
  start)   do_start "$@" ;;
  stop)    do_stop ;;
  restart) do_stop; do_start "$@" ;;
  status)  do_status ;;
  log)     remote_tty "tail -n 40 -f $REMOTE_LOG" ;;
  attach)  remote_tty "tmux attach -t $SESSION" ;;
  *)       usage; exit 1 ;;
esac
