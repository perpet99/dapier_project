"""
car2 방향키 조종 - 웹 UI + REST API 버전 (car2_arrow.py 의 웹판, 표준 라이브러리만 사용).

사용법:
    python car2_web.py --port /dev/ttyS0
    python car2_web.py --port /dev/ttyS0 --speed 1200 --http-port 8766
    python car2_web.py --dry-run          # 시리얼 없이 명령만 출력 (UI/API 확인용)

브라우저에서 http://<라즈베리파이 IP>:8766/ 접속
    ↑ 전진   ↓ 후진   ← 좌회전(제자리)   → 우회전(제자리)  (키보드 방향키 또는 화면 버튼)
    누르고 있는 동안만 움직이고, 떼면 정지.  Space 정지   +/- 속도 조절

REST API (JSON)
    GET  /api/status                       현재 상태
    POST /api/move     {"direction": "forward|backward|left|right", "speed": 800, "duration": 1.0}
    POST /api/velocity {"linear": 800, "angular": 0, "duration": 1.0}   angular 양수 = 좌회전
    POST /api/stop                         즉시 정지
    POST /api/speed    {"speed": 1200} 또는 {"delta": 200}             기본 속도 변경
    speed 생략 시 현재 기본 속도, duration 생략 시 HOLD_TIMEOUT(0.35초) 동안만 움직인다.
    -> 계속 움직이려면 duration 이내 간격으로 다시 보내거나 duration 을 지정 (최대 MAX_DURATION).

    예) curl -X POST localhost:8766/api/move -d '{"direction":"forward","duration":1.5}'

!! 첫 테스트는 반드시 차체를 들어 바퀴가 공중에 뜬 상태에서 진행 !!

안전: 명령이 끊기면(브라우저 닫힘, 네트워크 끊김) 기한이 지나 STOP 을 보낸다.
      car2.ino 자체에도 500ms 명령 타임아웃이 있다.
"""

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from car2_test import MOVE_PERIOD, Car

MAX_SPEED = 3400
MIN_SPEED = 200
SPEED_STEP = 200
HOLD_TIMEOUT = 0.35  # duration 없이 온 이동 명령의 유지 시간(초). 웹 UI는 누르는 동안 0.1초마다 다시 보낸다
MAX_DURATION = 10.0  # REST 로 한 번에 지정할 수 있는 최대 이동 시간(초)

# 방향 -> (이름, linear 부호, angular 부호), angular 양수 = 좌회전
DIRECTIONS = {
    "forward": ("전진", 1, 0),
    "backward": ("후진", -1, 0),
    "left": ("좌회전", 0, 1),
    "right": ("우회전", 0, -1),
}


class DryCar:
    """--dry-run 용: 시리얼 대신 명령을 출력만 한다."""

    def __init__(self):
        self.last = None

    def send(self, cmd):
        if cmd != self.last:  # 같은 명령 재전송은 생략해서 출력이 넘치지 않게
            print("  ->", cmd)
        self.last = cmd

    def drain(self):
        pass

    def close(self):
        self.send("STOP")


class Controller:
    """이동 명령 상태를 들고, 백그라운드 스레드에서 차에 주기적으로 전송한다."""

    def __init__(self, car, speed):
        self.car = car
        self.speed = speed
        self.lock = threading.Lock()
        self.action = "정지"
        self.linear = 0
        self.angular = 0
        self.deadline = 0.0  # 이 시각까지 이동, 지나면 STOP
        self.moving = False
        self.last_cmd = "-"
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def move(self, name, linear, angular, duration=None):
        duration = HOLD_TIMEOUT if duration is None else max(0.0, min(float(duration), MAX_DURATION))
        linear = max(-MAX_SPEED, min(int(linear), MAX_SPEED))
        angular = max(-MAX_SPEED, min(int(angular), MAX_SPEED))
        with self.lock:
            changed = (linear, angular) != (self.linear, self.angular)
            self.action, self.linear, self.angular = name, linear, angular
            self.deadline = time.time() + duration
            if changed:
                self.moving = False  # 방향이 바뀌면 다음 루프에서 즉시 전송

    def stop(self):
        with self.lock:
            self.deadline = 0.0

    def set_speed(self, speed):
        with self.lock:
            self.speed = max(MIN_SPEED, min(int(speed), MAX_SPEED))
            return self.speed

    def status(self):
        with self.lock:
            active = time.time() < self.deadline
            return {
                "action": self.action if active else "정지",
                "linear": self.linear if active else 0,
                "angular": self.angular if active else 0,
                "speed": self.speed,
                "moving": self.moving,
                "remaining": round(max(0.0, self.deadline - time.time()), 2),
                "last_cmd": self.last_cmd,
                "min_speed": MIN_SPEED,
                "max_speed": MAX_SPEED,
                "speed_step": SPEED_STEP,
            }

    def _loop(self):
        last_send = 0.0
        while self.running:
            now = time.time()
            with self.lock:
                active = now < self.deadline and (self.linear or self.angular)
                # 이동 중에는 MOVE_PERIOD마다 재전송(car2.ino 500ms 타임아웃 방지), 멈출 때는 STOP 1회
                if active and (not self.moving or now - last_send >= MOVE_PERIOD):
                    cmd = f"V,{self.linear},{self.angular}"
                    self.moving, last_send = True, now
                elif not active and self.moving:
                    cmd = "STOP"
                    self.moving = False
                    self.action, self.linear, self.angular = "정지", 0, 0
                else:
                    cmd = None
                if cmd:
                    self.last_cmd = cmd
            if cmd:
                self.car.send(cmd)
            self.car.drain()
            time.sleep(0.01)

    def close(self):
        self.running = False
        self.thread.join(timeout=1.0)


def serial_drain(car):
    car.ser.reset_input_buffer()  # ACK 응답은 버림


INDEX_HTML = """<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>car2 조종</title>
<style>
  :root { --bg:#f4f5f7; --panel:#fff; --text:#1d2330; --muted:#6b7280; --btn:#e5e7eb;
          --active:#2563eb; --stop:#dc2626; }
  @media (prefers-color-scheme: dark) {
    :root { --bg:#111418; --panel:#1b2027; --text:#e5e7eb; --muted:#9ca3af; --btn:#2b323c; }
  }
  * { box-sizing:border-box; }
  body { margin:0; background:var(--bg); color:var(--text);
         font-family:system-ui, -apple-system, "Noto Sans KR", sans-serif;
         -webkit-user-select:none; user-select:none; touch-action:manipulation; }
  main { max-width:420px; margin:0 auto; padding:16px; }
  h1 { font-size:20px; margin:0 0 12px; }
  .panel { background:var(--panel); border-radius:12px; padding:14px; margin-bottom:12px; }
  .status { display:grid; grid-template-columns:auto 1fr; gap:4px 12px; font-size:15px; }
  .status b { color:var(--muted); font-weight:500; }
  #action { font-weight:700; }
  #action.go { color:var(--active); }
  .pad { display:grid; grid-template-columns:repeat(3, 1fr); gap:10px; }
  .pad button { aspect-ratio:1; font-size:34px; border:0; border-radius:14px;
                background:var(--btn); color:var(--text); touch-action:none; cursor:pointer; }
  .pad button.on { background:var(--active); color:#fff; }
  .pad #stop { background:var(--stop); color:#fff; font-size:18px; font-weight:700; }
  .speed { display:flex; align-items:center; gap:10px; }
  .speed input { flex:1; }
  .speed button { width:44px; height:44px; font-size:22px; border:0; border-radius:10px;
                  background:var(--btn); color:var(--text); }
  .hint { color:var(--muted); font-size:13px; line-height:1.6; }
  #err { color:var(--stop); font-size:13px; min-height:1em; }
</style>
</head>
<body>
<main>
  <h1>car2 조종</h1>
  <div class="panel status">
    <b>상태</b><span id="action">정지</span>
    <b>속도</b><span id="speedText">-</span>
    <b>명령</b><span id="cmd">-</span>
  </div>
  <div class="panel pad">
    <span></span><button data-dir="forward" aria-label="전진">↑</button><span></span>
    <button data-dir="left" aria-label="좌회전">←</button>
    <button id="stop">정지</button>
    <button data-dir="right" aria-label="우회전">→</button>
    <span></span><button data-dir="backward" aria-label="후진">↓</button><span></span>
  </div>
  <div class="panel">
    <div class="speed">
      <button id="minus">−</button>
      <input id="speed" type="range" min="200" max="3400" step="200">
      <button id="plus">+</button>
    </div>
  </div>
  <div class="hint">
    방향키 또는 버튼을 누르고 있는 동안만 이동, 떼면 정지<br>
    Space 정지 · +/- 속도 조절<br>
    첫 테스트는 반드시 바퀴를 띄운 상태에서!
  </div>
  <div id="err"></div>
</main>
<script>
const KEYS = {ArrowUp:"forward", ArrowDown:"backward", ArrowLeft:"left", ArrowRight:"right"};
const $ = id => document.getElementById(id);
let dir = null, timer = null, step = 200;

function api(path, body) {
  return fetch(path, {method: body === undefined ? "GET" : "POST", keepalive: true,
                      headers: {"Content-Type": "application/json"},
                      body: body === undefined ? undefined : JSON.stringify(body)})
    .then(r => r.json())
    .then(j => { $("err").textContent = j.error || ""; return j; })
    .catch(() => { $("err").textContent = "서버 연결 끊김"; });
}

function start(d) {
  if (dir === d) return;
  dir = d;
  highlight();
  api("/api/move", {direction: d});
  clearInterval(timer);
  timer = setInterval(() => api("/api/move", {direction: d}), 100);  // 누르는 동안 유지 신호
}

function stop() {
  clearInterval(timer); timer = null;
  dir = null;
  highlight();
  api("/api/stop", {});
}

function highlight() {
  document.querySelectorAll(".pad [data-dir]").forEach(b => b.classList.toggle("on", b.dataset.dir === dir));
}

function setSpeed(body) { api("/api/speed", body).then(render); }

function render(s) {
  if (!s || s.speed === undefined) return;
  $("action").textContent = s.action;
  $("action").classList.toggle("go", s.action !== "정지");
  $("speedText").textContent = s.speed + " step/s";
  $("cmd").textContent = s.last_cmd;
  step = s.speed_step;
  $("speed").min = s.min_speed; $("speed").max = s.max_speed; $("speed").step = s.speed_step;
  if (document.activeElement !== $("speed")) $("speed").value = s.speed;
}

document.querySelectorAll(".pad [data-dir]").forEach(b => {
  b.addEventListener("pointerdown", e => { b.setPointerCapture(e.pointerId); start(b.dataset.dir); });
  ["pointerup", "pointercancel", "lostpointercapture"].forEach(ev => b.addEventListener(ev, () => {
    if (dir === b.dataset.dir) stop();
  }));
  b.addEventListener("contextmenu", e => e.preventDefault());
});
$("stop").addEventListener("pointerdown", stop);
$("plus").addEventListener("click", () => setSpeed({delta: step}));
$("minus").addEventListener("click", () => setSpeed({delta: -step}));
$("speed").addEventListener("change", e => setSpeed({speed: +e.target.value}));

document.addEventListener("keydown", e => {
  if (KEYS[e.key]) { e.preventDefault(); start(KEYS[e.key]); }
  else if (e.key === " ") { e.preventDefault(); stop(); }
  else if (e.key === "+" || e.key === "=") setSpeed({delta: step});
  else if (e.key === "-" || e.key === "_") setSpeed({delta: -step});
});
document.addEventListener("keyup", e => { if (KEYS[e.key] === dir) stop(); });
window.addEventListener("blur", () => { if (dir) stop(); });  // 창을 벗어나면 keyup 을 못 받으므로 정지
document.addEventListener("visibilitychange", () => { if (document.hidden && dir) stop(); });

setInterval(() => api("/api/status").then(render), 300);
api("/api/status").then(render);
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    controller = None  # main 에서 지정

    def log_message(self, fmt, *args):
        pass  # 0.1초마다 오는 요청 로그는 생략

    def _json(self, code, obj):
        data = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(data)

    def _body(self):
        length = int(self.headers.get("Content-Length") or 0)
        if not length:
            return {}
        body = json.loads(self.rfile.read(length).decode("utf-8"))
        if not isinstance(body, dict):
            raise ValueError("JSON 객체가 필요합니다")
        return body

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            data = INDEX_HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        elif self.path == "/api/status":
            self._json(200, self.controller.status())
        else:
            self._json(404, {"error": "not found"})

    def do_POST(self):
        ctl = self.controller
        try:
            body = self._body()
            if self.path == "/api/move":
                direction = body.get("direction")
                if direction not in DIRECTIONS:
                    raise ValueError(f"direction 은 {', '.join(DIRECTIONS)} 중 하나")
                name, lin, ang = DIRECTIONS[direction]
                speed = body.get("speed", ctl.speed)
                speed = max(MIN_SPEED, min(int(speed), MAX_SPEED))
                ctl.move(name, lin * speed, ang * speed, body.get("duration"))
            elif self.path == "/api/velocity":
                ctl.move("속도지정", body.get("linear", 0), body.get("angular", 0), body.get("duration"))
            elif self.path == "/api/stop":
                ctl.stop()
            elif self.path == "/api/speed":
                if "speed" in body:
                    ctl.set_speed(body["speed"])
                elif "delta" in body:
                    ctl.set_speed(ctl.speed + int(body["delta"]))
                else:
                    raise ValueError("speed 또는 delta 가 필요합니다")
            else:
                self._json(404, {"error": "not found"})
                return
        except (ValueError, TypeError) as e:  # json.JSONDecodeError 도 ValueError
            self._json(400, {"error": str(e)})
            return
        self._json(200, {"ok": True, **ctl.status()})


def main():
    parser = argparse.ArgumentParser(description="car2 웹 UI / REST API 조종")
    parser.add_argument("--port", help="예: /dev/ttyS0, /dev/ttyUSB0")
    parser.add_argument("--dry-run", action="store_true", help="시리얼 없이 명령만 출력")
    parser.add_argument("--speed", type=int, default=800, help=f"시작 속도 (step/s, {MIN_SPEED}~{MAX_SPEED})")
    parser.add_argument("--host", default="0.0.0.0", help="HTTP 바인드 주소")
    parser.add_argument("--http-port", type=int, default=8766, help="HTTP 포트")
    args = parser.parse_args()
    if not args.port and not args.dry_run:
        parser.error("--port 또는 --dry-run 이 필요합니다")

    if args.dry_run:
        car = DryCar()
    else:
        car = Car(args.port)
        car.drain = lambda: serial_drain(car)

    ctl = Controller(car, max(MIN_SPEED, min(args.speed, MAX_SPEED)))
    Handler.controller = ctl
    server = ThreadingHTTPServer((args.host, args.http_port), Handler)
    server.daemon_threads = True
    print(f"웹 UI: http://{args.host}:{args.http_port}/  (Ctrl+C 종료)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        ctl.close()
        car.close()  # 종료 시 반드시 STOP
        print("종료 (STOP 전송)")


if __name__ == "__main__":
    main()
