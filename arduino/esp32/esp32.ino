// ESP32 WROOM-32D - WiFi 조종 브리지 (AP 모드)
// exam2.ino(L298N 모터 2개, on/off 제어)를 Serial2로 조종하는 웹 UI.
//   - 배선: ESP32 TX2(GPIO17) -> Arduino D7(SoftwareSerial), GND 공통
//           (하드웨어 Serial0/RX0은 USB/PC 디버그 전용으로 남기기 위해 분리함)
//           (ESP32 RX2(GPIO16)는 이 용도로는 사용하지 않음)
//   - 사용법: PID-CAR WiFi에 접속 후 http://192.168.4.1 접속
//             화면의 방향 버튼을 누르고 있거나 키보드 방향키를 누르고 있는 동안 이동,
//             떼면 자동 정지 (RC 조종 방식)
//
// 시리얼 프로토콜 (exam2.ino와 동일, PWM 없이 on/off 제어이므로 부호만 사용)
//   "L,<value>\n"  : 왼쪽 모터 방향 설정 (양수=정방향, 음수=역방향, 0=정지)
//   "R,<value>\n"  : 오른쪽 모터 방향 설정 (양수=정방향, 음수=역방향, 0=정지)
//   "STOP\n"       : 두 모터 모두 정지
#include <WiFi.h>
#include <WebServer.h>

const char* AP_SSID = "perpet-CAR";
const char* AP_PW   = "12345678";   // 8자 이상 필수

WebServer server(80);

// --- 통신 로그 (Serial2로 보낸 명령 기록, 최근 LOG_SIZE개 순환 버퍼) ---
const int LOG_SIZE = 20;
struct LogEntry {
  unsigned long time;
  String text;
};
LogEntry logBuf[LOG_SIZE];
int logCount = 0;
int logHead = 0; // 다음에 기록할 위치

void addLog(const String &text) {
  logBuf[logHead].time = millis();
  logBuf[logHead].text = text;
  logHead = (logHead + 1) % LOG_SIZE;
  if (logCount < LOG_SIZE) logCount++;
}

void sendSerial(const String &line) {
  Serial2.println(line);
  addLog(line);
}

const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>PID-CAR 조종</title>
<style>
  body { font-family: sans-serif; text-align: center; background:#111; color:#eee; margin:0; padding:20px; }
  h2 { margin-bottom: 4px; }
  #status { color:#8f8; margin-bottom: 20px; }
  .pad { display:grid; grid-template-columns: 80px 80px 80px; grid-template-rows: 80px 80px 80px; gap:8px; justify-content:center; }
  .pad button {
    font-size: 28px; border-radius: 12px; border: none; background:#333; color:#fff;
    touch-action: manipulation; user-select: none;
  }
  .pad button:active, .pad button.active { background:#0a84ff; }
  #up    { grid-column: 2; grid-row: 1; }
  #left  { grid-column: 1; grid-row: 2; }
  #stop  { grid-column: 2; grid-row: 2; background:#a00; }
  #right { grid-column: 3; grid-row: 2; }
  #down  { grid-column: 2; grid-row: 3; }
  #conn { margin-bottom: 12px; font-size: 14px; }
  #logBox { margin-top: 24px; text-align: left; max-width: 360px; margin-left: auto; margin-right: auto; }
  #logBox h3 { font-size: 14px; color: #aaa; margin-bottom: 4px; }
  #logPanel {
    height: 160px; overflow-y: auto; background:#000; border:1px solid #333; border-radius:6px;
    padding:6px 8px; font-family: monospace; font-size: 12px; color:#8f8;
  }
  #logPanel div { white-space: nowrap; }
</style>
</head>
<body>
<h2>PID-CAR 조종</h2>
<div id="conn">접속 상태 확인 중...</div>
<div id="status">방향키 또는 버튼을 누르고 있는 동안 이동합니다</div>
<div class="pad">
  <button id="up"    data-dir="F">▲</button>
  <button id="left"  data-dir="L">◀</button>
  <button id="stop"  data-dir="S">■</button>
  <button id="right" data-dir="R">▶</button>
  <button id="down"  data-dir="B">▼</button>
</div>
<div id="logBox">
  <h3>통신 로그</h3>
  <div id="logPanel"></div>
</div>
<script>
  let current = null;

  function send(dir) {
    fetch('/cmd?dir=' + dir).catch(() => {});
  }

  function press(dir, btn) {
    if (current === dir) return;
    current = dir;
    document.querySelectorAll('.pad button').forEach(b => b.classList.remove('active'));
    if (btn) btn.classList.add('active');
    send(dir);
  }

  function release() {
    if (current === null) return;
    current = null;
    document.querySelectorAll('.pad button').forEach(b => b.classList.remove('active'));
    send('S');
  }

  // 화면 버튼 (마우스 + 터치)
  document.querySelectorAll('.pad button').forEach(btn => {
    const dir = btn.dataset.dir;
    btn.addEventListener('mousedown', () => press(dir, btn));
    btn.addEventListener('touchstart', (e) => { e.preventDefault(); press(dir, btn); });
    btn.addEventListener('mouseup', release);
    btn.addEventListener('mouseleave', release);
    btn.addEventListener('touchend', release);
    btn.addEventListener('touchcancel', release);
  });

  // 키보드 방향키
  const keyMap = { ArrowUp: 'F', ArrowDown: 'B', ArrowLeft: 'L', ArrowRight: 'R' };
  document.addEventListener('keydown', (e) => {
    const dir = keyMap[e.key];
    if (!dir || e.repeat) return;
    e.preventDefault();
    press(dir, document.getElementById({F:'up',B:'down',L:'left',R:'right'}[dir]));
  });
  document.addEventListener('keyup', (e) => {
    if (keyMap[e.key]) { e.preventDefault(); release(); }
  });
  window.addEventListener('blur', release);

  // 접속 상태 표시 (ESP32 응답 여부, 접속 기기 수, 가동시간)
  function updateStatus() {
    fetch('/status').then(r => r.json()).then(d => {
      const conn = document.getElementById('conn');
      conn.textContent = `ESP32 연결됨 · 접속 기기 ${d.clients}대 · 가동시간 ${d.uptime}s`;
      conn.style.color = '#8f8';
    }).catch(() => {
      const conn = document.getElementById('conn');
      conn.textContent = 'ESP32 연결 끊김';
      conn.style.color = '#f88';
    });
  }

  // 통신 로그 표시 (ESP32 -> Arduino로 보낸 시리얼 명령, 최신순)
  function updateLog() {
    fetch('/log').then(r => r.json()).then(list => {
      const panel = document.getElementById('logPanel');
      panel.innerHTML = list.slice().reverse().map(e => {
        const sec = (e.t / 1000).toFixed(1);
        return `<div>[${sec}s] ${e.msg}</div>`;
      }).join('');
    }).catch(() => {});
  }

  updateStatus();
  updateLog();
  setInterval(updateStatus, 2000);
  setInterval(updateLog, 1000);
</script>
</body>
</html>
)rawliteral";

void sendMotor(const String &target, int value) {
  sendSerial(target + "," + String(value));
}

void handleRoot() {
  server.send_P(200, "text/html", PAGE);
}

void handleStatus() {
  String json = "{\"clients\":" + String(WiFi.softAPgetStationNum()) +
                ",\"uptime\":" + String(millis() / 1000) + "}";
  server.send(200, "application/json", json);
}

void handleLog() {
  String json = "[";
  int start = (logCount < LOG_SIZE) ? 0 : logHead;
  for (int i = 0; i < logCount; i++) {
    int idx = (start + i) % LOG_SIZE;
    if (i > 0) json += ",";
    json += "{\"t\":" + String(logBuf[idx].time) + ",\"msg\":\"" + logBuf[idx].text + "\"}";
  }
  json += "]";
  server.send(200, "application/json", json);
}

void handleCmd() {
  String dir = server.arg("dir");

  if (dir == "F") {
    sendMotor("L", 1);
    sendMotor("R", 1);
  } else if (dir == "B") {
    sendMotor("L", -1);
    sendMotor("R", -1);
  } else if (dir == "L") {
    sendMotor("L", -1);
    sendMotor("R", 1);
  } else if (dir == "R") {
    sendMotor("L", 1);
    sendMotor("R", -1);
  } else if (dir == "S") {
    sendSerial("STOP");
  } else {
    server.send(400, "text/plain; charset=utf-8", "잘못된 방향");
    return;
  }

  server.send(200, "text/plain; charset=utf-8", dir);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);  // RX2=16(비움), TX2=17 -> Uno D7 (SoftwareSerial, 저속 고정)

  WiFi.softAP(AP_SSID, AP_PW);
  Serial.print("접속 주소: http://");
  Serial.println(WiFi.softAPIP());          // 기본 192.168.4.1

  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.on("/status", handleStatus);
  server.on("/log", handleLog);
  server.begin();
}

void loop() {
  server.handleClient();
}
