#!/usr/bin/env python3
"""
bot_modules/depth_stream_server.py
-----------------------------------
웹 브라우저로 실시간 영상(MJPEG) 스트리밍, 대시보드 웹 UI,
스냅샷 캡처 및 VLM AI 분석 결과 REST API를 제공하는 멀티스레드 서버 모듈
"""

import os
import sys
import time
import json
import socket
import urllib.parse
import threading
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import cv2


def get_local_ip():
    """현재 기기의 실제 Wi-Fi / 이더넷 IP 자동 감지"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return '127.0.0.1'


class SharedStreamState:
    """스레드 안전한 실시간 영상 프레임 및 VLM 결과 공유 컨텍스트"""
    def __init__(self, snapshot_dir=None):
        self.frame_lock = threading.Lock()
        self.output_frame = None

        self.vlm_lock = threading.Lock()
        self.vlm_result = {
            "object": "스냅샷 캡처 대기 중",
            "is_trash": None,
            "dist_cm": 0,
            "details": "전방 물체 캡처 후 AI에게 '앞에 있는 거 분석해줘'라고 요청하세요.",
            "timestamp": time.time(),
            "status": "idle"
        }

        self.capture_lock = threading.Lock()
        self.last_capture_time = 0.0
        self.last_cached_snapshot = None
        self.cooldown_sec = 10.0

        if snapshot_dir is None:
            cur_dir = os.path.dirname(os.path.abspath(__file__))
            ws_dir = os.path.dirname(cur_dir)
            self.snapshot_dir = os.path.join(ws_dir, "snapshots")
        else:
            self.snapshot_dir = snapshot_dir
        self.snapshot_file = os.path.join(self.snapshot_dir, "latest_object.jpg")

    def set_frame(self, frame):
        with self.frame_lock:
            self.output_frame = frame.copy() if frame is not None else None

    def get_frame(self):
        with self.frame_lock:
            return self.output_frame.copy() if self.output_frame is not None else None

    def trigger_snapshot(self, reason="manual"):
        """스냅샷 캡처 및 VLM 상태 초기화 (10초 쿨다운 적용)"""
        now = time.time()
        with self.capture_lock:
            elapsed = now - self.last_capture_time
            if elapsed < self.cooldown_sec:
                rem = round(self.cooldown_sec - elapsed, 1)
                return False, rem

            self.last_capture_time = now
            curr = self.get_frame()
            if curr is not None:
                os.makedirs(self.snapshot_dir, exist_ok=True)
                cv2.imwrite(self.snapshot_file, curr)
                ret, jpeg = cv2.imencode('.jpg', curr, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
                if ret:
                    self.last_cached_snapshot = jpeg.tobytes()

            with self.vlm_lock:
                self.vlm_result["object"] = "📸 방금 캡처됨 (AI 분석 대기 중...)"
                self.vlm_result["is_trash"] = None
                self.vlm_result["details"] = "새 스냅샷 저장 완료. AI 에이전트에게 '앞에 있는 거 분석해줘'라고 요청하세요."
                self.vlm_result["timestamp"] = now
                self.vlm_result["status"] = "captured"

            return True, self.cooldown_sec


# 전역 기본 공유 상태
stream_state = SharedStreamState()


class VideoStreamHandler(BaseHTTPRequestHandler):
    """웹 브라우저로 실시간 영상(MJPEG) 및 AI 분석 API 제공"""
    
    def do_POST(self):
        clean_path = urllib.parse.urlparse(self.path).path
        if clean_path in ('/api/vlm_result', '/api/vlm'):
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length)
            try:
                data = json.loads(body.decode('utf-8'))
                with stream_state.vlm_lock:
                    stream_state.vlm_result.update({
                        "object": data.get("object", stream_state.vlm_result.get("object")),
                        "is_trash": data.get("is_trash", stream_state.vlm_result.get("is_trash")),
                        "dist_cm": data.get("dist_cm", stream_state.vlm_result.get("dist_cm")),
                        "details": data.get("details", stream_state.vlm_result.get("details")),
                        "status": "completed",
                        "timestamp": time.time()
                    })
                resp = json.dumps({"status": "success", "data": stream_state.vlm_result}, ensure_ascii=False).encode('utf-8')
                self.send_response(200)
            except Exception as e:
                resp = json.dumps({"status": "error", "message": str(e)}).encode('utf-8')
                self.send_response(400)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(resp)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Connection', 'close')
            self.end_headers()
            self.wfile.write(resp)
            return

        self.send_response(404)
        self.send_header('Content-Type', 'text/plain; charset=utf-8')
        self.send_header('Connection', 'close')
        self.end_headers()
        self.wfile.write(b"404 Not Found")

    def do_GET(self):
        clean_path = urllib.parse.urlparse(self.path).path
        if clean_path in ('/', '/index.html'):
            html = self._render_html_dashboard()
            html_bytes = html.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(html_bytes)))
            self.send_header('Connection', 'close')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(html_bytes)
            return

        if clean_path == '/favicon.ico':
            self.send_response(204)
            self.send_header('Connection', 'close')
            self.end_headers()
            return

        # VLM 결과 조회 API
        if clean_path in ('/api/vlm_result', '/api/vlm'):
            with stream_state.vlm_lock:
                payload = json.dumps(stream_state.vlm_result, ensure_ascii=False).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Connection', 'close')
            self.end_headers()
            self.wfile.write(payload)
            return

        # 스냅샷 캡처 트리거 API (10초 쿨다운 적용)
        if clean_path in ('/api/trigger_inspect', '/api/inspect'):
            success, rem = stream_state.trigger_snapshot(reason="api")
            if not success:
                resp = json.dumps({
                    "status": "cooldown",
                    "cooldown": True,
                    "remaining": rem,
                    "message": f"캡처 쿨다운 중입니다 ({rem}초 남음)"
                }, ensure_ascii=False).encode('utf-8')
            else:
                resp = json.dumps({
                    "status": "success",
                    "cooldown": False,
                    "remaining": stream_state.cooldown_sec,
                    "path": stream_state.snapshot_file,
                    "message": "캡처 완료 (향후 10초간 쿨다운 적용)"
                }, ensure_ascii=False).encode('utf-8')

            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(resp)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Connection', 'close')
            self.end_headers()
            self.wfile.write(resp)
            return

        # 스냅샷 서빙 (10초 쿨다운 캐싱)
        if clean_path in ('/snapshot.jpg', '/snapshot', '/capture', '/capture.jpg', '/latest.jpg'):
            with stream_state.capture_lock:
                now = time.time()
                elapsed = now - stream_state.last_capture_time
                if elapsed >= stream_state.cooldown_sec or stream_state.last_cached_snapshot is None:
                    curr = stream_state.get_frame()
                    if curr is not None:
                        ret, jpeg = cv2.imencode('.jpg', curr, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
                        if ret:
                            stream_state.last_cached_snapshot = jpeg.tobytes()
                            stream_state.last_capture_time = now
                            os.makedirs(stream_state.snapshot_dir, exist_ok=True)
                            with open(stream_state.snapshot_file, "wb") as f:
                                f.write(stream_state.last_cached_snapshot)
                data = stream_state.last_cached_snapshot

            if data is not None:
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Connection', 'close')
                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                self.end_headers()
                self.wfile.write(data)
                return
            self.send_response(503)
            self.end_headers()
            return

        # MJPEG 실시간 스트리밍 피드
        if clean_path in ('/stream.mjpg', '/stream', '/video'):
            self.send_response(200)
            self.send_header('Age', '0')
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()

            while True:
                curr = stream_state.get_frame()
                if curr is None:
                    time.sleep(0.05)
                    continue

                ret, jpeg = cv2.imencode('.jpg', curr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if not ret:
                    time.sleep(0.02)
                    continue

                data = jpeg.tobytes()
                try:
                    self.wfile.write(b'--frame\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                    self.wfile.write(b'\r\n')
                    time.sleep(0.035)
                except (BrokenPipeError, ConnectionResetError):
                    break
                except Exception:
                    break
            return

        err_body = b"404 Not Found"
        self.send_response(404)
        self.send_header('Content-Type', 'text/plain; charset=utf-8')
        self.send_header('Content-Length', str(len(err_body)))
        self.send_header('Connection', 'close')
        self.end_headers()
        self.wfile.write(err_body)

    def log_message(self, format, *args):
        pass  # 불필요한 HTTP 액세스 로그 억제

    def _render_html_dashboard(self):
        return """<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>AMR 실시간 RGB-D 융합 물체 분리 뷰어</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { background: #0e0e10; color: white; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; text-align: center; margin: 0; padding: 15px; }
        h1 { font-size: 22px; margin-bottom: 6px; color: #00d2ff; }
        .stream-box { display: inline-block; margin-top: 10px; background: #1a1a24; border-radius: 12px; padding: 6px; }
        .stream { border: 2px solid #00d2ff; border-radius: 8px; max-width: 98%; height: auto; box-shadow: 0 6px 20px rgba(0, 210, 255, 0.3); display: block; }
        .legend { display: inline-flex; align-items: center; justify-content: center; gap: 15px; margin-top: 10px; padding: 8px 18px; background: #1a1d28; border-radius: 8px; font-size: 14px; }
        .dot { width: 14px; height: 14px; border-radius: 3px; display: inline-block; }
        .dot-blue { background: #00aaff; }
        .dot-green { background: #00ff66; }
        .dot-red { background: #ff2255; }
        
        .vlm-card { margin: 14px auto; max-width: 820px; background: #161b26; border-radius: 12px; padding: 14px 20px; border: 1px solid #26334d; box-shadow: 0 4px 20px rgba(0,0,0,0.5); text-align: left; }
        .vlm-header { display: flex; justify-content: space-between; align-items: center; border-bottom: 1px solid #232c40; padding-bottom: 8px; }
        .vlm-title { font-size: 16px; font-weight: bold; color: #a277ff; }
        .vlm-btn { background: #00d2ff; color: #0a0e17; border: none; padding: 6px 14px; border-radius: 6px; font-weight: bold; cursor: pointer; transition: 0.2s; }
        .vlm-btn:hover { background: #00b4db; transform: scale(1.02); }
        .vlm-grid { display: grid; grid-template-columns: 1.1fr 1fr 1.6fr; gap: 12px; margin-top: 10px; }
        .vlm-item { background: #1e2638; border-radius: 8px; padding: 10px; border: 1px solid #2a3650; }
        .vlm-label { font-size: 12px; color: #8892b0; margin-bottom: 4px; }
        .vlm-val { font-size: 15px; font-weight: bold; color: #eceff4; }
        .badge-safe { color: #00ff66; font-weight: bold; }
        .badge-trash { color: #ff3366; font-weight: bold; }
        
        .guide { margin-top: 12px; font-size: 13px; color: #8892b0; line-height: 1.6; }
        kbd { background: #2a3650; padding: 2px 6px; border-radius: 4px; color: #00d2ff; font-weight: bold; }
    </style>
</head>
<body>
    <h1>🤖 AMR 실시간 RGB-D 융합 바닥 시각화 & 물체 분리</h1>
    <div class="legend">
        <span><i class="dot dot-blue"></i> <b>푸른색 영역: 인식된 바닥 (Floor)</b></span>
        <span><i class="dot dot-green"></i> <b>녹색 외곽선: 분리된 물체 (Object)</b></span>
        <span><i class="dot dot-red"></i> <b>빨간 박스: 감지 바운딩 박스</b></span>
    </div>

    <!-- AI VLM 커스텀 스킬 분석 카드 -->
    <div class="vlm-card">
        <div class="vlm-header">
            <span class="vlm-title">✨ AI 시각 지능(VLM) 실시간 물체 식별 & 쓰레기 판별</span>
            <div style="display: flex; gap: 8px; align-items: center;">
                <span class="badge-safe" style="font-size: 12px; background: rgba(0, 255, 102, 0.12); padding: 4px 10px; border-radius: 6px; border: 1px solid rgba(0, 255, 102, 0.3);">⚡ 자동 감지 캡처 파이프라인 가동 중</span>
                <button class="vlm-btn" id="inspectBtn" onclick="triggerInspect()">🔍 수동 즉시 캡처</button>
            </div>
        </div>
        <div class="vlm-grid">
            <div class="vlm-item">
                <div class="vlm-label">식별된 물체 명칭</div>
                <div class="vlm-val" id="vlmObj">로딩 중...</div>
            </div>
            <div class="vlm-item">
                <div class="vlm-label">쓰레기 여부 판정</div>
                <div class="vlm-val" id="vlmTrash">로딩 중...</div>
            </div>
            <div class="vlm-item">
                <div class="vlm-label">파지/처리 권장 사항</div>
                <div class="vlm-val" id="vlmDetails">로딩 중...</div>
            </div>
        </div>
    </div>

    <div class="stream-box">
        <img class="stream" src="/stream.mjpg" alt="실시간 카메라 영상 로딩 중..." />
    </div>
    <div class="guide">
        💡 조작: <kbd>[a]</kbd> 자동 캡처 ON/OFF | <kbd>[d]</kbd> 3D 단차(Depth Sobel) | <kbd>[i]</kbd> 수동 캡처 | <kbd>[b]</kbd> 바닥 시각화 | <kbd>[[] / []]</kbd> 바닥 감도 | <kbd>[t]</kbd> 바닥 영점 | <kbd>[+] / [-]</kbd> 뎁스 감도 | <kbd>[f]</kbd> 융합 모드 | <kbd>[q]</kbd> 종료
        <br/>🤖 <b>자율 AI 파이프라인 가동:</b> 전방에 물체가 감지되면 10초마다 자동 스냅샷을 캡처하고 백그라운드 AI VLM 데몬이 즉시 분석하여 결과를 실시간 갱신합니다.
    </div>

    <script>
        function updateVlmCard() {
            fetch('/api/vlm_result')
                .then(r => r.json())
                .then(data => {
                    if (data.object) {
                        document.getElementById('vlmObj').textContent = data.object;
                        const trashEl = document.getElementById('vlmTrash');
                        if (data.is_trash === true) {
                            trashEl.innerHTML = '<span class="badge-trash">🗑️ 쓰레기 (폐기 대상)</span>';
                        } else if (data.is_trash === false) {
                            trashEl.innerHTML = '<span class="badge-safe">💎 유효 물품/공구 (쓰레기 아님)</span>';
                        } else {
                            trashEl.innerHTML = '<span style="color: #f6c177; font-weight: bold;">⏳ AI 분석 대기 중</span>';
                        }
                        document.getElementById('vlmDetails').textContent = data.details || '-';
                    }
                }).catch(() => {});
        }
        setInterval(updateVlmCard, 1500);
        updateVlmCard();

        function triggerInspect() {
            const btn = document.getElementById('inspectBtn');
            btn.disabled = true;
            btn.textContent = "캡처 요청 중...";
            document.getElementById('vlmObj').textContent = "📸 방금 캡처됨 (AI 분석 대기 중...)";
            document.getElementById('vlmTrash').innerHTML = '<span style="color: #f6c177; font-weight: bold;">⏳ AI 분석 대기 중</span>';
            document.getElementById('vlmDetails').textContent = "채팅창에 '앞에 물체 분석해줘'라고 요청하시면 즉시 갱신됩니다.";

            fetch('/api/trigger_inspect')
                .then(r => r.json())
                .then(res => {
                    let rem = res.remaining ? Math.ceil(res.remaining) : 10;
                    btn.textContent = "⏳ 쿨다운 (" + rem + "초)";
                    const cdTimer = setInterval(() => {
                        rem--;
                        if (rem <= 0) {
                            clearInterval(cdTimer);
                            btn.disabled = false;
                            btn.textContent = "🔍 스냅샷 캡처 요청";
                        } else {
                            btn.textContent = "⏳ 쿨다운 (" + rem + "초)";
                        }
                    }, 1000);
                }).catch(() => {
                    btn.disabled = false;
                    btn.textContent = "🔍 스냅샷 캡처 요청";
                });
        }
    </script>
</body>
</html>"""


def start_web_server(port=8080):
    """지정 포트(또는 가용 포트)로 백그라운드 웹 스트리밍 서버 기동"""
    server = None
    actual_port = port
    local_ip = get_local_ip()
    for p in range(port, port + 10):
        try:
            server = ThreadingHTTPServer(('0.0.0.0', p), VideoStreamHandler)
            server.daemon_threads = True
            actual_port = p
            break
        except OSError:
            continue

    if server:
        print(f"🌐 [웹 스트리밍 준비 완료] 👉 http://{local_ip}:{actual_port}")
        server.serve_forever()
    else:
        print(f"❌ [웹 서버 에러] 포트 {port}~{port+9}를 열 수 없습니다.")
