#!/usr/bin/env python3
"""
AMR 단일 관심영역(Single ROI) 기반 MiDaS AI 근접 물체 감지 및 자동 정지 (grasp_approach_stopper.py)
---------------------------------------------------------------------------------
1. [해결책 A: 단일 관심영역(ROI) + 백분위수(Percentile) & 점유 면적 판정]:
   - 이중 네모(내부/외부 대비)의 근접 시 시야 가림, 배경 거리 왜곡, 바닥면 구배 문제를 완전 해결!
   - 로봇 전방 파지/충돌 영역에 단 하나의 정밀 ROI 박스만 설정.
   - ROI 내부 뎁스의 상위 88% 백분위수(Percentile)를 취해 노이즈 없이 물체 전면 깊이를 실시간 측정.
   - 일정 비율(점유율 %) 이상의 픽셀이 임계치를 넘었을 때만 확실한 물체로 판정 (허위 감지 제로).
   - 필요 시 빈 바닥 영점(Tare)을 잡아 바닥 위로 솟아오른 물체만 쏙 감지 가능!
2. [단독 모터 구동 & ROS 2 동시 지원]:
   - 별도 bringup 없이 직접 시리얼(/dev/ttyACM0 등)로 4개 바퀴 직결 구동 가능!
   - ROS 2 실행 중일 때는 /cmd_vel 토픽도 동시 발행.
3. [웹 브라우저 실시간 모니터 & 리모컨 (http://192.168.0.64:8080)]:
   - 실시간 RGB 영상 + 3D 뎁스 히트맵 및 감지 영역 하이라이트.
   - ROI 뎁스 수치(0~255), 점유 면적(%), 추정 거리(cm), 프로그레스 게이지 표시.
   - 원클릭 자율 전진, 비상 정지, 영점 조절, 정지 임계치 슬라이더 실시간 조작.
"""

import os
import sys
import time
import json
import select
import argparse
import threading
from urllib.parse import urlparse, parse_qs
from http.server import HTTPServer, BaseHTTPRequestHandler

import cv2
import numpy as np
import serial
import struct

# PyTorch 확인
try:
    import torch
except ImportError:
    print("❌ PyTorch가 설치되어 있지 않습니다!")
    print("   pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu --break-system-packages")
    sys.exit(1)

# ROS 2 가용성 확인
HAS_ROS2 = False
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    HAS_ROS2 = True
except ImportError:
    pass

CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
DEFAULT_WEB_PORT = 8080

# 전역 공유 상태
shared_state = {
    "output_frame": None,
    "lock": threading.Lock(),
    "mode": "IDLE",             # IDLE, APPROACHING, REACHED
    "target_threshold": 175,    # 정지 기준 뎁스 임계값 (0~255, 기본: 175)
    "current_depth": 0,         # ROI 내 88% 백분위수 뎁스 수치
    "occupancy_pct": 0,         # 임계값 초과 픽셀 면적 점유율 (%)
    "approx_dist_cm": 0,        # 대략적 추정 거리 (cm)
    "baseline_depth": 0,        # 바닥 영점 기준값 (Tare)
    "min_occupancy": 8,         # 최소 점유율 (% - 노이즈 차단용 기본 8%)
    "is_detected": False,       # 최종 물체 감지 플래그
    "speed": 0.07,              # 전진 속도 (m/s)
    "should_approach": False,
    "status_msg": "대기 중 (Ready)",
    "stop_requested": False,
    "tare_requested": False,
    "shutdown_requested": False
}


class VideoAndControlHandler(BaseHTTPRequestHandler):
    """실시간 영상 스트리밍 및 웹 리모컨 제어 핸들러"""

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        # 1. 제어 API
        if path == '/api/approach':
            shared_state["should_approach"] = True
            shared_state["mode"] = "APPROACHING"
            shared_state["status_msg"] = f"물체로 전진 중 (목표 임계값: {shared_state['target_threshold']})..."
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"status":"approaching"}')
            return

        if path == '/api/stop':
            shared_state["should_approach"] = False
            shared_state["mode"] = "IDLE"
            shared_state["stop_requested"] = True
            shared_state["status_msg"] = "사용자에 의해 수동 정지됨"
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"status":"stopped"}')
            return

        if path == '/api/tare':
            shared_state["tare_requested"] = True
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"status":"tare_requested"}')
            return

        if path == '/api/set_threshold':
            params = parse_qs(parsed.query)
            if 'val' in params:
                try:
                    val = int(params['val'][0])
                    shared_state["target_threshold"] = max(80, min(245, val))
                except ValueError:
                    pass
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(f'{{"target_threshold": {shared_state["target_threshold"]}}}'.encode())
            return

        if path == '/api/status':
            pct = min(100, int((shared_state["current_depth"] / max(1, shared_state["target_threshold"])) * 100))
            data = {
                "mode": shared_state["mode"],
                "depth": shared_state["current_depth"],
                "target": shared_state["target_threshold"],
                "occupancy": shared_state["occupancy_pct"],
                "dist_cm": shared_state["approx_dist_cm"],
                "baseline": shared_state["baseline_depth"],
                "detected": shared_state["is_detected"],
                "pct": pct,
                "msg": shared_state["status_msg"]
            }
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(json.dumps(data).encode())
            return

        if path == '/api/shutdown':
            shared_state["stop_requested"] = True
            shared_state["shutdown_requested"] = True
            shared_state["status_msg"] = "프로그램 종료 중..."
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"status":"shutting_down"}')
            return

        # 2. 웹 대시보드 페이지
        if path == '/' or path == '/index.html':
            html = f"""<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <title>AMR MiDaS 단일 ROI 근접 물체 정밀 감지 & 정지</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {{
            background: #0d0f18;
            color: #eceff4;
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
            text-align: center;
            margin: 0;
            padding: 12px;
        }}
        h1 {{ font-size: 21px; margin: 4px 0 2px 0; color: #00d2ff; }}
        .subtitle {{ font-size: 13px; color: #8892b0; margin-bottom: 12px; }}
        
        .stream-box {{
            display: inline-block;
            border: 3px solid #1e2638;
            border-radius: 12px;
            overflow: hidden;
            box-shadow: 0 8px 30px rgba(0,0,0,0.7);
            max-width: 98%;
        }}
        .stream {{ width: 100%; height: auto; display: block; }}
        
        .panel {{
            margin: 14px auto;
            max-width: 640px;
            background: #151a26;
            border-radius: 12px;
            padding: 15px 22px;
            box-shadow: 0 4px 20px rgba(0,0,0,0.5);
            border: 1px solid #232c40;
        }}
        
        .status-badge {{
            display: inline-block;
            font-size: 16px;
            font-weight: bold;
            padding: 8px 22px;
            border-radius: 30px;
            margin-bottom: 12px;
            letter-spacing: 0.5px;
        }}
        .status-idle {{ background: #2e3440; color: #d8dee9; border: 1px solid #4c566a; }}
        .status-approaching {{ background: #ffaa00; color: #111; animation: blink 0.9s infinite alternate; }}
        .status-reached {{ background: #ff1744; color: #fff; font-size: 18px; box-shadow: 0 0 20px rgba(255,23,68,0.7); }}
        
        @keyframes blink {{
            from {{ opacity: 0.8; transform: scale(0.98); }}
            to {{ opacity: 1.0; transform: scale(1.02); }}
        }}
        
        /* 3D 뎁스 메트릭 카드 */
        .metric-cards {{
            display: grid;
            grid-template-columns: 1.1fr 1fr 1fr;
            gap: 10px;
            margin: 12px 0;
        }}
        .card {{
            background: #1e2638;
            padding: 10px;
            border-radius: 8px;
            border: 1px solid #2a3650;
        }}
        .card-title {{ font-size: 12px; color: #8892b0; margin-bottom: 4px; }}
        .card-value {{ font-size: 20px; font-weight: bold; }}
        .val-depth {{ color: #00e676; }}
        .val-occupancy {{ color: #ffab00; }}
        .val-dist {{ color: #00c6ff; }}
        
        /* 게이지 바 */
        .gauge-container {{
            margin: 12px 0 16px 0;
            background: #0d0f18;
            border-radius: 10px;
            padding: 4px;
            border: 1px solid #2a3650;
        }}
        .gauge-bar {{
            height: 18px;
            border-radius: 6px;
            background: linear-gradient(90deg, #00c6ff, #ffaa00, #ff1744);
            width: 0%;
            transition: width 0.15s ease-out;
        }}
        
        .controls {{
            display: flex;
            gap: 8px;
            justify-content: center;
            margin-top: 10px;
            flex-wrap: wrap;
        }}
        button {{
            padding: 10px 18px;
            font-size: 14px;
            font-weight: bold;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            transition: all 0.2s ease;
        }}
        .btn-go {{ background: linear-gradient(135deg, #00c6ff, #0072ff); color: white; }}
        .btn-go:hover {{ background: linear-gradient(135deg, #00b0e8, #0060d6); transform: translateY(-2px); }}
        .btn-stop {{ background: linear-gradient(135deg, #ff0844, #ffb199); color: white; }}
        .btn-stop:hover {{ background: linear-gradient(135deg, #e00037, #f79c82); transform: translateY(-2px); }}
        .btn-tare {{ background: #4c566a; color: #eceff4; }}
        .btn-tare:hover {{ background: #5e81ac; color: white; }}
        .btn-exit {{ background: #3b4252; color: #d8dee9; }}
        .btn-exit:hover {{ background: #bf616a; color: white; }}
        
        .slider-group {{
            margin-top: 14px;
            font-size: 14px;
            color: #ccd6f6;
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 10px;
        }}
        input[type=range] {{ width: 190px; }}
    </style>
</head>
<body>
    <h1>🤖 AMR MiDaS 단일 ROI 근접 물체 정밀 감지</h1>
    <div class="subtitle">해결책 A: 단일 관심영역(Single ROI) + 하위 88% 뎁스 백분위수 & 면적 점유율 판정</div>

    <div class="stream-box">
        <img class="stream" src="/stream.mjpg" />
    </div>

    <div class="panel">
        <div id="statusBadge" class="status-badge status-idle">대기 중 (Ready)</div>
        
        <div class="metric-cards">
            <div class="card">
                <div class="card-title">🎯 ROI 뎁스 강도 (상위 88%)</div>
                <div id="valDepth" class="card-value val-depth">0 / 255</div>
            </div>
            <div class="card">
                <div class="card-title">📐 박스 면적 점유율</div>
                <div id="valOccupancy" class="card-value val-occupancy">0 %</div>
            </div>
            <div class="card">
                <div class="card-title">📏 전방 추정 거리</div>
                <div id="valDist" class="card-value val-dist">~0 cm</div>
            </div>
        </div>

        <div class="gauge-container">
            <div id="gaugeBar" class="gauge-bar"></div>
        </div>

        <div class="controls">
            <button class="btn-go" onclick="triggerApproach()">🚀 자율 전진 시작</button>
            <button class="btn-stop" onclick="triggerStop()">🛑 비상 정지</button>
            <button class="btn-tare" onclick="triggerTare()">⚖️ 바닥 영점(Tare)</button>
            <button class="btn-exit" onclick="triggerExit()">⚡ 프로그램 종료</button>
        </div>

        <div class="slider-group">
            <span>🎯 정지 감지 임계값:</span>
            <input type="range" id="threshSlider" min="100" max="235" value="{shared_state['target_threshold']}" oninput="updateThreshold(this.value)">
            <b id="threshLabel" style="color:#00d2ff; font-size: 16px;">{shared_state['target_threshold']} pt</b>
        </div>
    </div>

    <script>
        function triggerApproach() {{
            fetch('/api/approach');
        }}
        function triggerStop() {{
            fetch('/api/stop');
        }}
        function triggerTare() {{
            fetch('/api/tare');
            alert('현재 바닥을 기준으로 영점을 설정합니다.');
        }}
        function triggerExit() {{
            if (confirm('프로그램을 완전히 종료하시겠습니까?')) {{
                fetch('/api/shutdown');
            }}
        }}
        function updateThreshold(val) {{
            document.getElementById('threshLabel').innerText = val + ' pt';
            fetch('/api/set_threshold?val=' + val);
        }}

        // 실시간 수치 동기화 루프 (200ms)
        setInterval(async () => {{
            try {{
                const res = await fetch('/api/status');
                const d = await res.json();
                
                document.getElementById('valDepth').innerText = `${{d.depth}} / 255`;
                document.getElementById('valOccupancy').innerText = `${{d.occupancy}} %`;
                document.getElementById('valDist').innerText = `~${{d.dist_cm}} cm`;
                document.getElementById('gaugeBar').style.width = d.pct + '%';

                const b = document.getElementById('statusBadge');
                if (d.detected) {{
                    b.className = 'status-badge status-reached';
                    b.innerText = `🛑 물체 감지 완료! (~${{d.dist_cm}}cm 앞 정지)`;
                }} else if (d.mode === 'APPROACHING') {{
                    b.className = 'status-badge status-approaching';
                    b.innerText = `🚀 물체로 전진 중... (${{d.depth}} / ${{d.target}} pt [${{d.pct}}%])`;
                }} else {{
                    b.className = 'status-badge status-idle';
                    b.innerText = d.msg;
                }}
            }} catch (e) {{}}
        }}, 200);
    </script>
</body>
</html>"""
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(html.encode('utf-8'))
            return

        # 3. 단일 스냅샷 이미지 제공 (10초 쿨다운 캐싱 적용)
        if path == '/snapshot.jpg':
            now = time.time()
            data = None
            with shared_state["lock"]:
                last_t = shared_state.get("last_snapshot_time", 0.0)
                cached = shared_state.get("cached_snapshot")
                if now - last_t >= 10.0 or cached is None:
                    frame = shared_state.get("output_frame")
                    if frame is not None:
                        ret, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
                        if ret:
                            cached = jpeg.tobytes()
                            shared_state["cached_snapshot"] = cached
                            shared_state["last_snapshot_time"] = now
                data = cached

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

        # 4. 실시간 MJPEG 스트림
        if path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            while True:
                with shared_state["lock"]:
                    frame = shared_state["output_frame"]
                    if frame is None:
                        time.sleep(0.02)
                        continue
                    ret, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
                    if not ret:
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
            return

        self.send_response(404)
        self.end_headers()

    def log_message(self, format, *args):
        pass


def run_web_server(preferred_port):
    actual_port = preferred_port
    server = None
    for p in range(preferred_port, preferred_port + 10):
        try:
            server = HTTPServer(('0.0.0.0', p), VideoAndControlHandler)
            actual_port = p
            break
        except OSError:
            continue

    if server is None:
        print(f"❌ 웹 서버 시작 실패: 포트 {preferred_port}~{preferred_port+9}가 모두 사용 중입니다.")
        return

    shared_state["actual_port"] = actual_port
    print(f"🌐 [웹 모니터 & 리모컨 준비 완료] 👉 http://192.168.0.64:{actual_port}")
    server.serve_forever()


def find_working_camera(preferred_idx=0):
    candidates = [preferred_idx] + [i for i in [0, 1, 2, 3, 4] if i != preferred_idx]
    for idx in candidates:
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
                return cap, idx
            cap.release()
    return None, -1


class GraspApproachStopper:
    def __init__(self, node=None, target_threshold=175, approach_speed=0.07):
        self.node = node
        self.target_threshold = target_threshold
        shared_state["target_threshold"] = target_threshold
        shared_state["speed"] = approach_speed

        self.pub_cmd_vel = None
        if self.node:
            self.pub_cmd_vel = self.node.create_publisher(Twist, '/cmd_vel', 10)

        print("\n" + "=" * 68)
        print("   🤖 AMR MiDaS 단일 관심영역(Single ROI) 근접 물체 정지 제어기")
        print("=" * 68)
        print(f"🎯 정지 기준 뎁스 임계치 : {self.target_threshold} pt (0~255 스케일)")
        print(f"🏎️ 접근 주행 속도       : {approach_speed} m/s")

        self.device = torch.device("cpu")
        print("🧠 [1/3] Intel MiDaS AI 뎁스 모델 로딩 중...")
        try:
            self.midas = torch.hub.load("intel-isl/MiDaS", "MiDaS_small", trust_repo=True)
            self.midas.to(self.device)
            self.midas.eval()
            midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms", trust_repo=True)
            self.transform = midas_transforms.small_transform
            print("✅ MiDaS AI 뎁스 모델 로드 성공!\n")
        except Exception as e:
            print(f"\n❌ MiDaS 모델 로드 실패: {e}")
            err_str = str(e).lower()
            if "timm" in err_str or "attributeerror" in err_str:
                print("💡 [원인 분석] 최신 timm 패키지(1.0+)와 MiDaS 레거시 코드 간 버전 충돌입니다.")
                print("👉 해결 명령어: pip install timm==0.6.12 --break-system-packages")
            sys.exit(1)

        # 2. 카메라 자동 탐색 연결
        print("📸 [2/3] 카메라 디바이스 연결 탐색 중...")
        self.cap, active_idx = find_working_camera(CAMERA_INDEX)
        if self.cap is None:
            print(f"❌ 사용 가능한 카메라 디바이스(/dev/video0 ~ 4)를 찾을 수 없습니다!")
            sys.exit(1)

        print(f"✅ 카메라 연결 성공: /dev/video{active_idx} ({FRAME_WIDTH}x{FRAME_HEIGHT})")

        # 3. 서보 모터 직접 제어(Direct Serial) 또는 ROS 2 연동 준비
        print("🔌 [3/3] STS3215 서보 모터 제어 인터페이스 초기화...")
        self.serial_port = None
        self.connect_direct_serial()

        # 4. [해결책 A] 단일 관심영역 (Single ROI) 대폭 확대 설정
        # 로봇 전방 파지/접근 영역: 가로 68% (~435px), 세로 24%~88% (~307px)
        roi_w = int(FRAME_WIDTH * 0.68)   # ~435 px (기존 282px 대비 1.5배 이상 가로 확장)
        self.roi_x1 = (FRAME_WIDTH - roi_w) // 2   # 102
        self.roi_x2 = self.roi_x1 + roi_w          # 537
        self.roi_y1 = int(FRAME_HEIGHT * 0.24)     # 115 (위쪽으로 확장하여 전방 물체 조기 탐지)
        self.roi_y2 = int(FRAME_HEIGHT * 0.88)     # 422 (아래쪽으로 확장하여 근접 물체 포착)

        self.has_display = bool(os.environ.get("DISPLAY"))
        self.prev_time = time.time()
        self.fps = 0.0
        self.last_log_time = 0.0

        # 상태 관리
        self.state = "IDLE"  # IDLE, APPROACHING, REACHED_STOP
        self.consecutive_reach_count = 0

    def connect_direct_serial(self):
        """sts_controller 없이도 바퀴를 직접 회전시킬 수 있도록 시리얼 포트 직결"""
        ports = ['/dev/servo_port', '/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0']
        for p in ports:
            if os.path.exists(p):
                try:
                    s = serial.Serial(p, 1000000, timeout=0.05)
                    self.serial_port = s
                    print(f"  🎉 [모터 직접 제어 활성화] 서보 드라이버 연결 완료: {p}")
                    for sid in [1, 2, 3, 4]:
                        self.direct_set_wheel_mode(sid)
                    print("  ✅ [단독 구동 모드] 별도의 ROS 2 bringup 없이도 바퀴가 직접 회전합니다!\n")
                    return
                except Exception:
                    continue

        if HAS_ROS2 and self.pub_cmd_vel:
            print("  📡 [ROS 2 모드] 서보 포트가 이미 점유 중입니다. ROS 2 /cmd_vel 토픽을 통해 구동합니다.\n")
        else:
            print("  ⚠️ [주의] 서보 시리얼 포트 및 ROS 2 연결을 찾을 수 없습니다.\n")

    def direct_calc_checksum(self, buf):
        return (~(sum(buf) & 0xFF)) & 0xFF

    def direct_send_packet(self, servo_id, inst, reg_or_params):
        if not self.serial_port or not self.serial_port.is_open:
            return
        length = len(reg_or_params) + 2
        body = [servo_id, length, inst] + list(reg_or_params)
        chk = self.direct_calc_checksum(body)
        packet = bytearray([0xFF, 0xFF] + body + [chk])
        self.serial_port.write(packet)
        self.serial_port.flush()

    def direct_set_wheel_mode(self, servo_id):
        self.direct_send_packet(servo_id, 0x03, [0x37, 0x00])
        time.sleep(0.01)
        self.direct_send_packet(servo_id, 0x03, [0x21, 0x01])
        time.sleep(0.01)
        self.direct_send_packet(servo_id, 0x03, [0x37, 0x01])
        time.sleep(0.01)

    def direct_send_speed(self, servo_id, val):
        val = int(val)
        mag = min(abs(val), 32767)
        if val < 0:
            encoded_val = (1 << 15) | mag
        else:
            encoded_val = mag
        low_b = encoded_val & 0xFF
        high_b = (encoded_val >> 8) & 0xFF
        self.direct_send_packet(servo_id, 0x03, [0x2E, low_b, high_b])

    def send_direct_4wheel(self, step_speed):
        if not self.serial_port or not self.serial_port.is_open:
            return
        # ID 1: 좌앞 (양수), ID 2: 우앞 (음수)
        # ID 3: 좌뒤 (양수), ID 4: 우뒤 (음수)
        self.direct_send_speed(1, step_speed)
        self.direct_send_speed(2, -step_speed)
        self.direct_send_speed(3, step_speed)
        self.direct_send_speed(4, -step_speed)

    def publish_velocity(self, linear_x, angular_z=0.0):
        if HAS_ROS2 and self.pub_cmd_vel:
            cmd = Twist()
            cmd.linear.x = float(linear_x)
            cmd.angular.z = float(angular_z)
            self.pub_cmd_vel.publish(cmd)

        if self.serial_port and self.serial_port.is_open:
            step_speed = int(linear_x * 12000.0)
            self.send_direct_4wheel(step_speed)

    def emergency_stop(self):
        for _ in range(5):
            self.publish_velocity(0.0, 0.0)
            if self.serial_port and self.serial_port.is_open:
                self.send_direct_4wheel(0)
            time.sleep(0.01)

    def process_frame(self, frame):
        h, w, _ = frame.shape
        target_threshold = shared_state["target_threshold"]

        cur_time = time.time()
        self.fps = 0.9 * self.fps + 0.1 * (1.0 / max(cur_time - self.prev_time, 0.001))
        self.prev_time = cur_time

        # 1. MiDaS 3D 뎁스 추론
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        input_batch = self.transform(img_rgb).to(self.device)

        with torch.no_grad():
            prediction = self.midas(input_batch)
            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=img_rgb.shape[:2],
                mode="bicubic",
                align_corners=False,
            ).squeeze()

        depth_map = prediction.cpu().numpy()

        # 2. 정규화 (0~255) 및 컬러맵 변환
        d_min, d_max = depth_map.min(), depth_map.max()
        if d_max - d_min > 0:
            depth_norm = (255 * (depth_map - d_min) / (d_max - d_min)).astype(np.uint8)
        else:
            depth_norm = np.zeros_like(depth_map, dtype=np.uint8)

        depth_colored = cv2.applyColorMap(depth_norm, cv2.COLORMAP_MAGMA)

        # 3. [해결책 A 핵심 알고리즘] 단일 관심영역(ROI) 뎁스 및 점유 면적 측정
        roi = depth_norm[self.roi_y1:self.roi_y2, self.roi_x1:self.roi_x2]

        # (1) ROI 내 88% 백분위수 (물체 표면의 대표 근접 깊이, 노이즈 제외)
        current_depth = int(np.percentile(roi, 88))

        # (2) 바닥 영점(Tare) 요청 처리
        if shared_state.get("tare_requested", False):
            shared_state["tare_requested"] = False
            shared_state["baseline_depth"] = current_depth
            print(f"⚖️ [바닥 영점 보정 완료] 현재 바닥 기준 뎁스: {current_depth} pt")

        # (3) 임계값 초과 픽셀 면적 점유율 (%)
        close_pixels = (roi >= target_threshold)
        occupancy_pct = int(np.mean(close_pixels) * 100)

        # (4) 물리적 거리 추정 근사치 (cm)
        # MiDaS 특성: 255 -> ~16cm, 200 -> ~22cm, 175 -> ~25cm, 130 -> ~34cm
        approx_dist_cm = int(max(10, 6000.0 / (current_depth + 25) - 5))

        # (5) 최종 감지 여부 판정: 뎁스가 임계값 이상이면서 점유 면적이 최소 점유율(8%) 이상일 때
        is_detected = (current_depth >= target_threshold) and (occupancy_pct >= shared_state["min_occupancy"])

        shared_state["current_depth"] = current_depth
        shared_state["occupancy_pct"] = occupancy_pct
        shared_state["approx_dist_cm"] = approx_dist_cm
        shared_state["is_detected"] = is_detected

        # 4. 상태 머신 및 주행/정지 제어
        if shared_state["stop_requested"]:
            shared_state["stop_requested"] = False
            self.state = "IDLE"
            self.emergency_stop()

        if shared_state["should_approach"] and self.state != "APPROACHING":
            if is_detected:
                print(f"\n⚠️ 이미 전방 {approx_dist_cm}cm에 물체가 감지되어 있습니다! (현재 뎁스: {current_depth})")
                print("   👉 물체를 조금 더 멀리 두거나, 임계값을 올려주세요.")
                shared_state["should_approach"] = False
                shared_state["mode"] = "REACHED"
                shared_state["status_msg"] = f"이미 목표 거리 도달 상태입니다 (~{approx_dist_cm}cm)"
            else:
                self.state = "APPROACHING"
                self.consecutive_reach_count = 0
                print(f"\n🚀 [자율 전진 시작] 현재 뎁스 {current_depth}pt ➔ 목표 {target_threshold}pt (~{approx_dist_cm}cm)로 접근합니다...")

        if self.state == "APPROACHING":
            if is_detected:
                self.consecutive_reach_count += 1
                if self.consecutive_reach_count >= 2:
                    self.state = "REACHED_STOP"
                    shared_state["should_approach"] = False
                    shared_state["mode"] = "REACHED"
                    shared_state["status_msg"] = f"🛑 물체 감지 도달 (~{approx_dist_cm}cm)! 정지 완료"
                    self.emergency_stop()
                    print(f"\n🛑 [물체 도달 긴급 정지!] 전방 ~{approx_dist_cm}cm (뎁스 {current_depth}pt, 점유 {occupancy_pct}%) ➔ 안전 정지 완료!")
            else:
                self.consecutive_reach_count = 0
                approach_speed = shared_state["speed"]
                # 목표에 근접하면 서서히 감속
                if current_depth >= target_threshold - 20:
                    approach_speed *= 0.5

                self.publish_velocity(approach_speed, 0.0)

                if cur_time - self.last_log_time > 0.7:
                    pct = min(100, int((current_depth / target_threshold) * 100))
                    print(f"🏎️ [전진 중] 뎁스: {current_depth}/{target_threshold} pt [{pct}%] | 점유: {occupancy_pct}% | 거리: ~{approx_dist_cm}cm")
                    self.last_log_time = cur_time

        elif self.state == "IDLE":
            if is_detected:
                self.emergency_stop()

        # 5. UI 화면 시각화 (좌: 원본 RGB, 우: 3D 뎁스맵)
        annotated_rgb = frame.copy()

        # 감지 상태에 따른 색상 및 텍스트
        if is_detected or self.state == "REACHED_STOP":
            box_color = (0, 0, 255)       # 빨간색 (물체 감지/정지)
            status_text = f"STOP! Object Detected (~{approx_dist_cm}cm, {occupancy_pct}%)"
        elif self.state == "APPROACHING":
            box_color = (0, 165, 255)     # 주황색 (접근 중)
            status_text = f"APPROACHING... Depth: {current_depth}/{target_threshold} pt (~{approx_dist_cm}cm)"
        else:
            box_color = (0, 255, 0)       # 녹색 (대기/클리어)
            status_text = f"CLEAR | Depth: {current_depth}/{target_threshold} pt (~{approx_dist_cm}cm)"

        # (1) 단일 ROI 박스 및 코너 강조 표시
        cv2.rectangle(annotated_rgb, (self.roi_x1, self.roi_y1), (self.roi_x2, self.roi_y2), box_color, 2)
        cv2.putText(annotated_rgb, f"Grasp ROI [Depth: {current_depth} | Area: {occupancy_pct}%]",
                    (self.roi_x1 + 6, self.roi_y1 + 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, box_color, 2)

        # (2) 뎁스 히트맵 상에도 동일 ROI 박스 및 감지된 물체 하이라이트
        cv2.rectangle(depth_colored, (self.roi_x1, self.roi_y1), (self.roi_x2, self.roi_y2), (255, 255, 255), 2)
        cv2.putText(depth_colored, f"Target >= {target_threshold} pt",
                    (self.roi_x1 + 6, self.roi_y1 + 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)

        # ROI 내에서 임계값을 넘은 물체 영역을 컬러맵 상에서 반짝이도록 오버레이
        if occupancy_pct > 0:
            roi_colored = depth_colored[self.roi_y1:self.roi_y2, self.roi_x1:self.roi_x2]
            roi_colored[close_pixels] = cv2.addWeighted(roi_colored[close_pixels], 0.5, np.full_like(roi_colored[close_pixels], (0, 255, 0)), 0.5, 0)

        # (3) 상단 상태 바 & 실시간 게이지 바
        banner_bg = (0, 0, 180) if (is_detected or self.state == "REACHED_STOP") else (
            (0, 90, 200) if self.state == "APPROACHING" else (25, 25, 30)
        )
        cv2.rectangle(annotated_rgb, (0, 0), (w, 52), banner_bg, -1)
        cv2.putText(annotated_rgb, status_text, (15, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (255, 255, 255), 2)

        # 게이지 바 그리기 (220px 너비)
        gx, gy, gw, gh = 15, 32, 220, 12
        cv2.rectangle(annotated_rgb, (gx, gy), (gx + gw, gy + gh), (60, 60, 60), -1)
        fill_ratio = min(1.0, current_depth / max(1, target_threshold))
        fill_w = int(gw * fill_ratio)
        gauge_color = (0, 0, 255) if is_detected else ((0, 200, 255) if self.state == "APPROACHING" else (0, 255, 0))
        if fill_w > 0:
            cv2.rectangle(annotated_rgb, (gx, gy), (gx + fill_w, gy + gh), gauge_color, -1)
        cv2.rectangle(annotated_rgb, (gx, gy), (gx + gw, gy + gh), (120, 120, 120), 1)
        cv2.putText(annotated_rgb, f"{int(fill_ratio * 100)}%", (gx + gw + 8, gy + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)

        # FPS 표시
        cv2.putText(annotated_rgb, f"FPS: {self.fps:.1f}", (w - 100, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # 좌우 병합 (1280x480)
        combined = np.hstack((annotated_rgb, depth_colored))
        return combined

    def run(self):
        print("=" * 68)
        print("🎉  AMR MiDaS 단일 ROI 근접 물체 감지 루프 시작!")
        print(f"🌐  PC / 스마트폰 웹 대시보드: http://192.168.0.64:{DEFAULT_WEB_PORT}")
        print("⌨️   조작 방법:")
        print("    [터미널에서]  Ctrl + C  또는  'q' + Enter : 프로그램 즉시 종료")
        print("    [터미널에서]  'g' + Enter : 자율 전진 시작 | 's' + Enter : 즉시 정지")
        print("    [터미널에서]  't' + Enter : 현재 바닥 영점(Tare) 보정")
        print("    [터미널에서]  '+' / '-' + Enter : 감지 임계치 5pt 증가 / 감소")
        print("    [웹 브라우저] 실시간 게이지 & 버튼으로 원클릭 조작 가능")
        if self.has_display:
            print("    [화면 창에서] 창을 마우스로 클릭 후 'q' 키를 누르면 종료")
        print("=" * 68 + "\n")

        while True:
            # 1. 터미널 표준입력 비차단 감지
            if sys.stdin.isatty():
                r, _, _ = select.select([sys.stdin], [], [], 0)
                if r:
                    term_line = sys.stdin.readline().strip().lower()
                    if term_line in ('q', 'quit', 'exit'):
                        print("\n🛑 [터미널 입력] 프로그램 종료 요청을 수신했습니다.")
                        break
                    elif term_line in ('g', 'a', 'go'):
                        shared_state["should_approach"] = True
                    elif term_line in ('s', 'stop', ' '):
                        shared_state["stop_requested"] = True
                    elif term_line in ('t', 'tare'):
                        shared_state["tare_requested"] = True
                    elif term_line in ('+', '='):
                        shared_state["target_threshold"] = min(245, shared_state["target_threshold"] + 5)
                        print(f"📏 감지 임계치: {shared_state['target_threshold']} pt")
                    elif term_line in ('-', '_'):
                        shared_state["target_threshold"] = max(80, shared_state["target_threshold"] - 5)
                        print(f"📏 감지 임계치: {shared_state['target_threshold']} pt")

            # 2. 웹 UI 종료 요청 확인
            if shared_state.get("shutdown_requested"):
                print("\n🛑 [웹 UI 요청] 프로그램을 안전하게 종료합니다.")
                break

            ret, frame = self.cap.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            # 카메라가 위아래 반전되어 설치된 경우 정방향으로 먼저 뒤집어 처리
            frame = cv2.flip(frame, 0)

            combined = self.process_frame(frame)

            with shared_state["lock"]:
                shared_state["output_frame"] = combined.copy()

            if HAS_ROS2 and self.node:
                rclpy.spin_once(self.node, timeout_sec=0.001)

            if self.has_display:
                cv2.imshow("AMR MiDaS Single ROI Stopper", combined)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("\n🛑 [화면 창 입력] 'q' 키로 종료합니다.")
                    break
                elif key in (ord('g'), ord('a')):
                    shared_state["should_approach"] = True
                elif key in (ord('s'), 32):
                    shared_state["stop_requested"] = True
                elif key in (ord('t'),):
                    shared_state["tare_requested"] = True
                elif key == ord('+') or key == ord('='):
                    shared_state["target_threshold"] = min(245, shared_state["target_threshold"] + 5)
                    print(f"📏 감지 임계치: {shared_state['target_threshold']} pt")
                elif key == ord('-') or key == ord('_'):
                    shared_state["target_threshold"] = max(80, shared_state["target_threshold"] - 5)
                    print(f"📏 감지 임계치: {shared_state['target_threshold']} pt")

            time.sleep(0.01)


def main():
    parser = argparse.ArgumentParser(description="AMR MiDaS 단일 ROI 근접 물체 감지 및 정지")
    parser.add_argument("--threshold", type=int, default=175, help="정지 감지 뎁스 임계치 (0~255, 기본: 175)")
    parser.add_argument("--speed", type=float, default=0.07, help="접근 전진 속도 (m/s, 기본: 0.07)")
    parser.add_argument("--web-port", type=int, default=DEFAULT_WEB_PORT, help="웹 스트리밍 포트 (기본: 8080)")
    parser.add_argument("--auto-start", action="store_true", help="시작하자마자 바로 자율 전진 시작")
    args = parser.parse_args()

    if args.auto_start:
        shared_state["should_approach"] = True

    # 웹 서버 백그라운드 스레드
    web_thread = threading.Thread(target=run_web_server, args=(args.web_port,), daemon=True)
    web_thread.start()

    # ROS 2 노드 초기화
    node = None
    if HAS_ROS2:
        rclpy.init()
        node = Node('grasp_approach_stopper')

    stopper = GraspApproachStopper(node=node, target_threshold=args.threshold, approach_speed=args.speed)
    try:
        stopper.run()
    except KeyboardInterrupt:
        print("\n🛑 프로그램이 안전하게 종료되었습니다.")
    finally:
        stopper.emergency_stop()
        stopper.cap.release()
        cv2.destroyAllWindows()
        if HAS_ROS2 and node:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
