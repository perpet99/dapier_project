#!/usr/bin/env python3
"""
라즈베리파이 / USB 카메라 실시간 웹 스트리밍 및 진단 도구 (test_camera.py)
-----------------------------------------------------------------
1. 시스템의 /dev/video* 및 V4L2 / libcamera 장치 자동 검색 및 상태 진단
2. OpenCV 카메라 프레임 실시간 캡처 (스레드 기반 비동기 파이프라인)
3. 브라우저에서 실시간으로 화면을 볼 수 있는 고성능 웹 스트리밍 서버 내장 (HTTP/MJPEG)
4. 스냅샷 다운로드, 실시간 FPS/해상도 모니터링, 가상 테스트 패턴 폴백 지원
"""

import os
os.environ.setdefault("OPENCV_LOG_LEVEL", "OFF")
import sys
import time
import glob
import json
import socket
import argparse
import threading
import subprocess
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

import cv2
import numpy as np


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """멀티 클라이언트 동시 접속을 지원하는 멀티스레드 HTTP 서버"""
    daemon_threads = True
    allow_reuse_address = True


class CameraStreamer:
    """OpenCV 카메라 캡처 및 프레임 버퍼 관리자"""

    def __init__(self, device_id=None, width=640, height=480, fps=30):
        self.device_id = device_id
        self.target_width = width
        self.target_height = height
        self.target_fps = fps

        self.cap = None
        self.active_device = None
        self.is_virtual = False
        self.running = False
        self.lock = threading.Lock()

        self.latest_raw_frame = None
        self.latest_jpeg = None
        self.frame_count = 0
        self.fps_actual = 0.0
        self.actual_width = width
        self.actual_height = height
        self.last_fps_calc_time = time.time()
        self.fps_counter = 0

        self.thread = None

    def find_working_camera(self):
        """사용 가능한 카메라 인덱스를 검색하여 반환"""
        if self.device_id is not None:
            # 사용자가 특정 장치를 지정한 경우
            try:
                idx = int(self.device_id)
            except ValueError:
                idx = self.device_id
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    cap.release()
                    return idx
                cap.release()
            return None

        # 자동 검색 (0~5번 인덱스 순회)
        for idx in range(6):
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    cap.release()
                    return idx
                cap.release()
        return None

    def start(self):
        """카메라 열기 및 캡처 스레드 시작"""
        cam_idx = self.find_working_camera()
        if cam_idx is not None:
            self.active_device = cam_idx
            self.cap = cv2.VideoCapture(cam_idx)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.target_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.target_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.target_fps)

            # 실제 적용된 해상도 확인
            actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if actual_w > 0 and actual_h > 0:
                self.actual_width = actual_w
                self.actual_height = actual_h

            self.is_virtual = False
            print(f"✅ 실제 카메라 장치 [Index: {self.active_device}] 연결 성공! ({self.actual_width}x{self.actual_height})")
        else:
            self.is_virtual = True
            self.active_device = "가상 테스트 패턴 (실제 카메라 미감지)"
            self.actual_width = self.target_width
            self.actual_height = self.target_height
            print("⚠️ 실제 카메라를 감지하지 못했습니다.")
            print("   -> 웹 스트리밍 동작 검증을 위해 [가상 로봇 테스트 패턴] 모드로 자동 시작합니다.")

        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _generate_virtual_frame(self, step):
        """카메라가 없을 때 시각적 피드백을 주는 시뮬레이션 프레임 생성"""
        w, h = self.actual_width, self.actual_height
        frame = np.zeros((h, w, 3), dtype=np.uint8)

        # 다크 블루/그레이 그리드 배경
        frame[:, :] = (24, 20, 20)
        grid_step = 40
        for x in range(0, w, grid_step):
            cv2.line(frame, (x, 0), (x, h), (40, 35, 35), 1)
        for y in range(0, h, grid_step):
            cv2.line(frame, (0, y), (w, y), (40, 35, 35), 1)

        # 애니메이션 스캐닝 바
        scan_x = int((math_sin := (np.sin(step * 0.08) + 1.0) / 2.0) * w)
        cv2.line(frame, (scan_x, 0), (scan_x, h), (0, 200, 255), 2)

        # 중앙 조준선 (Crosshair)
        cx, cy = w // 2, h // 2
        cv2.circle(frame, (cx, cy), 50, (0, 165, 255), 2)
        cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
        cv2.line(frame, (cx - 70, cy), (cx + 70, cy), (0, 165, 255), 1)
        cv2.line(frame, (cx, cy - 70), (cx, cy + 70), (0, 165, 255), 1)

        # 상태 텍스트
        now_str = time.strftime("%Y-%m-%d %H:%M:%S")
        cv2.putText(frame, "AMR CAMERA TEST - STANDBY PATTERN", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 200), 2, cv2.LINE_AA)
        cv2.putText(frame, f"TIME: {now_str}", (20, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(frame, "NO HARDWARE DETECTED - CONNECT CAMERA CABLE", (20, h - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 100, 255), 2, cv2.LINE_AA)

        return frame

    def _capture_loop(self):
        """백그라운드 캡처 루프"""
        step = 0
        while self.running:
            frame = None
            if not self.is_virtual and self.cap is not None:
                ret, frame = self.cap.read()
                if not ret or frame is None:
                    # 캡처 실패 시 잠시 대기 후 재시도
                    time.sleep(0.05)
                    continue
            else:
                frame = self._generate_virtual_frame(step)
                step += 1
                time.sleep(1.0 / self.target_fps)

            # FPS 계산
            self.fps_counter += 1
            now = time.time()
            elapsed = now - self.last_fps_calc_time
            if elapsed >= 1.0:
                self.fps_actual = round(self.fps_counter / elapsed, 1)
                self.fps_counter = 0
                self.last_fps_calc_time = now

            # JPEG 인코딩
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
            ret, jpeg = cv2.imencode('.jpg', frame, encode_param)

            if ret:
                with self.lock:
                    self.latest_raw_frame = frame
                    self.latest_jpeg = jpeg.tobytes()
                    self.frame_count += 1

    def get_jpeg(self):
        """가장 최근 인코딩된 JPEG 바이트 반환"""
        with self.lock:
            return self.latest_jpeg

    def get_snapshot(self):
        """스냅샷용 JPEG 바이트 반환"""
        with self.lock:
            if self.latest_raw_frame is not None:
                ret, jpeg = cv2.imencode('.jpg', self.latest_raw_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
                if ret:
                    return jpeg.tobytes()
            return self.latest_jpeg

    def get_status(self):
        """카메라 상태 딕셔너리 반환"""
        with self.lock:
            return {
                "active_device": str(self.active_device),
                "is_virtual": self.is_virtual,
                "resolution": f"{self.actual_width}x{self.actual_height}",
                "fps": self.fps_actual,
                "frame_count": self.frame_count,
            }

    def stop(self):
        """스트리밍 정지 및 자원 해제"""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.cap is not None:
            self.cap.release()
            self.cap = None


def get_system_diagnostics():
    """시스템 비디오 장치 진단 정보 수집"""
    devices = sorted(glob.glob('/dev/video*'))
    v4l2_info = "v4l2-ctl 명령어를 찾을 수 없거나 출력이 없습니다."
    try:
        res = subprocess.run(["v4l2-ctl", "--list-devices"], capture_output=True, text=True, timeout=3)
        if res.returncode == 0 and res.stdout.strip():
            v4l2_info = res.stdout.strip()
    except Exception:
        pass

    rpicam_info = "미감지"
    for cmd in ["rpicam-hello", "libcamera-hello"]:
        try:
            res = subprocess.run([cmd, "--list-cameras"], capture_output=True, text=True, timeout=3)
            if res.returncode == 0 and "Available cameras" in res.stdout:
                rpicam_info = f"[{cmd}]\n" + res.stdout.strip()
                break
        except Exception:
            pass

    return {
        "video_devices": devices,
        "v4l2_info": v4l2_info,
        "rpicam_info": rpicam_info
    }


def build_html_dashboard():
    """현대적인 반응형 웹 대시보드 HTML"""
    return """<!DOCTYPE html>
<html lang="ko">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>AMR Camera Live Monitor</title>
  <style>
    :root {
      --bg-primary: #0b0f19;
      --bg-card: #151d30;
      --bg-surface: #1e293b;
      --accent: #38bdf8;
      --accent-glow: rgba(56, 189, 248, 0.25);
      --success: #10b981;
      --warning: #f59e0b;
      --danger: #ef4444;
      --text-main: #f8fafc;
      --text-sub: #94a3b8;
      --border: #334155;
    }
    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
    }
    body {
      background: var(--bg-primary);
      color: var(--text-main);
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
      min-height: 100vh;
      display: flex;
      flex-direction: column;
    }
    header {
      background: rgba(21, 29, 48, 0.85);
      backdrop-filter: blur(12px);
      border-bottom: 1px solid var(--border);
      padding: 16px 24px;
      display: flex;
      justify-content: space-between;
      align-items: center;
      position: sticky;
      top: 0;
      z-index: 50;
    }
    .brand {
      display: flex;
      align-items: center;
      gap: 12px;
    }
    .brand-icon {
      width: 36px;
      height: 36px;
      border-radius: 10px;
      background: linear-gradient(135deg, #0284c7, #38bdf8);
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 18px;
      box-shadow: 0 0 15px var(--accent-glow);
    }
    .brand-text h1 {
      font-size: 1.15rem;
      font-weight: 700;
      letter-spacing: -0.02em;
    }
    .brand-text p {
      font-size: 0.75rem;
      color: var(--text-sub);
    }
    .status-badge {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      padding: 6px 14px;
      border-radius: 9999px;
      font-size: 0.82rem;
      font-weight: 600;
      background: rgba(16, 185, 129, 0.15);
      color: var(--success);
      border: 1px solid rgba(16, 185, 129, 0.3);
    }
    .status-badge.virtual {
      background: rgba(245, 158, 11, 0.15);
      color: var(--warning);
      border-color: rgba(245, 158, 11, 0.3);
    }
    .pulse-dot {
      width: 8px;
      height: 8px;
      border-radius: 50%;
      background: currentColor;
      box-shadow: 0 0 8px currentColor;
      animation: pulse 1.8s infinite;
    }
    @keyframes pulse {
      0%, 100% { opacity: 1; transform: scale(1); }
      50% { opacity: 0.4; transform: scale(0.85); }
    }
    main {
      flex: 1;
      max-width: 1400px;
      width: 100%;
      margin: 0 auto;
      padding: 24px;
      display: grid;
      grid-template-columns: 1fr 340px;
      gap: 24px;
    }
    @media (max-width: 980px) {
      main {
        grid-template-columns: 1fr;
      }
    }
    .stream-section {
      display: flex;
      flex-direction: column;
      gap: 16px;
    }
    .viewport-card {
      background: var(--bg-card);
      border: 1px solid var(--border);
      border-radius: 16px;
      overflow: hidden;
      box-shadow: 0 10px 30px rgba(0, 0, 0, 0.35);
      position: relative;
    }
    .stream-wrapper {
      position: relative;
      width: 100%;
      aspect-ratio: 4 / 3;
      background: #000;
      display: flex;
      align-items: center;
      justify-content: center;
      overflow: hidden;
    }
    .stream-wrapper img {
      width: 100%;
      height: 100%;
      object-fit: contain;
      user-select: none;
    }
    .stream-overlay {
      position: absolute;
      top: 14px;
      left: 14px;
      right: 14px;
      display: flex;
      justify-content: space-between;
      pointer-events: none;
    }
    .overlay-tag {
      background: rgba(15, 23, 42, 0.75);
      backdrop-filter: blur(8px);
      border: 1px solid rgba(255, 255, 255, 0.1);
      padding: 6px 12px;
      border-radius: 8px;
      font-size: 0.78rem;
      font-family: monospace;
      color: #e2e8f0;
    }
    .grid-lines {
      position: absolute;
      inset: 0;
      pointer-events: none;
      display: none;
      grid-template-columns: 1fr 1fr 1fr;
      grid-template-rows: 1fr 1fr 1fr;
      border: 1px solid rgba(56, 189, 248, 0.2);
    }
    .grid-lines.active {
      display: grid;
    }
    .grid-lines > div {
      border: 1px solid rgba(56, 189, 248, 0.15);
    }
    .controls-bar {
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      padding: 16px;
      background: var(--bg-card);
      border: 1px solid var(--border);
      border-radius: 16px;
    }
    .btn {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      padding: 10px 16px;
      border-radius: 10px;
      font-size: 0.88rem;
      font-weight: 600;
      border: 1px solid var(--border);
      background: var(--bg-surface);
      color: var(--text-main);
      cursor: pointer;
      transition: all 0.2s ease;
      text-decoration: none;
    }
    .btn:hover {
      background: #334155;
      border-color: #475569;
      transform: translateY(-1px);
    }
    .btn.primary {
      background: linear-gradient(135deg, #0284c7, #0ea5e9);
      border-color: #0284c7;
      color: #fff;
    }
    .btn.primary:hover {
      background: linear-gradient(135deg, #0369a1, #0284c7);
      box-shadow: 0 4px 14px var(--accent-glow);
    }
    .sidebar {
      display: flex;
      flex-direction: column;
      gap: 16px;
    }
    .card {
      background: var(--bg-card);
      border: 1px solid var(--border);
      border-radius: 16px;
      padding: 18px;
    }
    .card-title {
      font-size: 0.95rem;
      font-weight: 700;
      margin-bottom: 14px;
      display: flex;
      align-items: center;
      gap: 8px;
      color: #e2e8f0;
    }
    .metrics-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
    }
    .metric-box {
      background: var(--bg-surface);
      border: 1px solid rgba(255, 255, 255, 0.05);
      border-radius: 10px;
      padding: 12px;
    }
    .metric-label {
      font-size: 0.72rem;
      color: var(--text-sub);
      margin-bottom: 4px;
    }
    .metric-value {
      font-size: 1.15rem;
      font-weight: 700;
      color: var(--accent);
      font-family: monospace;
    }
    .device-list {
      list-style: none;
      font-family: monospace;
      font-size: 0.8rem;
    }
    .device-list li {
      padding: 6px 10px;
      background: var(--bg-surface);
      border-radius: 6px;
      margin-bottom: 6px;
      border: 1px solid rgba(255, 255, 255, 0.04);
      word-break: break-all;
    }
    .help-box {
      font-size: 0.8rem;
      color: var(--text-sub);
      line-height: 1.5;
    }
    .help-box code {
      background: rgba(0, 0, 0, 0.3);
      padding: 2px 6px;
      border-radius: 4px;
      color: #38bdf8;
      font-family: monospace;
    }
  </style>
</head>
<body>
  <header>
    <div class="brand">
      <div class="brand-icon">📹</div>
      <div class="brand-text">
        <h1>AMR Camera Live Monitor</h1>
        <p>실시간 비디오 스트리밍 & 진단 대시보드</p>
      </div>
    </div>
    <div id="liveBadge" class="status-badge">
      <span class="pulse-dot"></span>
      <span id="badgeText">연결 중...</span>
    </div>
  </header>

  <main>
    <section class="stream-section">
      <div class="viewport-card">
        <div class="stream-wrapper" id="videoContainer">
          <img id="streamImg" src="/stream.mjpg" alt="카메라 실시간 스트림" />
          <div class="grid-lines" id="gridOverlay">
            <div></div><div></div><div></div>
            <div></div><div></div><div></div>
            <div></div><div></div><div></div>
          </div>
          <div class="stream-overlay">
            <div class="overlay-tag" id="overlayFps">FPS: --</div>
            <div class="overlay-tag" id="overlayRes">RES: --</div>
          </div>
        </div>
      </div>

      <div class="controls-bar">
        <button class="btn primary" onclick="takeSnapshot()">
          📸 스냅샷 다운로드
        </button>
        <button class="btn" onclick="toggleGrid()">
          📐 조준 그리드 토글
        </button>
        <button class="btn" onclick="reconnectStream()">
          🔄 스트림 새로고침
        </button>
        <button class="btn" onclick="toggleFullscreen()">
          ⛶ 전체 화면
        </button>
      </div>
    </section>

    <aside class="sidebar">
      <div class="card">
        <div class="card-title">📊 실시간 성능 지표</div>
        <div class="metrics-grid">
          <div class="metric-box">
            <div class="metric-label">현재 FPS</div>
            <div class="metric-value" id="valFps">0.0</div>
          </div>
          <div class="metric-box">
            <div class="metric-label">해상도</div>
            <div class="metric-value" id="valRes">--</div>
          </div>
          <div class="metric-box">
            <div class="metric-label">총 프레임 수</div>
            <div class="metric-value" id="valFrames">0</div>
          </div>
          <div class="metric-box">
            <div class="metric-label">장치 모드</div>
            <div class="metric-value" id="valMode" style="font-size:0.85rem;">확인 중</div>
          </div>
        </div>
      </div>

      <div class="card">
        <div class="card-title">🔍 연결된 카메라 장치</div>
        <ul class="device-list" id="deviceList">
          <li>조회 중...</li>
        </ul>
      </div>

      <div class="card">
        <div class="card-title">💡 라즈베리파이 가이드</div>
        <div class="help-box">
          <p>카메라가 인식되지 않을 경우:</p>
          <p style="margin-top:6px;">1. 권한 확인: <code>sudo usermod -aG video $USER</code></p>
          <p style="margin-top:4px;">2. CSI 케이블 체결 및 접점 방향 점검</p>
          <p style="margin-top:4px;">3. <code>/boot/firmware/config.txt</code>의 카메라 오버레이 설정 확인 (예: <code>camera_auto_detect=1</code>)</p>
        </div>
      </div>
    </aside>
  </main>

  <script>
    let isGridOn = false;

    function toggleGrid() {
      isGridOn = !isGridOn;
      document.getElementById('gridOverlay').classList.toggle('active', isGridOn);
    }

    function reconnectStream() {
      const img = document.getElementById('streamImg');
      img.src = '/stream.mjpg?t=' + new Date().getTime();
    }

    function toggleFullscreen() {
      const container = document.getElementById('videoContainer');
      if (!document.fullscreenElement) {
        container.requestFullscreen().catch(err => {
          alert(`전체화면 오류: ${err.message}`);
        });
      } else {
        document.exitFullscreen();
      }
    }

    function takeSnapshot() {
      const link = document.createElement('a');
      link.href = '/snapshot?t=' + new Date().getTime();
      link.download = `amr_snapshot_${new Date().toISOString().replace(/[:.]/g, '-')}.jpg`;
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);
    }

    async function updateStatus() {
      try {
        const res = await fetch('/api/status');
        if (!res.ok) return;
        const data = await res.json();

        // 지표 업데이트
        document.getElementById('valFps').textContent = data.fps.toFixed(1);
        document.getElementById('overlayFps').textContent = `FPS: ${data.fps.toFixed(1)}`;
        document.getElementById('valRes').textContent = data.resolution;
        document.getElementById('overlayRes').textContent = `RES: ${data.resolution}`;
        document.getElementById('valFrames').textContent = data.frame_count.toLocaleString();

        const badge = document.getElementById('liveBadge');
        const badgeText = document.getElementById('badgeText');
        const modeEl = document.getElementById('valMode');

        if (data.is_virtual) {
          badge.className = 'status-badge virtual';
          badgeText.textContent = '가상 테스트 패턴';
          modeEl.textContent = 'VIRTUAL';
          modeEl.style.color = '#f59e0b';
        } else {
          badge.className = 'status-badge';
          badgeText.textContent = `LIVE (${data.active_device})`;
          modeEl.textContent = 'HARDWARE';
          modeEl.style.color = '#10b981';
        }

        if (data.devices) {
          const list = document.getElementById('deviceList');
          if (data.devices.length === 0) {
            list.innerHTML = '<li style="color:#ef4444;">/dev/video* 없음</li>';
          } else {
            list.innerHTML = data.devices.map(d => `<li>${d}</li>`).join('');
          }
        }
      } catch (e) {
        console.error('상태 조회 실패:', e);
      }
    }

    setInterval(updateStatus, 1000);
    updateStatus();
  </script>
</body>
</html>
"""


class WebStreamHandler(BaseHTTPRequestHandler):
    """실시간 웹 스트리밍 및 API 요청 핸들러"""

    streamer: CameraStreamer = None
    diagnostics = {}

    def log_message(self, format, *args):
        # 콘솔이 지저분해지지 않도록 스트림 청크 로그는 억제
        pass

    def do_GET(self):
        if self.path in ['/', '/index.html']:
            content = build_html_dashboard().encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(content)))
            self.end_headers()
            self.wfile.write(content)

        elif self.path.startswith('/stream.mjpg'):
            self.send_response(200)
            self.send_header('Age', '0')
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()

            try:
                while True:
                    frame_bytes = self.streamer.get_jpeg()
                    if frame_bytes is not None:
                        self.wfile.write(b'--FRAME\r\n')
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', str(len(frame_bytes)))
                        self.end_headers()
                        self.wfile.write(frame_bytes)
                        self.wfile.write(b'\r\n')
                    time.sleep(1.0 / max(1, self.streamer.target_fps))
            except (ConnectionResetError, BrokenPipeError):
                pass

        elif self.path.startswith('/snapshot'):
            snapshot_bytes = self.streamer.get_snapshot()
            if snapshot_bytes:
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(snapshot_bytes)))
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                self.wfile.write(snapshot_bytes)
            else:
                self.send_error(503, "카메라 프레임 획득 실패")

        elif self.path == '/api/status':
            status = self.streamer.get_status()
            status["devices"] = self.diagnostics.get("video_devices", [])
            data = json.dumps(status).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(data)

        else:
            self.send_error(404, "Not Found")


def get_local_ip():
    """LAN IP 주소 조회"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


def print_system_report(diagnostics):
    """터미널 진단 정보 출력"""
    print("=" * 65)
    print("🔍 1. 시스템 비디오 장치 확인")
    print("=" * 65)

    devices = diagnostics["video_devices"]
    if not devices:
        print("❌ /dev/video* 장치를 찾을 수 없습니다.")
    else:
        print(f"발견된 비디오 장치 목록: {devices}")

    if diagnostics["v4l2_info"] != "v4l2-ctl 명령어를 찾을 수 없거나 출력이 없습니다.":
        print("\n[v4l2-ctl 장치 상세 정보]")
        print(diagnostics["v4l2_info"])

    if diagnostics["rpicam_info"] != "미감지":
        print("\n[라즈베리파이 카메라 진단]")
        print(diagnostics["rpicam_info"])


def main():
    parser = argparse.ArgumentParser(description="라즈베리파이 / AMR 실시간 웹 카메라 스트리밍 도구")
    parser.add_argument("--device", default=None, help="카메라 인덱스 또는 디바이스 경로 (기본값: 자동 검색)")
    parser.add_argument("--port", type=int, default=8080, help="웹 스트리밍 포트 (기본값: 8080, 충돌 시 자동 탐색)")
    parser.add_argument("--host", default="0.0.0.0", help="바인딩 호스트 (기본값: 0.0.0.0)")
    parser.add_argument("--width", type=int, default=640, help="해상도 가로 너비 (기본값: 640)")
    parser.add_argument("--height", type=int, default=480, help="해상도 세로 높이 (기본값: 480)")
    parser.add_argument("--fps", type=int, default=30, help="목표 FPS (기본값: 30)")
    parser.add_argument("--snap-only", action="store_true", help="웹 서버를 켜지 않고 1회 캡처만 수행")

    args = parser.parse_args()

    diagnostics = get_system_diagnostics()
    print_system_report(diagnostics)

    # 카메라 스트리머 초기화
    streamer = CameraStreamer(
        device_id=args.device,
        width=args.width,
        height=args.height,
        fps=args.fps
    )
    streamer.start()

    # 1프레임 캡처 파일 저장 (기존 test_camera 호환성 유지)
    time.sleep(0.5)
    save_path = os.path.expanduser("~/ros2_amr_ws/camera_test.jpg")
    snap = streamer.get_snapshot()
    if snap:
        with open(save_path, "wb") as f:
            f.write(snap)
        print(f"📸 1프레임 캡처 이미지 저장 완료: {save_path}")

    if args.snap_only:
        streamer.stop()
        print("✅ 스냅샷 캡처 완료 (--snap-only). 종료합니다.")
        return

    WebStreamHandler.streamer = streamer
    WebStreamHandler.diagnostics = diagnostics

    # 사용 가능한 포트 자동 검색 및 바인딩
    server = None
    chosen_port = args.port
    for p in range(args.port, args.port + 20):
        try:
            server = ThreadedHTTPServer((args.host, p), WebStreamHandler)
            chosen_port = p
            break
        except OSError:
            continue

    if server is None:
        print(f"❌ 포트 바인딩 실패 ({args.port} ~ {args.port+20} 포트가 모두 점유되어 있습니다.)")
        streamer.stop()
        return

    if chosen_port != args.port:
        print(f"⚠️ 요청하신 포트 {args.port}번이 사용 중이어서 {chosen_port}번 포트로 자동 전환되었습니다.")

    local_ip = get_local_ip()

    print("\n" + "=" * 65)
    print("🌐 2. 실시간 웹 카메라 스트리밍 서버가 실행되었습니다!")
    print("=" * 65)
    print(f"  • 로컬 접속 URL      : http://localhost:{chosen_port}")
    print(f"  • 외부/모바일 접속 URL: http://{local_ip}:{chosen_port}")
    print(f"  • MJPEG 원시 스트림  : http://{local_ip}:{chosen_port}/stream.mjpg")
    print(f"  • 실시간 스냅샷      : http://{local_ip}:{chosen_port}/snapshot")
    print("=" * 65)
    print("💡 웹 브라우저에서 위 URL을 열어 실시간 화면을 확인하세요.")
    print("   종료하려면 [Ctrl + C]를 누르세요.\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n🛑 사용자에 의해 서버가 중단되었습니다. 자원을 정리합니다...")
    finally:
        server.server_close()
        streamer.stop()
        print("✅ 카메라 및 웹 서버가 안전하게 종료되었습니다.")


if __name__ == '__main__':
    main()
