#!/usr/bin/env python3
"""
scripts/camera_web_streamer.py
AMR 1회 후진 후 무조건 직진 전진(Lock-in) & 3축 정밀 도킹 시스템

1. 시작 시 가까우면: 딱 1회만 뒤로 후진하여 공간 확보
2. 후진 완료 후: 무조건 앞으로만 직진 전진 (다시 후진 금지 락-인)
3. 25cm 도달 시: 제자리 수평 정렬 후 완벽 도킹
"""

import os
import sys
import time
import math
import json
import socket
import argparse
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
import cv2
import numpy as np

HAS_ROS2 = False
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    HAS_ROS2 = True
except ImportError:
    pass


def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return 'localhost'


class UnifiedDockingStreamer:
    def __init__(self, camera_idx=0, width=640, height=480, port=8080,
                 tag_size=0.08, target_dist=0.10, max_speed=0.08, auto_start=False, flip_180=True):
        self.camera_idx = camera_idx
        self.width = width
        self.height = height
        self.port = port
        self.tag_size = tag_size
        self.target_dist = target_dist
        self.max_speed = max_speed

        self.running = True
        self.docking_active = auto_start
        self.substate = "IDLE"
        self.state_desc = "대기 중 (IDLE)"
        self.has_backed_up = False    # 딱 1회 후진 플래그 (후진 후 다시 후진 불가)
        self.backup_start_time = 0.0
        self.docked_counter = 0
        self.last_seen_time = 0.0
        self.last_center_x = width / 2.0
        self.current_metric = {"dist": 0.0, "x_px": 0, "yaw_deg": 0.0, "tag_id": None}

        # 정밀 튜닝 파라미터 (안정적 도킹 데드밴드)
        self.lin_tol = 0.030          # 거리 허용오차 (±3.0cm)
        self.px_center_tol = 25       # 중심 픽셀 허용오차 (±25px)
        self.yaw_tol_deg = 3.5        # 차체 각도 허용오차 (±3.5도)

        # 모터 제어 속도
        self.min_vx = 0.035
        self.min_wz = 0.07
        self.max_wz = 0.16
        self.search_wz = 0.10

        # OpenCV Aruco / AprilTag 딕셔너리
        self.dict_tag = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.params = cv2.aruco.DetectorParameters_create()
        elif hasattr(cv2.aruco, 'DetectorParameters'):
            self.params = cv2.aruco.DetectorParameters()
        else:
            self.params = None

        if self.params is not None:
            # 바닥 마커 심한 기울기 / 원근 왜곡 대응 파라미터 최적화
            if hasattr(cv2.aruco, 'CORNER_REFINE_SUBPIX'):
                self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.params.adaptiveThreshWinSizeMin = 3
            self.params.adaptiveThreshWinSizeMax = 23
            self.params.adaptiveThreshWinSizeStep = 10
            self.params.adaptiveThreshConstant = 7
            self.params.minMarkerPerimeterRate = 0.02
            self.params.maxErroneousBitsInBorderRate = 0.45
            self.params.perspectiveRemoveIgnoredMarginPerCell = 0.15
            self.params.polygonalApproxAccuracyRate = 0.05

        self.detector = None
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(self.dict_tag, self.params)

        self.camera_matrix = np.array([
            [530.0,   0.0, width / 2.0],
            [  0.0, 530.0, height / 2.0],
            [  0.0,   0.0,          1.0]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)

        self.latest_frame = None
        self.debug_frame = None
        self.lock = threading.Lock()

        # ROS 2 초기화
        self.ros_node = None
        self.cmd_pub = None
        if HAS_ROS2:
            try:
                if not rclpy.ok():
                    rclpy.init()
                self.ros_node = Node('unified_docking_streamer')
                self.cmd_pub = self.ros_node.create_publisher(Twist, '/cmd_vel', 10)
                print("🤖 [ROS 2] /cmd_vel 모터 퍼블리셔 연결 완료!")
            except Exception as e:
                print(f"⚠️ [ROS 2] 퍼블리셔 초기화 예외: {e}")

        # 카메라 열기
        print(f"📷 카메라 디바이스 열기 시도 중... (Index: {camera_idx})")
        self.cap = cv2.VideoCapture(camera_idx)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        if not self.cap.isOpened():
            print(f"❌ 카메라({camera_idx})를 열 수 없습니다. 연결 상태를 확인하세요.")
            sys.exit(1)
        self.flip_180 = flip_180
        print("✅ 카메라 연결 성공!")

    def publish_vel(self, vx, wz):
        if self.cmd_pub and self.ros_node:
            cmd = Twist()
            cmd.linear.x = float(vx)
            cmd.angular.z = float(wz)
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self.ros_node, timeout_sec=0.001)

    def detect_markers(self, gray):
        try:
            if self.detector is not None:
                return self.detector.detectMarkers(gray)
            elif hasattr(cv2.aruco, 'detectMarkers'):
                return cv2.aruco.detectMarkers(gray, self.dict_tag, parameters=self.params)
        except Exception:
            pass
        return None, None, None

    def estimate_pose(self, corner):
        if hasattr(cv2.aruco, 'estimatePoseSingleMarkers'):
            try:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    [corner], self.tag_size, self.camera_matrix, self.dist_coeffs
                )
                return rvecs[0][0], tvecs[0][0]
            except Exception:
                pass
        s = self.tag_size / 2.0
        obj_pts = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float32)
        img_pts = corner.reshape((4, 2)).astype(np.float32)
        flags = getattr(cv2, 'SOLVEPNP_IPPE_SQUARE', 0)
        _, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self.camera_matrix, self.dist_coeffs, flags=flags)
        return rvec.flatten(), tvec.flatten()

    def start_threads(self):
        def process_loop():
            cx_img = self.width / 2.0

            while self.running:
                ret, frame = self.cap.read()
                if not ret or frame is None:
                    time.sleep(0.04)
                    continue

                if self.flip_180:
                    frame = cv2.rotate(frame, cv2.ROTATE_180)

                h, w = frame.shape[:2]
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = self.detect_markers(gray)

                detected = None
                if ids is not None and len(ids) > 0:
                    flat_ids = np.asarray(ids).flatten()
                    corner = corners[0]
                    pts = corner.reshape((4, 2))
                    tid = int(flat_ids[0])

                    # 1. 2D 중심점
                    tc_x = float(np.mean(pts[:, 0]))
                    tc_y = float(np.mean(pts[:, 1]))

                    # 2. 3D 실제 위치 및 거리 (X: 좌우, Y: 상하, Z: 전방거리)
                    rvec, tvec = self.estimate_pose(corner)
                    x_3d = float(tvec[0])
                    y_3d = float(tvec[1])
                    z_3d = float(tvec[2])
                    actual_dist = math.sqrt(x_3d**2 + y_3d**2 + z_3d**2)

                    # 3. 3D 정밀 Yaw 각도 (벽면 마커 법선 벡터 기준, 정면 직각일 때 0.0 rad)
                    rot_mat, _ = cv2.Rodrigues(rvec)
                    tag_yaw_rad = math.atan2(rot_mat[0, 2], -rot_mat[2, 2])
                    tag_angle_deg = math.degrees(tag_yaw_rad)

                    norm_x_err = (tc_x - cx_img) / cx_img

                    detected = {
                        "id": tid,
                        "cx": tc_x, "cy": tc_y,
                        "dist": actual_dist,
                        "z_3d": z_3d,
                        "x_3d": x_3d,
                        "tag_angle_deg": tag_angle_deg,
                        "tag_angle_rad": tag_yaw_rad,
                        "norm_x_err": norm_x_err,
                        "corner": corner, "rvec": rvec, "tvec": tvec
                    }
                    self.last_seen_time = time.time()
                    self.last_center_x = tc_x
                    self.current_metric = {
                        "dist": round(actual_dist * 100, 1),
                        "x_px": int(tc_x - cx_img),
                        "yaw_deg": round(tag_angle_deg, 1),
                        "tag_id": tid
                    }

                vx = 0.0
                wz = 0.0

                if not self.docking_active:
                    self.substate = "IDLE"
                    self.state_desc = "대기 중 (IDLE)"
                    vx, wz = 0.0, 0.0
                elif detected is not None:
                    dist_err = detected["dist"] - self.target_dist
                    x_err_px = abs(detected["cx"] - cx_img)
                    yaw_err_deg = abs(detected["tag_angle_deg"])

                    # 0. 도킹 완료 조건 (거리 목표 ±3cm, 중심 ±25px, 각도 ±3.5도)
                    if abs(dist_err) <= self.lin_tol and x_err_px <= self.px_center_tol and yaw_err_deg <= self.yaw_tol_deg:
                        self.docked_counter += 1
                        if self.docked_counter >= 5:
                            self.substate = "DOCKED"
                            self.state_desc = "도킹완료! (DOCKED)"
                            vx, wz = 0.0, 0.0
                        else:
                            self.substate = "FINAL_ALIGN"
                            self.state_desc = "도킹 도달 (안정화 중)"
                            vx, wz = 0.0, 0.0

                    # 1. [공간 확보 후진]: 거리가 목표+10cm 이내인데 중심이 많이 벗어났거나(>35px) 각도가 많이 틀어진 경우
                    elif not self.has_backed_up and (detected["dist"] < (self.target_dist + 0.10) and (x_err_px > 35 or yaw_err_deg > 8.0)):
                        if self.backup_start_time == 0.0:
                            self.backup_start_time = time.time()

                        elapsed = time.time() - self.backup_start_time
                        if detected["dist"] < (self.target_dist + 0.20) and elapsed < 2.5:
                            self.substate = "ONESHOT_BACKUP"
                            self.state_desc = "1단계: 후진 공간확보 (직선후진)"
                            self.docked_counter = 0
                            vx = -0.055
                            wz = 0.0
                        else:
                            # 후진 완료 -> 전진 락 활성화
                            self.has_backed_up = True
                            self.substate = "FORWARD_APPROACH"
                            vx, wz = 0.0, 0.0

                    # 2. [전진 S-Curve 조향 접근]: 목표 거리를 향해 직진하면서 마커 중심축 및 벽면 각도를 동시에 일치
                    elif dist_err > self.lin_tol:
                        self.substate = "FORWARD_APPROACH"
                        self.state_desc = "2단계: S-Curve 정밀 직진정렬"
                        self.docked_counter = 0

                        raw_vx = 0.35 * dist_err
                        # 각도가 많이 틀어졌을 때는 감속하여 안정적인 조향 궤적 형성
                        if abs(yaw_err_deg) > 12.0 or x_err_px > 40:
                            raw_vx *= 0.65
                        vx = np.clip(raw_vx, self.min_vx, self.max_speed)

                        # 마커 중심 편차(norm_x_err) + 3D 벽면 수평각(tag_angle_rad) 복합 조향
                        raw_wz = - (0.45 * detected["norm_x_err"] + 0.35 * detected["tag_angle_rad"])
                        wz = np.clip(raw_wz, -self.max_wz, self.max_wz)

                        # 데드밴드: 중심과 각도가 모두 충분히 정렬되면 직진(wz=0)
                        if x_err_px <= 15 and yaw_err_deg <= self.yaw_tol_deg:
                            wz = 0.0
                        elif abs(wz) < self.min_wz and (x_err_px > 15 or yaw_err_deg > self.yaw_tol_deg):
                            wz = math.copysign(self.min_wz, wz)

                    # 3. [최종 미세 각도 트림]: 목표 거리 도착 후 제자리에서 각도만 정렬 (데드밴드 적용)
                    else:
                        self.substate = "FINAL_ALIGN"
                        self.state_desc = "3단계: 수평 각도정렬"
                        self.docked_counter = 0
                        vx = 0.0

                        # 각도 오차가 허용치(3.5도)를 넘을 때만 살짝 정렬하고, 범위 안이면 완벽 정지(wz=0)
                        if yaw_err_deg > self.yaw_tol_deg:
                            raw_wz = - 0.35 * detected["tag_angle_rad"]
                            wz = np.clip(raw_wz, -0.10, 0.10)
                            if abs(wz) < self.min_wz:
                                wz = math.copysign(self.min_wz, wz)
                        else:
                            wz = 0.0
                else:
                    self.docked_counter = 0
                    time_since_seen = time.time() - self.last_seen_time

                    if self.substate == "DOCKED":
                        vx, wz = 0.0, 0.0
                    elif time_since_seen < 1.2:
                        self.state_desc = "감속 대기 (HOLDING)"
                        vx, wz = 0.0, 0.0
                    else:
                        self.substate = "SEARCHING"
                        self.state_desc = "천천히 탐색 (SEARCHING)"
                        vx = 0.0
                        search_dir = -1.0 if self.last_center_x > cx_img else 1.0
                        wz = search_dir * self.search_wz

                if self.docking_active:
                    self.publish_vel(vx, wz)

                dbg = self.render_overlay(frame, detected, vx, wz)
                with self.lock:
                    self.debug_frame = dbg
                    self.latest_frame = frame

                if detected is not None:
                    sys.stdout.write(
                        f"\r[{self.state_desc:<25}] Tag:{detected['id']} | Dist:{detected['dist']*100:4.1f}cm | "
                        f"Yaw:{detected['tag_angle_deg']:+4.1f}° | ErrX:{detected['cx']-cx_img:+3.0f}px | Cmd: Vx={vx:+.2f}, Wz={wz:+.2f}   "
                    )
                else:
                    sys.stdout.write(
                        f"\r[{self.state_desc:<25}] Tag: None | Cmd: Vx={vx:+.2f}, Wz={wz:+.2f}                          "
                    )
                sys.stdout.flush()
                time.sleep(0.03)

        t_proc = threading.Thread(target=process_loop, daemon=True)
        t_proc.start()

        # 멀티스레드 웹 서버
        streamer = self
        local_ip = get_local_ip()

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, format, *args):
                pass

            def do_GET(self):
                if self.path == '/' or self.path == '/index.html':
                    html = f"""<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>AMR 직진 락-인 정밀 도킹 콘솔</title>
    <style>
        * {{ box-sizing: border-box; margin:0; padding:0; }}
        body {{
            background: #0b1120; color: #f8fafc; font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            display: flex; flex-direction: column; align-items: center; justify-content: center;
            min-height: 100vh; padding: 15px;
        }}
        .card {{
            background: #1e293b; border-radius: 16px; padding: 20px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.6); max-width: 680px; width: 100%;
            text-align: center; border: 1px solid #334155;
        }}
        h1 {{ font-size: 20px; color: #38bdf8; margin-bottom: 10px; }}
        .status-badge {{
            display: inline-block; padding: 6px 16px; border-radius: 20px;
            font-size: 14px; font-weight: bold; margin-bottom: 12px;
            background: #334155; color: #f8fafc; transition: 0.3s;
        }}
        .stream-box {{
            width: 100%; border-radius: 10px; overflow: hidden;
            background: #000; border: 2px solid #475569; margin-bottom: 15px;
        }}
        img {{ width: 100%; display: block; }}
        .btn-group {{ display: flex; gap: 10px; justify-content: center; margin-bottom: 12px; }}
        .btn {{
            background: #0284c7; color: white; border: none; padding: 12px 24px;
            border-radius: 8px; font-weight: bold; cursor: pointer; font-size: 15px;
            transition: 0.2s;
        }}
        .btn:hover {{ background: #0369a1; }}
        .btn-start {{ background: #10b981; }}
        .btn-start:hover {{ background: #059669; }}
        .btn-stop {{ background: #ef4444; }}
        .btn-stop:hover {{ background: #dc2626; }}
        .metric-grid {{
            display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 8px; margin-bottom: 10px;
        }}
        .metric-card {{
            background: #090d16; padding: 8px; border-radius: 6px; font-size: 12px; border: 1px solid #334155;
        }}
        .metric-card b {{ font-size: 15px; color: #38bdf8; display: block; margin-top: 2px; }}
    </style>
</head>
<body>
    <div class="card">
        <h1>🏷️ AMR 직진 락-인 정밀 도킹</h1>
        <div id="statusBadge" class="status-badge">상태: 대기 중 (IDLE)</div>
        <div class="stream-box">
            <img src="/stream.mjpg" alt="실시간 카메라 영상">
        </div>
        <div class="metric-grid">
            <div class="metric-card">거리 (목표 {streamer.target_dist*100:.0f}cm)<b id="valDist">0.0 cm</b></div>
            <div class="metric-card">중심 편차 (목표 0px)<b id="valCenter">0 px</b></div>
            <div class="metric-card">차체 각도 (목표 0.0°)<b id="valYaw">0.0°</b></div>
        </div>
        <div class="btn-group">
            <button class="btn btn-start" onclick="startDocking()">▶️ 자율 도킹 시작</button>
            <button class="btn btn-stop" onclick="stopDocking()">⏹️ 비상 정지</button>
        </div>
    </div>
    <script>
        function startDocking() {{
            fetch('/api/start').then(r=>r.json()).then(d=>{{
                document.getElementById('statusBadge').innerText = '상태: 도킹 주행 중 (ACTIVE)';
                document.getElementById('statusBadge').style.background = '#10b981';
            }});
        }}
        function stopDocking() {{
            fetch('/api/stop').then(r=>r.json()).then(d=>{{
                document.getElementById('statusBadge').innerText = '상태: 정지됨 (STOPPED)';
                document.getElementById('statusBadge').style.background = '#ef4444';
            }});
        }}
        setInterval(()=>{{
            fetch('/api/status').then(r=>r.json()).then(d=>{{
                const b = document.getElementById('statusBadge');
                b.innerText = '상태: ' + d.state;
                if(d.state.includes('도킹완료')) b.style.background = '#10b981';
                else if(d.state.includes('1단계') || d.state.includes('후진')) b.style.background = '#8b5cf6';
                else if(d.state.includes('2단계') || d.state.includes('전진')) b.style.background = '#0284c7';
                else if(d.state.includes('3단계') || d.state.includes('수평')) b.style.background = '#f59e0b';
                else b.style.background = '#334155';

                if(d.metrics) {{
                    document.getElementById('valDist').innerText = d.metrics.dist + ' cm';
                    document.getElementById('valCenter').innerText = (d.metrics.x_px > 0 ? '+' : '') + d.metrics.x_px + ' px';
                    document.getElementById('valYaw').innerText = (d.metrics.yaw_deg > 0 ? '+' : '') + d.metrics.yaw_deg + '°';
                }}
            }}).catch(()=>{{}});
        }}, 400);
    </script>
</body>
</html>"""
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.end_headers()
                    self.wfile.write(html.encode('utf-8'))

                elif self.path == '/api/start':
                    streamer.docking_active = True
                    streamer.has_backed_up = False
                    streamer.backup_start_time = 0.0
                    streamer.docked_counter = 0
                    print("\n▶️ [웹 제어] 자율 도킹 시작 명령 수신됨!")
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    self.wfile.write(b'{"status":"started"}')

                elif self.path == '/api/stop':
                    streamer.docking_active = False
                    streamer.publish_vel(0.0, 0.0)
                    print("\n⏹️ [웹 제어] 정지 명령 수신됨!")
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    self.wfile.write(b'{"status":"stopped"}')

                elif self.path == '/api/status':
                    res = json.dumps({"state": streamer.state_desc, "active": streamer.docking_active, "metrics": streamer.current_metric})
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    self.wfile.write(res.encode('utf-8'))

                elif self.path == '/stream.mjpg':
                    self.send_response(200)
                    self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                    self.end_headers()
                    try:
                        while streamer.running:
                            frame = None
                            with streamer.lock:
                                if streamer.debug_frame is not None:
                                    frame = streamer.debug_frame.copy()
                            if frame is not None:
                                ret, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                                if ret:
                                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
                            time.sleep(0.04)
                    except Exception:
                        pass
                else:
                    self.send_response(404)
                    self.end_headers()

        try:
            from http.server import ThreadingHTTPServer
            server = ThreadingHTTPServer(('0.0.0.0', self.port), Handler)
        except ImportError:
            from socketserver import ThreadingMixIn
            class ThreadedHTTPServer(ThreadingMixIn, HTTPServer): pass
            server = ThreadedHTTPServer(('0.0.0.0', self.port), Handler)

        print("\n" + "=" * 65)
        print("🚀 [AMR 직진 락-인 정밀 도킹 서버 준비 완료!]")
        print(f"👉 PC 브라우저 주소창에 아래 주소를 입력하세요:")
        print(f"   http://{local_ip}:{self.port}")
        print("=" * 65)
        print("💡 브라우저 화면에서 [▶️ 자율 도킹 시작] 버튼을 누르면 출발합니다!\n")

        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print("\n🛑 서버 종료")
        finally:
            self.running = False
            self.publish_vel(0.0, 0.0)
            self.cap.release()

    def render_overlay(self, frame, detected, vx, wz):
        dbg = frame.copy()
        h, w = dbg.shape[:2]
        cx, cy = w // 2, h // 2

        # 중앙 타겟 조준선
        cv2.line(dbg, (cx - 25, cy), (cx + 25, cy), (120, 120, 120), 1)
        cv2.line(dbg, (cx, cy - 25), (cx, cy + 25), (120, 120, 120), 1)
        cv2.rectangle(dbg, (cx - self.px_center_tol, 0), (cx + self.px_center_tol, h), (40, 60, 40), 1)

        if detected is not None:
            c = detected["corner"]
            pts = c.reshape((-1, 1, 2)).astype(np.int32)
            cv2.polylines(dbg, [pts], True, (0, 255, 0), 2)
            
            # 3D 좌표축 시각화 (X: 빨강, Y: 초록, Z: 파랑)
            if hasattr(cv2.aruco, 'drawAxis'):
                try:
                    cv2.aruco.drawAxis(dbg, self.camera_matrix, self.dist_coeffs,
                                       detected["rvec"], detected["tvec"], self.tag_size * 0.75)
                except Exception:
                    pass
            elif hasattr(cv2, 'drawFrameAxes'):
                try:
                    cv2.drawFrameAxes(dbg, self.camera_matrix, self.dist_coeffs,
                                      detected["rvec"], detected["tvec"], self.tag_size * 0.75)
                except Exception:
                    pass

            tc_x = int(detected["cx"])
            tc_y = int(detected["cy"])
            cv2.circle(dbg, (tc_x, tc_y), 5, (0, 0, 255), -1)
            cv2.line(dbg, (cx, cy), (tc_x, tc_y), (0, 255, 255), 1, cv2.LINE_AA)

            dist_txt = f"Tag:{detected['id']} | Dist:{detected['dist']*100:.1f}cm | Yaw:{detected['tag_angle_deg']:+.1f}deg"
            cv2.putText(dbg, dist_txt, (max(10, tc_x - 90), max(25, tc_y - 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

        overlay = dbg.copy()
        cv2.rectangle(overlay, (0, 0), (w, 50), (15, 15, 15), -1)
        cv2.addWeighted(overlay, 0.8, dbg, 0.2, 0, dbg)

        if "도킹완료" in self.state_desc:
            badge_color = (0, 200, 0)
        elif "1단계" in self.state_desc:
            badge_color = (139, 92, 246)  # 보라색 (후진)
        elif "2단계" in self.state_desc:
            badge_color = (0, 180, 255)   # 파란색 (직진)
        elif "3단계" in self.state_desc:
            badge_color = (245, 158, 11)  # 주황색 (수평정렬)
        else:
            badge_color = (100, 100, 100)

        cv2.rectangle(dbg, (8, 8), (250, 42), badge_color, -1)
        cv2.putText(dbg, self.state_desc.split(' ')[0], (15, 30), cv2.FONT_HERSHEY_DUPLEX, 0.52, (255, 255, 255), 1)

        info = f"목표: {self.target_dist*100:.0f}cm | Cmd: Vx={vx:+.2f}m/s, Wz={wz:+.2f}rad/s"
        cv2.putText(dbg, info, (265, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1, cv2.LINE_AA)

        return dbg


def main():
    parser = argparse.ArgumentParser(description="AMR AprilTag 도킹 & 웹 스트리머")
    parser.add_argument("--camera", default="0", help="카메라 디바이스 (0 또는 /dev/video0)")
    parser.add_argument("--no-flip", action="store_true", help="카메라 180도 회전(위아래 반전 보정) 끄기")
    parser.add_argument("--port", type=int, default=8080, help="웹 스트리밍 포트 (기본: 8080)")
    parser.add_argument("--tag-size", type=float, default=0.08, help="태그 검은색 사각형 크기 (m)")
    parser.add_argument("--target-dist", type=float, default=0.10, help="목표 정지 거리 (m, 기본: 0.10m = 10cm)")
    parser.add_argument("--max-speed", type=float, default=0.08, help="최대 선속도 (m/s)")
    parser.add_argument("--auto-start", action="store_true", help="실행하자마자 바로 자율 도킹 시작")

    args = parser.parse_args()
    cam_idx = int(args.camera) if args.camera.isdigit() else args.camera
    streamer = UnifiedDockingStreamer(
        camera_idx=cam_idx, port=args.port,
        tag_size=args.tag_size, target_dist=args.target_dist, max_speed=args.max_speed,
        auto_start=args.auto_start, flip_180=not args.no_flip
    )
    streamer.start_threads()


if __name__ == '__main__':
    main()
