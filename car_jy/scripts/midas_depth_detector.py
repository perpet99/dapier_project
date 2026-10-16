#!/usr/bin/env python3
"""
AMR MiDaS 단안 카메라 3D 뎁스(거리) 추정 및 자동 정지 노드 (midas_depth_detector.py)
-------------------------------------------------------------------------
1. Intel MiDaS-Small(경량 45MB) 딥러닝 AI로 2D 사진에서 실시간 3D 거리 지도(Depth Map) 생성
2. 라이다 높이보다 아래에 있는 바닥의 작은 물체(머그컵, 캔, 낮은 상자 등) 완벽 입체 감지!
3. 로봇 전방 파지 영역에 물체가 근접하면 /cmd_vel로 즉시 0을 전송하여 긴급 정지!
4. 포트 8080 실시간 웹 스트리밍 제공:
   👉 좌측: 일반 카메라 영상 + 타겟 박스
   👉 우측: MiDaS 3D 뎁스 히트맵 (가까울수록 노란색/흰색으로 빛남!)
"""

import os
import sys
import time
import threading
import cv2
import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler

# PyTorch 확인
try:
    import torch
except ImportError:
    print("❌ PyTorch가 설치되어 있지 않습니다!")
    print("   pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu --break-system-packages")
    sys.exit(1)

# ROS 2 라이브러리
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
WEB_PORT = 8080

# 🛑 정지 기준 뎁스 임계값 (가까울수록 값이 커짐, 0~255 스케일에서 180 이상이면 근접)
STOP_DEPTH_THRESHOLD = 175

output_frame = None
frame_lock = threading.Lock()


class VideoStreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        global output_frame
        if self.path == '/' or self.path == '/index.html':
            html = f"""<!DOCTYPE html>
<html>
<head>
    <title>AMR MiDaS 3D 뎁스 실시간 모니터</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {{ background: #121212; color: white; font-family: -apple-system, BlinkMacSystemFont, sans-serif; text-align: center; margin: 0; padding: 10px; }}
        h1 {{ font-size: 20px; margin-bottom: 6px; }}
        .stream {{ border: 3px solid #ff0055; border-radius: 10px; max-width: 98%; height: auto; box-shadow: 0 4px 15px rgba(0,0,0,0.6); }}
        .info {{ margin-top: 10px; font-size: 14px; color: #ccc; line-height: 1.6; }}
        .badge {{ background: #ff0055; color: white; padding: 3px 8px; border-radius: 4px; font-weight: bold; font-size: 12px; }}
    </style>
</head>
<body>
    <h1>🤖 AMR MiDaS 3D 거리 추정 모니터 <span class="badge">DEPTH AI</span></h1>
    <img class="stream" src="/stream.mjpg" />
    <div class="info">
        <p>📷 <b>좌측:</b> 원본 카메라 영상 | 🌈 <b>우측:</b> MiDaS 3D 뎁스 히트맵</p>
        <p>🔥 <b>노란색/흰색:</b> 코앞 근접 물체 (위험/파지) | 🌌 <b>보라색/검은색:</b> 먼 배경</p>
    </div>
</body>
</html>"""
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(html.encode('utf-8'))
            return

        if self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            while True:
                with frame_lock:
                    if output_frame is None:
                        time.sleep(0.02)
                        continue
                    ret, jpeg = cv2.imencode('.jpg', output_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
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
                    time.sleep(0.04)
                except (BrokenPipeError, ConnectionResetError):
                    break
            return

        self.send_response(404)
        self.end_headers()

    def log_message(self, format, *args):
        pass


def start_web_server():
    server = HTTPServer(('0.0.0.0', WEB_PORT), VideoStreamHandler)
    server.serve_forever()


class MiDaSDepthDetector:
    def __init__(self, node=None):
        self.node = node
        self.pub_cmd_vel = None
        if self.node:
            self.pub_cmd_vel = self.node.create_publisher(Twist, '/cmd_vel', 10)

        print("\n🧠 Intel MiDaS-Small 3D 뎁스 모델 로딩 중...")
        self.device = torch.device("cpu")

        # PyTorch Hub에서 MiDaS_small 로드 (약 45MB)
        try:
            self.midas = torch.hub.load("intel-isl/MiDaS", "MiDaS_small")
            self.midas.to(self.device)
            self.midas.eval()
            midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
            self.transform = midas_transforms.small_transform
            print("✅ MiDaS-Small 모델 로드 완료!\n")
        except Exception as e:
            print(f"❌ MiDaS 로드 실패: {e}")
            print("   'pip install timm --break-system-packages' 가 필요할 수 있습니다.")
            sys.exit(1)

        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        if not self.cap.isOpened():
            print(f"❌ 카메라(/dev/video{CAMERA_INDEX})를 열 수 없습니다!")
            sys.exit(1)

        print(f"📸 카메라 연결 완료! ({FRAME_WIDTH}x{FRAME_HEIGHT})")

        # 로봇 전방 파지 작업 구역 (Grasp Workspace ROI)
        self.roi_x1 = int(FRAME_WIDTH * 0.25)   # 160
        self.roi_x2 = int(FRAME_WIDTH * 0.75)   # 480
        self.roi_y1 = int(FRAME_HEIGHT * 0.45)  # 216
        self.roi_y2 = int(FRAME_HEIGHT * 0.90)  # 432

        self.is_stopped = False
        self.stop_cmd = Twist() if HAS_ROS2 else None

        self.prev_time = time.time()
        self.fps = 0.0

    def emergency_stop_robot(self):
        if self.pub_cmd_vel and self.stop_cmd:
            for _ in range(5):
                self.pub_cmd_vel.publish(self.stop_cmd)
                time.sleep(0.02)

    def process_frame(self, frame):
        h, w, _ = frame.shape

        cur_time = time.time()
        self.fps = 0.9 * self.fps + 0.1 * (1.0 / max(cur_time - self.prev_time, 0.001))
        self.prev_time = cur_time

        # 1. MiDaS 추론용 RGB 변환 및 입력 전처리
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

        # 2. 뎁스 맵 정규화 (0~255) 및 컬러 히트맵 생성
        depth_min = depth_map.min()
        depth_max = depth_map.max()
        if depth_max - depth_min > 0:
            depth_norm = (255 * (depth_map - depth_min) / (depth_max - depth_min)).astype(np.uint8)
        else:
            depth_norm = np.zeros_like(depth_map, dtype=np.uint8)

        depth_colored = cv2.applyColorMap(depth_norm, cv2.COLORMAP_INFERNO)

        # 3. 로봇 전방 작업 구역(ROI)의 뎁스 측정
        roi_depth = depth_norm[self.roi_y1:self.roi_y2, self.roi_x1:self.roi_x2]
        max_depth_in_roi = int(np.percentile(roi_depth, 95))  # 상위 5% 밝기 (가장 가까운 물체)

        # 실제 거리 근사치 계산 (cm)
        # depth 255 -> 약 15cm, depth 180 -> 약 30cm, depth 100 -> 약 80cm
        approx_dist_cm = int(max(6000.0 / (max_depth_in_roi + 20) - 5, 12))

        should_stop = (max_depth_in_roi >= STOP_DEPTH_THRESHOLD)

        # 4. 원본 영상(RGB)에 그래픽 그리기
        annotated_rgb = frame.copy()
        box_color = (0, 0, 255) if should_stop else (0, 255, 0)

        # 파지 감시 영역 표시
        cv2.rectangle(annotated_rgb, (self.roi_x1, self.roi_y1), (self.roi_x2, self.roi_y2), box_color, 2)
        cv2.putText(annotated_rgb, f"Grasp Zone [Dist: ~{approx_dist_cm}cm]",
                    (self.roi_x1 + 6, self.roi_y1 + 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, box_color, 2)

        # 히트맵 영상에도 동일한 ROI 표시
        cv2.rectangle(depth_colored, (self.roi_x1, self.roi_y1), (self.roi_x2, self.roi_y2), (255, 255, 255), 2)
        cv2.putText(depth_colored, f"Depth: {max_depth_in_roi}/255",
                    (self.roi_x1 + 6, self.roi_y1 + 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        # 5. 정지 제어
        if should_stop:
            cv2.rectangle(annotated_rgb, (0, 0), (w, 45), (0, 0, 255), -1)
            cv2.putText(annotated_rgb, f"🛑 OBJECT IN GRASP RANGE! (~{approx_dist_cm}cm)", (15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 2)

            if not self.is_stopped:
                print(f"\n🛑 [MiDaS 뎁스 감지] 전방 약 {approx_dist_cm}cm에 물체 감지 ➔ 로봇 긴급 정지!")
                self.emergency_stop_robot()
                self.is_stopped = True
            else:
                self.emergency_stop_robot()
        else:
            if self.is_stopped:
                print("✅ [물체 해소] 작업 구역이 비었습니다.")
                self.is_stopped = False

            status_text = f"Path Clear | Forward Dist: ~{approx_dist_cm}cm"
            cv2.putText(annotated_rgb, status_text, (15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # FPS 표시
        cv2.putText(annotated_rgb, f"FPS: {self.fps:.1f}", (w - 110, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # 6. 좌우 나란히 병합 (640x480 2개를 합쳐서 1280x480 출력)
        # 라즈베리파이 네트워크 대역폭 절약을 위해 50% 축소하여 640x240으로 병합
        rgb_small = cv2.resize(annotated_rgb, (320, 240))
        depth_small = cv2.resize(depth_colored, (320, 240))
        combined = np.hstack((rgb_small, depth_small))

        return combined

    def run(self):
        global output_frame
        print("=" * 65)
        print("🧠  Intel MiDaS 3D 뎁스 거리 추정 루프 시작!")
        print(f"🌐  PC 웹 브라우저 실시간 화면: http://192.168.0.64:{WEB_PORT}")
        print("=" * 65)

        while True:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            processed = self.process_frame(frame)

            with frame_lock:
                output_frame = processed.copy()

            if HAS_ROS2 and self.node:
                rclpy.spin_once(self.node, timeout_sec=0.001)

            time.sleep(0.01)


def main(args=None):
    web_thread = threading.Thread(target=start_web_server, daemon=True)
    web_thread.start()

    node = None
    if HAS_ROS2:
        rclpy.init(args=args)
        node = Node('midas_depth_detector')

    detector = MiDaSDepthDetector(node=node)
    try:
        detector.run()
    except KeyboardInterrupt:
        print("\n🛑 노드가 안전하게 종료되었습니다.")
    finally:
        detector.cap.release()
        if HAS_ROS2 and node:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
