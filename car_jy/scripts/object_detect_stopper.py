#!/usr/bin/env python3
"""
AMR 카메라 전방 물체 감지 및 자동 정지 노드 (object_detect_stopper.py)
-------------------------------------------------------------------
1. 라즈베리파이 카메라(/dev/video0)로 실시간 영상 획득 (OpenCV 기반)
2. 상단(벽, 천장, 책상 상판)을 제외하고 바닥 주행 경로(ROI)만 감시
3. 바닥에 놓인 물체(상자, 신발, 컵 등)를 실시간 감지하여 경계 박스(Bounding Box) 추적
4. 물체가 정지선(약 30~40cm 앞)에 도달하면 즉시 /cmd_vel에 0을 전송하여 긴급 정지!
5. 포트 8080 웹 스트리밍 제공 (PC 크롬에서 http://192.168.0.64:8080 으로 실시간 확인!)
"""

import os
import sys
import time
import threading
import cv2
import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler

# ROS 2 라이브러리 (ROS 2 환경이 없어도 영상 테스트 가능하도록 예외 처리)
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

# 전역 프레임 저장소 (웹 스트리밍용)
output_frame = None
frame_lock = threading.Lock()


class VideoStreamHandler(BaseHTTPRequestHandler):
    """웹 브라우저로 실시간 영상(MJPEG)을 쏴주는 핸들러"""
    def do_GET(self):
        global output_frame
        if self.path == '/' or self.path == '/index.html':
            html = f"""<!DOCTYPE html>
<html>
<head>
    <title>AMR 전방 물체 감지 실시간 모니터</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {{ background: #121212; color: white; font-family: sans-serif; text-align: center; margin: 0; padding: 15px; }}
        h1 {{ font-size: 20px; margin-bottom: 10px; }}
        .stream {{ border: 3px solid #00ff66; border-radius: 8px; max-width: 95%; height: auto; }}
        .info {{ margin-top: 10px; font-size: 14px; color: #aaa; }}
    </style>
</head>
<body>
    <h1>🤖 AMR 실시간 전방 물체 감지 모니터</h1>
    <img class="stream" src="/stream.mjpg" />
    <div class="info">
        <p>🟢 <b>초록 사각형:</b> 바닥 감시 영역(ROI) | 🔴 <b>빨간 점선:</b> 정지 기준선 (약 35cm 앞)</p>
        <p>물체가 빨간 선을 넘으면 모터가 즉시 자동 정지합니다!</p>
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
                        time.sleep(0.03)
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
                    time.sleep(0.04)  # ~25 FPS
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


class ObjectDetectStopper:
    def __init__(self, node=None):
        self.node = node
        self.pub_cmd_vel = None
        if self.node:
            self.pub_cmd_vel = self.node.create_publisher(Twist, '/cmd_vel', 10)

        # 카메라 초기화
        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        if not self.cap.isOpened():
            print(f"❌ 카메라(/dev/video{CAMERA_INDEX})를 열 수 없습니다! 권한을 확인하세요.")
            sys.exit(1)

        print(f"✅ 카메라(/dev/video{CAMERA_INDEX}) 열기 성공! ({FRAME_WIDTH}x{FRAME_HEIGHT})")

        # 바닥 영역 ROI (Region of Interest) 설정
        # 상단 45%는 벽/책상이므로 무시, 하단 55% 바닥 주행면만 검사
        self.roi_top = int(FRAME_HEIGHT * 0.45)      # Y = 216
        self.roi_bottom = FRAME_HEIGHT - 10           # Y = 470
        self.roi_left = int(FRAME_WIDTH * 0.15)       # X = 96
        self.roi_right = int(FRAME_WIDTH * 0.85)     # X = 544

        # 🛑 정지 기준선 (물체의 밑변이 이 높이보다 아래로 내려오면 약 35cm 앞 ➔ 정지!)
        self.stop_line_y = int(FRAME_HEIGHT * 0.78)   # Y = 374

        self.is_stopped = False
        self.stop_cmd = None
        if HAS_ROS2:
            self.stop_cmd = Twist()

        # 배경 분리 학습기 (바닥면과 다른 물체 분리)
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=200, varThreshold=25, detectShadows=False
        )

    def emergency_stop_robot(self):
        """로봇에게 즉시 정지 신호 전송"""
        if self.pub_cmd_vel and self.stop_cmd:
            for _ in range(5):
                self.pub_cmd_vel.publish(self.stop_cmd)
                time.sleep(0.02)

    def process_frame(self, frame):
        """프레임에서 물체 검출 및 정지 판단"""
        h, w, _ = frame.shape
        annotated = frame.copy()

        # 1. 감시 영역(ROI) 설정 표시
        cv2.rectangle(annotated, (self.roi_left, self.roi_top), (self.roi_right, self.roi_bottom), (0, 255, 0), 2)
        cv2.putText(annotated, "Floor ROI (Detection Zone)", (self.roi_left + 5, self.roi_top + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # 2. 🛑 빨간색 정지 기준선 그리기
        cv2.line(annotated, (self.roi_left, self.stop_line_y), (self.roi_right, self.stop_line_y), (0, 0, 255), 2)
        cv2.putText(annotated, "STOP LINE (~35cm)", (self.roi_left + 5, self.stop_line_y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # 3. 바닥 ROI 추출
        roi = frame[self.roi_top:self.roi_bottom, self.roi_left:self.roi_right]
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray_roi, (7, 7), 0)

        # 바닥면 엣지 및 이질도 검출 (Adaptive Thresholding + Canny)
        edges = cv2.Canny(blurred, 35, 100)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)

        # 윤곽선(Contour) 검출
        contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_objects = []
        should_stop = False

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 800:  # 너무 작은 잡음 제거
                continue

            bx, by, bw, bh = cv2.boundingRect(cnt)

            # 글로벌 좌표로 변환
            gx = bx + self.roi_left
            gy = by + self.roi_top
            bottom_y = gy + bh  # 물체의 바닥 접촉점

            detected_objects.append((gx, gy, bw, bh, bottom_y))

            # 물체 바운딩 박스 표시 (노란색)
            cv2.rectangle(annotated, (gx, gy), (gx + bw, gy + bh), (0, 255, 255), 2)
            cv2.circle(annotated, (gx + bw // 2, bottom_y), 4, (0, 0, 255), -1)

            # 물체의 바닥이 정지 기준선을 넘었는지 확인!
            if bottom_y >= self.stop_line_y:
                should_stop = True

        # 4. 정지 제어
        if should_stop:
            cv2.rectangle(annotated, (0, 0), (w, 50), (0, 0, 255), -1)
            cv2.putText(annotated, "🛑 OBJECT DETECTED! STOPPING ROBOT!", (20, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            if not self.is_stopped:
                print("\n🛑 [긴급 정지!] 전방 35cm에 물체가 감지되었습니다! 로봇을 정지합니다.")
                self.emergency_stop_robot()
                self.is_stopped = True
            else:
                self.emergency_stop_robot()
        else:
            if self.is_stopped:
                print("✅ [물체 해소] 전방에 물체가 사라졌습니다.")
                self.is_stopped = False

            if detected_objects:
                cv2.putText(annotated, f"Tracking {len(detected_objects)} Objects (Approaching...)", (20, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            else:
                cv2.putText(annotated, "Path Clear - Monitoring...", (20, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        return annotated

    def run(self):
        global output_frame
        print("=" * 65)
        print(f"👁️  전방 물체 감지 및 자동 정지 루프가 시작되었습니다!")
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

            time.sleep(0.02)


def main(args=None):
    # 웹 스트리밍 서버 백그라운드 시작
    web_thread = threading.Thread(target=start_web_server, daemon=True)
    web_thread.start()

    node = None
    if HAS_ROS2:
        rclpy.init(args=args)
        node = Node('object_detect_stopper')

    stopper = ObjectDetectStopper(node=node)
    try:
        stopper.run()
    except KeyboardInterrupt:
        print("\n🛑 노드가 안전하게 종료되었습니다.")
    finally:
        stopper.cap.release()
        if HAS_ROS2 and node:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
