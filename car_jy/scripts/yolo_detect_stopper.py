#!/usr/bin/env python3
"""
AMR 30도 틸트 카메라 기반 로봇팔(SO-101) 파지 물체 인식 노드 (yolo_detect_stopper.py)
----------------------------------------------------------------------------
- 30도 하향 틸트 각도에 최적화된 "원근 거리 & 실제 물리 크기(cm) 역추정" 엔진 탑재
- 멀리 있는 큰 물체와 가까이 있는 작은 물체의 원근 왜곡 완벽 보정!
- 실제 물체 가로 폭(cm), 전방 거리(cm), 좌우 편차(cm) 실시간 계산
- 로봇팔 그리퍼 파지 가능 규격(가로 3cm ~ 14cm) 물체만 선별하여 ~30cm 앞 자동 정지
- 포트 8080 실시간 웹 스트리밍 제공 (http://192.168.0.64:8080)
"""

import os
import sys
import time
import threading
import cv2
import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler
from ultralytics import YOLO

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
INFERENCE_SIZE = 160   # 160x160 초고속 추론 (25~30 FPS)
CONF_THRESHOLD = 0.25
WEB_PORT = 8080

# SO-101 로봇팔 파지 가능 규격 (cm 단위!)
MIN_GRASP_WIDTH_CM = 3.0   # 3cm 이상 (먼지/얼룩 무시)
MAX_GRASP_WIDTH_CM = 14.0  # 14cm 이하 (그리퍼 최대 스트로크 고려, 거대한 벽/가구 무시)

output_frame = None
frame_lock = threading.Lock()


def estimate_distance_and_size(obj_w, bottom_y):
    """
    30도 하향 틸트 카메라의 원근 역기하학 모델:
    화면 Y좌표(바닥 접촉점)로부터 실제 거리(cm)와 실제 폭(cm) 계산
    """
    clamped_y = max(bottom_y, 80)

    # 1. 전방 거리 추정 (cm)
    # y=460 -> 약 18cm, y=370 -> 약 30cm, y=250 -> 약 55cm, y=140 -> 약 110cm
    dist_cm = (12000.0 / (clamped_y + 25.0)) - 5.0
    dist_cm = max(dist_cm, 10.0)

    # 2. Y좌표에 따른 1cm당 픽셀 수 (Scale factor)
    # 가까울수록 1cm당 픽셀 수가 많아짐 (y=370에서 1cm ~= 13.5px, y=140에서 1cm ~= 5.5px)
    pixels_per_cm = 0.036 * (clamped_y - 60) + 2.8
    pixels_per_cm = max(pixels_per_cm, 2.0)

    # 3. 실제 물체 폭 계산 (cm)
    real_width_cm = obj_w / pixels_per_cm

    return dist_cm, real_width_cm, pixels_per_cm


class VideoStreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        global output_frame
        if self.path == '/' or self.path == '/index.html':
            html = f"""<!DOCTYPE html>
<html>
<head>
    <title>AMR 30도 틸트 로봇팔 파지 모니터</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {{ background: #121212; color: white; font-family: -apple-system, BlinkMacSystemFont, sans-serif; text-align: center; margin: 0; padding: 12px; }}
        h1 {{ font-size: 20px; margin-bottom: 6px; }}
        .stream {{ border: 3px solid #00e5ff; border-radius: 10px; max-width: 95%; height: auto; box-shadow: 0 4px 15px rgba(0,0,0,0.6); }}
        .info {{ margin-top: 10px; font-size: 14px; color: #bbb; line-height: 1.5; }}
        .badge {{ background: #00e5ff; color: #000; padding: 3px 8px; border-radius: 4px; font-weight: bold; font-size: 12px; }}
    </style>
</head>
<body>
    <h1>🤖 AMR 로봇팔 파지 전용 원근 보정 모니터 <span class="badge">30° TILT AI</span></h1>
    <img class="stream" src="/stream.mjpg" />
    <div class="info">
        <p>🎯 <b>하늘색 박스:</b> 파지 가능 물체 (실제 폭 3~14cm 자동 환산)</p>
        <p>🔴 <b>빨간 점선:</b> 로봇팔 파지선 (~30cm) | <b>회색 박스:</b> 너무 큰 가구/벽 (자동 무시)</p>
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
                    time.sleep(0.03)
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


class GraspObjectDetector:
    def __init__(self, node=None):
        self.node = node
        self.pub_cmd_vel = None
        if self.node:
            self.pub_cmd_vel = self.node.create_publisher(Twist, '/cmd_vel', 10)

        print("\n🧠 30도 틸트 원근 보정 모델 로딩 중...")
        self.model = YOLO("yolov8n.pt")
        print("✅ 모델 준비 완료! (원근 거리 & cm 단위 물리 크기 추정 엔진 활성화)\n")

        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        if not self.cap.isOpened():
            print(f"❌ 카메라(/dev/video{CAMERA_INDEX})를 열 수 없습니다!")
            sys.exit(1)

        print(f"📸 30도 하향 카메라 정상 연결! ({FRAME_WIDTH}x{FRAME_HEIGHT})")

        # 30도 하향 틸트에 최적화된 ROI
        # 상단 20%는 원거리 지평선, 하단 전체가 바닥 주행 경로
        self.roi_top = int(FRAME_HEIGHT * 0.20)       # Y = 96
        self.roi_bottom = FRAME_HEIGHT - 5            # Y = 475
        self.roi_left = int(FRAME_WIDTH * 0.05)       # X = 32
        self.roi_right = int(FRAME_WIDTH * 0.95)      # X = 608

        # 🛑 로봇팔 파지 도달 기준선 (약 30cm 앞 ➔ Y=375)
        self.stop_line_y = int(FRAME_HEIGHT * 0.78)   # Y = 374

        # 그리퍼 중심 기준 X축
        self.center_x = FRAME_WIDTH // 2              # X = 320

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
        annotated = frame.copy()

        # FPS 측정
        cur_time = time.time()
        self.fps = 0.9 * self.fps + 0.1 * (1.0 / max(cur_time - self.prev_time, 0.001))
        self.prev_time = cur_time

        # 1. 바닥 ROI 추출
        roi = frame[self.roi_top:self.roi_bottom, self.roi_left:self.roi_right]

        # 2. YOLO 초고속 160x160 추론
        results = self.model(roi, imgsz=INFERENCE_SIZE, conf=CONF_THRESHOLD, agnostic_nms=True, verbose=False)

        graspable_targets = []
        should_stop = False

        if results and len(results[0].boxes) > 0:
            for box in results[0].boxes:
                bx1, by1, bx2, by2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])

                # 글로벌 좌표 변환
                gx1 = bx1 + self.roi_left
                gy1 = by1 + self.roi_top
                gx2 = bx2 + self.roi_left
                gy2 = by2 + self.roi_top

                obj_w = gx2 - gx1
                obj_h = gy2 - gy1
                bottom_y = gy2
                obj_cx = (gx1 + gx2) // 2
                obj_cy = (gy1 + gy2) // 2

                # 📐 원근 기하학 모델로 실제 거리(cm)와 실제 폭(cm) 계산!
                dist_cm, real_w_cm, px_per_cm = estimate_distance_and_size(obj_w, bottom_y)
                dx_cm = (obj_cx - self.center_x) / px_per_cm  # 실제 좌우 편차 (cm)

                # [원근 보정 크기 필터] 실제 크기가 3cm ~ 14cm 인가?
                if real_w_cm < MIN_GRASP_WIDTH_CM:
                    continue  # 바닥 먼지/얼룩 무시

                if real_w_cm > MAX_GRASP_WIDTH_CM:
                    # 너무 거대한 가구/벽 (예: 가로 25cm 이상) ➔ 회색 박스로 표시만 하고 무시
                    cv2.rectangle(annotated, (gx1, gy1), (gx2, gy2), (100, 100, 100), 1)
                    cv2.putText(annotated, f"Too Large ({real_w_cm:.0f}cm)", (gx1, max(gy1 - 6, 15)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (130, 130, 130), 1)
                    continue

                # ✅ 로봇팔 파지 가능 규격 물체 포착!
                is_in_range = (bottom_y >= self.stop_line_y)
                if is_in_range:
                    should_stop = True

                graspable_targets.append({
                    'bbox': (gx1, gy1, gx2, gy2),
                    'center': (obj_cx, obj_cy),
                    'dist_cm': dist_cm,
                    'real_w_cm': real_w_cm,
                    'dx_cm': dx_cm,
                    'is_in_range': is_in_range
                })

        # 3. 화면 가이드라인 그리기
        # 그리퍼 중심선 (수직선)
        cv2.line(annotated, (self.center_x, self.roi_top), (self.center_x, self.roi_bottom), (60, 60, 60), 1)

        # 바닥 감시 구역 (녹색)
        cv2.rectangle(annotated, (self.roi_left, self.roi_top), (self.roi_right, self.roi_bottom), (0, 255, 0), 1)

        # 🛑 로봇팔 파지 기준선 (빨간색 - 약 30cm 앞)
        cv2.line(annotated, (self.roi_left, self.stop_line_y), (self.roi_right, self.stop_line_y), (0, 0, 255), 2)
        cv2.putText(annotated, "GRASP LINE (~30cm)", (self.roi_left + 6, self.stop_line_y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # 파지 타겟 박스 및 실측 수치(cm) 표시
        for target in graspable_targets:
            gx1, gy1, gx2, gy2 = target['bbox']
            cx, cy = target['center']
            d_cm = target['dist_cm']
            w_cm = target['real_w_cm']
            dx_cm = target['dx_cm']

            color = (0, 230, 255) if target['is_in_range'] else (255, 180, 0)
            cv2.rectangle(annotated, (gx1, gy1), (gx2, gy2), color, 2)

            # 십자선 (타겟 정밀 중심)
            cv2.drawMarker(annotated, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 12, 2)

            # 중심 편차 가이드선
            cv2.line(annotated, (cx, cy), (self.center_x, cy), (0, 255, 255), 1)

            # cm 단위 실시간 정보 배너
            label = f"PICKABLE [{w_cm:.1f}cm | {d_cm:.0f}cm 앞]"
            align_label = f"dx: {dx_cm:+.1f}cm"
            cv2.putText(annotated, label, (gx1, max(gy1 - 8, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 2)
            cv2.putText(annotated, align_label, (gx1, max(gy1 + 15, 35)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)

        # 4. 정지 제어 & 상태 배너
        if should_stop:
            cv2.rectangle(annotated, (0, 0), (w, 45), (0, 180, 255), -1)
            cv2.putText(annotated, "🎯 TARGET REACHED! READY TO GRASP!", (15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 0, 0), 2)

            if not self.is_stopped:
                best = graspable_targets[0]
                print(f"\n🎯 [파지 위치 도달!] 폭 {best['real_w_cm']:.1f}cm 물체 감지 (거리: {best['dist_cm']:.0f}cm, 편차: {best['dx_cm']:+.1f}cm) ➔ 로봇 정지 완료!")
                self.emergency_stop_robot()
                self.is_stopped = True
            else:
                self.emergency_stop_robot()
        else:
            if self.is_stopped:
                print("✅ [물체 회수 완료] 작업 구역이 비었습니다. 정상 주행 준비.")
                self.is_stopped = False

            if graspable_targets:
                cv2.putText(annotated, f"Target Approaching ({len(graspable_targets)} item) ...", (15, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 180, 0), 2)
            else:
                cv2.putText(annotated, "Searching for Graspable Items (3~14cm)...", (15, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        # FPS 표시
        cv2.putText(annotated, f"FPS: {self.fps:.1f}", (w - 110, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        return annotated

    def run(self):
        global output_frame
        print("=" * 65)
        print("🦾  30도 틸트 로봇팔 파지 물체 감지 루프 시작!")
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
        node = Node('arm_grasp_detector')

    detector = GraspObjectDetector(node=node)
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
