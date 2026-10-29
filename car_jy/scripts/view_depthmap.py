#!/usr/bin/env python3
"""
카메라 실시간 3D 뎁스맵 & RGB-D 융합 물체 분리 뷰어 (view_depthmap.py)
------------------------------------------------------------------
- Intel MiDaS 3D 뎁스 추론 및 바닥 자동 분리 (RGB-D Fusion)
- 웹 실시간 MJPEG 스트리밍 (포트 8080) & VLM AI 판별 REST API
- 다중 쓰레드 모듈화 아키텍처
"""

import os
import sys
import time
import select
import threading
import cv2

# bot_modules 임포트 경로 보장
_cur_dir = os.path.dirname(os.path.abspath(__file__))
_root_dir = os.path.dirname(_cur_dir) if os.path.basename(_cur_dir) == 'scripts' else _cur_dir
for _p in [_root_dir, os.path.expanduser("~/ros2_amr_ws")]:
    if os.path.exists(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from bot_modules.depth_stream_server import start_web_server, stream_state, get_local_ip
from bot_modules.rgbd_segmentor import RGBDSegmentor
from bot_modules.depth_renderer import DepthRenderer

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CAMERA_INDEX = 0
WEB_PORT = 8080


def find_working_camera(preferred_idx=0):
    """연결 가능한 카메라 디바이스 자동 탐색"""
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


class DepthMapViewerApp:
    """AMR 실시간 RGB-D 융합 뷰어 통합 애플리케이션"""
    def __init__(self):
        print("\n" + "=" * 68)
        print("   🤖 Intel MiDaS 실시간 RGB-D 융합 물체 분리 뷰어 초기화")
        print("=" * 68)

        # 1. 카메라 연결
        print("📸 [1/2] 카메라 디바이스 자동 탐색 중...")
        self.cap, self.camera_idx = find_working_camera(CAMERA_INDEX)
        if self.cap is None:
            print("❌ 카메라(/dev/video0 ~ 4)를 열 수 없습니다! USB 케이블과 권한을 확인하세요.")
            sys.exit(1)
        print(f"✅ 카메라 정상 연결! /dev/video{self.camera_idx} ({FRAME_WIDTH}x{FRAME_HEIGHT})")

        # 2. 알고리즘 & 렌더러 초기화
        self.segmentor = RGBDSegmentor(FRAME_WIDTH, FRAME_HEIGHT)
        self.renderer = DepthRenderer(FRAME_WIDTH, FRAME_HEIGHT)

        # 3. 제어 변수
        self.fps = 0.0
        self.prev_time = time.time()
        self.auto_capture_enabled = True
        self.auto_capture_max_dist_cm = 55
        self.has_display = bool(os.environ.get("DISPLAY"))

    def handle_keyboard_char(self, char):
        """키보드 입력(터미널 또는 OpenCV 윈도우) 공통 처리"""
        if char in ('q', 'quit', 'exit'):
            return False
        elif char in ('i', 'v', 'inspect', 'vlm'):
            success, rem = stream_state.trigger_snapshot(reason="manual")
            if success:
                print(f"📸 [스냅샷 저장 완료] {stream_state.snapshot_file} (10초 쿨다운 시작)")
                print("    👉 채팅창에 '앞에 물체 분석해줘'라고 하시면 AI가 즉시 판별합니다!")
            else:
                print(f"⏳ [캡처 쿨다운 중] 아직 10초가 지나지 않았습니다. (남은 시간: {rem}초)")
        elif char in ('b', 'floor'):
            self.renderer.show_floor_overlay = not self.renderer.show_floor_overlay
            print(f"🟦 바닥 시각화 오버레이: {'[ON] 푸른색 표시' if self.renderer.show_floor_overlay else '[OFF] 숨김'}")
        elif char in ('d', 'sobel', 'step'):
            self.renderer.show_depth_edges = not self.renderer.show_depth_edges
            print(f"📐 Depth Sobel 3D 단차 모서리선: {'[ON] 형광 노란색 표시' if self.renderer.show_depth_edges else '[OFF] 숨김'}")
        elif char in ('a', 'auto'):
            self.auto_capture_enabled = not self.auto_capture_enabled
            print(f"⚡ 자동 감지 캡처: {'[ON] 물체 감지 시 10초마다 자동 캡처' if self.auto_capture_enabled else '[OFF] 수동 캡처 전용'}")
        elif char in (']', '}'):
            self.segmentor.color_thresh = min(100, self.segmentor.color_thresh + 3)
            print(f"🎨 바닥 색상 허용 오차: {self.segmentor.color_thresh}")
        elif char in ('[', '{'):
            self.segmentor.color_thresh = max(8, self.segmentor.color_thresh - 3)
            print(f"🎨 바닥 색상 허용 오차: {self.segmentor.color_thresh}")
        elif char in ('f', 'fusion'):
            self.segmentor.fusion_mode = not self.segmentor.fusion_mode
            print(f"🔄 융합 모드: {'[ON] 바닥 제거 활성화' if self.segmentor.fusion_mode else '[OFF] 순수 뎁스 모드'}")
        elif char in ('t', 'tare'):
            self.segmentor.lock_floor_color = not self.segmentor.lock_floor_color
            state_str = "🔒 바닥 색상 고정 (Locked)" if self.segmentor.lock_floor_color else "🔄 바닥 색상 자동 추적 (Auto)"
            print(f"⚖️ {state_str}")
        elif char in ('+', '='):
            self.segmentor.target_threshold = min(245, self.segmentor.target_threshold + 5)
            print(f"📏 뎁스 임계치: {self.segmentor.target_threshold} pt")
        elif char in ('-', '_'):
            self.segmentor.target_threshold = max(70, self.segmentor.target_threshold - 5)
            print(f"📏 뎁스 임계치: {self.segmentor.target_threshold} pt")
        elif char in ('c', 'color'):
            self.renderer.use_colormap = not self.renderer.use_colormap
            print(f"🎨 컬러맵 모드: {'MAGMA' if self.renderer.use_colormap else 'Grayscale'}")
        return True

    def run(self):
        local_ip = get_local_ip()
        print("=" * 68)
        print("🎉  실시간 RGB-D 융합 물체 분리 뷰어 스트리밍 시작!")
        print(f"🌐  웹 브라우저 접속 주소: http://{local_ip}:{WEB_PORT}")
        print("⌨️   키보드 조작: 'i' 캡처 | 'a' 자동캡처 | 'd' 3D단차 | 'b' 바닥표시 | 'f' 융합모드 | 'q' 종료")
        print("=" * 68 + "\n")

        while True:
            # 1. 터미널 표준입력 비차단 감지
            if sys.stdin.isatty():
                r, _, _ = select.select([sys.stdin], [], [], 0)
                if r:
                    line = sys.stdin.readline().strip().lower()
                    if not self.handle_keyboard_char(line):
                        break

            # 2. 카메라 프레임 캡처
            ret, frame = self.cap.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            # 카메라 상하 반전 보정
            frame = cv2.flip(frame, 0)

            # FPS 연산
            cur_time = time.time()
            self.fps = 0.9 * self.fps + 0.1 * (1.0 / max(cur_time - self.prev_time, 0.001))
            self.prev_time = cur_time

            # 3. 3D 뎁스 및 바닥 분리 추론
            seg_res = self.segmentor.process(frame)

            # 4. 자동 감지 캡처 발동 체크
            if self.auto_capture_enabled and seg_res.nearest_dist_cm and seg_res.nearest_dist_cm <= self.auto_capture_max_dist_cm:
                success, _ = stream_state.trigger_snapshot(reason="auto")
                if success:
                    print(f"\n⚡ [자동 캡처 발동] 전방 ~{seg_res.nearest_dist_cm}cm 물체 감지 (10초 쿨다운 적용)")

            # 5. 프레임 렌더링 및 웹 스트림 공유
            with stream_state.vlm_lock:
                vlm_copy = stream_state.vlm_result.copy()

            combined = self.renderer.compose_frame(
                frame, seg_res, self.segmentor, self.fps, vlm_copy, self.auto_capture_enabled
            )
            stream_state.set_frame(combined)

            # 6. 로컬 GUI 윈도우 (GUI 지원 환경)
            if self.has_display:
                cv2.imshow("AMR RGB-D Fusion Object Segmentation", combined)
                key = cv2.waitKey(1) & 0xFF
                if key != 255:
                    if not self.handle_keyboard_char(chr(key)):
                        break

            time.sleep(0.01)


def main():
    # 웹 서버 백그라운드 구동
    web_thread = threading.Thread(target=start_web_server, args=(WEB_PORT,), daemon=True)
    web_thread.start()

    viewer = DepthMapViewerApp()
    try:
        viewer.run()
    except KeyboardInterrupt:
        print("\n🛑 뷰어가 안전하게 종료되었습니다.")
    finally:
        viewer.cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
