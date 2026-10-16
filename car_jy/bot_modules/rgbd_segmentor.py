#!/usr/bin/env python3
"""
bot_modules/rgbd_segmentor.py
------------------------------
Intel MiDaS 3D 뎁스 추론 및 RGB-D 융합 바닥 제거 / 물체 분리(Segmentation) 전담 모듈
"""

import sys
import numpy as np
import cv2

try:
    import torch
except ImportError:
    print("❌ PyTorch가 설치되어 있지 않습니다!")
    print("   설치 명령어: pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu --break-system-packages")
    sys.exit(1)


class SegmentationResult:
    """RGB-D 물체 분리 결과 데이터 컨테이너"""
    def __init__(self):
        self.depth_norm = None
        self.clean_mask = None
        self.is_floor_mask = None
        self.depth_step_edges = None
        self.detected_objects = []
        self.floor_coverage_pct = 0
        self.floor_color_bgr = None
        self.nearest_dist_cm = None


class RGBDSegmentor:
    """RGB-D 융합 바닥 분리 및 물체 감지 엔진"""
    def __init__(self, frame_width=640, frame_height=480, target_threshold=145):
        self.w = frame_width
        self.h = frame_height
        self.device = torch.device("cpu")

        # 1. MiDaS-Small 모델 로드
        print("🧠 [1/2] MiDaS AI 모델 로딩 중... (가중치 45MB)")
        try:
            self.midas = torch.hub.load("intel-isl/MiDaS", "MiDaS_small", trust_repo=True)
            self.midas.to(self.device)
            self.midas.eval()
            midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms", trust_repo=True)
            self.transform = midas_transforms.small_transform
            print("✅ MiDaS AI 뎁스 모델 준비 완료!\n")
        except Exception as e:
            print(f"\n❌ MiDaS 로드 실패: {e}")
            err_str = str(e).lower()
            if "timm" in err_str or "attributeerror" in err_str:
                print("💡 [원인 분석] 최신 timm(1.0+)과 MiDaS 간 버전 충돌입니다.")
                print("👉 해결 명령어: pip install timm==0.6.12 --break-system-packages")
            sys.exit(1)

        # [통로 영역 ROI] (가로 72%, 세로 22%~88%)
        roi_w = int(self.w * 0.72)
        self.roi_x1 = (self.w - roi_w) // 2
        self.roi_x2 = self.roi_x1 + roi_w
        self.roi_y1 = int(self.h * 0.22)
        self.roi_y2 = int(self.h * 0.88)

        # [바닥 기준 색상 샘플링 구역] (좌/우 하단 코너)
        self.floor_boxes = [
            (25, 428, 115, 475),
            (525, 428, 615, 475)
        ]

        # 알고리즘 튜닝 파라미터
        self.target_threshold = target_threshold  # 뎁스 근접 임계치
        self.color_thresh = 18                    # 바닥 Lab 색상 차이 임계치
        self.protrusion_thresh = 14               # 3D 솟아오름 돌출도 임계치
        self.min_object_area = 450                # 최소 물체 면적
        self.depth_sobel_thresh = 22.0            # 3D 단차(Step Edge) 임계치

        # 상태 플래그
        self.fusion_mode = True
        self.lock_floor_color = False
        self.floor_color_bgr = None
        self.floor_depth_top = 85.0
        self.floor_depth_bottom = 155.0

    def infer_depth(self, frame):
        """MiDaS 딥러닝 뎁스맵 추론 및 0~255 정규화 (입력 프레임은 이미 뒤집기 완료된 상태)"""
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
        d_min, d_max = depth_map.min(), depth_map.max()
        if d_max - d_min > 0:
            return (255 * (depth_map - d_min) / (d_max - d_min)).astype(np.uint8)
        return np.zeros_like(depth_map, dtype=np.uint8)

    def update_floor_sample(self, frame, depth_norm):
        """바닥 기준 색상 및 하단 뎁스 실시간 자동 샘플링 (EMA 적용)"""
        sample_pixels = []
        sample_depths = []
        for (fx1, fy1, fx2, fy2) in self.floor_boxes:
            patch = frame[fy1:fy2, fx1:fx2]
            d_patch = depth_norm[fy1:fy2, fx1:fx2]
            if patch.size > 0:
                sample_pixels.append(patch.reshape(-1, 3))
                sample_depths.append(d_patch.flatten())

        if sample_pixels and not self.lock_floor_color:
            current_floor = np.median(np.vstack(sample_pixels), axis=0).astype(np.float32)
            current_bottom_d = float(np.median(np.concatenate(sample_depths))) if sample_depths else 155.0
            if self.floor_color_bgr is None:
                self.floor_color_bgr = current_floor
                self.floor_depth_bottom = current_bottom_d
            else:
                self.floor_color_bgr = 0.95 * self.floor_color_bgr + 0.05 * current_floor
                self.floor_depth_bottom = 0.95 * self.floor_depth_bottom + 0.05 * current_bottom_d
        elif self.floor_color_bgr is None:
            self.floor_color_bgr = np.array([120.0, 120.0, 120.0], dtype=np.float32)

    def process(self, frame):
        """단일 RGB 프레임으로부터 바닥 제거 및 물체 감지 수행"""
        res = SegmentationResult()

        # 1. 뎁스 추론
        depth_norm = self.infer_depth(frame)
        res.depth_norm = depth_norm

        # 2. 바닥 샘플링
        self.update_floor_sample(frame, depth_norm)
        res.floor_color_bgr = self.floor_color_bgr

        # 3. 색상 거리 & 3D 솟아오름 모델링
        frame_lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB).astype(np.float32)
        floor_bgr_1px = np.uint8([[self.floor_color_bgr.astype(np.uint8)]])
        floor_lab = cv2.cvtColor(floor_bgr_1px, cv2.COLOR_BGR2LAB).astype(np.float32)[0, 0]

        dL = (frame_lab[:, :, 0] - floor_lab[0]) * 0.35
        da = frame_lab[:, :, 1] - floor_lab[1]
        db = frame_lab[:, :, 2] - floor_lab[2]
        color_diff = np.sqrt(dL**2 + da**2 + db**2)

        # 3D 경사면 기대 뎁스 모델
        y_indices = np.arange(self.h, dtype=np.float32)
        expected_floor_d = np.clip(
            self.floor_depth_top + (y_indices - self.roi_y1) * (self.floor_depth_bottom - self.floor_depth_top) / max(1.0, float(self.roi_y2 - self.roi_y1)),
            40.0, 240.0
        ).reshape(self.h, 1)

        depth_protrusion = depth_norm.astype(np.float32) - expected_floor_d
        is_protruding = (depth_protrusion > self.protrusion_thresh)

        # 4. 2D Canny 에지 + 3D Depth Sobel 단차 결합
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        canny_edges = cv2.Canny(gray, 35, 110)

        depth_blurred = cv2.GaussianBlur(depth_norm, (5, 5), 0)
        depth_sobel_x = cv2.Sobel(depth_blurred, cv2.CV_32F, 1, 0, ksize=3)
        depth_sobel_y = cv2.Sobel(depth_blurred, cv2.CV_32F, 0, 1, ksize=3)
        depth_grad_mag = cv2.magnitude(depth_sobel_x, depth_sobel_y)

        depth_step_edges = (depth_grad_mag > self.depth_sobel_thresh)
        res.depth_step_edges = depth_step_edges

        # 5. 동일 색상 구출 및 마스크 생성
        is_similar_to_floor = (color_diff <= self.color_thresh)
        has_3d_presence = is_protruding | (depth_step_edges & (depth_norm >= self.target_threshold - 25))
        is_same_color_object = is_similar_to_floor & has_3d_presence & (depth_norm >= self.target_threshold - 15)

        is_floor_mask = is_similar_to_floor & (~has_3d_presence)
        res.is_floor_mask = is_floor_mask

        is_diff_color_object = (~is_similar_to_floor) & (depth_norm >= self.target_threshold)
        raw_object_mask = (is_diff_color_object | is_same_color_object)

        corridor_mask = np.zeros_like(depth_norm, dtype=np.uint8)
        corridor_mask[self.roi_y1:self.roi_y2, self.roi_x1:self.roi_x2] = 255

        if self.fusion_mode:
            fused_mask = (raw_object_mask.astype(np.uint8) * 255)
            fused_mask = cv2.bitwise_and(fused_mask, corridor_mask)
        else:
            close_depth_mask = (depth_norm >= self.target_threshold).astype(np.uint8) * 255
            fused_mask = cv2.bitwise_and(close_depth_mask, corridor_mask)

        # 6. 모폴로지 연산 (노이즈 정제)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        clean_mask = cv2.morphologyEx(fused_mask, cv2.MORPH_OPEN, kernel, iterations=2)
        clean_mask = cv2.morphologyEx(clean_mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        res.clean_mask = clean_mask

        # 7. 외곽선 및 바운딩 박스 추출
        contours, _ = cv2.findContours(clean_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area >= self.min_object_area:
                ox, oy, ow, oh = cv2.boundingRect(cnt)
                cnt_mask = np.zeros_like(depth_norm, dtype=np.uint8)
                cv2.drawContours(cnt_mask, [cnt], -1, 255, -1)
                obj_depths = depth_norm[cnt_mask == 255]
                median_depth = int(np.median(obj_depths)) if len(obj_depths) > 0 else 0
                approx_cm = int(max(10, 6000.0 / (median_depth + 25) - 5))
                is_rescued = bool(np.mean(is_same_color_object[cnt_mask == 255]) > 0.35)

                res.detected_objects.append({
                    "cnt": cnt,
                    "bbox": (ox, oy, ow, oh),
                    "area": area,
                    "depth": median_depth,
                    "dist_cm": approx_cm,
                    "is_rescued": is_rescued,
                    "cx": ox + ow // 2,
                    "cy": oy + oh // 2
                })

        # 거리 오름차순 정렬
        if res.detected_objects:
            res.detected_objects.sort(key=lambda o: o["dist_cm"])
            res.nearest_dist_cm = res.detected_objects[0]["dist_cm"]

        # 통로 내 바닥 점유율 계산
        corridor_pixels = (corridor_mask > 0)
        floor_in_corridor = is_floor_mask & corridor_pixels
        res.floor_coverage_pct = int((np.sum(floor_in_corridor) / max(1, np.sum(corridor_pixels))) * 100)

        return res
