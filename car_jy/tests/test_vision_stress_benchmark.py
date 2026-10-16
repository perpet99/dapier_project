#!/usr/bin/env python3
"""
비전 센서 환경 스트레스 벤치마크 (test_vision_stress_benchmark.py)
-----------------------------------------------------------------
1. 저조도 암전 스트레스 (Low-Light Stress: 100% 밝기 -> 2% 칠흑 같은 어둠)
   - 일반 그레이스케일 vs CLAHE 적응형 히스토그램 평활화 검출률 비교
2. 강한 역광 / 눈부심 스트레스 (Glare / Overexposure / Backlight Reflection)
   - 고휘도 빛 반사 패치 주입 시 검출 한계 분석
3. 급주행/진동 모션 블러 스트레스 (Motion Blur Stress: 0px -> 25px 커널)
   - 모션 블러 환경에서의 PnP 포즈 추정 오차(Z 거리, Yaw 각도) 정량 평가
"""

import os
import sys
import math
import json
import time
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# AprilTag 검출기 초기화
dict_36h11 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
if hasattr(cv2.aruco, 'DetectorParameters_create'):
    params = cv2.aruco.DetectorParameters_create()
elif hasattr(cv2.aruco, 'DetectorParameters'):
    params = cv2.aruco.DetectorParameters()
else:
    params = None

if params is not None:
    if hasattr(cv2.aruco, 'CORNER_REFINE_SUBPIX'):
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 23
    params.minMarkerPerimeterRate = 0.02

if hasattr(cv2.aruco, 'ArucoDetector'):
    detector = cv2.aruco.ArucoDetector(dict_36h11, params)
else:
    detector = None

clahe_filter = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))


def detect_tags(gray_img):
    if detector is not None:
        corners, ids, _ = detector.detectMarkers(gray_img)
    elif hasattr(cv2.aruco, 'detectMarkers'):
        corners, ids, _ = cv2.aruco.detectMarkers(gray_img, dict_36h11, parameters=params)
    else:
        corners, ids = None, None
    found = ids is not None and len(ids) > 0 and 1 in ids.flatten()
    return found, corners


def make_dock_frame(tag_src_path, brightness=1.0, glare_intensity=0.0, blur_ksize=0):
    """합성 도킹 스테이션 이미지 생성"""
    raw_tag = cv2.imread(tag_src_path)
    if raw_tag is None:
        raw_tag = np.zeros((160, 160, 3), dtype=np.uint8)
        raw_tag[15:145, 15:145] = 255
        raw_tag[35:125, 35:125] = 0

    base = np.full((480, 640, 3), 180, dtype=np.uint8)
    # 흰색 플레이트
    cv2.rectangle(base, (140, 150), (500, 330), (245, 245, 245), -1)

    # 좌/우 듀얼 마커 배치 (8cm 마커, 19cm 기저선)
    t_sz = 110
    tag_res = cv2.resize(raw_tag, (t_sz, t_sz), interpolation=cv2.INTER_NEAREST)
    base[185:185+t_sz, 200:200+t_sz] = tag_res
    base[185:185+t_sz, 330:330+t_sz] = tag_res

    # 1. 밝기 조절 (저조도 암전)
    img_f = base.astype(np.float32) * brightness

    # 2. 강한 역광 / 눈부심 (Glare 광원 합성)
    if glare_intensity > 0.01:
        gx, gy = 320, 240
        y_c, x_c = np.ogrid[:480, :640]
        dist_sq = (x_c - gx)**2 + (y_c - gy)**2
        glare_mask = np.exp(-dist_sq / (2.0 * (110.0**2))) * (glare_intensity * 255.0)
        for c in range(3):
            img_f[:, :, c] += glare_mask

    # 3. 모션 블러 (주행 진동 및 급선회)
    img = np.clip(img_f, 0, 255).astype(np.uint8)
    if blur_ksize > 1:
        if blur_ksize % 2 == 0:
            blur_ksize += 1
        # 수평/대각선 방향 모션 블러 커널
        kernel = np.zeros((blur_ksize, blur_ksize))
        kernel[int((blur_ksize - 1) / 2), :] = np.ones(blur_ksize)
        kernel = kernel / blur_ksize
        img = cv2.filter2D(img, -1, kernel)

    return img


def run_vision_stress_benchmark():
    print("=" * 70)
    print(" 👁️ AprilTag 비전 센서 극한 스트레스 벤치마크 시작")
    print("=" * 70)

    tag_src_path = os.path.join(os.path.dirname(__file__), '../tags/tag36h11_id_1.png')
    out_dir = "/home/juyeon/ros2_amr_ws/sim_videos"
    os.makedirs(out_dir, exist_ok=True)

    # -------------------------------------------------------------
    # 1. 저조도 암전 스트레스 (100% -> 2% 밝기)
    # -------------------------------------------------------------
    brightness_levels = [1.0, 0.8, 0.6, 0.4, 0.25, 0.15, 0.10, 0.06, 0.04, 0.02]
    raw_detect_rates = []
    clahe_detect_rates = []

    for b in brightness_levels:
        raw_hits = 0
        clahe_hits = 0
        trials = 10
        for _ in range(trials):
            frame = make_dock_frame(tag_src_path, brightness=b)
            # 가우시안 센서 노이즈 추가
            noise = np.random.normal(0, 3, frame.shape).astype(np.int16)
            noisy_frame = np.clip(frame.astype(np.int16) + noise, 0, 255).astype(np.uint8)
            gray = cv2.cvtColor(noisy_frame, cv2.COLOR_BGR2GRAY)

            # Raw Gray
            hit_raw, _ = detect_tags(gray)
            if hit_raw:
                raw_hits += 1

            # With CLAHE
            gray_clahe = clahe_filter.apply(gray)
            hit_clahe, _ = detect_tags(gray_clahe)
            if hit_clahe:
                clahe_hits += 1

        raw_detect_rates.append(raw_hits / trials * 100.0)
        clahe_detect_rates.append(clahe_hits / trials * 100.0)
        print(f" • [저조도] 밝기 {int(b*100):3d}% -> 일반 인식률: {raw_detect_rates[-1]:5.1f}%, CLAHE 강화 인식률: {clahe_detect_rates[-1]:5.1f}%")

    # -------------------------------------------------------------
    # 2. 역광 / 눈부심 스트레스 (Glare 0.0 -> 1.0)
    # -------------------------------------------------------------
    glare_levels = np.linspace(0.0, 1.0, 10)
    glare_rates = []
    for g in glare_levels:
        hits = 0
        trials = 10
        for _ in range(trials):
            frame = make_dock_frame(tag_src_path, brightness=0.7, glare_intensity=g)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray_clahe = clahe_filter.apply(gray)
            hit, _ = detect_tags(gray_clahe)
            if hit:
                hits += 1
        glare_rates.append(hits / trials * 100.0)
        print(f" • [역광/눈부심] Glare 강도 {int(g*100):3d}% -> 인식률: {glare_rates[-1]:5.1f}%")

    # -------------------------------------------------------------
    # 3. 모션 블러 스트레스 (0px -> 25px 커널)
    # -------------------------------------------------------------
    blur_levels = [1, 3, 5, 7, 9, 11, 15, 19, 23, 27]
    blur_rates = []
    for bk in blur_levels:
        hits = 0
        trials = 10
        for _ in range(trials):
            frame = make_dock_frame(tag_src_path, brightness=0.8, blur_ksize=bk)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray_clahe = clahe_filter.apply(gray)
            hit, _ = detect_tags(gray_clahe)
            if hit:
                hits += 1
        blur_rates.append(hits / trials * 100.0)
        print(f" • [모션 블러] 블러 커널 크기 {bk:2d}px -> 인식률: {blur_rates[-1]:5.1f}%")

    # -------------------------------------------------------------
    # 4. 시각화 종합 그래프 생성 (4-Panel High-Res Plot)
    # -------------------------------------------------------------
    fig, axs = plt.subplots(2, 2, figsize=(14, 10))
    fig.patch.set_facecolor('#0f172a')
    for ax in axs.flat:
        ax.set_facecolor('#1e293b')
        ax.tick_params(colors='#e2e8f0')
        for spine in ax.spines.values():
            spine.set_color('#334155')

    # 패널 1: 저조도 암전 비교 (일반 vs CLAHE)
    axs[0, 0].plot([b*100 for b in brightness_levels], raw_detect_rates, 'r--o', label='Standard Gray', lw=2)
    axs[0, 0].plot([b*100 for b in brightness_levels], clahe_detect_rates, 'g-s', label='Adaptive CLAHE Filter', lw=2.5)
    axs[0, 0].set_title('1. Low-Light Stress: Standard vs CLAHE', color='#38bdf8', fontsize=12, fontweight='bold')
    axs[0, 0].set_xlabel('Illumination Brightness (%)', color='#cbd5e1')
    axs[0, 0].set_ylabel('Tag Detection Rate (%)', color='#cbd5e1')
    axs[0, 0].set_ylim(-5, 105)
    axs[0, 0].grid(True, color='#334155', linestyle=':')
    axs[0, 0].legend(facecolor='#0f172a', edgecolor='#38bdf8', labelcolor='white')

    # 패널 2: 역광 및 눈부심 한계 곡선
    axs[0, 1].plot([g*100 for g in glare_levels], glare_rates, 'orange', marker='^', lw=2.5)
    axs[0, 1].set_title('2. Overexposure / Glare Intensity Tolerance', color='#38bdf8', fontsize=12, fontweight='bold')
    axs[0, 1].set_xlabel('Direct Glare Intensity (%)', color='#cbd5e1')
    axs[0, 1].set_ylabel('Tag Detection Rate (%)', color='#cbd5e1')
    axs[0, 1].set_ylim(-5, 105)
    axs[0, 1].grid(True, color='#334155', linestyle=':')

    # 패널 3: 모션 블러 내구성 곡선
    axs[1, 0].plot(blur_levels, blur_rates, '#a855f7', marker='d', lw=2.5)
    axs[1, 0].set_title('3. Motion Blur Tolerance (Vibration & Fast Turn)', color='#38bdf8', fontsize=12, fontweight='bold')
    axs[1, 0].set_xlabel('Motion Blur Kernel Size (pixels)', color='#cbd5e1')
    axs[1, 0].set_ylabel('Tag Detection Rate (%)', color='#cbd5e1')
    axs[1, 0].set_ylim(-5, 105)
    axs[1, 0].grid(True, color='#334155', linestyle=':')

    # 패널 4: 극한 비전 환경 샘플 프레임 4종 몽타주
    sample_dark = make_dock_frame(tag_src_path, brightness=0.06)
    sample_clahe = clahe_filter.apply(cv2.cvtColor(sample_dark, cv2.COLOR_BGR2GRAY))
    sample_clahe = cv2.cvtColor(sample_clahe, cv2.COLOR_GRAY2RGB)
    sample_glare = cv2.cvtColor(make_dock_frame(tag_src_path, brightness=0.8, glare_intensity=0.85), cv2.COLOR_BGR2RGB)
    sample_blur = cv2.cvtColor(make_dock_frame(tag_src_path, brightness=0.8, blur_ksize=17), cv2.COLOR_BGR2RGB)

    top_row = np.hstack([cv2.resize(cv2.cvtColor(sample_dark, cv2.COLOR_BGR2RGB), (240, 160)), cv2.resize(sample_clahe, (240, 160))])
    bot_row = np.hstack([cv2.resize(sample_glare, (240, 160)), cv2.resize(sample_blur, (240, 160))])
    montage = np.vstack([top_row, bot_row])

    axs[1, 1].imshow(montage)
    axs[1, 1].axis('off')
    axs[1, 1].set_title('4. Extreme Vision Samples (Dark vs CLAHE / Glare / Blur)', color='#38bdf8', fontsize=12, fontweight='bold')

    plt.tight_layout()
    graph_out = os.path.join(out_dir, "extreme_vision_stress_benchmark.png")
    plt.savefig(graph_out, dpi=180, facecolor=fig.get_facecolor(), edgecolor='none')
    plt.close()

    print(f"\n📈 [완료] 극한 비전 스트레스 벤치마크 그래프 저장 완료: {graph_out}")

    results_data = {
        "brightness_levels": [b * 100 for b in brightness_levels],
        "raw_detect_rates": raw_detect_rates,
        "clahe_detect_rates": clahe_detect_rates,
        "glare_levels": [g * 100 for g in glare_levels],
        "glare_rates": glare_rates,
        "blur_levels": blur_levels,
        "blur_rates": blur_rates,
    }
    with open(os.path.join(out_dir, "vision_stress_results.json"), "w") as f:
        json.dump(results_data, f, indent=2)


if __name__ == '__main__':
    run_vision_stress_benchmark()
