#!/usr/bin/env python3
"""
bot_modules/depth_renderer.py
------------------------------
RGB-D 융합 시각화, 바닥 오버레이, 물체 트래킹 박스 및 HUD 대시보드 렌더링 전담 모듈
"""

import time
import re
import cv2
import numpy as np


class DepthRenderer:
    """RGB-D 시각화 및 HUD 오버레이 렌더러"""
    def __init__(self, frame_width=640, frame_height=480):
        self.w = frame_width
        self.h = frame_height

        # 중앙 프로브 마커 위치
        self.probe_x = self.w // 2
        self.probe_y = self.h // 2

        # 렌더링 옵션 플래그
        self.show_floor_overlay = True
        self.show_depth_edges = False
        self.use_colormap = True

    def render_left_rgb(self, frame, seg_res, segmentor, fps):
        """좌측: 원본 RGB + 바닥 오버레이 + 물체 트래킹 화면 생성"""
        rgb_view = frame.copy()

        # 1. 인식된 바닥 영역 푸른색(Cyan/Blue) 반투명 오버레이
        if self.show_floor_overlay and seg_res.is_floor_mask is not None:
            floor_tint = np.zeros_like(rgb_view)
            floor_tint[seg_res.is_floor_mask] = (255, 140, 0)
            rgb_view = cv2.addWeighted(rgb_view, 1.0, floor_tint, 0.38, 0)

        # 2. 바닥 샘플 박스
        if seg_res.floor_color_bgr is not None:
            sample_color = tuple(int(c) for c in seg_res.floor_color_bgr)
            for (fx1, fy1, fx2, fy2) in segmentor.floor_boxes:
                cv2.rectangle(rgb_view, (fx1, fy1), (fx2, fy2), sample_color, -1)
                cv2.rectangle(rgb_view, (fx1, fy1), (fx2, fy2), (0, 255, 255), 2)
                cv2.putText(rgb_view, "Floor Sample", (fx1 + 2, fy1 + 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 0, 0), 1)

        # 3. 감시 통로 박스
        cv2.rectangle(rgb_view, (segmentor.roi_x1, segmentor.roi_y1), (segmentor.roi_x2, segmentor.roi_y2), (255, 180, 0), 1)
        cv2.putText(rgb_view, f"Corridor [Floor: {seg_res.floor_coverage_pct}%]", (segmentor.roi_x1 + 6, segmentor.roi_y1 + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 180, 0), 1)

        # 4. 검출된 물체 윤곽선 및 바운딩 박스
        for i, obj in enumerate(seg_res.detected_objects):
            cnt = obj["cnt"]
            ox, oy, ow, oh = obj["bbox"]
            dist_cm = obj["dist_cm"]
            is_rescued = obj.get("is_rescued", False)

            cnt_color = (0, 255, 255) if is_rescued else (0, 255, 0)
            cv2.drawContours(rgb_view, [cnt], -1, cnt_color, 2)
            cv2.rectangle(rgb_view, (ox, oy), (ox + ow, oy + oh), (0, 0, 255), 2)
            cv2.circle(rgb_view, (obj["cx"], obj["cy"]), 4, (0, 0, 255), -1)

            rescue_badge = " [Same-Color]" if is_rescued else ""
            tag_text = f"Obj #{i+1}{rescue_badge}: ~{dist_cm}cm ({ow}x{oh}px)"
            tag_y = max(18, oy - 8)
            cv2.rectangle(rgb_view, (ox, tag_y - 14), (ox + len(tag_text) * 8 + 6, tag_y + 4), (0, 0, 180), -1)
            cv2.putText(rgb_view, tag_text, (ox + 2, tag_y), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (255, 255, 255), 1)

        # 5. 중앙 거리 프로브 마커
        center_region = seg_res.depth_norm[self.probe_y - 10:self.probe_y + 10, self.probe_x - 10:self.probe_x + 10]
        center_val = int(np.median(center_region)) if center_region.size > 0 else 0
        probe_cm = int(max(5500.0 / (center_val + 15.0) - 4.0, 12))

        cv2.drawMarker(rgb_view, (self.probe_x, self.probe_y), (0, 255, 255), cv2.MARKER_CROSS, 18, 1)
        cv2.putText(rgb_view, f"Center: ~{probe_cm}cm", (self.probe_x + 12, self.probe_y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1)

        cv2.putText(rgb_view, "Original RGB + Contour Track", (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2)
        cv2.putText(rgb_view, f"FPS: {fps:.1f}", (self.w - 105, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        return rgb_view

    def render_right_depth(self, seg_res, segmentor):
        """우측: 3D 뎁스맵 + 바닥 분리 마스크 하이라이트 화면 생성"""
        depth_norm = seg_res.depth_norm
        if self.use_colormap:
            depth_colored = cv2.applyColorMap(depth_norm, cv2.COLORMAP_MAGMA)
        else:
            depth_colored = cv2.cvtColor(depth_norm, cv2.COLOR_GRAY2BGR)

        # 바닥 오버레이 (어두운 파란색)
        if self.show_floor_overlay and seg_res.is_floor_mask is not None:
            depth_floor_tint = np.zeros_like(depth_colored)
            depth_floor_tint[seg_res.is_floor_mask] = (160, 60, 0)
            depth_colored = cv2.addWeighted(depth_colored, 1.0, depth_floor_tint, 0.35, 0)

        # 감시 통로 박스
        cv2.rectangle(depth_colored, (segmentor.roi_x1, segmentor.roi_y1), (segmentor.roi_x2, segmentor.roi_y2), (255, 255, 255), 1)

        # 3D 단차 모서리선 오버레이 (Depth Sobel)
        if self.show_depth_edges and seg_res.depth_step_edges is not None:
            sobel_overlay = np.zeros_like(depth_colored)
            sobel_overlay[seg_res.depth_step_edges] = (0, 255, 255)
            depth_colored = cv2.addWeighted(depth_colored, 1.0, sobel_overlay, 0.90, 0)
            cv2.putText(depth_colored, "[Depth Sobel 3D Edges ON]", (segmentor.roi_x1 + 6, segmentor.roi_y2 - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 255), 1)

        # 물체 세그멘테이션 마스크 형광 연두색 오버레이
        if len(seg_res.detected_objects) > 0 and seg_res.clean_mask is not None:
            mask_bool = (seg_res.clean_mask > 0)
            overlay = depth_colored.copy()
            overlay[mask_bool] = (0, 255, 0)
            depth_colored = cv2.addWeighted(depth_colored, 0.65, overlay, 0.35, 0)

            for obj in seg_res.detected_objects:
                ox, oy, ow, oh = obj["bbox"]
                cv2.drawContours(depth_colored, [obj["cnt"]], -1, (255, 255, 255), 2)
                cv2.rectangle(depth_colored, (ox, oy), (ox + ow, oy + oh), (0, 255, 0), 2)
                cv2.putText(depth_colored, f"~{obj['dist_cm']}cm", (ox + 4, oy + 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        mode_str = 'FUSION' if segmentor.fusion_mode else 'RAW'
        cv2.putText(depth_colored, f"Depth Map ({mode_str})", (15, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        return depth_colored

    def compose_frame(self, frame, seg_res, segmentor, fps, vlm_state, auto_capture_enabled):
        """좌우 화면 융합 및 HUD 상태 배너 합성"""
        rgb_view = self.render_left_rgb(frame, seg_res, segmentor, fps)
        depth_colored = self.render_right_depth(seg_res, segmentor)

        # 1. 상단 상태 배너
        floor_vis_txt = "BLUE=FLOOR" if self.show_floor_overlay else "FLOOR VIS OFF"
        if len(seg_res.detected_objects) > 0:
            nearest_cm = seg_res.nearest_dist_cm
            banner_bg = (0, 0, 180)
            banner_txt = f"🛑 {len(seg_res.detected_objects)} OBJECT(S) DETECTED! Nearest: ~{nearest_cm}cm [{floor_vis_txt}]"
        else:
            banner_bg = (0, 110, 0) if segmentor.fusion_mode else (25, 25, 30)
            banner_txt = f"✅ CLEAR | Floor Isolated ({seg_res.floor_coverage_pct}%) [{floor_vis_txt}]" if segmentor.fusion_mode else f"IDLE | Raw Depth Mode [{floor_vis_txt}]"

        cv2.rectangle(rgb_view, (0, 0), (self.w, 32), banner_bg, -1)
        cv2.putText(rgb_view, banner_txt, (15, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 2)

        # 2. AI VLM 커스텀 스킬 분석 배너
        if vlm_state:
            v_obj = vlm_state.get("object")
            v_trash = vlm_state.get("is_trash")
            v_time = vlm_state.get("timestamp", 0)

            if v_obj and (time.time() - v_time < 300):
                trash_badge = "TRASH" if v_trash is True else ("SAFE ITEM" if v_trash is False else "CAPTURED")
                m = re.search(r'\(([^)]+)\)', str(v_obj))
                if m and m.group(1).isascii():
                    clean_obj = m.group(1).strip()
                else:
                    ascii_part = "".join([c for c in str(v_obj) if ord(c) < 128]).strip()
                    clean_obj = ascii_part if len(ascii_part) > 1 else ("Captured" if v_trash is None else "Object")

                vlm_txt = f"AI VLM: {clean_obj} | {trash_badge}"
                vlm_bg = (50, 15, 120) if v_trash is True else ((20, 100, 40) if v_trash is False else (120, 80, 20))
                cv2.rectangle(rgb_view, (0, 32), (self.w, 58), vlm_bg, -1)
                cv2.putText(rgb_view, vlm_txt, (15, 51), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (255, 255, 255), 1)

        # 3. 하단 상태 요약 배너
        f_bgr = [int(c) for c in seg_res.floor_color_bgr] if seg_res.floor_color_bgr is not None else [0, 0, 0]
        tare_str = "[LOCKED]" if segmentor.lock_floor_color else "[AUTO]"
        autocap_str = "[ON]" if auto_capture_enabled else "[OFF]"
        summary_txt = f"Floor: {f_bgr} {tare_str} | AutoCap: {autocap_str} | FloorCov: {seg_res.floor_coverage_pct}% | ColorTol: {segmentor.color_thresh} | DepthThresh: {segmentor.target_threshold}"

        cv2.rectangle(depth_colored, (0, self.h - 26), (self.w, self.h), (20, 20, 20), -1)
        cv2.putText(depth_colored, summary_txt, (10, self.h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 255), 1)
        cv2.rectangle(rgb_view, (0, self.h - 26), (self.w, self.h), (20, 20, 20), -1)
        cv2.putText(rgb_view, summary_txt, (10, self.h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 255), 1)

        # 4. 좌우 결합 (1280 x 480)
        return np.hstack((rgb_view, depth_colored))
