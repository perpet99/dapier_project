#!/usr/bin/env python3
"""
tests/test_apriltag_docking.py
AprilTag 정밀 도킹 인식 및 제어 알고리즘 단위 테스트
"""

import math
import unittest
import cv2
import numpy as np


class TestAprilTagDocking(unittest.TestCase):
    def setUp(self):
        self.dict_36h11 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.params = cv2.aruco.DetectorParameters_create()
        else:
            self.params = cv2.aruco.DetectorParameters()

        self.camera_matrix = np.array([
            [530.0,   0.0, 320.0],
            [  0.0, 530.0, 240.0],
            [  0.0,   0.0,   1.0]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        self.tag_size = 0.08  # 8cm

    def test_apriltag_generation_and_detection(self):
        """가상 AprilTag 이미지 생성 및 인식 테스트"""
        for test_id in [0, 1, 5, 12]:
            marker_size = 200
            marker_img = cv2.aruco.drawMarker(self.dict_36h11, test_id, marker_size, borderBits=1)

            # 640x480 캔버스 중앙에 합성
            canvas = np.full((480, 640), 255, dtype=np.uint8)
            y_start = (480 - marker_size) // 2
            x_start = (640 - marker_size) // 2
            canvas[y_start:y_start+marker_size, x_start:x_start+marker_size] = marker_img

            corners, ids, _ = cv2.aruco.detectMarkers(canvas, self.dict_36h11, parameters=self.params)
            self.assertIsNotNone(ids, f"Tag {test_id} 감지 실패")
            self.assertEqual(int(ids[0][0]), test_id, f"Tag ID 불일치 (기대: {test_id}, 실제: {ids[0][0]})")

    def test_pose_estimation_and_distance(self):
        """3D Pose 추정 및 거리 계산 유효성 테스트"""
        marker_size = 180
        marker_img = cv2.aruco.drawMarker(self.dict_36h11, 0, marker_size, borderBits=1)

        canvas = np.full((480, 640), 255, dtype=np.uint8)
        y_start = (480 - marker_size) // 2
        x_start = (640 - marker_size) // 2
        canvas[y_start:y_start+marker_size, x_start:x_start+marker_size] = marker_img

        corners, ids, _ = cv2.aruco.detectMarkers(canvas, self.dict_36h11, parameters=self.params)
        self.assertIsNotNone(ids)

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.tag_size, self.camera_matrix, self.dist_coeffs
        )
        self.assertEqual(len(tvecs), 1)
        z_dist = float(tvecs[0][0][2])
        self.assertGreater(z_dist, 0.1, "Z 거리가 10cm 이상이어야 함")
        self.assertLess(z_dist, 2.0, "Z 거리가 2.0m 이내여야 함")

    def test_control_logic_far_approach(self):
        """원거리 감지 시 전진 및 조향 속도 산출 테스트"""
        target_dist = 0.25
        z_current = 0.80  # 80cm 전방
        x_current = 0.10  # 10cm 우측 편차
        kp_lin = 0.50
        kp_bearing = 0.85

        dist_err = z_current - target_dist
        bearing = math.atan2(x_current, z_current)

        raw_vx = kp_lin * dist_err
        vx = np.clip(raw_vx, 0.03, 0.12)
        wz = np.clip(kp_bearing * bearing, -0.40, 0.40)

        self.assertGreater(vx, 0.05, "전진 속도가 적절히 생성되어야 함")
        self.assertGreater(wz, 0.05, "우측 편차에 대해 양의 각속도가 생성되어야 함")

    def test_control_logic_docked_convergence(self):
        """목표 위치 도달 시 수렴 판정 및 정지 테스트"""
        target_dist = 0.25
        z_current = 0.255  # 5mm 오차 (허용오차 2cm 이내)
        x_current = 0.005  # 5mm 오차 (허용오차 2cm 이내)
        yaw_current = math.radians(1.0) # 1도 오차 (허용오차 2.5도 이내)

        dist_ok = abs(z_current - target_dist) <= 0.02
        lat_ok = abs(x_current) <= 0.02
        yaw_ok = abs(yaw_current) <= math.radians(2.5)

        self.assertTrue(dist_ok and lat_ok and yaw_ok, "허용오차 범위 내 도킹 수렴 판정 확인")


if __name__ == '__main__':
    unittest.main()
