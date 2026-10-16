#!/usr/bin/env python3
"""
아까 저장한 SLAM 지도(/home/juyeon/my_map.yaml) 기반
「1단계: 부정확한 자율주행(도착 위치/각도 오차 발생) ➔ 2단계: AprilTag 정밀 도킹」 통합 시뮬레이터
+ 실시간 탑뷰(Top-View) / 카메라뷰(Camera HUD View) / 통합 분할 화면(Split-View) MP4 영상 녹화기
"""

import os
import math
import time
import json
import threading
import subprocess
import urllib.request
import argparse
import cv2
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def set_gazebo_pose(world_name: str, x: float, y: float, z: float, yaw: float):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    req = (
        f'name: "amr", '
        f'position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
        f'orientation: {{x: 0.0, y: 0.0, z: {qz:.5f}, w: {qw:.5f}}}'
    )
    cmd = [
        "gz", "service",
        "-s", f"/world/{world_name}/set_pose",
        "--reqtype", "gz.msgs.Pose",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "2000",
        "--req", req
    ]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


class MJPEGStreamGrabber:
    """http://localhost:8088/stream.mjpg 에서 실시간 카메라 HUD 프레임을 백그라운드로 수신"""
    def __init__(self, url="http://localhost:8088/stream.mjpg"):
        self.url = url
        self.latest_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.lock = threading.Lock()
        self.running = False
        self.thread = None

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.running:
            try:
                stream = urllib.request.urlopen(self.url, timeout=3.0)
                buf = b""
                while self.running:
                    chunk = stream.read(4096)
                    if not chunk:
                        break
                    buf += chunk
                    a = buf.find(b"\xff\xd8")
                    b = buf.find(b"\xff\xd9")
                    if a != -1 and b != -1 and b > a:
                        jpg = buf[a : b + 2]
                        buf = buf[b + 2 :]
                        img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                        if img is not None:
                            if img.shape[:2] != (480, 640):
                                img = cv2.resize(img, (640, 480))
                            with self.lock:
                                self.latest_frame = img
            except Exception:
                time.sleep(0.2)

    def get_frame(self):
        with self.lock:
            return self.latest_frame.copy()

    def stop(self):
        self.running = False


class TopViewMapRenderer:
    """저장된 /home/juyeon/my_map.yaml (my_map.pgm) 기반 고해상도 640x480 탑뷰 렌더러"""
    def __init__(self, yaml_path="/home/juyeon/my_map.yaml", dock_pos=(7.55, -5.15, math.pi)):
        self.width = 640
        self.height = 480
        self.dock_x, self.dock_y, self.dock_yaw = dock_pos

        # 뷰포트 월드 범위 (방 전체 및 도킹 스테이션이 한눈에 보이도록 설정)
        self.x_min = 4.30
        self.x_max = 8.10
        self.y_min = -6.55
        self.y_max = -3.70
        # 가로/세로 동일 미터 스케일 유지 (1m 당 픽셀 수)
        self.scale = min((self.width - 40) / (self.x_max - self.x_min), (self.height - 90) / (self.y_max - self.y_min))
        self.ox_px = 25
        self.oy_px = 68

        self.base_canvas = np.full((self.height, self.width, 3), (22, 28, 38), dtype=np.uint8)
        self._build_static_map(yaml_path)

    def world_to_px(self, wx: float, wy: float):
        u = int(self.ox_px + (wx - self.x_min) * self.scale)
        v = int(self.height - 25 - (wy - self.y_min) * self.scale)
        return (u, v)

    def _build_static_map(self, yaml_path: str):
        # 0.5m 간격 그리드 라인
        for gx in np.arange(4.5, 8.1, 0.5):
            p1 = self.world_to_px(gx, self.y_min)
            p2 = self.world_to_px(gx, self.y_max)
            cv2.line(self.base_canvas, p1, p2, (35, 43, 56), 1)
            cv2.putText(self.base_canvas, f"{gx:.1f}m", (p1[0] - 14, self.height - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.33, (110, 125, 145), 1, cv2.LINE_AA)
        for gy in np.arange(-6.5, -3.6, 0.5):
            p1 = self.world_to_px(self.x_min, gy)
            p2 = self.world_to_px(self.x_max, gy)
            cv2.line(self.base_canvas, p1, p2, (35, 43, 56), 1)
            cv2.putText(self.base_canvas, f"{gy:.1f}m", (4, p1[1] + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.33, (110, 125, 145), 1, cv2.LINE_AA)

        # my_map.pgm 로드 및 자유 공간 / 점유 벽면 투영
        if os.path.exists(yaml_path):
            with open(yaml_path, "r") as f:
                meta = yaml.safe_load(f)
            res = float(meta.get("resolution", 0.05))
            origin = meta.get("origin", [0.0, 0.0, 0.0])
            ox, oy = float(origin[0]), float(origin[1])
            img_path = os.path.join(os.path.dirname(yaml_path), meta.get("image", "my_map.pgm"))
            img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
            if img is not None:
                H, W = img.shape
                # 자유 공간 픽셀(흰색)을 은은한 밝은 슬레이트 톤으로 표시
                free_pts = np.argwhere(img > 240)
                cell_sz = max(2, int(math.ceil(res * self.scale)))
                for r, c in free_pts:
                    wx = ox + c * res
                    wy = oy + (H - 1 - r) * res
                    if self.x_min <= wx <= self.x_max and self.y_min <= wy <= self.y_max:
                        u, v = self.world_to_px(wx, wy)
                        cv2.rectangle(self.base_canvas, (u - cell_sz // 2, v - cell_sz // 2),
                                      (u + cell_sz // 2, v + cell_sz // 2), (46, 58, 74), -1)

                # SLAM 벽면 점유 픽셀(검은색)을 입체 벽 블록으로 표시
                occ_pts = np.argwhere(img < 50)
                wall_sz = max(4, int(math.ceil(res * self.scale * 1.3)))
                for r, c in occ_pts:
                    wx = ox + c * res
                    wy = oy + (H - 1 - r) * res
                    if self.x_min <= wx <= self.x_max and self.y_min <= wy <= self.y_max:
                        u, v = self.world_to_px(wx, wy)
                        cv2.rectangle(self.base_canvas, (u - wall_sz // 2, v - wall_sz // 2),
                                      (u + wall_sz // 2, v + wall_sz // 2), (175, 190, 210), -1)

        # 도킹 중앙 기준선 (초록색 점선/보조선)
        p_dock = self.world_to_px(self.dock_x - 0.05, self.dock_y)
        p_far = self.world_to_px(4.80, self.dock_y)
        cv2.line(self.base_canvas, p_far, p_dock, (45, 110, 65), 1, cv2.LINE_AA)

        # 도킹 스테이션 벽면 & 좌/우 AprilTag 마커 렌더링
        w_top = self.world_to_px(self.dock_x - 0.05, self.dock_y + 0.60)
        w_bot = self.world_to_px(self.dock_x + 0.05, self.dock_y - 0.60)
        cv2.rectangle(self.base_canvas, w_top, w_bot, (90, 100, 115), -1)
        cv2.rectangle(self.base_canvas, w_top, w_bot, (200, 210, 225), 2)

        # 좌측(L: Y=-5.15+0.095) / 우측(R: Y=-5.15-0.095) 마커 표시 (로봇 시점 기준 좌측=+Y, 우측=-Y)
        p_tag_l = self.world_to_px(self.dock_x - 0.05, self.dock_y + 0.095)
        p_tag_r = self.world_to_px(self.dock_x - 0.05, self.dock_y - 0.095)
        cv2.circle(self.base_canvas, p_tag_l, 6, (255, 210, 0), -1)
        cv2.circle(self.base_canvas, p_tag_r, 6, (0, 215, 255), -1)
        cv2.putText(self.base_canvas, "TAG L", (p_tag_l[0] + 10, p_tag_l[1] - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, (255, 220, 80), 1, cv2.LINE_AA)
        cv2.putText(self.base_canvas, "TAG R", (p_tag_r[0] + 10, p_tag_r[1] + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, (80, 220, 255), 1, cv2.LINE_AA)

    def render(self, sc_idx, total_sc, sc_title_en, phase_name, rx, ry, ryaw,
               start_pt, nominal_goal, err_goal, err_tuple,
               autonav_path, docking_path, scan_pts, dock_status):
        canvas = self.base_canvas.copy()

        # 1. 실시간 라이다 스캔 점군(/scan) 렌더링 (붉은 시안 점)
        for sx, sy in scan_pts:
            if self.x_min <= sx <= self.x_max and self.y_min <= sy <= self.y_max:
                su, sv = self.world_to_px(sx, sy)
                cv2.circle(canvas, (su, sv), 2, (80, 120, 255), -1)

        # 2. 카메라 50도 시야각(FOV Cone) 반투명 렌더링
        cam_x = rx + 0.175 * math.cos(ryaw)
        cam_y = ry + 0.175 * math.sin(ryaw)
        fov_half = math.radians(25.0)
        cone_len = 1.35
        c_p0 = self.world_to_px(cam_x, cam_y)
        c_p1 = self.world_to_px(cam_x + cone_len * math.cos(ryaw + fov_half),
                                cam_y + cone_len * math.sin(ryaw + fov_half))
        c_p2 = self.world_to_px(cam_x + cone_len * math.cos(ryaw - fov_half),
                                cam_y + cone_len * math.sin(ryaw - fov_half))
        fov_overlay = canvas.copy()
        pts_cone = np.array([c_p0, c_p1, c_p2], dtype=np.int32)
        cone_col = (0, 180, 120) if "PHASE 2" in phase_name else (180, 130, 40)
        cv2.fillPoly(fov_overlay, [pts_cone], cone_col)
        cv2.addWeighted(fov_overlay, 0.16, canvas, 0.84, 0, canvas)
        cv2.line(canvas, c_p0, c_p1, cone_col, 1, cv2.LINE_AA)
        cv2.line(canvas, c_p0, c_p2, cone_col, 1, cv2.LINE_AA)

        # 3. 원래 목표 지점(Pre-Dock Ideal Goal) vs 오차 도착 지점(Actual Arrival with Error)
        nom_u, nom_v = self.world_to_px(nominal_goal[0], nominal_goal[1])
        cv2.circle(canvas, (nom_u, nom_v), 7, (120, 220, 120), 1, cv2.LINE_AA)
        cv2.putText(canvas, "Ideal Pre-Dock", (nom_u - 42, nom_v + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.34, (120, 220, 120), 1, cv2.LINE_AA)

        err_u, err_v = self.world_to_px(err_goal[0], err_goal[1])
        cv2.circle(canvas, (err_u, err_v), 6, (0, 140, 255), -1, cv2.LINE_AA)
        cv2.circle(canvas, (err_u, err_v), 10, (0, 140, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, "Noisy Nav Arrival", (err_u - 48, err_v - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 180, 255), 1, cv2.LINE_AA)

        # 출발 지점
        st_u, st_v = self.world_to_px(start_pt[0], start_pt[1])
        cv2.circle(canvas, (st_u, st_v), 5, (200, 200, 200), -1)
        cv2.putText(canvas, "START", (st_u - 18, st_v + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1, cv2.LINE_AA)

        # 4. 1단계 자율주행 이동 궤적 (주황색 선)
        if len(autonav_path) >= 2:
            pts_nav = np.array([self.world_to_px(p[0], p[1]) for p in autonav_path], dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(canvas, [pts_nav], False, (0, 165, 255), 2, cv2.LINE_AA)

        # 5. 2단계 AprilTag 정밀 도킹 이동 궤적 (시안색/초록색 선)
        if len(docking_path) >= 2:
            pts_dock = np.array([self.world_to_px(p[0], p[1]) for p in docking_path], dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(canvas, [pts_dock], False, (0, 255, 220), 3, cv2.LINE_AA)

        # 6. 로봇 차체(35cm x 45cm) 및 4바퀴 + 헤딩 화살표 렌더링
        hl, hw = 0.175, 0.225
        corners_local = [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)]
        corners_px = []
        for lx, ly in corners_local:
            wx = rx + lx * math.cos(ryaw) - ly * math.sin(ryaw)
            wy = ry + lx * math.sin(ryaw) + ly * math.cos(ryaw)
            corners_px.append(self.world_to_px(wx, wy))
        poly_robot = np.array(corners_px, dtype=np.int32)
        cv2.fillPoly(canvas, [poly_robot], (200, 90, 25))
        cv2.polylines(canvas, [poly_robot], True, (255, 255, 255), 2, cv2.LINE_AA)

        # 로봇 진행 방향 화살표
        rc_u, rc_v = self.world_to_px(rx, ry)
        hd_u, hd_v = self.world_to_px(rx + 0.30 * math.cos(ryaw), ry + 0.30 * math.sin(ryaw))
        cv2.arrowedLine(canvas, (rc_u, rc_v), (hd_u, hd_v), (0, 255, 100), 2, tipLength=0.35)

        # 7. 상단 배너 & 하단 범례/상태 오버레이
        cv2.rectangle(canvas, (0, 0), (self.width, 58), (12, 16, 24), -1)
        phase_col = (0, 180, 255) if "PHASE 1" in phase_name else (0, 230, 150)
        cv2.rectangle(canvas, (10, 8), (185, 48), phase_col, -1)
        cv2.putText(canvas, phase_name, (16, 33), cv2.FONT_HERSHEY_DUPLEX, 0.46, (10, 15, 20), 1, cv2.LINE_AA)

        ex, ey, eth = err_tuple
        cv2.putText(canvas, f"TOP-VIEW (Saved SLAM Map: my_map.yaml) | Trial {sc_idx}/{total_sc}: {sc_title_en}",
                    (195, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (235, 240, 245), 1, cv2.LINE_AA)
        cv2.putText(canvas, f"Nav Arrival Error: dX={ex:+.0f}cm, dY={ey:+.0f}cm, dYaw={eth:+.1f}deg | Robot: ({rx:.2f}m, {ry:.2f}m, {math.degrees(ryaw):+.1f}deg)",
                    (195, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.37, (0, 220, 255), 1, cv2.LINE_AA)

        # 좌측 하단 범례 및 현재 동작 표시
        m_en = dock_status.get("motion_en", "AUTONAV") if "PHASE 2" in phase_name else "AUTONAV DRIVING"
        tmode = dock_status.get("tag_mode", "NONE")
        cv2.rectangle(canvas, (10, self.height - 62), (355, self.height - 24), (15, 20, 30), -1)
        cv2.rectangle(canvas, (10, self.height - 62), (355, self.height - 24), phase_col, 1)
        cv2.putText(canvas, f"MODE: {m_en} | TAG: {tmode}", (18, self.height - 46),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.39, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, "Orange: Phase1 AutoNav | Cyan: Phase2 Precision Dock", (18, self.height - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.34, (180, 200, 220), 1, cv2.LINE_AA)

        return canvas


class AutoNavDockSimNode(Node):
    def __init__(self, dock_pos=(7.55, -5.15)):
        super().__init__('autonav_dock_sim_node')
        self.dock_x, self.dock_y = dock_pos
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_enable_dock = self.create_publisher(Bool, '/apriltag/enable_docking', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.sub_status = self.create_subscription(String, '/apriltag/docking_status', self.status_cb, 10)

        self.odom_x = None
        self.odom_y = None
        self.odom_yaw = None
        self.last_odom_x = None
        self.last_odom_y = None
        self.last_odom_yaw = None

        self.world_x = 0.0
        self.world_y = 0.0
        self.world_yaw = 0.0
        self.in_phase2 = False
        self.dock_status = {}
        self.scan_world_pts = []

    def reset_world_pose_tracker(self, x: float, y: float, yaw: float):
        self.world_x = x
        self.world_y = y
        self.world_yaw = yaw
        self.last_odom_x = self.odom_x
        self.last_odom_y = self.odom_y
        self.last_odom_yaw = self.odom_yaw

    def odom_cb(self, msg: Odometry):
        ox = float(msg.pose.pose.position.x)
        oy = float(msg.pose.pose.position.y)
        qx = float(msg.pose.pose.orientation.x)
        qy = float(msg.pose.pose.orientation.y)
        qz = float(msg.pose.pose.orientation.z)
        qw = float(msg.pose.pose.orientation.w)
        oyaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

        if self.last_odom_x is not None:
            dx_o = ox - self.last_odom_x
            dy_o = oy - self.last_odom_y
            dyaw = normalize_angle(oyaw - self.last_odom_yaw)
            ds = dx_o * math.cos(self.last_odom_yaw) + dy_o * math.sin(self.last_odom_yaw)
            if abs(ds) < 0.25:
                mid_yaw = self.world_yaw + 0.5 * dyaw
                self.world_x += ds * math.cos(mid_yaw)
                self.world_y += ds * math.sin(mid_yaw)
                self.world_yaw = normalize_angle(self.world_yaw + dyaw)

            # 2단계 정밀 도킹 중 마커가 감지되면 비전 거리/횡방향 오차로 월드 위치를 부드럽게 보정
            if self.in_phase2 and self.dock_status.get("tag_detected"):
                dist_m = self.dock_status.get("dist_m")
                lat_m = self.dock_status.get("lat_err_m")
                if dist_m is not None and lat_m is not None and 0.15 < dist_m < 1.50:
                    vis_x = (self.dock_x - 0.05) - dist_m - 0.175
                    vis_y = self.dock_y + lat_m
                    self.world_x = 0.88 * self.world_x + 0.12 * vis_x
                    self.world_y = 0.88 * self.world_y + 0.12 * vis_y

        self.odom_x = ox
        self.odom_y = oy
        self.odom_yaw = oyaw
        self.last_odom_x = ox
        self.last_odom_y = oy
        self.last_odom_yaw = oyaw

    def scan_cb(self, msg: LaserScan):
        pts = []
        angle = msg.angle_min
        lx = self.world_x + 0.095 * math.cos(self.world_yaw)
        ly = self.world_y + 0.095 * math.sin(self.world_yaw)
        for i, r in enumerate(msg.ranges):
            if i % 3 == 0 and 0.08 < r < 4.5 and math.isfinite(r):
                wa = self.world_yaw + angle
                pts.append((lx + r * math.cos(wa), ly + r * math.sin(wa)))
            angle += msg.angle_increment
        self.scan_world_pts = pts

    def status_cb(self, msg: String):
        try:
            self.dock_status = json.loads(msg.data)
        except Exception:
            pass

    def send_cmd(self, vx: float, wz: float):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.pub_cmd.publish(msg)

    def set_docking_active(self, active: bool):
        self.in_phase2 = bool(active)
        msg = Bool()
        msg.data = bool(active)
        for _ in range(4):
            self.pub_enable_dock.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.04)


def run_simulation(trials_to_run: int = 3, out_dir: str = "/home/juyeon/ros2_amr_ws/sim_videos"):
    os.makedirs(out_dir, exist_ok=True)
    rclpy.init()
    node = AutoNavDockSimNode()
    grabber = MJPEGStreamGrabber("http://localhost:8088/stream.mjpg")
    grabber.start()
    top_renderer = TopViewMapRenderer("/home/juyeon/my_map.yaml")

    # MP4 VideoWriters (20 FPS, H.264 avc1)
    fps = 20.0
    fourcc = cv2.VideoWriter_fourcc(*'avc1')
    top_video_path = os.path.join(out_dir, "autonav_docking_topview.mp4")
    cam_video_path = os.path.join(out_dir, "autonav_docking_cameraview.mp4")
    comb_video_path = os.path.join(out_dir, "autonav_docking_combined.mp4")

    vw_top = cv2.VideoWriter(top_video_path, fourcc, fps, (640, 480))
    vw_cam = cv2.VideoWriter(cam_video_path, fourcc, fps, (640, 480))
    vw_comb = cv2.VideoWriter(comb_video_path, fourcc, fps, (1280, 480))

    print("⏳ Gazebo 및 AprilTag 정밀 도킹 노드 연결 대기 중...")
    t0 = time.time()
    while rclpy.ok() and (node.odom_x is None or not node.dock_status) and (time.time() - t0 < 15.0):
        rclpy.spin_once(node, timeout_sec=0.1)

    nominal_goal_x = 6.40
    nominal_goal_y = -5.15
    nominal_goal_yaw_deg = 0.0

    scenarios = [
        {
            "name": "시나리오 1: 자율주행이 왼쪽(+22cm)으로 치우치고 우측(-18°)으로 삐딱하게 도착 (우측 마커 1개만 보이는 상태)",
            "title_en": "Left Offset (+22cm) & Right Skew (-18 deg)",
            "start_x": 5.05, "start_y": -5.55, "start_yaw_deg": 12.0,
            "err_x": -0.05, "err_y": +0.22, "err_yaw_deg": -18.0,
        },
        {
            "name": "시나리오 2: 자율주행이 오른쪽(-19cm)으로 치우치고 좌측(+16°)으로 삐딱하게 도착 (좌측 마커 1개만 보이는 상태)",
            "title_en": "Right Offset (-19cm) & Left Skew (+16 deg)",
            "start_x": 5.05, "start_y": -4.80, "start_yaw_deg": -10.0,
            "err_x": +0.05, "err_y": -0.19, "err_yaw_deg": +16.0,
        },
        {
            "name": "시나리오 3: 자율주행이 너무 가까이(+25cm) 및 좌측(+15cm, -12°)으로 부정확하게 도착 (자동 후진 재정렬 유발)",
            "title_en": "Close Overshoot (+25cm) & Left Skew (+15cm, -12 deg)",
            "start_x": 5.15, "start_y": -5.35, "start_yaw_deg": 5.0,
            "err_x": +0.25, "err_y": +0.15, "err_yaw_deg": -12.0,
        },
    ]

    results = []
    total_frames = 0

    def record_step(sc_idx, sc, phase_label, autonav_path, docking_path):
        nonlocal total_frames
        top_img = top_renderer.render(
            sc_idx=sc_idx,
            total_sc=trials_to_run,
            sc_title_en=sc["title_en"],
            phase_name=phase_label,
            rx=node.world_x,
            ry=node.world_y,
            ryaw=node.world_yaw,
            start_pt=(sc["start_x"], sc["start_y"]),
            nominal_goal=(nominal_goal_x, nominal_goal_y),
            err_goal=(nominal_goal_x + sc["err_x"], nominal_goal_y + sc["err_y"]),
            err_tuple=(sc["err_x"] * 100.0, sc["err_y"] * 100.0, sc["err_yaw_deg"]),
            autonav_path=autonav_path,
            docking_path=docking_path,
            scan_pts=node.scan_world_pts,
            dock_status=node.dock_status,
        )
        cam_img = grabber.get_frame()
        comb_img = np.hstack([top_img, cam_img])
        vw_top.write(top_img)
        vw_cam.write(cam_img)
        vw_comb.write(comb_img)
        total_frames += 1
        return comb_img

    for idx, sc in enumerate(scenarios[:trials_to_run], start=1):
        print("\n" + "=" * 78)
        print(f"🚀 [{idx}/{trials_to_run}] {sc['name']}")
        print("=" * 78)

        autonav_path = []
        docking_path = []

        node.set_docking_active(False)
        node.send_cmd(0.0, 0.0)
        time.sleep(0.3)

        sx, sy = sc["start_x"], sc["start_y"]
        syaw = math.radians(sc["start_yaw_deg"])
        set_gazebo_pose("my_map_world", sx, sy, 0.06, syaw)
        for _ in range(10):
            node.send_cmd(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
        node.reset_world_pose_tracker(sx, sy, syaw)
        autonav_path.append((node.world_x, node.world_y))

        goal_x = nominal_goal_x + sc["err_x"]
        goal_y = nominal_goal_y + sc["err_y"]
        goal_yaw = math.radians(nominal_goal_yaw_deg + sc["err_yaw_deg"])

        # 출발 전 0.5초간 대기 프레임 기록
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.02)
            record_step(idx, sc, "PHASE 1: AUTONAV", autonav_path, docking_path)
            time.sleep(0.04)

        # 1단계: 자율주행 주행 및 녹화 (출발점 -> 부정확한 도착점)
        nav_start = time.time()
        while rclpy.ok() and (time.time() - nav_start < 18.0):
            t_step = time.time()
            rclpy.spin_once(node, timeout_sec=0.02)
            dx = goal_x - node.world_x
            dy = goal_y - node.world_y
            dist = math.hypot(dx, dy)
            target_hdg = math.atan2(dy, dx)
            hdg_err = normalize_angle(target_hdg - node.world_yaw)

            if len(autonav_path) == 0 or math.hypot(node.world_x - autonav_path[-1][0], node.world_y - autonav_path[-1][1]) > 0.02:
                autonav_path.append((node.world_x, node.world_y))

            if dist < 0.06:
                break

            if abs(hdg_err) > 0.45:
                vx = 0.04
                wz = max(-0.7, min(0.7, 1.8 * hdg_err))
            else:
                vx = max(0.08, min(0.24, 0.65 * dist))
                wz = max(-0.6, min(0.6, 1.6 * hdg_err))
            node.send_cmd(vx, wz)
            record_step(idx, sc, "PHASE 1: AUTONAV", autonav_path, docking_path)
            elapsed = time.time() - t_step
            if elapsed < 0.05:
                time.sleep(0.05 - elapsed)

        # 도착 지점에서 부정확한 도착 각도로 회전
        rot_start = time.time()
        while rclpy.ok() and (time.time() - rot_start < 6.0):
            t_step = time.time()
            rclpy.spin_once(node, timeout_sec=0.02)
            yaw_err = normalize_angle(goal_yaw - node.world_yaw)
            if abs(yaw_err) < math.radians(2.0):
                break
            wz = max(-0.6, min(0.6, 1.8 * yaw_err))
            node.send_cmd(0.0, wz)
            record_step(idx, sc, "PHASE 1: AUTONAV", autonav_path, docking_path)
            elapsed = time.time() - t_step
            if elapsed < 0.05:
                time.sleep(0.05 - elapsed)

        node.send_cmd(0.0, 0.0)
        set_gazebo_pose("my_map_world", goal_x, goal_y, 0.06, goal_yaw)
        for _ in range(10):
            node.send_cmd(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.04)
        node.reset_world_pose_tracker(goal_x, goal_y, goal_yaw)
        autonav_path.append((goal_x, goal_y))
        docking_path.append((goal_x, goal_y))

        # 인계 직전 0.6초간 프레임 기록
        for _ in range(12):
            rclpy.spin_once(node, timeout_sec=0.02)
            record_step(idx, sc, "HANDOVER -> DOCK", autonav_path, docking_path)
            time.sleep(0.04)

        init_tag_mode = node.dock_status.get("tag_mode", "NONE")
        node.set_docking_active(True)
        dock_start = time.time()
        best_lat_cm = 999.0
        best_yaw_deg = 999.0
        used_backup = False
        docked_ok = False
        last_comb_img = None

        while rclpy.ok() and (time.time() - dock_start < 40.0):
            t_step = time.time()
            rclpy.spin_once(node, timeout_sec=0.02)
            st = node.dock_status.get("state", "UNKNOWN")
            dist_m = node.dock_status.get("dist_m")
            lat_m = node.dock_status.get("lat_err_m")
            yaw_d = node.dock_status.get("yaw_deg", 0.0)

            if len(docking_path) == 0 or math.hypot(node.world_x - docking_path[-1][0], node.world_y - docking_path[-1][1]) > 0.015:
                docking_path.append((node.world_x, node.world_y))

            if st == "BACKING_UP":
                used_backup = True
            if lat_m is not None and dist_m is not None and dist_m <= 0.36:
                best_lat_cm = abs(lat_m) * 100.0
                best_yaw_deg = abs(yaw_d)

            last_comb_img = record_step(idx, sc, f"PHASE 2: {st}", autonav_path, docking_path)

            if st == "DOCKED" and (time.time() - dock_start > 3.0):
                docked_ok = True
                node.world_x = 7.50 - 0.19
                node.world_y = -5.15
                node.world_yaw = 0.0
                docking_path.append((node.world_x, node.world_y))
                for _ in range(24):
                    rclpy.spin_once(node, timeout_sec=0.02)
                    last_comb_img = record_step(idx, sc, "PHASE 2: DOCKED!", autonav_path, docking_path)
                    time.sleep(0.04)
                print(f"✅ [시나리오 {idx} 정밀 도킹 및 녹화 완료!] (LatErr={best_lat_cm:.1f}cm, Yaw={best_yaw_deg:.1f}°)")
                break

            elapsed = time.time() - t_step
            if elapsed < 0.05:
                time.sleep(0.05 - elapsed)

        if last_comb_img is not None:
            snap_path = os.path.join(out_dir, f"scenario_{idx}_summary.png")
            cv2.imwrite(snap_path, last_comb_img)

        results.append({
            "scenario": sc["name"],
            "arr_err_cm": (sc["err_x"] * 100.0, sc["err_y"] * 100.0, sc["err_yaw_deg"]),
            "init_tag_mode": init_tag_mode,
            "used_backup": used_backup,
            "final_lat_err_cm": best_lat_cm,
            "final_yaw_err_deg": best_yaw_deg,
            "docked": docked_ok,
        })

    vw_top.release()
    vw_cam.release()
    vw_comb.release()
    grabber.stop()

    print("\n" + "=" * 78)
    print(f"🎬 [시뮬레이션 영상 녹화 완료!] 총 {total_frames} 프레임 ({total_frames/fps:.1f}초 분량)")
    print(f" • 통합 분할 영상 (탑뷰 + 카메라뷰): {comb_video_path}")
    print(f" • 탑뷰 단독 영상 (Top-View):        {top_video_path}")
    print(f" • 카메라뷰 단독 영상 (Camera-View): {cam_video_path}")
    print("=" * 78)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=3, help="실행할 오차 시나리오 개수 (기본 3)")
    parser.add_argument("--out-dir", default="/home/juyeon/ros2_amr_ws/sim_videos", help="영상 저장 폴더")
    args = parser.parse_args()
    run_simulation(trials_to_run=args.trials, out_dir=args.out_dir)
