#!/usr/bin/env python3
"""
AMR AprilTag 정밀 위치 정렬 및 도킹 노드 (apriltag_docking_node.py)
------------------------------------------------------------------
실제 로봇(Physical AMR) 및 Gazebo 시뮬레이션 환경 모두를 지원하는
초정밀 AprilTag 도킹 ROS 2 노드입니다.

주요 특징:
1. 듀얼 태그 8-포인트 광역 기저선 solvePnP 알고리즘 (코너 순서 [TR, BR, BL, TL] 완벽 일치)
2. 대각선/경사각 진입 시에도 번개처럼 감지하는 다단계 고감도 AprilTag 검출 파이프라인
3. 스테이션 좌표계(Station Frame) 기반 3차 대칭 베지어 곡선(Cubic Bézier Curve) 생성
   - 대각선 진입 시 오버슈트 및 횡방향 출렁거림(S-Turn) 원천 배제
4. 횡방향 오차 가중 속도 스케일링 (중앙선 수렴 전 벽면 도달 방지)
5. 카메라 화각 초과 시 라이다(/scan) 기반 초근접 핸드오버 및 직진 정렬
6. 도킹 직전 최종 각도 정밀 정렬(In-place Yaw Squaring)
7. USB 웹캠(/dev/video*) 직접 캡처 및 ROS 2 토픽(/camera/image_raw) 동시 지원
8. 실시간 웹 모니터링 HUD 스트리밍 (기본 포트 8088, Start/Stop 버튼 지원)
"""

import os
import sys
import time
import math
import json
import threading
from http.server import HTTPServer, ThreadingHTTPServer, BaseHTTPRequestHandler
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, CameraInfo, LaserScan




TAG_DICT_MAP = {
    'tag36h11': cv2.aruco.DICT_APRILTAG_36h11,
    'tag25h9': cv2.aruco.DICT_APRILTAG_25h9,
    'tag16h5': cv2.aruco.DICT_APRILTAG_16h5,
    'aruco_original': cv2.aruco.DICT_ARUCO_ORIGINAL,
    'aruco_4x4_50': cv2.aruco.DICT_4X4_50,
}


class ApriltagDockingNode(Node):
    def __init__(self, **kwargs):
        super().__init__('apriltag_docking_node', **kwargs)

        # -----------------------------------------------------------------
        # 1. ROS 2 파라미터 선언
        # -----------------------------------------------------------------
        # 마커 및 배치 정보
        self.declare_parameter('tag_family', 'tag36h11')
        self.declare_parameter('target_tag_id', -1)            # -1: 임의의 감지된 첫 태그, >=0: 특정 ID
        self.declare_parameter('left_tag_id', 0)              # 왼쪽 마커 특정 ID (기본 0: Left Tag ID 0)
        self.declare_parameter('right_tag_id', 1)             # 오른쪽 마커 특정 ID (기본 1: Right Tag ID 1)
        self.declare_parameter('tag_size', 0.14)                # 실제 마커 외곽 크기 (미터, 기본 0.14 = 14cm)
        self.declare_parameter('tag_spacing', 0.19)             # 두 마커 중심 간격 (미터, 기본 0.19 = 19cm)
        self.declare_parameter('is_gazebo_simulation', False)   # Gazebo 텍스처(2/3 마진) 적용 여부 (현실 로봇은 False)

        # 도킹 목표 및 허용오차
        self.declare_parameter('target_dist', 0.0)              # 목표 정지 거리 (미터, 0.0 = 벽면 완전 밀착)
        self.declare_parameter('target_x_offset', 0.0)          # 목표 횡방향 오프셋 (미터)
        self.declare_parameter('target_yaw_offset', 0.0)        # 목표 방위각 (rad)
        self.declare_parameter('linear_tolerance', 0.02)        # 거리 허용오차 (미터, 기본 ±2cm)
        self.declare_parameter('lateral_tolerance', 0.025)      # 측면 편차 허용오차 (미터, 기본 ±2.5cm)
        self.declare_parameter('angular_tolerance', 0.045)      # 각도 허용오차 (rad, 약 ±2.5도)

        # 속도 및 제어 게인
        self.declare_parameter('max_linear_speed', 0.15)        # 최대 전진 속도 (m/s)
        self.declare_parameter('min_linear_speed', 0.03)        # 최소 전진 속도 (m/s)
        self.declare_parameter('max_angular_speed', 0.70)       # 최대 회전 속도 (rad/s)
        self.declare_parameter('search_angular_speed', 0.65)    # 탐색/역회전 회전 속도 (rad/s, 고속 탐색)
        self.declare_parameter('enable_search', True)           # 태그 미감지 시 자동 탐색/역회전 여부
        self.declare_parameter('auto_start', False)            # 노드 시작 시 즉시 도킹 제어 활성화 (기본: False, 시작 버튼 대기)
        self.declare_parameter('max_backup_count', 10)          # 최대 후진 리커버리 허용 횟수

        # 카메라 및 라이다 센서
        self.declare_parameter('use_ros_image', False)          # True: ROS 토픽 사용, False: OpenCV V4L2 직접 캡처
        self.declare_parameter('camera_device', '/dev/video0')   # V4L2 비디오 디바이스 경로 또는 인덱스
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('use_lidar', False)              # 라이다 센서 연동 여부 (기본: False, 순수 카메라 비전 모드)
        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('lidar_offset', 0.14)            # 라이다 중심과 로봇 전방 범퍼/카메라 간 거리 (미터)
        self.declare_parameter('flip_180', False)               # 카메라 180도 상하 반전 (기본: False)
        self.declare_parameter('show_gui', True)                # OpenCV 실시간 카메라 시점 GUI 창 표시 여부
        self.declare_parameter('straight_drive_duration', 7.5)  # 중앙 정렬 후 무조건 직진 밀착 시간 (완전 벽 밀착: 7.5초)
        self.declare_parameter('straight_drive_speed', 0.07)    # 무조건 직진 시 주행 속도 (m/s)
        self.declare_parameter('straight_dock_dist', 0.42)      # 최종 무조건 직진 밀착을 시작하는 도킹 진입 거리 (미터, 기본: 0.42m = 42cm, 14cm 태그 화각 이탈 전 조기 직진 전환)
        self.declare_parameter('web_port', 8088)                # 실시간 웹 모니터링 포트
        self.declare_parameter('show_axes', True)              # 도킹 스테이션 중심 3D 좌표축(Frame Axes) 렌더링 여부

        # -----------------------------------------------------------------
        # 2. 파라미터 값 추출
        # -----------------------------------------------------------------
        self.tag_family_str = self.get_parameter('tag_family').value
        self.target_tag_id = int(self.get_parameter('target_tag_id').value)
        self.left_tag_id = int(self.get_parameter('left_tag_id').value)
        self.right_tag_id = int(self.get_parameter('right_tag_id').value)
        self.tag_size = float(self.get_parameter('tag_size').value)
        self.tag_spacing = float(self.get_parameter('tag_spacing').value)
        self.is_gazebo = bool(self.get_parameter('is_gazebo_simulation').value)
        self.use_ros_image = bool(self.get_parameter('use_ros_image').value)

        # Gazebo 시뮬레이션에서는 8cm 평면 내에 태그가 2/3 비율로 렌더링되지만, 실제 로봇은 출력된 태그 실측 크기 그대로 사용
        # (is_gazebo_simulation=True 또는 use_ros_image=True 시 자동 적용)
        if self.is_gazebo or self.use_ros_image:
            self.tag_inner_size = self.tag_size * (320.0 / 480.0)
        else:
            self.tag_inner_size = self.tag_size

        self.target_dist = float(self.get_parameter('target_dist').value)
        self.target_x_offset = float(self.get_parameter('target_x_offset').value)
        self.target_yaw_offset = float(self.get_parameter('target_yaw_offset').value)
        self.linear_tolerance = float(self.get_parameter('linear_tolerance').value)
        self.lateral_tolerance = float(self.get_parameter('lateral_tolerance').value)
        self.angular_tolerance = float(self.get_parameter('angular_tolerance').value)

        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.min_linear_speed = float(self.get_parameter('min_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.search_angular_speed = float(self.get_parameter('search_angular_speed').value)
        self.enable_search = bool(self.get_parameter('enable_search').value)
        self.is_active = bool(self.get_parameter('auto_start').value)

        self.use_ros_image = bool(self.get_parameter('use_ros_image').value)
        self.camera_device = self.get_parameter('camera_device').value
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.use_lidar = bool(self.get_parameter('use_lidar').value)
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.lidar_offset = float(self.get_parameter('lidar_offset').value)
        self.flip_180 = bool(self.get_parameter('flip_180').value)
        self.show_gui = bool(self.get_parameter('show_gui').value)
        if self.show_gui and not os.environ.get('DISPLAY'):
            self.get_logger().info("🖥️ [디스플레이] DISPLAY 환경변수가 없어 show_gui를 비활성화합니다. (웹 뷰어 포트 8088 이용 가능)")
            self.show_gui = False
        elif self.show_gui:
            self.get_logger().info("🖥️ [디스플레이] OpenCV 실시간 카메라 시점 창(show_gui=True) 활성화")
        self.straight_drive_duration = float(self.get_parameter('straight_drive_duration').value)
        self.straight_drive_speed = float(self.get_parameter('straight_drive_speed').value)
        self.straight_dock_dist = float(self.get_parameter('straight_dock_dist').value)
        self.straight_drive_start_time = None
        self.web_port = int(self.get_parameter('web_port').value)
        self.show_axes = bool(self.get_parameter('show_axes').value)
        self.filt_rvec = None
        self.filt_tvec = None

        # -----------------------------------------------------------------
        # 3. OpenCV ArUco / AprilTag 검출기 초기화
        # -----------------------------------------------------------------
        dict_id = TAG_DICT_MAP.get(self.tag_family_str.lower(), cv2.aruco.DICT_APRILTAG_36h11)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

        # 기본 최적화 파라미터 (OpenCV 4.6 이하에서는 DetectorParameters_create 필수)
        self.default_params = self._create_detector_parameters()
        self.aruco_params = self._create_detector_parameters()

        if self.aruco_params is not None:
            if hasattr(cv2.aruco, 'CORNER_REFINE_CONTOUR'):
                self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
            elif hasattr(cv2.aruco, 'CORNER_REFINE_SUBPIX'):
                self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.aruco_params.adaptiveThreshWinSizeMin = 3
            self.aruco_params.adaptiveThreshWinSizeMax = 23
            self.aruco_params.adaptiveThreshWinSizeStep = 4
            self.aruco_params.adaptiveThreshConstant = 5
            self.aruco_params.minMarkerPerimeterRate = 0.01
            self.aruco_params.maxMarkerPerimeterRate = 5.0
            self.aruco_params.maxErroneousBitsInBorderRate = 0.50
            self.aruco_params.perspectiveRemoveIgnoredMarginPerCell = 0.13
            self.aruco_params.polygonalApproxAccuracyRate = 0.05

        # 대각선/경사각 진입 시 고감도 파라미터
        self.aruco_params_sensitive = self._create_detector_parameters()
        if self.aruco_params_sensitive is not None:
            self.aruco_params_sensitive.cornerRefinementMethod = getattr(cv2.aruco, 'CORNER_REFINE_NONE', 0)
            self.aruco_params_sensitive.adaptiveThreshWinSizeStep = 2
            self.aruco_params_sensitive.adaptiveThreshConstant = 7
            self.aruco_params_sensitive.minMarkerPerimeterRate = 0.01
            self.aruco_params_sensitive.maxMarkerPerimeterRate = 5.0

        self.detector = None
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # 카메라 내부 파라미터 (기본 640x480, CameraInfo 수신 시 동적 업데이트)
        self.camera_matrix = np.array([
            [530.0,   0.0, 320.0],
            [  0.0, 530.0, 240.0],
            [  0.0,   0.0,   1.0]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        self.cam_info_received = False

        # -----------------------------------------------------------------
        # 4. 상태 변수 및 필터 초기화
        # -----------------------------------------------------------------
        self.state = "SEARCHING" if self.is_active else "IDLE"
        self.docked_consecutive_count = 0
        self.last_detection_time = 0.0
        self.last_seen_dist = 999.0
        self.last_seen_x = 0.0
        self.front_obstacle_dist = 999.0
        self.closest_wall_angle = 0.0
        self.last_turn_dir = None
        self.handover_active = False
        self.has_seen_tag = False
        self.start_time = time.time()
        self.last_cmd_vx = 0.0
        self.last_cmd_wz = 0.0
        self.smoothed_wz = 0.0
        self.prev_heading_err = 0.0
        self.prev_control_time = time.time()

        # 🔄 [후진 리커버리(Backup Realignment) 변수]
        # 벽 앞에서 헤딩이 비스듬할 경우 뒤로 물러나 평행 공간을 확보하고 재진입
        self.is_reversing = False
        self.reverse_start_time = None
        self.reverse_duration = 5.5   # 후진 주행 시간 (초) -> 약 0.60m 안전 후퇴 (넓은 런웨이로 1~2회 내 도킹 보장)
        self.reverse_speed = -0.11    # 후진 선속도 (m/s)
        self.backup_count = 0         # 후진 재정렬 시도 횟수
        self.max_backup_count = int(self.get_parameter('max_backup_count').value)     # 최대 후진 허용 횟수
        self.is_realigning = False    # 후진 후 제자리 평행 회전 정렬 플래그

        # 라이다 기반 벽면 피팅 변수
        self.lidar_wall_yaw = 0.0
        self.lidar_wall_dist = 999.0
        self.lidar_wall_valid = False

        # 궤적 및 시각화 데이터
        self.bezier_pts = []
        self.lookahead_pt = None
        self.latest_frame = None
        self.debug_frame = None
        self.frame_lock = threading.Lock()
        self.cap = None

        # 로봇 실제 주행 궤적 기록 (선으로 시각화할 이동 경로 데이터)
        self.history_path = []  # [(x_st, z_st, yaw), ...]
        self.last_path_pt = None
        self.total_traveled_distance = 0.0

        # 단일 태그 지속 추적 래치 (18cm 깜빡임 및 좌우 헌팅 진동 방지)
        self.tracked_single_role = None  # "LEFT" or "RIGHT"
        self.last_dual_left_u = None
        self.last_dual_right_u = None
        self.last_dual_left_tvec = None
        self.last_dual_right_tvec = None
        self.has_seen_dual = False

        # 오도메트리 기반 정밀 주행 궤적 및 상대 변위 추적
        self.odom_pose = None       # (x, y, yaw)
        self.last_odom_pose = None  # (x, y, yaw)
        self.odom_start_pose = None
        self.station_heading = 0.0  # 도킹 스테이션 기준 로봇 머리 방향 (rad)
        self.odom_yaw_offset = None

        # -----------------------------------------------------------------
        # 5. ROS 2 통신 (Publisher & Subscriber)
        # -----------------------------------------------------------------
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/apriltag/docking_status', 10)
        self.pub_path = self.create_publisher(Path, '/apriltag/docking_path', 10)

        # 오도메트리 구독 (/odom: 매끄러운 실제 주행 경로 및 이동 거리 기록)
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.sub_enable = self.create_subscription(
            Bool,
            '/apriltag/enable_docking',
            self.enable_docking_callback,
            10
        )

        # 라이다 구독 (/scan: use_lidar=True 시에만 활성화)
        if self.use_lidar:
            self.sub_scan = self.create_subscription(
                LaserScan,
                self.lidar_topic,
                self.scan_callback,
                10
            )
            self.get_logger().info("📡 [센서] 라이다(/scan) 연동 활성화")
        else:
            self.sub_scan = None
            self.get_logger().info("🎯 [센서] 순수 카메라 비전(Pure Vision) 주행 모드 (라이다 완전 비활성화/미사용)")

        if self.use_ros_image:
            self.sub_image = self.create_subscription(
                Image,
                self.image_topic,
                self.image_callback,
                10
            )
            self.sub_camera_info = self.create_subscription(
                CameraInfo,
                self.camera_info_topic,
                self.camera_info_callback,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            )
            self.get_logger().info(f"📷 [ROS 2 이미지 구독] 토픽: {self.image_topic}")
        else:
            self.start_camera_capture()

        # 제어 주기 타이머 (20Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)

        # 실시간 웹 HUD 스트리밍 서버
        if self.web_port > 0:
            self.start_web_server()

        self.get_logger().info(
            f"🏷️ [AprilTag Docking] 노드 준비 완료! "
            f"(Family: {self.tag_family_str}, TargetID: {self.target_tag_id}, "
            f"TagSize: {self.tag_size*100:.1f}cm, Spacing: {self.tag_spacing*100:.1f}cm, "
            f"TargetDist: {self.target_dist*100:.1f}cm, WebPort: {self.web_port})"
        )

    # ---------------------------------------------------------------------
    # 카메라 캡처 및 이미지 콜백
    # ---------------------------------------------------------------------
    def start_camera_capture(self):
        """OpenCV 비디오 캡처 백그라운드 스레드 (V4L2)"""
        def capture_worker():
            cam_idx = 0
            if isinstance(self.camera_device, int) or (isinstance(self.camera_device, str) and self.camera_device.isdigit()):
                cam_idx = int(self.camera_device)
            elif isinstance(self.camera_device, str) and self.camera_device.startswith('/dev/video'):
                try:
                    cam_idx = int(self.camera_device.replace('/dev/video', ''))
                except ValueError:
                    cam_idx = self.camera_device

            self.get_logger().info(f"📷 [카메라] 장치: {self.camera_device} (ID: {cam_idx}) 연결 시도...")
            self.cap = cv2.VideoCapture(cam_idx, cv2.CAP_V4L2) if isinstance(cam_idx, int) else cv2.VideoCapture(cam_idx)
            if not self.cap.isOpened() and isinstance(cam_idx, int):
                self.cap = cv2.VideoCapture(cam_idx)

            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.cap.set(cv2.CAP_PROP_FPS, 30)
                self.get_logger().info(f"✅ [카메라] 장치 연결 성공! (640x480)")
            else:
                self.get_logger().error(f"❌ [카메라 오류] {self.camera_device}를 열 수 없습니다! 'ls -l /dev/video*'로 카메라 포트를 확인하세요.")

            while rclpy.ok():
                if not self.cap or not self.cap.isOpened():
                    time.sleep(1.0)
                    self.cap = cv2.VideoCapture(cam_idx, cv2.CAP_V4L2) if isinstance(cam_idx, int) else cv2.VideoCapture(cam_idx)
                    continue

                ret, frame = self.cap.read()
                if ret and frame is not None:
                    if self.flip_180:
                        frame = cv2.rotate(frame, cv2.ROTATE_180)
                    with self.frame_lock:
                        self.latest_frame = frame
                else:
                    time.sleep(0.03)

            if self.cap:
                self.cap.release()

        t = threading.Thread(target=capture_worker, daemon=True)
        t.start()

    def image_callback(self, msg: Image):
        try:
            if msg.encoding in ['rgb8', 'bgr8']:
                np_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                frame = np_arr[:, :, ::-1] if msg.encoding == 'rgb8' else np_arr.copy()
            elif msg.encoding == 'mono8':
                np_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
                frame = cv2.cvtColor(np_arr, cv2.COLOR_GRAY2BGR)
            elif msg.encoding in ['rgba8', 'bgra8']:
                np_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 4))
                frame = cv2.cvtColor(np_arr, cv2.COLOR_RGBA2BGR if msg.encoding == 'rgba8' else cv2.COLOR_BGRA2BGR)
            elif msg.encoding in ['yuv422', 'yuyv']:
                np_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 2))
                frame = cv2.cvtColor(np_arr, cv2.COLOR_YUV2BGR_YUYV)
            else:
                self.get_logger().warn_once(f"지원하지 않는 이미지 인코딩: {msg.encoding}")
                return

            if self.flip_180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)

            with self.frame_lock:
                self.latest_frame = frame
        except Exception as e:
            self.get_logger().warn(f"이미지 변환 오류: {e}")

    def camera_info_callback(self, msg: CameraInfo):
        if not self.cam_info_received:
            K = np.array(msg.k, dtype=np.float32).reshape((3, 3))
            if K[0, 0] > 10.0:
                self.camera_matrix = K
                if len(msg.d) >= 4:
                    self.dist_coeffs = np.array(msg.d, dtype=np.float32)
                self.cam_info_received = True
                self.get_logger().info(f"📐 [카메라 파라미터 수신] fx={K[0,0]:.1f}, fy={K[1,1]:.1f}, cx={K[0,2]:.1f}, cy={K[1,2]:.1f}")

    # ---------------------------------------------------------------------
    # 라이다 콜백: 전방 장애물 및 도킹 벽면 평면 피팅
    # ---------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        try:
            n = len(msg.ranges)
            if n == 0:
                return

            pts_x, pts_y = [], []
            d_min_front = 999.0
            d_min_wall = 999.0
            angle_min_wall = 0.0

            for i in range(n):
                angle = msg.angle_min + i * msg.angle_increment
                r = msg.ranges[i]
                if msg.range_min <= r <= msg.range_max and not math.isnan(r) and not math.isinf(r):
                    xi = r * math.cos(angle)
                    yi = r * math.sin(angle)

                    # 로봇 본체 자기 반사 필터링 (라이다 중심 기준)
                    if (-0.40 <= xi <= 0.05 and abs(yi) <= 0.20) or r < 0.05:
                        continue

                    # 전방 110도 내 가장 가까운 벽면 탐색 (태그 탐색 회전용)
                    if abs(angle) <= math.radians(110.0) and r < d_min_wall and xi > 0.05:
                        d_min_wall = r
                        angle_min_wall = angle

                    # 전방 +-25도 정면 장애물 거리
                    if abs(angle) <= math.radians(25.0) and xi > 0.05:
                        if r < d_min_front:
                            d_min_front = r

                    # 전방 +-55도 도킹 벽면 포인트 수집
                    if abs(angle) <= math.radians(55.0) and 0.05 <= xi <= 2.0 and abs(yi) <= 0.58:
                        pts_x.append(xi)
                        pts_y.append(yi)

            if d_min_front < 900.0:
                self.front_obstacle_dist = float(d_min_front)
            if d_min_wall < 4.0:
                self.closest_wall_angle = float(angle_min_wall)

            # 2D 평면 직선 피팅 (TLS: Total Least Squares / SVD)
            if len(pts_x) >= 8:
                pts = np.column_stack((pts_x, pts_y))
                mx = float(np.mean(pts[:, 0]))
                my = float(np.mean(pts[:, 1]))
                pts_c = pts - np.array([mx, my])
                cov = pts_c.T @ pts_c
                eigvals, eigvecs = np.linalg.eigh(cov)

                # 최소 고윳값의 고유벡터가 벽면 법선 (nx, ny)
                n_est = eigvecs[:, 0]
                if n_est[0] < 0:
                    n_est = -n_est

                perp_dist = float(mx * n_est[0] + my * n_est[1])
                raw_yaw = math.atan2(-n_est[1], n_est[0])

                if 0.05 <= perp_dist <= 2.5:
                    if not hasattr(self, 'filt_lidar_yaw'):
                        self.filt_lidar_yaw = raw_yaw
                        self.filt_lidar_dist = perp_dist
                    else:
                        self.filt_lidar_yaw = 0.70 * self.filt_lidar_yaw + 0.30 * raw_yaw
                        self.filt_lidar_dist = 0.70 * self.filt_lidar_dist + 0.30 * perp_dist

                    self.lidar_wall_yaw = float(np.clip(self.filt_lidar_yaw, -math.radians(35), math.radians(35)))
                    self.lidar_wall_dist = float(self.filt_lidar_dist)
                    self.lidar_wall_valid = True
                else:
                    self.lidar_wall_valid = False
            else:
                self.lidar_wall_valid = False
        except Exception:
            pass

    # ---------------------------------------------------------------------
    # 오도메트리 콜백 (/odom: 실제 휠 주행 이동량 및 매끄러운 궤적 누적)
    # ---------------------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        try:
            x = float(msg.pose.pose.position.x)
            y = float(msg.pose.pose.position.y)
            qx = float(msg.pose.pose.orientation.x)
            qy = float(msg.pose.pose.orientation.y)
            qz = float(msg.pose.pose.orientation.z)
            qw = float(msg.pose.pose.orientation.w)
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

            if self.last_odom_pose is not None:
                dx = x - self.last_odom_pose[0]
                dy = y - self.last_odom_pose[1]
                ds = math.hypot(dx, dy)
                if self.is_active and ds < 0.20:
                    self.total_traveled_distance += ds

            self.odom_pose = (x, y, yaw)
            self.last_odom_pose = (x, y, yaw)
        except Exception:
            pass

    # ---------------------------------------------------------------------
    # 제어 활성화 토픽 콜백 및 속도 발행
    # ---------------------------------------------------------------------
    def enable_docking_callback(self, msg: Bool):
        self.is_active = msg.data
        if self.is_active:
            self.state = "SEARCHING"
            self.docked_consecutive_count = 0
            self.handover_active = False
            self.history_path = []
            self.last_path_pt = None
            self.total_traveled_distance = 0.0
            self.odom_start_pose = self.odom_pose
            self.tracked_single_role = None
            self.has_seen_dual = False
            self.last_dual_left_tvec = None
            self.last_dual_right_tvec = None
            self.station_heading = 0.0
            self.odom_yaw_offset = None
            self.get_logger().info("▶️ [AprilTag Docking] 도킹 제어 활성화 요청 수신 (주행 궤적 선 초기화)")
        else:
            self.state = "IDLE"
            self.publish_cmd_vel(0.0, 0.0)
            self.get_logger().info("⏹️ [AprilTag Docking] 도킹 제어 정지 요청 수신")

    def publish_cmd_vel(self, vx: float, wz: float):
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.angular.z = float(wz)
        self.pub_cmd_vel.publish(cmd)

    # ---------------------------------------------------------------------
    # 다단계 방탄 AprilTag / ArUco 감지 알고리즘
    # ---------------------------------------------------------------------
    def _create_detector_parameters(self):
        """OpenCV 버전에 안전한 파라미터 인스턴스 생성 (OpenCV 4.6 Segfault 원천 차단)"""
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            return cv2.aruco.DetectorParameters_create()
        elif hasattr(cv2.aruco, 'DetectorParameters'):
            return cv2.aruco.DetectorParameters()
        return None

    def _run_single_detect(self, img, aruco_dict, params):
        try:
            if hasattr(cv2.aruco, 'ArucoDetector'):
                p = params if params is not None else self.aruco_params
                detector = cv2.aruco.ArucoDetector(aruco_dict, p)
                c, ids, rej = detector.detectMarkers(img)
                if ids is not None and len(ids) > 0:
                    return c, ids, rej
            if hasattr(cv2.aruco, 'detectMarkers'):
                if params is not None:
                    c, ids, rej = cv2.aruco.detectMarkers(img, aruco_dict, parameters=params)
                else:
                    c, ids, rej = cv2.aruco.detectMarkers(img, aruco_dict)
                if ids is not None and len(ids) > 0:
                    return c, ids, rej
        except Exception:
            pass
        return None, None, None

    def detect_markers(self, gray):
        try:
            # 1단계: 지정된 딕셔너리 + 튜닝된 파라미터 감지
            c, ids, rej = self._run_single_detect(gray, self.aruco_dict, self.aruco_params)
            if ids is not None and len(ids) >= 2:
                return c, ids, rej

            # 1-B단계: 대각선/경사각 고감도 파라미터 감지
            if hasattr(self, 'aruco_params_sensitive') and self.aruco_params_sensitive is not None:
                c_s, ids_s, rej_s = self._run_single_detect(gray, self.aruco_dict, self.aruco_params_sensitive)
                if ids_s is not None and len(ids_s) >= 2:
                    return c_s, ids_s, rej_s
                elif (ids is None or len(ids) == 0) and ids_s is not None and len(ids_s) > 0:
                    c, ids, rej = c_s, ids_s, rej_s

            if ids is not None and len(ids) > 0:
                return c, ids, rej

            # 2단계: OpenCV 기본 파라미터 감지 (오버라이드 없는 표준 검출)
            default_params = self.default_params
            c, ids, rej = self._run_single_detect(gray, self.aruco_dict, default_params)
            if ids is not None and len(ids) > 0:
                return c, ids, rej

            # 3단계: CLAHE 조명 대비 강화 감지
            clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
            clahe_gray = clahe.apply(gray)
            c, ids, rej = self._run_single_detect(clahe_gray, self.aruco_dict, default_params)
            if ids is not None and len(ids) > 0:
                return c, ids, rej

            # 4단계: 좌우 반전(Horizontal Mirror) 복원 검출
            h, w = gray.shape[:2]
            gray_hf = cv2.flip(gray, 1)
            c_hf, ids_hf, rej_hf = self._run_single_detect(gray_hf, self.aruco_dict, self.aruco_params)
            if ids_hf is None or len(ids_hf) == 0:
                c_hf, ids_hf, rej_hf = self._run_single_detect(gray_hf, self.aruco_dict, default_params)

            if ids_hf is not None and len(ids_hf) > 0:
                restored = []
                for corner_item in c_hf:
                    raw = corner_item[0].copy()
                    raw[:, 0] = (w - 1) - raw[:, 0]
                    cw = np.array([[raw[1], raw[0], raw[3], raw[2]]], dtype=np.float32)
                    restored.append(cw)
                return restored, ids_hf, rej_hf

            # 5단계: 멀티 패밀리 자동 폴백 (tag25h9, tag16h5, aruco 4x4 등 다른 마커를 인쇄했을 때 자동 지원)
            fallback_dicts = [
                cv2.aruco.DICT_APRILTAG_25h9,
                cv2.aruco.DICT_APRILTAG_16h5,
                cv2.aruco.DICT_4X4_50,
                cv2.aruco.DICT_ARUCO_ORIGINAL
            ]
            for f_id in fallback_dicts:
                f_dict = cv2.aruco.getPredefinedDictionary(f_id)
                c_f, ids_f, rej_f = self._run_single_detect(gray, f_dict, default_params)
                if ids_f is not None and len(ids_f) > 0:
                    if not hasattr(self, '_last_family_warn'):
                        self._last_family_warn = 0.0
                    if time.time() - self._last_family_warn > 3.0:
                        self._last_family_warn = time.time()
                        self.get_logger().info(f"💡 [마커 자동 감지] 기본 {self.tag_family_str} 대신 다른 마커(Dict: {f_id}) 감지 성공! (IDs: {ids_f.flatten()})")
                    return c_f, ids_f, rej_f

        except Exception as e:
            pass
        return None, None, None

    def estimate_single_pose(self, corner):
        s = self.tag_inner_size / 2.0
        # OpenCV AprilTag 코너 검출 순서 [TL, TR, BR, BL]와 완벽 일치하는 3D 모델 좌표
        tag_pts = np.array([[-s, -s, 0.0], [s, -s, 0.0], [s, s, 0.0], [-s, s, 0.0]], dtype=np.float32)
        img_pts = corner.reshape((4, 2)).astype(np.float32)
        flags = getattr(cv2, 'SOLVEPNP_IPPE_SQUARE', cv2.SOLVEPNP_ITERATIVE)
        ret, rvec, tvec = cv2.solvePnP(tag_pts, img_pts, self.camera_matrix, self.dist_coeffs, flags=flags)
        if ret:
            return rvec.flatten(), tvec.flatten()
        return np.zeros(3), np.zeros(3)

    # ---------------------------------------------------------------------
    # 메인 도킹 제어 루프 (20Hz)
    # ---------------------------------------------------------------------
    def control_loop(self):
        frame = None
        with self.frame_lock:
            if self.latest_frame is not None:
                frame = self.latest_frame.copy()

        if frame is None:
            if not hasattr(self, '_last_cam_warn'):
                self._last_cam_warn = 0.0
            if time.time() - self._last_cam_warn > 2.5:
                self._last_cam_warn = time.time()
                src_name = f"ROS 2 이미지 토픽 '{self.image_topic}'" if self.use_ros_image else f"카메라 디바이스 '{self.camera_device}'"
                self.get_logger().warn(
                    f"📷 [카메라 대기] {src_name}로부터 영상 프레임을 수신 대기 중입니다..."
                )
            return

        h, w = frame.shape[:2]
        self.camera_matrix[0, 2] = w / 2.0
        self.camera_matrix[1, 2] = h / 2.0

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detect_markers(gray)

        detected = None
        if ids is not None and len(ids) > 0:
            flat_ids = ids.flatten()
            if not hasattr(self, '_last_detect_log'):
                self._last_detect_log = 0.0
            if time.time() - self._last_detect_log > 1.5:
                self._last_detect_log = time.time()
                self.get_logger().info(f"🏷️ [마커 감지 성공!] 카메라에 감지된 마커 IDs: {flat_ids.tolist()}")
            matching_indices = []
            valid_docking_ids = []
            if getattr(self, 'left_tag_id', -1) >= 0:
                valid_docking_ids.append(self.left_tag_id)
            if getattr(self, 'right_tag_id', -1) >= 0:
                valid_docking_ids.append(self.right_tag_id)
            if getattr(self, 'target_tag_id', -1) >= 0:
                valid_docking_ids.append(self.target_tag_id)

            for idx, tid in enumerate(flat_ids):
                if valid_docking_ids:
                    if tid in valid_docking_ids:
                        matching_indices.append(idx)
                else:
                    matching_indices.append(idx)

            candidates = []
            for idx in matching_indices:
                corner = corners[idx]
                tid = int(flat_ids[idx])
                pts = corner.reshape((4, 2)).astype(np.float32)
                u = float(pts[:, 0].mean())
                v = float(pts[:, 1].mean())
                peri = float(cv2.arcLength(pts, True))
                rvec, tvec = self.estimate_single_pose(corner)
                candidates.append({
                    "idx": idx,
                    "tid": tid,
                    "corner": corner,
                    "pts": pts,
                    "u": u,
                    "v": v,
                    "peri": peri,
                    "rvec": rvec,
                    "tvec": tvec,
                    "x": float(tvec[0]),
                    "y": float(tvec[1]),
                    "z": float(tvec[2])
                })

            candidates.sort(key=lambda c: c["u"])

            cx = self.camera_matrix[0, 2]
            fx = self.camera_matrix[0, 0]

            left_c = None
            right_c = None
            tag_mode = "NONE"
            baseline = 0.0
            rvec_use = None
            z_mid = 1.0
            y_mid = 0.0
            x_mid = 0.0
            raw_tag_yaw = 0.0
            rot_mat_use = None
            station_lat_err = 0.0
            s = self.tag_inner_size / 2.0

            has_explicit_lr = (
                getattr(self, 'left_tag_id', -1) >= 0
                and getattr(self, 'right_tag_id', -1) >= 0
                and self.left_tag_id != self.right_tag_id
            )
            cand_by_id = {c["tid"]: c for c in candidates}

            if has_explicit_lr and (self.left_tag_id in cand_by_id) and (self.right_tag_id in cand_by_id):
                # 🌟 [고유 태그 ID 기반 절대 좌/우 듀얼 매칭 (Left=0, Right=1)]
                left_c = cand_by_id[self.left_tag_id]
                right_c = cand_by_id[self.right_tag_id]
                is_dual = True
            elif not has_explicit_lr and len(candidates) >= 2:
                # 🌟 [자동 화면 좌표 u 기반 듀얼 매칭]
                left_c = candidates[0]
                right_c = candidates[-1]
                is_dual = True
            else:
                is_dual = False

            if is_dual:
                # 🌟 [듀얼 태그 8-포인트 광역 기저선 solvePnP]
                tag_mode = "DUAL_TAG"
                self.last_dual_left_u = float(left_c["u"])
                self.last_dual_right_u = float(right_c["u"])
                self.last_dual_left_id = int(left_c["tid"])
                self.last_dual_right_id = int(right_c["tid"])
                self.last_dual_left_tvec = np.array(left_c["tvec"], dtype=np.float32)
                self.last_dual_right_tvec = np.array(right_c["tvec"], dtype=np.float32)
                self.has_seen_dual = True
                self.tracked_single_role = None

                s = self.tag_inner_size / 2.0
                tag_l_x = -self.tag_spacing / 2.0
                tag_r_x = self.tag_spacing / 2.0

                # 코너 순서 [TL, TR, BR, BL]와 완벽 일치하는 3D 모델
                tag_pts = np.array([[-s, -s, 0.0], [s, -s, 0.0], [s, s, 0.0], [-s, s, 0.0]], dtype=np.float32)
                obj_8pts = np.vstack([tag_pts + [tag_l_x, 0.0, 0.0], tag_pts + [tag_r_x, 0.0, 0.0]]).astype(np.float32)
                img_8pts = np.vstack([left_c["pts"], right_c["pts"]])

                ret_pnp, rvec_multi, tvec_multi = cv2.solvePnP(
                    obj_8pts, img_8pts, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
                )

                if ret_pnp and 0.05 < tvec_multi[2][0] < 5.0:
                    x_mid = float(tvec_multi[0][0])
                    y_mid = float(tvec_multi[1][0])
                    z_mid = float(tvec_multi[2][0])
                    rot_mat, _ = cv2.Rodrigues(rvec_multi)
                    nx = float(rot_mat[0, 2])
                    nz = float(rot_mat[2, 2])
                    raw_tag_yaw = math.atan2(nx, nz)
                    rvec_use = rvec_multi
                    rot_mat_use = rot_mat

                    # 🌟 [참 3D 강체 역기구학: 도킹 스테이션 기준 로봇 절대 물리 좌표 (-R.T @ tvec)]
                    # 로봇이 제자리에서 회전(Yaw 회전)해도 궤적 좌표가 변하지 않는 진정한 3D 회전 불변 좌표계!
                    p_robot_dock = - rot_mat.T @ tvec_multi
                    robot_station_x = float(p_robot_dock[0][0])
                    robot_station_z = abs(float(p_robot_dock[2][0]))  # 전방 거리는 항상 벽 앞 양수(+)
                    station_lat_err = -robot_station_x
                    station_dist_z = robot_station_z
                else:
                    z_mid = (left_c["z"] + right_c["z"]) / 2.0
                    y_mid = (left_c["y"] + right_c["y"]) / 2.0
                    u_mid = (float(left_c["u"]) + float(right_c["u"])) / 2.0
                    x_mid = float((u_mid - cx) * z_mid / fx)
                    rot_mat, _ = cv2.Rodrigues(left_c["rvec"])
                    nx = float(rot_mat[0, 2])
                    nz = float(rot_mat[2, 2])
                    raw_tag_yaw = math.atan2(nx, nz)
                    rvec_use = left_c["rvec"]
                    rot_mat_use = rot_mat
                    robot_station_x = -x_mid
                    robot_station_z = z_mid
                    station_lat_err = x_mid
                    station_dist_z = z_mid

                baseline = abs(right_c["x"] - left_c["x"])

            elif len(candidates) >= 1:
                # 🌟 [단일 태그 관측 시 가상 중심점 투영]
                if has_explicit_lr and self.left_tag_id in cand_by_id:
                    c = cand_by_id[self.left_tag_id]
                elif has_explicit_lr and self.right_tag_id in cand_by_id:
                    c = cand_by_id[self.right_tag_id]
                else:
                    c = candidates[0]
                s = self.tag_inner_size / 2.0
                # 코너 순서 [TL, TR, BR, BL]와 완벽 일치하는 3D 모델
                tag_pts = np.array([[-s, -s, 0.0], [s, -s, 0.0], [s, s, 0.0], [-s, s, 0.0]], dtype=np.float32)
                ret_s, rv, tv = cv2.solvePnP(
                    tag_pts, c["pts"], self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
                )

                if ret_s and 0.05 < tv[2][0] < 5.0:
                    rot_mat, _ = cv2.Rodrigues(rv)
                    nx = float(rot_mat[0, 2])
                    nz = float(rot_mat[2, 2])
                    raw_tag_yaw = math.atan2(nx, nz)
                    rot_mat_use = rot_mat

                    half_spacing = self.tag_spacing / 2.0
                    # 카메라의 태그 기준 3D 상대 위치: - R.T @ tv
                    p_cam_in_tag = - rot_mat.T @ tv
                    # 도킹 중심 기준 로봇 위치 (X 오프셋: Left tag = -half_spacing, Right tag = +half_spacing)
                    p_cand_L = p_cam_in_tag.flatten() + np.array([-half_spacing, 0.0, 0.0])
                    p_cand_R = p_cam_in_tag.flatten() + np.array([half_spacing, 0.0, 0.0])

                    lat_err_L = float(-p_cand_L[0])
                    dist_z_L = abs(float(p_cand_L[2]))  # 전방 거리는 항상 벽 앞 양수(+)
                    lat_err_R = float(-p_cand_R[0])
                    dist_z_R = abs(float(p_cand_R[2]))  # 전방 거리는 항상 벽 앞 양수(+)

                    # 🌟 [단일 태그 좌/우 판별]
                    # 0단계: 파라미터로 지정된 좌/우 고유 태그 ID가 있는 경우 (절대 기준 확정)
                    if getattr(self, 'left_tag_id', -1) >= 0 and c["tid"] == self.left_tag_id:
                        self.tracked_single_role = "LEFT"
                    elif getattr(self, 'right_tag_id', -1) >= 0 and c["tid"] == self.right_tag_id:
                        self.tracked_single_role = "RIGHT"
                    # 1단계: 듀얼 마커에서 학습된 좌/우 ID가 서로 다른 경우 ID로 즉시 판별
                    elif (
                        self.has_seen_dual
                        and getattr(self, 'last_dual_left_id', -1) != getattr(self, 'last_dual_right_id', -1)
                    ):
                        if c["tid"] == self.last_dual_left_id:
                            self.tracked_single_role = "LEFT"
                        elif c["tid"] == self.last_dual_right_id:
                            self.tracked_single_role = "RIGHT"
                    elif hasattr(self, 'filt_lat_err') and abs(self.filt_lat_err) < 2.0:
                        diff_L = abs(lat_err_L - self.filt_lat_err)
                        diff_R = abs(lat_err_R - self.filt_lat_err)
                        self.tracked_single_role = "LEFT" if diff_L <= diff_R else "RIGHT"
                    elif self.has_seen_dual and self.last_dual_left_tvec is not None and self.last_dual_right_tvec is not None:
                        cur_tv = tv.flatten()
                        d_left_3d = float(np.linalg.norm(cur_tv - self.last_dual_left_tvec.flatten()))
                        d_right_3d = float(np.linalg.norm(cur_tv - self.last_dual_right_tvec.flatten()))
                        self.tracked_single_role = "LEFT" if d_left_3d <= d_right_3d else "RIGHT"
                    elif self.tracked_single_role is None:
                        self.tracked_single_role = "LEFT" if (float(tv[0][0]) < 0 or float(c["u"]) < cx) else "RIGHT"

                    is_left = (self.tracked_single_role == "LEFT")

                    if is_left:
                        robot_station_x = float(p_cand_L[0])
                        robot_station_z = dist_z_L
                        station_lat_err = lat_err_L
                        station_dist_z = dist_z_L
                        left_c = c
                        right_c = None
                        tvec_mid = - rot_mat @ (p_cand_L.reshape(3, 1))
                    else:
                        robot_station_x = float(p_cand_R[0])
                        robot_station_z = dist_z_R
                        station_lat_err = lat_err_R
                        station_dist_z = dist_z_R
                        left_c = None
                        right_c = c
                        tvec_mid = - rot_mat @ (p_cand_R.reshape(3, 1))

                    x_mid = float(tvec_mid[0])
                    y_mid = float(tvec_mid[1])
                    z_mid = float(tvec_mid[2])
                    rvec_use = rv
                else:
                    is_left = (self.tracked_single_role == "LEFT") if self.tracked_single_role else (float(c["tvec"][0]) < 0)
                    offset_x = (self.tag_spacing / 2.0) if is_left else (-self.tag_spacing / 2.0)
                    x_mid = float(c["tvec"][0]) + offset_x
                    y_mid = float(c["tvec"][1])
                    z_mid = float(c["tvec"][2])
                    raw_tag_yaw = 0.0
                    station_lat_err = x_mid
                    station_dist_z = z_mid
                    rvec_use = c["rvec"]

                baseline = 0.0
                tag_mode = "SINGLE_TAG"

            if candidates:
                raw_bearing = math.atan2(x_mid, max(0.08, z_mid))

                # 지터 방지 EMA 필터
                if not hasattr(self, 'filt_yaw') or self.filt_yaw is None:
                    self.filt_yaw = raw_tag_yaw
                    self.filt_bearing = raw_bearing
                    self.filt_z = z_mid
                    self.filt_lat_err = station_lat_err
                    self.filt_station_z = station_dist_z
                else:
                    alpha = 0.80 if len(candidates) >= 2 else 0.70
                    self.filt_yaw = (1.0 - alpha) * self.filt_yaw + alpha * raw_tag_yaw
                    self.filt_bearing = (1.0 - alpha) * self.filt_bearing + alpha * raw_bearing
                    self.filt_z = (1.0 - alpha) * self.filt_z + alpha * z_mid
                    self.filt_lat_err = (1.0 - alpha) * self.filt_lat_err + alpha * station_lat_err
                    self.filt_station_z = (1.0 - alpha) * self.filt_station_z + alpha * station_dist_z

                # 3D 축 좌표계(rvec, tvec) EMA 안정화 필터 (화면 중앙 축 튐 완전 제거)
                if rvec_use is not None:
                    rvec_arr = np.array(rvec_use, dtype=np.float32).flatten()
                    tvec_arr = np.array([x_mid, y_mid, self.filt_z], dtype=np.float32).flatten()
                    if self.filt_rvec is None:
                        self.filt_rvec = rvec_arr.copy()
                        self.filt_tvec = tvec_arr.copy()
                    else:
                        self.filt_rvec = 0.80 * self.filt_rvec + 0.20 * rvec_arr
                        self.filt_tvec = 0.80 * self.filt_tvec + 0.20 * tvec_arr
                    draw_rvec = self.filt_rvec.copy()
                    draw_tvec = self.filt_tvec.copy()
                else:
                    draw_rvec = None
                    draw_tvec = None

                detected = {
                    "id": candidates[0]["tid"],
                    "tag_mode": tag_mode,
                    "is_dual": len(candidates) >= 2,
                    "baseline": baseline,
                    "candidates": candidates,
                    "left_c": left_c,
                    "right_c": right_c,
                    "rvec": draw_rvec,
                    "rot_mat": rot_mat_use,
                    "tvec": draw_tvec if draw_tvec is not None else np.array([x_mid, y_mid, self.filt_z]),
                    "x": x_mid,
                    "y": y_mid,
                    "z": self.filt_z,
                    "lat_err": self.filt_lat_err,
                    "station_x": -self.filt_lat_err,
                    "station_z": self.filt_station_z,
                    "bearing": self.filt_bearing,
                    "yaw": self.filt_yaw
                }
                self.last_detection_time = time.time()
                self.last_seen_dist = float(self.filt_station_z)
                self.last_seen_x = float(self.filt_lat_err)
                self.has_seen_tag = True

            # 🌟 [도킹 스테이션 기준 로봇 실제 물리 헤딩 각도 산출]
            # 카메라 전방 벡터([0, 0, 1]^T)를 도킹 스테이션 좌표계로 역변환하여 진짜 진행 방향 계산
            if rot_mat_use is not None:
                cam_fwd = rot_mat_use.T @ np.array([[0.0], [0.0], [1.0]], dtype=np.float32)
                cur_hdg = math.atan2(float(cam_fwd[0][0]), float(cam_fwd[2][0]))
                self.station_heading = cur_hdg
                if self.odom_pose is not None:
                    self.odom_yaw_offset = cur_hdg - self.odom_pose[2]
            elif self.odom_pose is not None and self.odom_yaw_offset is not None:
                self.station_heading = self.odom_pose[2] + self.odom_yaw_offset

        # -----------------------------------------------------------------
        # 0. 비활성(STOP / IDLE) 상태 시 속도 0 발행 및 카메라 대기 화면 송출
        # -----------------------------------------------------------------
        if not self.is_active:
            self.state = "IDLE"
            self.publish_cmd_vel(0.0, 0.0)
            self.smoothed_wz = 0.0
            curr_dist = float(detected["station_z"]) if detected else 999.0
            dist_err = (curr_dist - self.target_dist) if detected else 999.0
            yaw_err = float(detected["yaw"]) if detected else 0.0
            motion_code = "STOP"
            motion_kr = "🛑 정지 대기 중입니다 (시작 버튼 대기)"
            motion_en = "IDLE - READY TO DOCK"

            status_info = {
                "state": "IDLE",
                "tag_detected": detected is not None,
                "tag_id": detected["id"] if detected else None,
                "tag_mode": detected.get("tag_mode", "NONE") if detected else "NONE",
                "dist_m": round(curr_dist, 3) if curr_dist < 900.0 else None,
                "dist_err_m": round(dist_err, 3) if curr_dist < 900.0 else None,
                "lat_err_m": round(detected.get("lat_err", 0.0), 3) if detected else None,
                "yaw_deg": round(math.degrees(yaw_err), 1),
                "handover_active": self.handover_active,
                "cmd_vx": 0.0,
                "cmd_wz": 0.0,
                "motion_code": motion_code,
                "motion_kr": motion_kr,
                "motion_en": motion_en,
                "flip_180": self.flip_180,
                "timestamp": time.time()
            }
            self.latest_status = status_info
            status_msg = String()
            status_msg.data = json.dumps(status_info, ensure_ascii=False)
            self.pub_status.publish(status_msg)

            dbg = self.render_overlay(frame, detected, 0.0, 0.0, curr_dist, dist_err)
            with self.frame_lock:
                self.debug_frame = dbg
            if self.show_gui:
                try:
                    cv2.imshow("AMR AprilTag Precision Docking", dbg)
                    cv2.waitKey(1)
                except Exception:
                    pass
            return

        # -----------------------------------------------------------------
        # 5-1. 실시간 주행 궤적(선) 기록 및 ROS 2 Path 토픽 발행
        # -----------------------------------------------------------------
        # 🛡️ [매끄러운 주행 궤적 생성 및 이상치 필터링]
        # PnP 노이즈로 인한 삐쭉빼쭉한 튐을 방지하고, 궤적이 초기화되어 사라지는 버그를 원천 해결합니다.
        if detected is not None and self.state in ["APPROACHING", "ALIGNING", "STRAIGHT_IN", "BACKING_UP", "REALIGNING"] and abs(self.last_cmd_vx) > 0.015:
            # 로봇의 도킹 스테이션 기준 실제 물리 위치
            raw_x_pos = float(detected.get("station_x", 0.0))
            raw_z_pos = float(detected.get("station_z", 1.0))
            yaw_val = float(detected.get("yaw", 0.0))

            if len(self.history_path) == 0:
                self.history_path.append((raw_x_pos, raw_z_pos, yaw_val))
                self.last_path_pt = (raw_x_pos, raw_z_pos)
            else:
                lx, lz = self.last_path_pt
                d_moved = math.hypot(raw_x_pos - lx, raw_z_pos - lz)

                if d_moved > 0.25:
                    # 🛡️ 순간적인 PnP 점프/노이즈 이상치는 무시(Reject)하여 궤적이 사방으로 튀는 현상 원천 차단!
                    # (절대 기존 누적 궤적을 삭제하지 않음)
                    pass
                elif d_moved >= 0.010:
                    # 🌟 궤적 스무딩(EMA 필터): 미세 진동을 흡수하여 실제 주행선과 동일한 매끄러운 곡선 생성
                    sm_x = 0.75 * raw_x_pos + 0.25 * lx
                    sm_z = 0.75 * raw_z_pos + 0.25 * lz
                    actual_d = math.hypot(sm_x - lx, sm_z - lz)

                    self.history_path.append((sm_x, sm_z, yaw_val))
                    # 휠 오도메트리가 없는 경우에만 비전 이동량 누적 (오도메트리 우선)
                    if self.odom_pose is None:
                        self.total_traveled_distance += actual_d
                    self.last_path_pt = (sm_x, sm_z)
                    if len(self.history_path) > 1200:
                        self.history_path.pop(0)

        elif self.state == "STRAIGHT_IN" and len(self.history_path) > 0:
            # 최종 직진 밀착 구간에서 태그가 화각 밖으로 벗어나도 데드레코닝으로 궤적선 부드럽게 연장
            lx, lz, lyaw = self.history_path[-1]
            nz = max(0.02, lz - (self.straight_drive_speed * 0.05))
            if (lz - nz) >= 0.010:
                self.history_path.append((lx, nz, lyaw))
                if self.odom_pose is None:
                    self.total_traveled_distance += (lz - nz)
                self.last_path_pt = (lx, nz)

        # ROS 2 Path 토픽 발행 (/apriltag/docking_path: RViz2 등 시각화 도구 연동)
        if self.pub_path.get_subscription_count() > 0 and len(self.history_path) > 0:
            p_msg = Path()
            p_msg.header.stamp = self.get_clock().now().to_msg()
            p_msg.header.frame_id = "dock_station"
            for hx, hz, _ in self.history_path:
                pose = PoseStamped()
                pose.header = p_msg.header
                # RViz2 바닥 평면(X-Y) 표시를 위해 전방 거리를 Y에 매핑 (수직 솟구침 방지)
                pose.pose.position.x = float(hx)
                pose.pose.position.y = float(hz)
                pose.pose.position.z = 0.0
                p_msg.poses.append(pose)
            self.pub_path.publish(p_msg)

        # -----------------------------------------------------------------
        # 6. 거리 및 각도 오차 산출 (순수 비전 우선, 라이다 옵션)
        # -----------------------------------------------------------------
        tag_z = detected["z"] if detected is not None else 999.0
        if self.use_lidar:
            if self.lidar_wall_valid:
                lidar_front = max(0.0, self.lidar_wall_dist - self.lidar_offset)
            elif self.front_obstacle_dist < 900.0:
                lidar_front = max(0.0, self.front_obstacle_dist - self.lidar_offset)
            else:
                lidar_front = 999.0
        else:
            lidar_front = 999.0

        if detected is not None:
            curr_dist = tag_z
        elif self.use_lidar and lidar_front < 900.0:
            curr_dist = lidar_front
        else:
            curr_dist = 999.0

        dist_err = curr_dist - self.target_dist

        # 각도 및 횡방향 오차
        if detected is not None:
            yaw_err = float(detected.get("yaw", 0.0))
            bearing = float(detected.get("bearing", 0.0))
            x_target = float(detected.get("x", 0.0))
        else:
            yaw_err = 0.0
            bearing = 0.0
            x_target = 0.0

        # -----------------------------------------------------------------
        # 7. 주행 제어 명령 산출 (정 중앙 정렬 시 무조건 직진 락 제어)
        # -----------------------------------------------------------------
        vx = 0.0
        wz = 0.0

        if not self.is_active:
            self.state = "IDLE"
            vx, wz = 0.0, 0.0
            self.smoothed_wz = 0.0
            self.straight_drive_start_time = None

        elif self.state == "DOCKED":
            vx, wz = 0.0, 0.0
            self.smoothed_wz = 0.0

        # 🌟 [무조건 3초 직진 주행 모드 (최종 밀착 단계)]
        # 도킹 진입 거리 이내에서 중앙 정렬되었거나 화각 이탈 시 3초 동안 무조건 직진!
        elif self.straight_drive_start_time is not None:
            elapsed = time.time() - self.straight_drive_start_time
            if elapsed < self.straight_drive_duration:
                self.state = "STRAIGHT_IN"
                wz = 0.0
                self.smoothed_wz = 0.0
                vx = float(self.straight_drive_speed)
            else:
                self.state = "DOCKED"
                vx = 0.0
                wz = 0.0
                self.smoothed_wz = 0.0
                self.straight_drive_start_time = None
                self.get_logger().info(f"🎉 [도킹 완료] 최종 {self.straight_drive_duration:.1f}초간 무조건 직진 밀착 완료 -> 도킹 안착 성공!")

        # 🔄 [1] 후진 리커버리 주행 모드 (벽 앞 안전 공간 확보)
        elif self.is_reversing:
            elapsed = time.time() - self.reverse_start_time
            if elapsed < self.reverse_duration:
                self.state = "BACKING_UP"
                vx = float(self.reverse_speed)
                # 후진하면서 마커를 보고 있다면 차체를 벽면과 완벽 수직(Yaw=0)으로 적극 교정!
                if detected is not None:
                    raw_wz = float(np.clip(-0.80 * yaw_err, -0.22, 0.22))
                else:
                    raw_wz = 0.0
                delta_wz = float(np.clip(raw_wz - self.smoothed_wz, -0.06, 0.06))
                self.smoothed_wz += delta_wz
                wz = self.smoothed_wz
            else:
                self.is_reversing = False
                self.reverse_start_time = None
                self.is_realigning = True
                vx, wz = 0.0, 0.0
                self.smoothed_wz = 0.0
                self.get_logger().info("🔄 [후진 완료] 0.5m+ 안전 공간 확보 완료 -> 제자리에서 벽면과 완전 평행 정렬 개시!")

        # 🔄 [2] 후진 후 제자리 평행 회전 정렬 모드
        elif self.is_realigning:
            self.state = "REALIGNING"
            vx = 0.0
            if detected is not None:
                # 완벽하게 평행(약 3.8도 이내)해질 때까지 제자리 정밀 회전
                if abs(yaw_err) <= 0.065:
                    self.is_realigning = False
                    wz = 0.0
                    self.smoothed_wz = 0.0
                    self.get_logger().info(f"✨ [평행 정렬 완료] 벽면과 완전 평행 달성 (Yaw={math.degrees(yaw_err):+.1f}°) -> 직진 정밀 도킹 재개!")
                else:
                    raw_wz = -1.00 * yaw_err
                    # 4WD 스키드 제자리 회전 정지 마찰 극복 (최소 0.14 rad/s)
                    min_turn = 0.14
                    if abs(raw_wz) < min_turn:
                        raw_wz = math.copysign(min_turn, raw_wz)
                    wz = float(np.clip(raw_wz, -0.25, 0.25))
                    self.smoothed_wz = wz
            else:
                self.is_realigning = False
                wz = 0.0
                self.smoothed_wz = 0.0

        # 🎯 마커 감지 상태 (중앙 조향 및 정렬 판별)
        elif detected is not None:
            is_dual = detected.get("is_dual", False)
            rot_mat_cur = detected.get("rot_mat", None)
            lat_err = float(detected.get("lat_err", 0.0))

            # 🎯 완벽 평행 밀착 판별 (실물 카메라 노이즈 고려: 각도 약 4.6도 이내, 횡오차 4.5cm 이내)
            is_parallel = (abs(yaw_err) <= 0.080) and (abs(lat_err) <= 0.045) and (abs(bearing) <= 0.085)
            # 허용 도킹 진입 (약 5.7도 이내, 횡오차 5.5cm 이내: 1회 이상 후진 후 신속 도킹)
            is_good_enough = (abs(yaw_err) <= 0.100) and (abs(lat_err) <= 0.055)

            # -------------------------------------------------------------
            # Case 1: [최종 도킹 진입 조건 (완벽 평행 밀착)]
            # 14cm 태그 화각 이탈 전(0.42m 내외)에 평행 정렬 확인 시 조기 직진 밀착 실행!
            # -------------------------------------------------------------
            can_straight_dock = (is_parallel and (
                curr_dist <= (self.straight_dock_dist + 0.05) or dist_err <= 0.12
            )) or (
                self.backup_count >= 1 and is_good_enough and curr_dist <= (self.straight_dock_dist + 0.06)
            )

            if can_straight_dock:
                self.straight_drive_start_time = time.time()
                self.state = "STRAIGHT_IN"
                wz = 0.0
                self.smoothed_wz = 0.0
                vx = float(self.straight_drive_speed)
                self.bezier_pts = []
                self.lookahead_pt = None
                self.get_logger().info(
                    f"🚀 [최종 도킹 진입!] 근접 거리({curr_dist:.2f}m)에서 벽면과 완전 평행 확인(Yaw={math.degrees(yaw_err):+.1f}°, LatErr={lat_err*100:+.1f}cm) -> "
                    f"지금부터 {self.straight_drive_duration:.1f}초간 직진 평행 밀착 주행 (Vx={vx:+.2f}m/s, Wz=0.0) 시작!"
                )

            # -------------------------------------------------------------
            # Case 2: [후진 리커버리 트리거: 벽 코앞인데 각도가 심하게 틀어져 있을 때]
            # 도킹대 코앞(0.28m 이하)에서 벽면과 각도(>5.5°) 또는 횡오차(>5cm)가 심할 때만 후진
            # -------------------------------------------------------------
            elif curr_dist <= 0.28 and (abs(yaw_err) > 0.095 or abs(lat_err) > 0.050) and self.backup_count < self.max_backup_count:
                self.is_reversing = True
                self.reverse_start_time = time.time()
                self.backup_count += 1
                self.state = "BACKING_UP"
                vx = float(self.reverse_speed)
                wz = 0.0
                self.smoothed_wz = 0.0
                self.bezier_pts = []
                self.lookahead_pt = None
                self.get_logger().info(
                    f"🔄 [후진 리커버리 발동] 벽 앞({curr_dist:.2f}m)에서 심한 오차(Yaw={math.degrees(yaw_err):+.1f}°, LatErr={lat_err*100:+.1f}cm) 감지 -> "
                    f"후진 재정렬을 수행합니다! (시도 {self.backup_count}/{self.max_backup_count})"
                )

            # -------------------------------------------------------------
            # Case 3: [도킹 경로 주행 및 연속 정렬 (ALIGNING / APPROACHING)]
            # -------------------------------------------------------------
            else:
                self.docked_consecutive_count = 0
                if abs(lat_err) <= 0.035 and abs(yaw_err) <= 0.045:
                    self.state = "APPROACHING"
                else:
                    self.state = "ALIGNING"

                # 🌟 [거리 적응형 Stanley + 거리 연동 평행화(Distance-based Heading Flattening) 제어]
                lookahead_d = max(0.40, min(1.00, curr_dist * 0.90))
                dist_flatten = float(np.clip((curr_dist - 0.22) / 0.28, 0.0, 1.0))
                # 횡오차 게인 1.60 (오버슈트 억제), 최대 각도 35도 제한
                raw_psi = -math.atan2(1.60 * lat_err, lookahead_d)
                psi_des = float(np.clip(raw_psi, -math.radians(35), math.radians(35))) * dist_flatten
                heading_err = float(yaw_err - psi_des)

                # 🌟 [PD 댐핑 제어: P게인 하향(0.85), D게인 상향(0.12)으로 좌우 요동(Hunting) 완벽 억제]
                now_t = time.time()
                dt = max(0.01, min(0.10, now_t - self.prev_control_time))
                self.prev_control_time = now_t
                d_heading = (heading_err - self.prev_heading_err) / dt
                d_heading = float(np.clip(d_heading, -0.8, 0.8))
                self.prev_heading_err = heading_err

                raw_wz = -0.85 * heading_err - 0.12 * d_heading

                # 🌟 [불감대(Deadband): 중심선 정렬 시 미세 조향 요동 방지]
                if abs(heading_err) < math.radians(1.8) and abs(lat_err) < 0.020:
                    raw_wz = 0.0
                else:
                    # 4WD 스키드 조향 타이어 바닥 정지 마찰 극복 (Tire Scrub Booster)
                    # 각속도가 너무 작으면 바퀴가 돌지 않고 직진하므로 최소 회전 속도(0.11 rad/s) 보장
                    min_skid_wz = 0.11
                    if abs(raw_wz) < min_skid_wz:
                        raw_wz = math.copysign(min_skid_wz, raw_wz)

                # 🛡️ [부드러운 카메라 화각 보호(Soft FOV Guard): 마커가 화면 밖으로 이탈 방지]
                if bearing > 0.22 and raw_wz > 0:
                    scale = float(np.clip((0.32 - bearing) / 0.10, 0.0, 1.0))
                    raw_wz *= scale
                elif bearing < -0.22 and raw_wz < 0:
                    scale = float(np.clip((0.32 - abs(bearing)) / 0.10, 0.0, 1.0))
                    raw_wz *= scale

                # 🕊️ [거리 연동 각속도 제한 & 부드러운 Slew-Rate Limiter (부드러운 조향 가속)]
                max_wz_cur = float(np.clip(0.35 * (curr_dist / 0.70), 0.13, 0.35))
                max_dwz = 0.045
                delta_wz = float(np.clip(raw_wz - self.smoothed_wz, -max_dwz, max_dwz))
                self.smoothed_wz += delta_wz
                wz = float(np.clip(self.smoothed_wz, -max_wz_cur, max_wz_cur))

                # 🎯 [횡오차 및 헤딩 수렴 우선 전진 감속: 중심선에 진입할 때까지 벽 쪽 돌진을 제한]
                min_v = max(0.06, float(self.min_linear_speed))
                speed_scale_mid = 1.0 / (1.0 + 5.0 * (abs(lat_err) ** 1.1))
                speed_scale_yaw = 1.0 / (1.0 + 6.0 * abs(heading_err))
                dist_scale = float(np.clip(dist_err / 0.48, 0.35, 1.0))
                raw_vx = self.max_linear_speed * speed_scale_mid * speed_scale_yaw * dist_scale
                vx = float(np.clip(raw_vx, min_v, self.max_linear_speed))

                if abs(wz) > 0.02:
                    self.last_turn_dir = "RIGHT" if wz < 0 else "LEFT"

        else:
            # 태그 미감지 상태 (오도메트리로 실시간 로봇 머리 방향 연속 추적)
            if self.odom_pose is not None and self.odom_yaw_offset is not None:
                self.station_heading = self.odom_pose[2] + self.odom_yaw_offset

            if (time.time() - self.last_detection_time) > 1.0:
                self.tracked_single_role = None

            # 🚀 [초근접 화각 이탈 대응: 14cm 태그 잘림 시 안정적 직진 밀착!]
            is_was_centered = abs(self.last_seen_x) <= 0.060 or (hasattr(self, 'filt_lat_err') and abs(self.filt_lat_err) <= 0.060)
            is_was_parallel = (hasattr(self, 'filt_yaw') and abs(self.filt_yaw) <= 0.090) or (abs(self.station_heading) <= 0.090)

            if self.last_seen_dist <= (self.straight_dock_dist + 0.10) and is_was_centered and is_was_parallel and self.straight_drive_start_time is None and not self.is_reversing:
                self.straight_drive_start_time = time.time()
                self.state = "STRAIGHT_IN"
                wz = 0.0
                self.smoothed_wz = 0.0
                vx = float(self.straight_drive_speed)
                self.get_logger().info(
                    f"🚀 [초근접 화각 이탈 도킹] 도킹 직전 거리({self.last_seen_dist:.2f}m)에서 태그 화각 이탈 -> "
                    f"{self.straight_drive_duration:.1f}초간 무조건 직진 밀착 푸시 개시!"
                )
            elif self.last_seen_dist <= 0.30 and (not is_was_centered or not is_was_parallel) and self.backup_count < self.max_backup_count and not self.is_reversing:
                # 벽 코앞에서 크게 틀어진 채 태그가 화각을 벗어났다면, 안전하게 후진 리커버리!
                self.is_reversing = True
                self.reverse_start_time = time.time()
                self.backup_count += 1
                self.state = "BACKING_UP"
                vx = float(self.reverse_speed)
                wz = 0.0
                self.smoothed_wz = 0.0
                self.get_logger().info(
                    f"🔄 [초근접 비정렬 태그 소실] 벽 앞({self.last_seen_dist:.2f}m)에서 태그 이탈 -> "
                    f"충돌 및 비스듬 밀착 방지를 위해 후진 리커버리를 시작합니다! (시도 {self.backup_count}/{self.max_backup_count})"
                )
            elif (time.time() - self.last_detection_time) < 0.60:
                self.state = "ALIGNING"
                vx = self.last_cmd_vx * 0.50
                wz = 0.0
            elif self.enable_search:
                # 벽 코앞(0.40m 이하)에서는 제자리 고속 회전 금지 (벽 충돌 방지 및 정지 대기)
                if self.last_seen_dist < 0.40:
                    self.state = "SEARCHING"
                    vx = 0.0
                    wz = 0.0
                else:
                    self.state = "SEARCHING"
                    vx = 0.0
                    if (time.time() - self.start_time) < 0.3:
                        wz = 0.0
                    elif self.last_turn_dir == "RIGHT":
                        wz = self.search_angular_speed
                    elif self.last_turn_dir == "LEFT":
                        wz = -self.search_angular_speed
                    elif abs(self.last_seen_x) > 0.02:
                        wz = -self.search_angular_speed if self.last_seen_x > 0 else self.search_angular_speed
                    else:
                        wz = self.search_angular_speed
            else:
                self.state = "IDLE"
                vx, wz = 0.0, 0.0

        self.last_cmd_vx = vx
        self.last_cmd_wz = wz

        # 속도 명령 발행
        self.publish_cmd_vel(vx, wz)

        # JSON 상태 피드백 발행
        status_info = {
            "state": self.state,
            "tag_detected": detected is not None,
            "tag_id": detected["id"] if detected else None,
            "tag_mode": detected.get("tag_mode", "NONE") if detected else "NONE",
            "dist_m": round(curr_dist, 3) if curr_dist < 900.0 else None,
            "dist_err_m": round(dist_err, 3) if curr_dist < 900.0 else None,
            "lat_err_m": round(detected.get("lat_err", 0.0), 3) if detected else None,
            "yaw_deg": round(math.degrees(yaw_err), 1),
            "handover_active": self.handover_active,
            "cmd_vx": round(vx, 3),
            "cmd_wz": round(wz, 3),
            "timestamp": time.time()
        }
        status_msg = String()
        status_msg.data = json.dumps(status_info, ensure_ascii=False)
        self.pub_status.publish(status_msg)

        # 터미널 주기적 상태 로깅 (0.5초마다 1회)
        if not hasattr(self, '_loop_counter'):
            self._loop_counter = 0
        self._loop_counter += 1
        if self._loop_counter % 10 == 0:
            if detected is not None:
                self.get_logger().info(
                    f"[{self.state}] Tag:{detected.get('tag_mode')} | Dist:{detected['z']*100:.1f}cm (Err:{dist_err*100:+.1f}cm) | "
                    f"LatErr:{detected.get('lat_err', 0.0)*100:+.1f}cm | Yaw:{math.degrees(yaw_err):+.1f}° | Cmd: Vx={vx:+.2f}m/s, Wz={wz:+.2f}rad/s"
                )
            else:
                self.get_logger().info(
                    f"[{self.state}] Tag: None | Cmd: Vx={vx:+.2f}m/s, Wz={wz:+.2f}rad/s"
                )

        # 비전 HUD 오버레이 렌더링
        dbg = self.render_overlay(frame, detected, vx, wz, curr_dist, dist_err)
        with self.frame_lock:
            self.debug_frame = dbg

        # OpenCV 실시간 카메라 시점 창 표시 (show_gui=True 일 때 화면에 윈도우 팝업)
        if self.show_gui:
            try:
                cv2.imshow("AMR AprilTag Precision Docking", dbg)
                cv2.waitKey(1)
            except Exception:
                pass

    # ---------------------------------------------------------------------
    # HUD 오버레이 렌더링
    # ---------------------------------------------------------------------
    def render_overlay(self, frame, detected, vx, wz, curr_dist, dist_err):
        dbg = frame.copy()
        h, w = dbg.shape[:2]
        cx, cy = w // 2, h // 2

        cv2.line(dbg, (cx - 20, cy), (cx - 6, cy), (120, 120, 120), 1)
        cv2.line(dbg, (cx + 6, cy), (cx + 20, cy), (120, 120, 120), 1)
        cv2.line(dbg, (cx, cy - 20), (cx, cy - 6), (120, 120, 120), 1)
        cv2.line(dbg, (cx, cy + 6), (cx, cy + 20), (120, 120, 120), 1)

        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]

        # 1. 녹색 베지어 궤적 렌더링
        if len(self.bezier_pts) > 1:
            img_curve = []
            y_mid_val = detected["y"] if (detected and "y" in detected) else 0.0
            for i, pt in enumerate(self.bezier_pts):
                xi, zi = pt[0], pt[1]
                if zi > 0.04:
                    yi = (i / float(len(self.bezier_pts) - 1)) * y_mid_val
                    ui = int(cx + (xi * fx / zi))
                    vi = int(cy + (yi * fy / zi))
                    if 0 <= ui < w and 0 <= vi < h:
                        img_curve.append((ui, vi))

            if len(img_curve) >= 2:
                curve_arr = np.array(img_curve, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(dbg, [curve_arr], False, (0, 240, 0), 3, cv2.LINE_AA)

            if self.lookahead_pt is not None:
                lx, lz = self.lookahead_pt[0], self.lookahead_pt[1]
                if lz > 0.04:
                    lu = int(cx + (lx * fx / lz))
                    lv = int(cy + (0.5 * y_mid_val * fy / lz))
                    if 0 <= lu < w and 0 <= lv < h:
                        cv2.circle(dbg, (lu, lv), 6, (0, 255, 255), -1)

            if len(self.bezier_pts) > 0:
                gx, gz = self.bezier_pts[-1][0], self.bezier_pts[-1][1]
                if gz > 0.04:
                    gu = int(cx + (gx * fx / gz))
                    gv = int(cy + (y_mid_val * fy / gz))
                    if 0 <= gu < w and 0 <= gv < h:
                        cv2.circle(dbg, (gu, gv), 8, (0, 0, 255), -1)
                        cv2.circle(dbg, (gu, gv), 12, (0, 0, 255), 2)
                        cv2.putText(dbg, "DOCK", (gu - 22, gv - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 0, 255), 1)

        # 2. 태그 렌더링
        if detected is not None:
            candidates = detected.get("candidates", [])
            for cand in candidates:
                pts = cand["pts"].astype(np.int32).reshape((-1, 1, 2))
                cv2.polylines(dbg, [pts], True, (0, 255, 255), 2)

            if detected.get("left_c") is not None:
                c = detected["left_c"]
                tid_str = f"ID:{c['tid']}" if 'tid' in c else "L"
                cv2.putText(dbg, f"L ({tid_str})", (int(c["u"]) - 35, int(c["v"]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 200, 0), 1)
            if detected.get("right_c") is not None:
                c = detected["right_c"]
                tid_str = f"ID:{c['tid']}" if 'tid' in c else "R"
                cv2.putText(dbg, f"R ({tid_str})", (int(c["u"]) - 35, int(c["v"]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 200, 0), 1)

            if len(candidates) >= 2:
                u_pts = [(int(c["u"]), int(c["v"])) for c in candidates]
                for i in range(len(u_pts) - 1):
                    cv2.line(dbg, u_pts[i], u_pts[i+1], (255, 255, 0), 2, cv2.LINE_AA)

            if getattr(self, 'show_axes', True) and hasattr(cv2, 'drawFrameAxes') and detected.get("rvec") is not None:
                try:
                    cv2.drawFrameAxes(dbg, self.camera_matrix, self.dist_coeffs,
                                      detected["rvec"], detected["tvec"], self.tag_size * 0.75, 2)
                except Exception:
                    pass

        # 2-1. 정 중앙 직진 가이드 라인 (Corridor)
        corridor_w = 30
        if self.state == "STRAIGHT_IN":
            rem_s = 0.0
            if hasattr(self, 'straight_drive_start_time') and self.straight_drive_start_time is not None:
                rem_s = max(0.0, self.straight_drive_duration - (time.time() - self.straight_drive_start_time))
            cv2.line(dbg, (cx - corridor_w, 60), (cx - corridor_w, h - 20), (0, 255, 0), 2)
            cv2.line(dbg, (cx + corridor_w, 60), (cx + corridor_w, h - 20), (0, 255, 0), 2)
            cv2.line(dbg, (cx - 15, cy), (cx + 15, cy), (0, 255, 0), 2)
            cv2.putText(dbg, f">> STRAIGHT DOCKING ({rem_s:.1f}s) <<", (cx - 150, h - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 0), 2)
        elif self.state == "APPROACHING":
            cv2.line(dbg, (cx - corridor_w, 60), (cx - corridor_w, h - 20), (0, 255, 0), 2)
            cv2.line(dbg, (cx + corridor_w, 60), (cx + corridor_w, h - 20), (0, 255, 0), 2)
            cv2.line(dbg, (cx - 15, cy), (cx + 15, cy), (0, 255, 0), 2)
            cv2.putText(dbg, ">> CENTER LOCKED (STRAIGHT) <<", (cx - 150, h - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 0), 2)
        elif self.state == "FINAL_ALIGN":
            cv2.line(dbg, (cx - corridor_w, 60), (cx - corridor_w, h - 20), (0, 200, 255), 2)
            cv2.line(dbg, (cx + corridor_w, 60), (cx + corridor_w, h - 20), (0, 200, 255), 2)
            cv2.putText(dbg, ">> FINAL HEADING ALIGN <<", (cx - 130, h - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 200, 255), 2)
        elif self.state == "ALIGNING":
            cv2.line(dbg, (cx - corridor_w, 60), (cx - corridor_w, h - 20), (0, 200, 255), 1)
            cv2.line(dbg, (cx + corridor_w, 60), (cx + corridor_w, h - 20), (0, 200, 255), 1)
            cv2.putText(dbg, "AIMING CENTER...", (cx - 70, h - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 200, 255), 2)

        # -----------------------------------------------------------------
        # 2-2. 실시간 탑다운 주행 궤적 미니맵 (BEV Mini-map & Trajectory Line)
        # -----------------------------------------------------------------
        map_w, map_h = 166, 166
        mx0 = w - map_w - 10
        my0 = 62
        mx1 = mx0 + map_w
        my1 = my0 + map_h

        if 0 <= mx0 < mx1 <= w and 0 <= my0 < my1 <= h:
            # 반투명 배경 패널
            roi = dbg[my0:my1, mx0:mx1]
            panel = np.full_like(roi, (18, 24, 32))
            cv2.addWeighted(roi, 0.20, panel, 0.80, 0, roi)
            dbg[my0:my1, mx0:mx1] = roi

            # 패널 외곽선 및 헤더
            cv2.rectangle(dbg, (mx0, my0), (mx1, my1), (65, 85, 110), 1)
            cv2.rectangle(dbg, (mx0, my0), (mx1, my0 + 20), (28, 40, 54), -1)
            cv2.putText(dbg, "TRAJECTORY (BEV)", (mx0 + 8, my0 + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 220, 255), 1, cv2.LINE_AA)

            dock_u = mx0 + map_w // 2
            dock_v = my0 + 28

            # 미니맵 스케일 (X: ±0.90m -> 폭 1.80m, Z: 0~2.00m)
            max_range_x = 1.80
            max_range_z = 2.00
            scale_x = (map_w - 20) / max_range_x
            scale_z = (map_h - 42) / max_range_z

            def st_to_bev(x_st, z_st):
                u = int(dock_u + x_st * scale_x)
                v = int(dock_v + z_st * scale_z)
                u = int(np.clip(u, mx0 + 4, mx1 - 4))
                v = int(np.clip(v, my0 + 22, my1 - 4))
                return (u, v)

            # 도킹 벽면 베이스라인 & 듀얼 마커
            cv2.line(dbg, (dock_u - 40, dock_v), (dock_u + 40, dock_v), (160, 160, 160), 2)
            half_sp_px = int((self.tag_spacing / 2.0) * scale_x)
            cv2.rectangle(dbg, (dock_u - half_sp_px - 4, dock_v - 4),
                               (dock_u - half_sp_px + 4, dock_v + 1), (0, 220, 255), -1)
            cv2.rectangle(dbg, (dock_u + half_sp_px - 4, dock_v - 4),
                               (dock_u + half_sp_px + 4, dock_v + 1), (0, 220, 255), -1)

            # 중앙 도킹 기준선 (녹색 타겟 라인)
            cv2.line(dbg, (dock_u, dock_v), (dock_u, my1 - 6), (40, 90, 50), 1)

            # 목표 정지 지점 (타겟)
            tgt_v = int(dock_v + self.target_dist * scale_z)
            cv2.circle(dbg, (dock_u, tgt_v), 3, (0, 255, 0), -1)
            cv2.circle(dbg, (dock_u, tgt_v), 6, (0, 255, 0), 1)

            # 현재 로봇 위치 및 진행 방향 화살표 (참 3D 강체 역기구학 좌표 기준)
            curr_x_pos = float(detected.get("station_x", 0.0)) if detected else (self.history_path[-1][0] if self.history_path else 0.0)
            curr_z_pos = float(detected.get("station_z", 1.0)) if detected else (self.history_path[-1][1] if self.history_path else 0.8)
            curr_yaw_val = detected.get("yaw", 0.0) if detected else 0.0

            # 🌟 [로봇이 주행해 온 실제 궤적 선 (Cyan Polyline) 및 현재 위치 실시간 연결]
            pts_bev = [st_to_bev(p[0], p[1]) for p in self.history_path]
            if detected is not None:
                cur_pt = st_to_bev(curr_x_pos, curr_z_pos)
                if len(pts_bev) == 0 or pts_bev[-1] != cur_pt:
                    pts_bev.append(cur_pt)

            if len(pts_bev) >= 2:
                arr_bev = np.array(pts_bev, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(dbg, [arr_bev], False, (0, 255, 255), 2, cv2.LINE_AA)

                # 주행 시작 지점 (Magenta)
                cv2.circle(dbg, pts_bev[0], 4, (255, 100, 255), -1)

            r_u, r_v = st_to_bev(curr_x_pos, curr_z_pos)
            cv2.circle(dbg, (r_u, r_v), 5, (0, 0, 255), -1)
            cv2.circle(dbg, (r_u, r_v), 6, (255, 255, 255), 1)

            # 로봇 머리 방향 화살표 (도킹 스테이션 기준 실제 물리 진행 방향)
            hdg = float(self.station_heading)
            arr_len = 14
            # 스테이션 기준: 우측이 +X(u 방향 증가), 전방(벽면)이 +Z(v 방향 감소)
            arr_u = int(r_u + arr_len * math.sin(hdg))
            arr_v = int(r_v - arr_len * math.cos(hdg))
            cv2.arrowedLine(dbg, (r_u, r_v), (arr_u, arr_v), (0, 255, 120), 2, tipLength=0.35)

            # 하단 누적 주행 거리 표시
            cv2.putText(dbg, f"Path: {self.total_traveled_distance:.2f}m", (mx0 + 8, my1 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (220, 220, 220), 1, cv2.LINE_AA)

        # 3. 상단 대시보드 HUD
        state_colors = {
            "IDLE": (120, 120, 120),
            "SEARCHING": (0, 165, 255),
            "ALIGNING": (0, 215, 255),
            "APPROACHING": (255, 140, 0),
            "STRAIGHT_IN": (0, 220, 0),
            "BACKING_UP": (0, 140, 255),
            "REALIGNING": (255, 180, 0),
            "RETRY_BACKUP": (0, 165, 255),
            "FINAL_TUNE": (255, 200, 0),
            "DOCKED": (0, 220, 0),
            "LOST_SAFETY": (0, 0, 220)
        }
        color = state_colors.get(self.state, (100, 100, 100))

        overlay = dbg.copy()
        cv2.rectangle(overlay, (0, 0), (w, 55), (15, 15, 15), -1)
        cv2.addWeighted(overlay, 0.8, dbg, 0.2, 0, dbg)

        cv2.rectangle(dbg, (10, 8), (145, 48), color, -1)
        cv2.putText(dbg, self.state, (16, 33), cv2.FONT_HERSHEY_DUPLEX, 0.48, (255, 255, 255), 1)

        sensor_mode = " [LIDAR]" if self.use_lidar else " [PURE-VISION]"
        mode_str = f" [{detected.get('tag_mode', '')}]" if detected else ""
        lat_str = f" X:{detected.get('x', 0.0)*100:+.1f}cm" if detected else ""
        deg_str = f" Ang:{math.degrees(detected.get('bearing', 0.0)):+.1f}deg" if detected else ""
        info1 = f"Target: Dist={self.target_dist*100:.1f}cm{lat_str}{deg_str}{sensor_mode}{mode_str}"
        info2 = f"Cmd: Vx={vx:+.2f}m/s Wz={wz:+.2f}rad/s | Dist: {curr_dist*100:.1f}cm | Path: {self.total_traveled_distance:.2f}m" if curr_dist < 900.0 else f"Cmd: Vx={vx:+.2f}m/s Wz={wz:+.2f}rad/s | Path: {self.total_traveled_distance:.2f}m"
        cv2.putText(dbg, info1, (155, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 220, 220), 1)
        cv2.putText(dbg, info2, (155, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 200), 1)
        return dbg

    # ---------------------------------------------------------------------
    # 실시간 웹 모니터링 서버
    # ---------------------------------------------------------------------
    def start_web_server(self):
        node_ref = self

        class WebHandler(BaseHTTPRequestHandler):
            def log_message(self, format, *args):
                pass

            def do_GET(self):
                if self.path == '/' or self.path == '/index.html':
                    html = f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>AMR AprilTag Precision Docking Monitor</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {{ margin:0; background:#0f172a; color:#f8fafc; font-family:'Segoe UI',sans-serif; text-align:center; padding:15px; }}
        h2 {{ margin:5px 0 15px 0; color:#38bdf8; }}
        .container {{ max-width:700px; margin:auto; background:#1e293b; padding:15px; border-radius:12px; box-shadow:0 8px 24px rgba(0,0,0,0.5); }}
        .stream-box {{ width:100%; border-radius:8px; overflow:hidden; border:2px solid #334155; margin-bottom:15px; background:#000; }}
        img {{ width:100%; display:block; }}
        .btn {{ background:#0284c7; color:#fff; border:none; padding:10px 20px; font-size:15px; font-weight:bold; border-radius:6px; cursor:pointer; margin:5px; transition:0.2s; }}
        .btn:hover {{ background:#0369a1; }}
        .btn-stop {{ background:#ef4444; }}
        .btn-stop:hover {{ background:#dc2626; }}
        .btn-axes {{ background:#6366f1; }}
        .btn-axes:hover {{ background:#4f46e5; }}
    </style>
</head>
<body>
    <div class="container">
        <h2>🏷️ AMR AprilTag Precision Docking Monitor</h2>
        <div class="stream-box">
            <img src="/stream.mjpg" alt="Live Camera Stream">
        </div>
        <div>
            <button class="btn" onclick="startDocking()">▶️ Start Docking</button>
            <button class="btn btn-stop" onclick="stopDocking()">⏹️ Stop</button>
            <button class="btn btn-axes" onclick="toggleAxes()">📐 3D 좌표축 On/Off</button>
        </div>
        <div id="status-badge" style="margin-top:12px; font-weight:bold; font-size:16px; color:#38bdf8;">Status: {node_ref.state}</div>
        <p>Target Dist: <b>{node_ref.target_dist * 100:.1f} cm</b> | Spacing: <b>{node_ref.tag_spacing * 100:.1f} cm</b></p>
        <p style="font-size:13px; color:#94a3b8; margin-top:8px;">🗺️ <b>주행 궤적 안내</b>: 영상 우측 상단 <b>BEV 미니맵</b>의 <span style="color:#38bdf8; font-weight:bold;">하늘색 실선</span>으로 실시간 이동 경로가 표시되며, ROS 2 토픽 <code>/apriltag/docking_path</code> (RViz2)로도 발행됩니다.</p>
    </div>
    <script>
        function startDocking() {{
            document.getElementById('status-badge').innerText = 'Starting...';
            fetch('/api/start').then(function(r) {{ return r.json(); }}).then(function(d) {{
                document.getElementById('status-badge').innerText = 'Status: ACTIVE (SEARCHING)';
                document.getElementById('status-badge').style.color = '#38bdf8';
            }});
        }}
        function stopDocking() {{
            document.getElementById('status-badge').innerText = 'Stopping...';
            fetch('/api/stop').then(function(r) {{ return r.json(); }}).then(function(d) {{
                document.getElementById('status-badge').innerText = 'Status: STOPPED (IDLE)';
                document.getElementById('status-badge').style.color = '#ef4444';
            }});
        }}
        function toggleAxes() {{
            fetch('/api/toggle_axes').then(function(r) {{ return r.json(); }});
        }}
    </script>
</body>
</html>"""
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.end_headers()
                    self.wfile.write(html.encode('utf-8'))

                elif self.path == '/api/toggle_axes':
                    node_ref.show_axes = not getattr(node_ref, 'show_axes', True)
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    self.wfile.write(json.dumps({"show_axes": node_ref.show_axes}).encode('utf-8'))

                elif self.path == '/api/start':
                    node_ref.is_active = True
                    node_ref.state = "SEARCHING"
                    node_ref.docked_consecutive_count = 0
                    node_ref.handover_active = False
                    node_ref.history_path = []
                    node_ref.last_path_pt = None
                    node_ref.total_traveled_distance = 0.0
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    self.wfile.write(b'{"status":"started"}')

                elif self.path == '/api/stop':
                    node_ref.is_active = False
                    node_ref.state = "IDLE"
                    for _ in range(5):
                        node_ref.publish_cmd_vel(0.0, 0.0)
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    self.wfile.write(b'{"status":"stopped"}')

                elif self.path == '/stream.mjpg':
                    self.send_response(200)
                    self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                    self.end_headers()
                    try:
                        while rclpy.ok():
                            frame = None
                            with node_ref.frame_lock:
                                if node_ref.debug_frame is not None:
                                    frame = node_ref.debug_frame.copy()
                                elif node_ref.latest_frame is not None:
                                    frame = node_ref.latest_frame.copy()

                            if frame is not None:
                                ret, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
                                if ret:
                                    self.wfile.write(b'--frame\r\n')
                                    self.send_header('Content-Type', 'image/jpeg')
                                    self.send_header('Content-Length', str(len(jpeg)))
                                    self.end_headers()
                                    self.wfile.write(jpeg.tobytes())
                                    self.wfile.write(b'\r\n')
                            time.sleep(0.04)
                    except Exception:
                        pass
                else:
                    self.send_response(404)
                    self.end_headers()

        def server_worker():
            try:
                server = ThreadingHTTPServer(('0.0.0.0', self.web_port), WebHandler)
                self.get_logger().info(f"🌐 [Web Monitor] http://0.0.0.0:{self.web_port} 에서 실시간 도킹 뷰어 실행 중")
                server.serve_forever()
            except Exception as e:
                self.get_logger().warn(f"웹 서버 시작 오류 (포트 {self.web_port}): {e}")

        server_thread = threading.Thread(target=server_worker, daemon=True)
        server_thread.start()


def main(args=None):
    rclpy.init(args=args)
    node = ApriltagDockingNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.publish_cmd_vel(0.0, 0.0)
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
