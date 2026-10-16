#!/usr/bin/env python3
"""
AMR 2단계 자율주행 & 조기 태그 감지 비전 정밀 도킹 통합 실행기 (navigate_and_dock.py)
-----------------------------------------------------------------------------
1. Nav2 자율주행으로 목표(home 등)를 향해 출발
2. 이동 도중 카메라가 AprilTag를 감지하면 즉시 Nav2를 중단하고 비전 정밀 도킹으로 조기 전환
3. 마커를 보면서 S-Curve 정밀 궤적으로 전방 35cm에 칼각(Yaw=0.0°)으로 안착
"""

import os
import sys
import time
import argparse
import threading
import cv2
import numpy as np
import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from navigate_to_coords import RobotNavigatorClient
from run_xml_waypoints import load_waypoints, find_waypoint
from apriltag_docking import AprilTagDocker, TAG_DICT_MAP


class TagWatcher:
    """Nav2 주행 중 실시간으로 AprilTag 출현을 감시하는 백그라운드 워처"""
    def __init__(self, camera_idx="0", tag_family="tag36h11", target_id=-1, tag_size=0.10, flip_180=True,
                 use_ros_cam=False, topic="/camera/image_raw"):
        self.camera_idx = int(camera_idx) if str(camera_idx).isdigit() else camera_idx
        self.tag_family = tag_family
        self.target_id = target_id
        self.tag_size = tag_size
        self.flip_180 = flip_180
        self.use_ros_cam = use_ros_cam
        self.topic = topic
        self.tag_found = False
        self.detected_info = None
        self.running = False
        self.consecutive_hits = 0
        self.cap = None
        self.ros_node = None
        self.executor = None

        # Aruco / AprilTag 검출기 초기화
        dict_id = TAG_DICT_MAP.get(self.tag_family.lower(), cv2.aruco.DICT_APRILTAG_36h11)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        elif hasattr(cv2.aruco, 'DetectorParameters'):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = None

        if self.aruco_params is not None:
            if hasattr(cv2.aruco, 'CORNER_REFINE_SUBPIX'):
                self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.aruco_params.adaptiveThreshWinSizeMin = 3
            self.aruco_params.adaptiveThreshWinSizeMax = 23
            self.aruco_params.minMarkerPerimeterRate = 0.02

        self.detector = None
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

    def _process_frame(self, frame):
        if frame is None:
            return

        if self.flip_180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        elif hasattr(cv2.aruco, 'detectMarkers'):
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        else:
            corners, ids = None, None

        hit = False
        if ids is not None and len(ids) > 0:
            flat_ids = ids.flatten()
            for idx, tid in enumerate(flat_ids):
                if self.target_id < 0 or tid == self.target_id:
                    hit = True
                    self.detected_info = {"id": int(tid)}
                    break

        if hit:
            self.consecutive_hits += 1
            if self.consecutive_hits >= 2:
                self.tag_found = True
        else:
            self.consecutive_hits = max(0, self.consecutive_hits - 1)

    def start(self):
        self.running = True
        self.tag_found = False
        self.consecutive_hits = 0

        if self.use_ros_cam:
            def ros_watch_loop():
                try:
                    from rclpy.node import Node
                    from rclpy.executors import SingleThreadedExecutor
                    from sensor_msgs.msg import Image as RosImage

                    if not rclpy.ok():
                        rclpy.init()
                    self.ros_node = Node('tag_watcher_node')

                    def img_callback(msg):
                        if not self.running:
                            return
                        try:
                            if msg.encoding in ['rgb8', 'bgr8']:
                                np_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                                frame = np_arr[:, :, ::-1] if msg.encoding == 'rgb8' else np_arr.copy()
                            elif msg.encoding == 'mono8':
                                np_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
                                frame = cv2.cvtColor(np_arr, cv2.COLOR_GRAY2BGR)
                            else:
                                frame = None

                            if frame is not None:
                                self._process_frame(frame)
                        except Exception:
                            pass

                    self.ros_sub = self.ros_node.create_subscription(RosImage, self.topic, img_callback, 10)
                    self.executor = SingleThreadedExecutor()
                    self.executor.add_node(self.ros_node)

                    while self.running:
                        self.executor.spin_once(timeout_sec=0.05)
                except Exception as e:
                    pass
                finally:
                    if self.ros_node:
                        try:
                            self.ros_node.destroy_node()
                        except Exception:
                            pass
                        self.ros_node = None

            self.thread = threading.Thread(target=ros_watch_loop, daemon=True)
            self.thread.start()
        else:
            def cv_watch_loop():
                try:
                    self.cap = cv2.VideoCapture(self.camera_idx)
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    self.cap.set(cv2.CAP_PROP_FPS, 30)

                    while self.running:
                        ret, frame = self.cap.read()
                        if not ret or frame is None:
                            time.sleep(0.04)
                            continue

                        self._process_frame(frame)
                        time.sleep(0.03)
                except Exception:
                    pass
                finally:
                    if self.cap:
                        self.cap.release()
                        self.cap = None

            self.thread = threading.Thread(target=cv_watch_loop, daemon=True)
            self.thread.start()

    def stop(self):
        self.running = False
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.cap:
            self.cap.release()
            self.cap = None


def run_nav_and_dock(target_query="home", target_dist=0.10, tag_id=-1, tag_size=0.10,
                     camera="0", ros_camera=False, topic="/camera/image_raw",
                     lidar_offset=0.20, web_port=8088, flip_180=True):
    # 1. 웨이포인트 로드
    waypoints = load_waypoints()
    wp = find_waypoint(waypoints, target_query)
    
    if wp is None:
        print(f"❌ 웨이포인트 '{target_query}'를 찾을 수 없습니다.")
        print(f"   등록된 웨이포인트: {list(waypoints.keys())}")
        return False

    # 타겟 태그 ID 확정 (XML에 정의된 태그 ID 또는 home의 경우 1번 태그)
    target_tag_id = tag_id
    if target_tag_id is None or target_tag_id < 0:
        if wp.get('tag_id') is not None:
            target_tag_id = wp['tag_id']
        elif wp['id'] == 'home':
            target_tag_id = 1
        else:
            target_tag_id = -1

    print("\n" + "=" * 70)
    print("🚀 [AMR 2단계 자율주행 & AprilTag 실시간 감지 정밀 도킹 미션 시작]")
    print("=" * 70)
    print(f" • 목표 지점:         {wp['name']} (ID: {wp['id']})")
    print(f" • 도킹 정지 거리:     {target_dist * 100:.1f} cm")
    print(f" • 타겟 전용 마커:     AprilTag ID #{target_tag_id if target_tag_id >= 0 else 'ANY'}")
    print(f" • 카메라 입력:       {'ROS 2 Topic (' + topic + ')' if ros_camera else 'Device ' + str(camera)}")
    print("=" * 70 + "\n")

    print(f"📍 [1단계: Nav2 자율주행 & 전용 마커(ID #{target_tag_id}) 감시]")
    print(f"   - 이동 목표: {wp['name']} (X={wp['x']:.2f}, Y={wp['y']:.2f}, Yaw={wp['yaw']:.1f}°)")
    if target_tag_id >= 0:
        print(f"   - 주행 중 오직 'AprilTag #{target_tag_id}'만 감지 시 조기 도킹으로 전환합니다. (다른 태그는 무시)\n")
    else:
        print(f"   - 도착 후 AprilTag 도킹을 수행합니다.\n")

    # 2. 실시간 전용 태그 워처 가동 (해당 웨이포인트 전용 tag_id만 감지)
    watcher = None
    check_cb = None
    if target_tag_id >= 0:
        watcher = TagWatcher(
            camera_idx=camera,
            target_id=target_tag_id,
            tag_size=tag_size,
            flip_180=flip_180,
            use_ros_cam=ros_camera,
            topic=topic
        )
        watcher.start()
        check_cb = lambda: watcher.tag_found

    # 3. Nav2 주행 실행
    nav_client = RobotNavigatorClient()
    nav_success = nav_client.go_to(
        x=wp['x'], y=wp['y'], yaw_deg=wp['yaw'],
        timeout_sec=120.0,
        check_callback=check_cb
    )

    # 태그 워처 카메라 점유 해제
    if watcher is not None:
        watcher.stop()
        time.sleep(0.6)

    if not nav_success:
        print("\n❌ [1단계 실패] Nav2 자율주행이 실패했습니다. 미션을 중단합니다.")
        return False

    print("\n" + "-" * 70)
    if watcher and watcher.tag_found:
        print(f"🎯 [조기 감지 성공] Nav2 주행 도중 목표 태그(ID #{target_tag_id})를 포착하여 즉시 도킹 모드로 전환합니다!")
    else:
        print(f"✅ [목표 도달 완료] Nav2로 {wp['name']} 전방에 도착했습니다! AprilTag(ID #{target_tag_id}) 도킹을 시작합니다.")
    print("📷 [2단계: AprilTag 비전 정밀 도킹 가동 중...]")
    print("-" * 70 + "\n")

    # 4. AprilTag 비전 도킹 실행 (S-Curve & 정밀 도킹)
    dock_args = argparse.Namespace(
        family="tag36h11",
        tag_id=target_tag_id,
        tag_size=tag_size,
        target_dist=target_dist,
        target_x=0.0,
        target_yaw_deg=0.0,
        linear_tol=0.03,
        lateral_tol=0.03,
        angular_tol_deg=3.5,
        max_speed=0.15,
        min_speed=0.03,
        max_ang_speed=0.85,
        search_ang_speed=0.65,
        kp_linear=0.45,
        kp_bearing=0.60,
        kp_yaw=0.35,
        no_search=False,
        no_flip=not flip_180,
        camera=str(camera),
        ros_camera=ros_camera,
        topic=topic,
        lidar_offset=lidar_offset,
        web_port=web_port,
        headless=True
    )

    docker = AprilTagDocker(dock_args)
    docker.run()

    print("\n" + "=" * 70)
    print(f"🎉 [미션 완수!] {wp['name']}에 안전하게 자율주행 및 정밀 도킹을 완료했습니다!")
    print("=" * 70 + "\n")
    return True


def main():
    parser = argparse.ArgumentParser(description="AMR Nav2 + AprilTag Precision Docking Integration")
    parser.add_argument("target", nargs="?", default=None, help="목표 웨이포인트 ID 또는 이름 (기본: home)")
    parser.add_argument("--waypoint", "-w", default=None, help="목표 웨이포인트 ID 또는 이름 (예: home, point_a)")
    parser.add_argument("--target-dist", type=float, default=0.10, help="도킹 목표 거리 (미터, 기본 0.10m = 10cm)")
    parser.add_argument("--tag-id", type=int, default=-1, help="타겟 AprilTag ID (-1이면 감지된 마커)")
    parser.add_argument("--tag-size", type=float, default=0.10, help="AprilTag 실측 한 변의 길이 (미터, 기본: 0.10m)")
    parser.add_argument("--camera", default="0", help="카메라 인덱스 또는 디바이스 경로 (기본: 0)")
    parser.add_argument("--ros-camera", action="store_true", help="ROS 2 이미지 토픽 구독 모드 사용 (Gazebo/ROS 카메라)")
    parser.add_argument("--topic", default="/camera/image_raw", help="ROS 2 카메라 이미지 토픽 이름 (기본: /camera/image_raw)")
    parser.add_argument("--no-flip", action="store_true", help="카메라 180도 회전(위아래 반전 보정) 끄기")
    parser.add_argument("--lidar-offset", type=float, default=0.20, help="라이다와 전방 카메라 간 전후 거리 오프셋 (미터, 기본 0.20m = 20cm)")
    parser.add_argument("--web-port", type=int, default=8088, help="웹 실시간 모니터링 포트 (기본: 8088)")

    args = parser.parse_args()
    target = args.waypoint or args.target or "home"

    run_nav_and_dock(
        target_query=target,
        target_dist=args.target_dist,
        tag_id=args.tag_id,
        tag_size=args.tag_size,
        camera=args.camera,
        ros_camera=args.ros_camera,
        topic=args.topic,
        lidar_offset=args.lidar_offset,
        web_port=args.web_port
    )


if __name__ == '__main__':
    main()
