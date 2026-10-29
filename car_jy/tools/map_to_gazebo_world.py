#!/usr/bin/env python3
"""
AMR 2D SLAM Map to Gazebo Harmonic 3D World Converter
Converts a 2D occupancy grid map (YAML + PGM) into an optimized 3D Gazebo World (.sdf)
with embedded physics, lighting, and an AprilTag docking station.
"""

import os
import sys
import yaml
import math
import cv2
import numpy as np


def generate_apriltag_texture(out_path):
    """AprilTag #1 고해상도 텍스처 이미지 자동 생성"""
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    dict_36h11 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, 'drawMarker'):
        tag_img = cv2.aruco.drawMarker(dict_36h11, 1, 400)
    elif hasattr(cv2.aruco, 'generateImageMarker'):
        tag_img = cv2.aruco.generateImageMarker(dict_36h11, 1, 400)
    else:
        tag_img = None

    if tag_img is not None:
        bordered = cv2.copyMakeBorder(tag_img, 80, 80, 80, 80, cv2.BORDER_CONSTANT, value=255)
        cv2.imwrite(out_path, bordered)
        print(f"✅ AprilTag 36h11 #1 정품 텍스처 생성 완료: {out_path}")


def convert_map_to_sdf(yaml_path, output_sdf_path, wall_height=1.0, dock_pos=(2.7, 2.0, -116.5)):
    """2D SLAM 맵을 읽어 Gazebo Harmonic 3D SDF 월드로 변환"""
    print(f"\n📂 SLAM 지도 로드 중: {yaml_path}")
    
    with open(yaml_path, 'r') as f:
        map_meta = yaml.safe_load(f)

    map_dir = os.path.dirname(yaml_path)
    image_file = map_meta.get('image', 'map.pgm')
    if not os.path.isabs(image_file):
        image_path = os.path.join(map_dir, image_file)
    else:
        image_path = image_file

    if not os.path.exists(image_path):
        raise FileNotFoundError(f"지도 이미지 파일을 찾을 수 없습니다: {image_path}")

    resolution = float(map_meta.get('resolution', 0.05))
    origin = map_meta.get('origin', [0.0, 0.0, 0.0])
    origin_x = float(origin[0])
    origin_y = float(origin[1])
    negate = int(map_meta.get('negate', 0))
    occ_thresh = float(map_meta.get('occupied_thresh', 0.65))

    # PGM / PNG 이미지 로드
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError(f"이미지 로드 실패: {image_path}")

    h_img, w_img = img.shape
    print(f" • 해상도: {resolution} m/pixel | 크기: {w_img}x{h_img} 픽셀")
    print(f" • 원점(Origin): X={origin_x:.3f}m, Y={origin_y:.3f}m")

    # 점유(Occupied) 벽면 픽셀 마스크 생성
    # ROS map 규격: 0=검은색(점유 벽면), 254=흰색(자유 공간), 205=회색(미탐색)
    if negate == 0:
        # 검은색(픽셀값 < (1.0 - occ_thresh)*255)이 벽
        wall_mask = (img < int((1.0 - occ_thresh) * 255)).astype(np.uint8) * 255
    else:
        wall_mask = (img > int(occ_thresh * 255)).astype(np.uint8) * 255

    # 1. 2D 윤곽선(Contours) 검출 및 박스 클러스터링으로 벽 픽셀을 사각형 블록으로 변환
    # 인접한 1px 라이다 벽면 점들을 부드럽게 연결 (MORPH_CLOSE)
    kernel = np.ones((2, 2), np.uint8)
    clean_mask = cv2.morphologyEx(wall_mask, cv2.MORPH_CLOSE, kernel)

    # 연결 성분(Connected Components) 또는 Contours 탐색
    contours, _ = cv2.findContours(clean_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    wall_boxes = []
    min_area = 0 # 얇은 라이다 벽면 픽셀도 모두 3D 벽 블록으로 보존

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            # 단일 점인 경우
            x, y, w, h = cv2.boundingRect(cnt)
        else:
            x, y, w, h = cv2.boundingRect(cnt)

        # 픽셀 좌표 -> 월드 3D 좌표 변환
        # ROS 맵 이미지: 행(y)이 위에서 아래로 증가, 월드 Y는 아래에서 위로 증가
        center_px_x = x + w / 2.0
        center_px_y = y + h / 2.0

        world_x = origin_x + center_px_x * resolution
        world_y = origin_y + (h_img - center_px_y) * resolution
        size_x = max(w * resolution, resolution)
        size_y = max(h * resolution, resolution)

        wall_boxes.append({
            'x': world_x,
            'y': world_y,
            'z': wall_height / 2.0,
            'sx': size_x,
            'sy': size_y,
            'sz': wall_height
        })

    print(f"🧱 추출된 3D 벽면 블록 수: {len(wall_boxes)} 개")

    # AprilTag 텍스처 경로 준비
    ws_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    tag_texture_path = os.path.join(ws_root, 'src/amr_bringup/models/apriltag_marker/materials/textures/tag36h11_1.png')
    generate_apriltag_texture(tag_texture_path)

    # 2. Gazebo Harmonic SDF World 파일 구성
    dock_x, dock_y, dock_yaw_deg = dock_pos
    dock_yaw_rad = math.radians(dock_yaw_deg)

    # 도킹 스테이션 벽면과 겹치는 기존 SLAM 벽 블록 제거 (마커 가림 방지)
    filtered_boxes = []
    for b in wall_boxes:
        if abs(b['x'] - dock_x) < 0.35 and abs(b['y'] - dock_y) < 0.75:
            continue
        filtered_boxes.append(b)
    wall_boxes = filtered_boxes

    sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="my_map_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Essential System Plugins -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>

    <!-- 태양광 조명 -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- 보조 조명 (AprilTag 시인성 강화) -->
    <light type="point" name="dock_light">
      <pose>{dock_x - 1.0 * math.cos(dock_yaw_rad):.3f} {dock_y - 1.0 * math.sin(dock_yaw_rad):.3f} 2.0 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <attenuation>
        <range>12</range>
        <constant>0.5</constant>
        <linear>0.05</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
    </light>

    <!-- 바닥면 (Ground Plane) -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.45 0.45 0.45 1</ambient>
            <diffuse>0.45 0.45 0.45 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- 🗺️ SLAM 2D 지도 기반 3D 벽면 구조물 -->
    <model name="slam_map_walls">
      <static>true</static>
      <link name="walls_link">
"""

    for i, b in enumerate(wall_boxes):
        sdf_content += f"""
        <!-- Wall Box {i} -->
        <collision name="wall_col_{i}">
          <pose>{b['x']:.4f} {b['y']:.4f} {b['z']:.4f} 0 0 0</pose>
          <geometry>
            <box>
              <size>{b['sx']:.4f} {b['sy']:.4f} {b['sz']:.4f}</size>
            </box>
          </geometry>
        </collision>
        <visual name="wall_vis_{i}">
          <pose>{b['x']:.4f} {b['y']:.4f} {b['z']:.4f} 0 0 0</pose>
          <geometry>
            <box>
              <size>{b['sx']:.4f} {b['sy']:.4f} {b['sz']:.4f}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.65 0.7 1</ambient>
            <diffuse>0.6 0.65 0.7 1</diffuse>
          </material>
        </visual>
"""

    sdf_content += f"""
      </link>
    </model>

    <!-- 🏷️ AprilTag #1 듀얼 도킹 스테이션 벽면 모델 (카메라 높이 60cm, 좌/우 19cm 기저선) -->
    <model name="apriltag_docking_wall">
      <static>true</static>
      <pose>{dock_x:.4f} {dock_y:.4f} 0.6000 0 0 {dock_yaw_rad:.4f}</pose>
      <link name="wall_link">
        <collision name="wall_collision">
          <geometry>
            <box>
              <size>0.10 1.20 1.20</size>
            </box>
          </geometry>
        </collision>
        <visual name="wall_visual">
          <geometry>
            <box>
              <size>0.10 1.20 1.20</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
          </material>
        </visual>

        <!-- 마커 흰색 지지 플레이트 -->
        <visual name="plate_visual">
          <pose>0.052 0 0.0 0 0 0</pose>
          <geometry>
            <box>
              <size>0.003 0.38 0.18</size>
            </box>
          </geometry>
          <material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>

        <!-- AprilTag #1 좌측 마커 (Y = -0.095m) -->
        <visual name="tag_left_visual">
          <pose>0.055 -0.095 0.0 0 1.570796 0</pose>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>0.08 0.08</size>
            </plane>
          </geometry>
          <material>
            <ambient>1.0 1.0 1.0 1</ambient>
            <diffuse>1.0 1.0 1.0 1</diffuse>
            <emissive>0.4 0.4 0.4 1</emissive>
            <pbr>
              <metal>
                <albedo_map>{tag_texture_path}</albedo_map>
                <roughness>0.1</roughness>
                <metalness>0.0</metalness>
              </metal>
            </pbr>
          </material>
        </visual>

        <!-- AprilTag #1 우측 마커 (Y = +0.095m) -->
        <visual name="tag_right_visual">
          <pose>0.055 0.095 0.0 0 1.570796 0</pose>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>0.08 0.08</size>
            </plane>
          </geometry>
          <material>
            <ambient>1.0 1.0 1.0 1</ambient>
            <diffuse>1.0 1.0 1.0 1</diffuse>
            <emissive>0.4 0.4 0.4 1</emissive>
            <pbr>
              <metal>
                <albedo_map>{tag_texture_path}</albedo_map>
                <roughness>0.1</roughness>
                <metalness>0.0</metalness>
              </metal>
            </pbr>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
"""

    os.makedirs(os.path.dirname(output_sdf_path), exist_ok=True)
    with open(output_sdf_path, 'w', encoding='utf-8') as f:
        f.write(sdf_content)

    print(f"\n🎉 [성공] Gazebo 3D 월드 생성 완료: {output_sdf_path}")


def main():
    import argparse
    parser = argparse.ArgumentParser(description="2D SLAM Map to Gazebo Harmonic 3D World Converter")
    parser.add_argument("--yaml", default="/home/juyeon/my_map.yaml", help="입력 2D 맵 YAML 파일 경로")
    parser.add_argument("--out", default="src/amr_bringup/worlds/my_map.sdf", help="출력 Gazebo SDF 월드 파일 경로")
    parser.add_argument("--wall-height", type=float, default=1.0, help="3D 벽면 높이 (미터, 기본 1.0m)")
    parser.add_argument("--dock-x", type=float, default=7.55, help="AprilTag 도킹 벽면 X 좌표")
    parser.add_argument("--dock-y", type=float, default=-5.15, help="AprilTag 도킹 벽면 Y 좌표")
    parser.add_argument("--dock-yaw", type=float, default=180.0, help="AprilTag 도킹 벽면이 바라보는 Yaw 각도 (도, 180.0 = -X 방향)")
    args = parser.parse_args()

    yaml_p = os.path.abspath(args.yaml)
    out_p = os.path.abspath(args.out)
    convert_map_to_sdf(yaml_p, out_p, wall_height=args.wall_height, dock_pos=(args.dock_x, args.dock_y, args.dock_yaw))


if __name__ == "__main__":
    main()

