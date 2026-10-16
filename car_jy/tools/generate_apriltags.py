#!/usr/bin/env python3
"""
tools/generate_apriltags.py
AMR 도킹 및 정밀 정렬용 AprilTag 이미지 생성 도구

지정된 태그 ID의 AprilTag 고화질 이미지(PNG)를 생성하여 출력 디렉토리에 저장합니다.
"""

import os
import argparse
import cv2
import numpy as np


TAG_DICT_MAP = {
    'tag36h11': cv2.aruco.DICT_APRILTAG_36h11,
    'tag25h9': cv2.aruco.DICT_APRILTAG_25h9,
    'tag16h5': cv2.aruco.DICT_APRILTAG_16h5,
    'aruco_original': cv2.aruco.DICT_ARUCO_ORIGINAL,
}


def generate_tag(family='tag36h11', tag_id=0, img_size=500, margin=50, output_dir='tags'):
    os.makedirs(output_dir, exist_ok=True)
    dict_id = TAG_DICT_MAP.get(family.lower(), cv2.aruco.DICT_APRILTAG_36h11)
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

    marker_size = img_size - (2 * margin)
    marker_img = cv2.aruco.drawMarker(aruco_dict, tag_id, marker_size, borderBits=1)

    canvas = np.full((img_size, img_size), 255, dtype=np.uint8)
    canvas[margin:margin+marker_size, margin:margin+marker_size] = marker_img

    # 라벨 텍스트 추가
    label = f"{family.upper()} - ID: {tag_id}"
    cv2.putText(canvas, label, (margin, img_size - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2, cv2.LINE_AA)

    filename = os.path.join(output_dir, f"{family}_id_{tag_id}.png")
    cv2.imwrite(filename, canvas)
    print(f"✅ 생성 완료: {filename} ({img_size}x{img_size}px)")
    return filename


def main():
    parser = argparse.ArgumentParser(description="AprilTag 이미지 생성기")
    parser.add_argument("--family", default="tag36h11", help="AprilTag 패밀리 (기본: tag36h11)")
    parser.add_argument("--ids", nargs="+", type=int, default=[0, 1, 2, 3, 4, 5], help="생성할 태그 ID 목록")
    parser.add_argument("--size", type=int, default=600, help="이미지 크기(px)")
    parser.add_argument("--output-dir", default="/home/juyeon/ros2_amr_ws/tags", help="저장 디렉토리")

    args = parser.parse_args()
    for tid in args.ids:
        generate_tag(args.family, tid, args.size, output_dir=args.output_dir)


if __name__ == '__main__':
    main()

