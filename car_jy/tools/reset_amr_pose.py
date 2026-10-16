#!/usr/bin/env python3
"""
AMR Robot Pose Reset Tool (Gazebo Simulation)
Allows instant repositioning of the robot to test straight or diagonal docking approaches without restarting Gazebo.
"""

import sys
import math
import argparse
import subprocess

def euler_to_quaternion(yaw_rad):
    # roll = 0, pitch = 0, yaw = yaw_rad
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)
    return 0.0, 0.0, qz, qw

def set_robot_pose(x, y, z, yaw_rad, world_name="docking_arena", model_name="amr"):
    qx, qy, qz, qw = euler_to_quaternion(yaw_rad)
    cmd = [
        "gz", "service",
        "-s", f"/world/{world_name}/set_pose",
        "--reqtype", "gz.msgs.Pose",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "2000",
        "--req",
        f'name: "{model_name}", position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}, orientation: {{x: {qx:.5f}, y: {qy:.5f}, z: {qz:.5f}, w: {qw:.5f}}}'
    ]
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, check=True)
        if "data: true" in res.stdout:
            print(f"✅ [위치 리셋 성공] {model_name} -> X={x:.2f}m, Y={y:.2f}m, Yaw={math.degrees(yaw_rad):.1f}°")
            return True
        else:
            print(f"⚠️ [리셋 응답 이상] {res.stdout.strip()}")
            return False
    except subprocess.CalledProcessError as e:
        print(f"❌ [오류 발생] Gazebo 서비스 호출 실패: {e.stderr.strip()}")
        return False

def main():
    parser = argparse.ArgumentParser(description="Gazebo AMR 로봇 위치 즉시 재설정 도구")
    parser.add_argument("--diagonal-right", "-dr", action="store_true", help="우측 대각선 위치 (X=1.4m, Y=-0.55m, Yaw=155°)")
    parser.add_argument("--diagonal-left", "-dl", action="store_true", help="좌측 대각선 위치 (X=1.4m, Y=+0.55m, Yaw=205°)")
    parser.add_argument("--center", "-c", action="store_true", help="정면 중앙 위치 (X=1.2m, Y=0.0m, Yaw=180°)")
    parser.add_argument("--x", type=float, default=None, help="임의 X 좌표 (m)")
    parser.add_argument("--y", type=float, default=None, help="임의 Y 좌표 (m)")
    parser.add_argument("--yaw-deg", type=float, default=None, help="임의 Yaw 각도 (도)")

    args = parser.parse_args()

    if args.diagonal_right:
        x, y, yaw_deg = 1.40, -0.55, 155.0
    elif args.diagonal_left:
        x, y, yaw_deg = 1.40, 0.55, 205.0
    elif args.center:
        x, y, yaw_deg = 1.20, 0.0, 180.0
    elif args.x is not None or args.y is not None or args.yaw_deg is not None:
        x = args.x if args.x is not None else 1.20
        y = args.y if args.y is not None else 0.0
        yaw_deg = args.yaw_deg if args.yaw_deg is not None else 180.0
    else:
        # 기본값: 우측 대각선 안내
        print("선택 가능한 프리셋:")
        print("  python3 tools/reset_amr_pose.py -dr  # 우측 대각선 진입 (X=1.4m, Y=-0.55m, 155°)")
        print("  python3 tools/reset_amr_pose.py -dl  # 좌측 대각선 진입 (X=1.4m, Y=+0.55m, 205°)")
        print("  python3 tools/reset_amr_pose.py -c   # 정면 중앙 진입   (X=1.2m, Y=0.00m, 180°)")
        x, y, yaw_deg = 1.40, -0.55, 155.0
        print(f"\n기본 설정인 우측 대각선(-dr)으로 로봇을 배치합니다...")

    yaw_rad = math.radians(yaw_deg)
    set_robot_pose(x, y, 0.10, yaw_rad)

if __name__ == "__main__":
    main()
