#!/usr/bin/env python3
"""
AMR 안전 언도킹(Undocking) CLI 도구 (undock_amr.py)
------------------------------------------------
실행 예시:
  python3 tools/undock_amr.py
  python3 tools/undock_amr.py --dist 0.50 --turn180
"""

import os
import sys
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "../src/amr_bringup"))
from amr_bringup.undock_node import AMRUndocker
import rclpy


def undock(dist=0.45, speed=0.06, turn180=False, turn_speed=0.30):
    if not rclpy.ok():
        rclpy.init()
    
    undocker = AMRUndocker(backup_dist=dist, backup_speed=speed, turn_180=turn180, turn_speed=turn_speed)
    try:
        return undocker.execute_undock()
    finally:
        undocker.destroy_node()


def main():
    parser = argparse.ArgumentParser(description="AMR Undocking Tool")
    parser.add_argument("--dist", type=float, default=0.45, help="후진 안전 거리 (미터, 기본 0.45m = 45cm)")
    parser.add_argument("--speed", type=float, default=0.06, help="후진 선속도 (m/s, 기본 0.06)")
    parser.add_argument("--turn180", action="store_true", help="후진 완료 후 180도 회전 실행")
    parser.add_argument("--turn-speed", type=float, default=0.30, help="회전 각속도 (rad/s, 기본 0.30)")

    args = parser.parse_args()
    if not rclpy.ok():
        rclpy.init()
    undock(dist=args.dist, speed=args.speed, turn180=args.turn180, turn_speed=args.turn_speed)
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
