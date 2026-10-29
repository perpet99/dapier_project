#!/usr/bin/env python3
"""
AMR 안전 언도킹(Undocking) 노드 (undock_node.py)
---------------------------------------------
도킹 스테이션 또는 벽면에 밀착된 로봇을 안전하게 후진시켜
장애물 및 벽면과의 안전 거리(기본 45cm)를 확보하고, 옵션에 따라 180도 회전합니다.
"""

import sys
import time
import math
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class AMRUndocker(Node):
    def __init__(self, backup_dist=0.45, backup_speed=0.06, turn_180=False, turn_speed=0.30):
        super().__init__('amr_undocker')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.backup_dist = float(backup_dist)
        self.backup_speed = abs(float(backup_speed))
        self.turn_180 = bool(turn_180)
        self.turn_speed = abs(float(turn_speed))

    def publish_vel(self, vx, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def execute_undock(self):
        print("=" * 60)
        print("🚀 [AMR 언도킹(Undocking) 시퀀스 시작]")
        print(f" • 후진 거리:   {self.backup_dist * 100:.1f} cm")
        print(f" • 후진 속도:   {self.backup_speed * 100:.1f} cm/s")
        print(f" • 180° 회전:  {'활성화' if self.turn_180 else '비활성화'}")
        print("=" * 60)

        # 1. 안전 대기 (0.5초)
        for _ in range(5):
            self.publish_vel(0.0, 0.0)
            time.sleep(0.1)

        # 2. 직선 후진 (지정 거리 이동)
        duration = self.backup_dist / self.backup_speed
        print(f"\n🔙 [1단계: 안전 후진] {self.backup_dist*100:.0f}cm 후진 중 (약 {duration:.1f}초)...")

        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            self.publish_vel(-self.backup_speed, 0.0)
            time.sleep(0.05)

        # 감속 정지
        for _ in range(5):
            self.publish_vel(0.0, 0.0)
            time.sleep(0.1)

        print("✓ 안전 후진 완료! (도킹 해제됨)")

        # 3. 옵션: 180도 회전
        if self.turn_180 and rclpy.ok():
            turn_duration = math.pi / self.turn_speed
            print(f"\n🔄 [2단계: 180° 회전] 열린 공간을 향해 회전 중 (약 {turn_duration:.1f}초)...")
            start_turn = time.time()
            while time.time() - start_turn < turn_duration and rclpy.ok():
                self.publish_vel(0.0, self.turn_speed)
                time.sleep(0.05)

            # 정지
            for _ in range(5):
                self.publish_vel(0.0, 0.0)
                time.sleep(0.1)
            print("✓ 180도 회전 완료!")

        print("\n" + "=" * 60)
        print("🎉 [언도킹 완료] 이제 Nav2 자율주행이나 다른 미션을 안전하게 시작할 수 있습니다.")
        print("=" * 60 + "\n")
        return True


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description="AMR Safe Undocking Routine")
    parser.add_argument("--dist", type=float, default=0.45, help="후진 안전 거리 (미터, 기본 0.45m = 45cm)")
    parser.add_argument("--speed", type=float, default=0.06, help="후진 선속도 (m/s, 기본 0.06)")
    parser.add_argument("--turn180", action="store_true", help="후진 완료 후 180도 회전 실행")
    parser.add_argument("--turn-speed", type=float, default=0.30, help="회전 각속도 (rad/s, 기본 0.30)")

    parsed_args, _ = parser.parse_known_args()

    undocker = AMRUndocker(
        backup_dist=parsed_args.dist,
        backup_speed=parsed_args.speed,
        turn_180=parsed_args.turn180,
        turn_speed=parsed_args.turn_speed
    )

    try:
        undocker.execute_undock()
    except KeyboardInterrupt:
        print("\n🛑 사용자에 의해 언도킹 중단됨.")
    finally:
        undocker.publish_vel(0.0, 0.0)
        undocker.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
