#!/usr/bin/env python3
"""
커스텀 스킬 테스트용 왕복 주행 스크립트 (test_round_trip.py)
--------------------------------------------------
1. 1초간 전진 (약 0.2m 이동)
2. 180도 (반 바퀴) 제자리 회전
3. 1초간 전진 (원위치 복귀)
4. 180도 제자리 회전하여 원래 바라보던 방향 정렬 후 정지
"""

import sys
import time
import signal
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SkillRoundTripNode(Node):
    def __init__(self):
        super().__init__('skill_round_trip_test')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.is_running = True
        self.get_logger().info("🚀 커스텀 스킬 주행 테스트 노드 시작")

    def publish_cmd(self, linear_x: float, angular_z: float, duration: float, step_name: str):
        """특정 속도로 지정된 시간 동안 /cmd_vel 발행"""
        print(f"\n▶ [{step_name}] 실행 중... (시간: {duration:.1f}초)")
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)

        start_time = time.time()
        while self.is_running and (time.time() - start_time < duration):
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            remaining = duration - (time.time() - start_time)
            print(f"   진행 중... 남은 시간: {max(0.0, remaining):.1f}초", end='\r')
            time.sleep(0.04)
        
        # 단계 종료 후 정지 패킷 송출
        self.stop_robot()
        time.sleep(0.3)

    def stop_robot(self):
        """로봇 즉시 정지"""
        stop_msg = Twist()
        for _ in range(10):
            self.cmd_pub.publish(stop_msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)
        print("\n🛑 로봇 완벽 정지 완료!")


def main(args=None):
    rclpy.init(args=args)
    node = SkillRoundTripNode()

    # SIGINT / SIGTERM 수신 시 안전 정지
    def signal_handler(sig, frame):
        print(f"\n⚠️ 중단 신호({sig}) 수신! 로봇을 즉시 정지합니다.")
        node.is_running = False
        node.stop_robot()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    straight_only = "--straight-only" in sys.argv

    # 기본 주행 파라미터
    drive_speed = 0.2   # m/s (직진 속도)
    drive_time = 1.0    # 초 (직진 시간: 1초 전진)
    turn_speed = 0.5    # rad/s (회전 속도)
    turn_time = 3.14159265 / turn_speed  # 180도(π rad) 회전 소요 시간 (~6.28초)

    if straight_only:
        print("=" * 60)
        print(" 🏎️ 커스텀 스킬 테스트: 1초 직진 후 즉시 정지")
        print("=" * 60)
        try:
            node.publish_cmd(linear_x=drive_speed, angular_z=0.0, duration=drive_time, step_name="1초 직진 주행")
            node.stop_robot()
            print("\n🎉 [성공] 1초 직진 주행 완료!")
        finally:
            node.stop_robot()
            node.destroy_node()
            rclpy.shutdown()
        return

    print("=" * 60)
    print(" 🤖 커스텀 스킬 테스트: 1초 직진 ➔ 180도 회전 ➔ 복귀 주행")
    print("=" * 60)

    try:
        # 1단계: 1초간 직진 전진
        node.publish_cmd(linear_x=drive_speed, angular_z=0.0, duration=drive_time, step_name="1단계: 1초 직진 전진")

        # 2단계: 반 바퀴 (180도) 제자리 회전
        node.publish_cmd(linear_x=0.0, angular_z=turn_speed, duration=turn_time, step_name="2단계: 180도 (반 바퀴) 제자리 회전")

        # 3단계: 다시 1초간 직진하여 원위치 복귀
        node.publish_cmd(linear_x=drive_speed, angular_z=0.0, duration=drive_time, step_name="3단계: 1초 직진 복귀")

        # 4단계: 원래 바라보던 정면 방향으로 180도 정렬 회전
        node.publish_cmd(linear_x=0.0, angular_z=turn_speed, duration=turn_time, step_name="4단계: 정면 180도 방향 정렬 회전")

        # 최종 완료 및 정지
        node.stop_robot()
        print("\n🎉 [성공] 커스텀 스킬 1초 직진-반바퀴-복귀 테스트 완료!")

    except (KeyboardInterrupt, SystemExit):
        print("\n⚠️ 사용자 강제 중단!")
        node.stop_robot()
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
