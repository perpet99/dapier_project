#!/usr/bin/env python3
"""
Nav2 파이썬 좌표 기반 자율주행 제어 스크립트
- 원하는 좌표 (x, y, yaw 각도)를 입력받아 자율주행 실행
- 실시간 이동 거리 및 남은 시간 터미널 표시
- 다른 파이썬 코드에서 import하여 함수로 사용 가능:
    from navigate_to_coords import RobotNavigatorClient
    bot = RobotNavigatorClient()
    bot.go_to(x=1.5, y=2.0, yaw_deg=90.0)
"""
import math
import sys
import time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy


class RobotNavigatorClient:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.navigator = BasicNavigator()

    def euler_to_quaternion(self, yaw_deg):
        """yaw 각도(도)를 쿼터니언 (z, w)으로 변환"""
        yaw_rad = math.radians(yaw_deg)
        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)
        return qz, qw

    def set_initial_pose(self, x=0.0, y=0.0, yaw_deg=0.0):
        """초기 위치 설정 (2D Pose Estimate 기능)"""
        qz, qw = self.euler_to_quaternion(yaw_deg)
        init_pose = PoseStamped()
        init_pose.header.frame_id = 'map'
        init_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        init_pose.pose.position.x = float(x)
        init_pose.pose.position.y = float(y)
        init_pose.pose.position.z = 0.0
        init_pose.pose.orientation.z = qz
        init_pose.pose.orientation.w = qw
        self.navigator.setInitialPose(init_pose)
        print(f"✓ 초기 위치 설정 완료: x={x}, y={y}, yaw={yaw_deg}°")

    def go_to(self, x, y, yaw_deg=0.0, timeout_sec=120.0, check_callback=None):
        """
        특정 좌표로 자율주행 이동 함수
        :param x: 목표 x 좌표 (미터)
        :param y: 목표 y 좌표 (미터)
        :param yaw_deg: 목표 도달 후 바라볼 각도 (-180° ~ +180°)
        :param timeout_sec: 최대 주행 대기 시간 (초)
        :param check_callback: 주행 중 매 주기 호출되는 콜백 함수 (True 반환 시 Nav2 주행을 즉시 조기 종료하고 True 반환)
        :return: True(성공 또는 조기 감지 완료), False(실패)
        """
        qz, qw = self.euler_to_quaternion(yaw_deg)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw

        print("=" * 60)
        print(f"🚀 목표 좌표로 자율주행 출발!")
        print(f"   - 목표 위치: X = {x:.2f} m, Y = {y:.2f} m")
        print(f"   - 최종 방향: {yaw_deg:.1f}°")
        print("=" * 60)

        # Nav2 액션 서버 연결 상태 3초 타임아웃 검사 (무한 대기 방지)
        if hasattr(self.navigator, 'nav_to_pose_client') and self.navigator.nav_to_pose_client is not None:
            if not self.navigator.nav_to_pose_client.wait_for_server(timeout_sec=3.0):
                print("\n❌ [오류] Nav2 내비게이션 노드가 실행되어 있지 않습니다!")
                print("   💡 터미널에서 'ros2 launch amr_bringup navigation.launch.py map:=...' 를 실행해 주세요.\n")
                return False

        # 자율주행 실행
        sent = self.navigator.goToPose(goal_pose)
        if not sent:
            print("\n❌ [목표 거부] Nav2가 목표 지점을 수락하지 않았습니다!")
            print("   (원인: 목표가 벽/장애물 내부이거나, 내비게이션 노드가 준비되지 않음)\n")
            return False

        start_time = time.time()
        while not self.navigator.isTaskComplete():
            elapsed = time.time() - start_time
            if elapsed > timeout_sec:
                print("\n⚠️ 주행 시간 초과! 주행을 취소합니다.")
                self.navigator.cancelTask()
                return False

            # 외부 조기 감지 콜백 확인 (예: AprilTag 마커 감지)
            if check_callback is not None:
                try:
                    if check_callback():
                        print("\n🎯 [태그 발견!] Nav2 주행 중 AprilTag 마커가 감지되어 즉시 비전 도킹으로 전환합니다.")
                        self.navigator.cancelTask()
                        time.sleep(0.5)
                        return True
                except Exception as e:
                    pass

            feedback = self.navigator.getFeedback()
            if feedback:
                dist_remaining = feedback.distance_remaining
                nav_time = feedback.navigation_time.sec
                print(f"   🚗 이동 중... 남은 거리: {dist_remaining:.2f} m (주행시간: {nav_time}s)", end='\r')

            time.sleep(0.2)

        # 결과 확인
        result = self.navigator.getResult()
        print()
        if result == TaskResult.SUCCEEDED:
            print(f"🎉 [도착 성공] 목표 좌표 ({x}, {y})에 완벽히 도달했습니다!\n")
            return True
        elif result == TaskResult.CANCELED:
            print("⚠️ [주행 취소] 주행이 취소되었습니다.\n")
            return False
        elif result == TaskResult.FAILED:
            print("❌ [주행 실패] 경로를 찾을 수 없거나 장애물로 인해 실패했습니다.\n")
            return False
        else:
            print("❓ [알 수 없는 상태] 주행이 완료되지 못했습니다.\n")
            return False


def main():
    bot = RobotNavigatorClient()

    # 명령행 인자로 바로 실행하는 경우: python3 navigate_to_coords.py 1.5 2.0 90
    if len(sys.argv) >= 3:
        try:
            target_x = float(sys.argv[1])
            target_y = float(sys.argv[2])
            target_yaw = float(sys.argv[3]) if len(sys.argv) >= 4 else 0.0
            success = bot.go_to(target_x, target_y, target_yaw)
            sys.exit(0 if success else 1)
        except ValueError:
            print("❌ 좌표는 숫자(실수)여야 합니다. 예: python3 navigate_to_coords.py 1.5 2.0 90")
            sys.exit(1)

    # 인자가 없을 때 대화형 메뉴
    print("=" * 60)
    print("        Nav2 좌표 입력 자율주행 프로그램")
    print("=" * 60)
    print("💡 등록된 프리셋 장소로 이동하거나 원하는 좌표(X, Y)를 직접 입력하세요.\n")

    # 미리 정의해둘 수 있는 장소 목록 (좌표는 방 환경에 맞게 수정 가능)
    PRESETS = {
        "1": ("충전 스테이션 / 홈 지점", 2.66, 2.11, -116.5),
        "2": ("지점 A (거실 앞)", 1.2, 0.5, 0.0),
        "3": ("지점 B (방 입구)", 0.8, -1.0, 90.0),
    }

    print("[프리셋 위치 목록]")
    for k, v in PRESETS.items():
        print(f"  {k}. {v[0]} (X={v[1]}, Y={v[2]}, Yaw={v[3]}°)")
    print("  4. 직접 좌표(X, Y, Yaw) 입력하기")
    print("  q. 종료")
    print("=" * 60)

    choice = input("선택할 번호를 입력하세요: ").strip()

    if choice in PRESETS:
        name, x, y, yaw = PRESETS[choice]
        print(f"\n👉 '{name}'(으)로 이동을 시작합니다...")
        bot.go_to(x, y, yaw)
    elif choice == "4":
        try:
            x_in = float(input("목표 X 좌표 (미터): ").strip())
            y_in = float(input("목표 Y 좌표 (미터): ").strip())
            yaw_in = float(input("목표 각도 Yaw (도, 예: 0, 90, 180): ").strip() or "0")
            bot.go_to(x_in, y_in, yaw_in)
        except ValueError:
            print("올바른 숫자를 입력해 주세요.")
    elif choice.lower() == 'q':
        print("종료합니다.")
    else:
        print("올바르지 않은 번호입니다.")


if __name__ == '__main__':
    main()
