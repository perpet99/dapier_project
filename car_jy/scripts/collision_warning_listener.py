#!/usr/bin/env python3
"""
실시간 충돌 방지 및 안전 경고 리스너 노드 (collision_warning_listener.py)
----------------------------------------------------------------------
Nav2 Collision Monitor의 상태(/collision_monitor_state)를 실시간 감시하여:
- 장애물 접근 시 감속 서행 안내 (SLOWDOWN)
- 위험 구역 진입 시 긴급 정지 경고 (STOP)
를 터미널에 실시간으로 안내합니다.
"""

import rclpy
from rclpy.node import Node
from nav2_msgs.msg import CollisionMonitorState


class CollisionWarningListener(Node):
    def __init__(self):
        super().__init__('collision_warning_listener')
        self.sub_state = self.create_subscription(
            CollisionMonitorState,
            '/collision_monitor_state',
            self.state_callback,
            10
        )
        self.last_state = -1
        self.get_logger().info("🛡️ 실시간 충돌 방지 및 안전 경고 시스템이 활성화되었습니다.")

    def state_callback(self, msg: CollisionMonitorState):
        if msg.action_type == self.last_state:
            return

        self.last_state = msg.action_type

        # 0: DO_NOTHING, 1: STOP, 2: SLOWDOWN, 3: APPROACH, 4: LIMIT
        if msg.action_type == 2:  # SLOWDOWN
            print(f"\n⚠️  [충돌 주의 경고] 장애물 접근 감지! (구역: {msg.polygon_name})")
            print("    👉 안전을 위해 50% 속도로 감속 서행하며 회피 주행 중입니다...")
        elif msg.action_type == 1:  # STOP
            print(f"\n🚨 [충돌 위험 긴급 정지!] 장애물이 로봇 차체 바로 코앞에 감지되었습니다! (구역: {msg.polygon_name})")
            print("    👉 충돌 방지를 위해 로봇이 정지했습니다. 장애물이 치워지면 자동으로 주행을 재개합니다.")
        elif msg.action_type == 0:  # NORMAL
            print("\n✅ [안전 구역 복귀] 전방 장애물이 해소되어 정상 속도로 자율주행을 재개합니다.")


def main(args=None):
    rclpy.init(args=args)
    node = CollisionWarningListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
