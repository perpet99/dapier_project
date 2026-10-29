#!/usr/bin/env python3
"""
RViz에서 마우스 클릭(2D Goal Pose 또는 Publish Point)으로 웨이포인트 자동 저장 도구
-----------------------------------------------------------------------------
사용법:
  python3 tools/click_to_save_waypoint.py home
  (실행 후 RViz 상단 툴바에서 '2D Goal Pose' 또는 'Publish Point'를 누르고 지도 위를 클릭하세요!)
"""

import sys
import math
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from set_waypoint import set_waypoint


def quaternion_to_yaw(z, w, x=0.0, y=0.0):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


class RVizClickListener(Node):
    def __init__(self, wp_id="home"):
        super().__init__('rviz_click_listener')
        self.wp_id = wp_id
        self.received = False

        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 10)

        print("\n" + "=" * 65)
        print(f"🖱️  [RViz 마우스 클릭 대기 중: 대상 '{self.wp_id}']")
        print("=" * 65)
        print("👉 RViz 화면 상단 툴바에서 다음 중 하나를 선택하고 지도를 클릭하세요:")
        print("   1) [2D Goal Pose] 클릭 (화살표 드래그로 X, Y, Yaw 각도까지 지정)")
        print("   2) [Publish Point] 클릭 (점 클릭으로 X, Y 좌표 지정)")
        print("=" * 65 + "\n")

    def goal_pose_callback(self, msg: PoseStamped):
        if self.received:
            return
        self.received = True
        x = msg.pose.position.x
        y = msg.pose.position.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        yaw_deg = quaternion_to_yaw(qz, qw)

        print(f"🎯 [2D Goal Pose 수신!] X={x:.2f}, Y={y:.2f}, Yaw={yaw_deg:.1f}°")
        set_waypoint(self.wp_id, x, y, yaw_deg)
        rclpy.shutdown()

    def clicked_point_callback(self, msg: PointStamped):
        if self.received:
            return
        self.received = True
        x = msg.point.x
        y = msg.point.y
        print(f"🎯 [Publish Point 수신!] X={x:.2f}, Y={y:.2f}")
        set_waypoint(self.wp_id, x, y, None)
        rclpy.shutdown()


def main():
    wp_id = sys.argv[1].strip() if len(sys.argv) >= 2 else "home"
    rclpy.init()
    node = RVizClickListener(wp_id=wp_id)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass


if __name__ == '__main__':
    main()
