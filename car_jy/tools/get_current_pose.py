#!/usr/bin/env python3
"""
현재 로봇의 RViz 지도(map) 기준 실시간 좌표 및 각도 조회 도구 (get_current_pose.py)
-------------------------------------------------------------------------
- RViz 상의 map -> base_link (또는 /amcl_pose)로부터 현재 위치 (X, Y, Yaw)를 읽어옵니다.
"""

import sys
import math
import time
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseWithCovarianceStamped


def quaternion_to_yaw(z, w, x=0.0, y=0.0):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class CurrentPoseReader(Node):
    def __init__(self):
        super().__init__('current_pose_reader')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.amcl_pose = None
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)

    def amcl_callback(self, msg):
        self.amcl_pose = msg.pose.pose

    def get_pose(self, timeout_sec=3.0):
        start = time.time()
        # 1. TF에서 map -> base_link 조회
        while time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                x = tf.transform.translation.x
                y = tf.transform.translation.y
                qz = tf.transform.rotation.z
                qw = tf.transform.rotation.w
                qx = tf.transform.rotation.x
                qy = tf.transform.rotation.y
                yaw_rad = quaternion_to_yaw(qz, qw, qx, qy)
                yaw_deg = math.degrees(yaw_rad)
                return x, y, yaw_deg, "TF (map -> base_link)"
            except Exception:
                pass

        # 2. /amcl_pose 조회
        if self.amcl_pose is not None:
            x = self.amcl_pose.position.x
            y = self.amcl_pose.position.y
            qz = self.amcl_pose.orientation.z
            qw = self.amcl_pose.orientation.w
            yaw_rad = quaternion_to_yaw(qz, qw)
            yaw_deg = math.degrees(yaw_rad)
            return x, y, yaw_deg, "/amcl_pose"

        return None, None, None, None


def main():
    rclpy.init()
    reader = CurrentPoseReader()
    x, y, yaw_deg, source = reader.get_pose(timeout_sec=3.0)
    reader.destroy_node()
    rclpy.shutdown()

    print("\n" + "=" * 65)
    print("📍 [현재 로봇의 RViz 지도(map) 기준 좌표]")
    print("=" * 65)
    if x is not None:
        print(f" • X 좌표 (전후):    {x:+.3f} m")
        print(f" • Y 좌표 (좌우):    {y:+.3f} m")
        print(f" • Yaw 각도 (회전):  {yaw_deg:+.1f}°")
        print(f" • 데이터 출처:      {source}")
        print("=" * 65)
        print(f"💡 waypoints.xml 추천 형식:")
        print(f"   <x>{x:.2f}</x>")
        print(f"   <y>{y:.2f}</y>")
        print(f"   <yaw>{yaw_deg:.1f}</yaw>\n")
    else:
        print("❌ 지도(map) 기준 로봇 위치를 가져올 수 없습니다.")
        print("   (Nav2 또는 SLAM / AMCL 노드가 실행 중인지 확인해 주세요)\n")


if __name__ == '__main__':
    main()
