#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class CmdVelOdomNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_odom_node')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('timeout_sec', 0.5)
        self.declare_parameter('invert_angular', False)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.invert_angular = bool(self.get_parameter('invert_angular').value)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.wz = 0.0

        self.last_cmd_time = self.get_clock().now()
        self.last_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # 50Hz 주기 오도메트리 적분 및 TF 브로드캐스트
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info("🚀 [AMR Odom] CmdVel 기반 동적 오도메트리 브로드캐스터 시작 (50Hz)")

    def cmd_vel_callback(self, msg: Twist):
        self.vx = msg.linear.x
        self.wz = -msg.angular.z if self.invert_angular else msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # 명령 타임아웃 감시 (명령이 안 들어오면 속도 0)
        time_since_cmd = (now - self.last_cmd_time).nanoseconds / 1e9
        if time_since_cmd > self.timeout_sec:
            self.vx = 0.0
            self.wz = 0.0

        if dt > 0.0:
            delta_x = (self.vx * math.cos(self.th)) * dt
            delta_y = (self.vx * math.sin(self.th)) * dt
            delta_th = self.wz * dt

            self.x += delta_x
            self.y += delta_y
            self.th += delta_th

        quat = yaw_to_quaternion(self.th)

        # 1. TF 브로드캐스트 (odom -> base_footprint)
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = quat
            self.tf_broadcaster.sendTransform(t)

        # 2. Odometry 토픽 발행 (/odom)
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.wz

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelOdomNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
