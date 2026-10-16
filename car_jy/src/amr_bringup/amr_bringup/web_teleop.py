import json
import math
import os
import socket
import threading
import time
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


WEB_APP_DIR = os.path.expanduser('~/ros2_amr_ws/web_app')

WAYPOINTS = {
    'home': (2.7, 2.0, -116.5),
    'point_a': (1.0, 1.0, 0.0),
    'point_b': (0.0, 2.0, 90.0),
    'point_c': (-2.0, 3.5, 0.0)
}


class WebTeleopRequestHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            html_path = os.path.join(WEB_APP_DIR, 'index.html')
            if os.path.exists(html_path):
                with open(html_path, 'rb') as f:
                    data = f.read()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_response(404)
                self.end_headers()
        elif self.path == '/style.css':
            css_path = os.path.join(WEB_APP_DIR, 'style.css')
            if os.path.exists(css_path):
                with open(css_path, 'rb') as f:
                    data = f.read()
                self.send_response(200)
                self.send_header('Content-Type', 'text/css; charset=utf-8')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_response(404)
                self.end_headers()
        elif self.path == '/app.js':
            js_path = os.path.join(WEB_APP_DIR, 'app.js')
            if os.path.exists(js_path):
                with open(js_path, 'rb') as f:
                    data = f.read()
                self.send_response(200)
                self.send_header('Content-Type', 'application/javascript; charset=utf-8')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_response(404)
                self.end_headers()
        elif self.path.startswith('/api/map_image'):
            map_path = os.path.expanduser('~/my_map.png')
            if os.path.exists(map_path):
                with open(map_path, 'rb') as f:
                    data = f.read()
                self.send_response(200)
                self.send_header('Content-Type', 'image/png')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_response(404)
                self.end_headers()
        elif self.path == '/api/status':
            pose = self.server.ros_node.current_pose
            body = json.dumps({'status': 'ok', 'pose': pose}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            # Fallback static file serving from WEB_APP_DIR
            clean_path = self.path.split('?')[0].lstrip('/')
            file_path = os.path.join(WEB_APP_DIR, clean_path)
            if os.path.exists(file_path) and os.path.isfile(file_path):
                ext = os.path.splitext(file_path)[1].lower()
                mime = 'text/plain'
                if ext == '.html':
                    mime = 'text/html; charset=utf-8'
                elif ext == '.css':
                    mime = 'text/css; charset=utf-8'
                elif ext == '.js':
                    mime = 'application/javascript; charset=utf-8'
                elif ext == '.png':
                    mime = 'image/png'
                elif ext in ('.jpg', '.jpeg'):
                    mime = 'image/jpeg'
                elif ext == '.svg':
                    mime = 'image/svg+xml'

                with open(file_path, 'rb') as f:
                    data = f.read()
                self.send_response(200)
                self.send_header('Content-Type', mime)
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_response(404)
                self.end_headers()

    def _send_json_ok(self, payload=b'{"status":"ok"}'):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(payload)))
        self.send_header('Connection', 'close')
        self.end_headers()
        self.wfile.write(payload)
        self.close_connection = True

    def do_POST(self):
        content_len = int(self.headers.get('Content-Length', 0))
        post_body = self.rfile.read(content_len) if content_len > 0 else b'{}'
        try:
            data = json.loads(post_body.decode('utf-8'))
        except Exception:
            data = {}

        if self.path == '/cmd':
            linear = float(data.get('linear', 0.0))
            angular = float(data.get('angular', 0.0))
            self.server.ros_node.update_cmd(linear, angular)
            self._send_json_ok()
        elif self.path == '/api/initial_pose':
            x = float(data.get('x', 0.0))
            y = float(data.get('y', 0.0))
            yaw = float(data.get('yaw', 0.0))
            self.server.ros_node.set_initial_pose(x, y, yaw)
            self._send_json_ok()
        elif self.path == '/api/goal_pose':
            x = float(data.get('x', 0.0))
            y = float(data.get('y', 0.0))
            yaw = float(data.get('yaw', 0.0))
            self.server.ros_node.send_goal_pose(x, y, yaw)
            self._send_json_ok()
        elif self.path == '/api/navigate':
            wp_id = data.get('waypoint')
            self.server.ros_node.navigate_to_waypoint(wp_id)
            self._send_json_ok()
        elif self.path == '/api/stop':
            self.server.ros_node.stop_robot()
            self._send_json_ok()
        elif self.path == '/api/action':
            action = data.get('action')
            self.server.ros_node.execute_action(action)
            self._send_json_ok()
        else:
            self.send_response(404)
            self.end_headers()


class WebTeleopNode(Node):
    def __init__(self):
        super().__init__('web_teleop')
        self.declare_parameter('port', 8080)
        self.port = self.get_parameter('port').value

        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pub_initial_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.current_pose = {'x': 2.70, 'y': 2.00, 'yaw': -2.0333}
        self.base_map_pose = {'x': 2.70, 'y': 2.00, 'yaw': -2.0333}
        self.base_odom_pose = None

        # Subscribe to AMCL pose for real-time map location
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )
        # Fallback to Odometry if AMCL is not yet initialized
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.target_linear = 0.0
        self.target_angular = 0.0
        self.last_cmd_time = time.time()
        self.has_amcl_pose = False
        self.action_thread = None

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.http_server = None
        self.server_thread = threading.Thread(target=self.run_http_server, daemon=True)
        self.server_thread.start()

        self.print_welcome_banner()

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        self.has_amcl_pose = True
        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_pose['yaw'] = 2.0 * math.atan2(qz, qw)

    def odom_callback(self, msg: Odometry):
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        oyaw = 2.0 * math.atan2(qz, qw)

        if not self.has_amcl_pose:
            if self.base_odom_pose is None:
                self.base_odom_pose = {'x': ox, 'y': oy, 'yaw': oyaw}

            # Calculate delta in odom local frame
            dx_odom = ox - self.base_odom_pose['x']
            dy_odom = oy - self.base_odom_pose['y']
            dyaw = oyaw - self.base_odom_pose['yaw']

            # Rotate delta into map frame using base yaw
            base_yaw = self.base_map_pose['yaw']
            dx_map = dx_odom * math.cos(base_yaw) - dy_odom * math.sin(base_yaw)
            dy_map = dx_odom * math.sin(base_yaw) + dy_odom * math.cos(base_yaw)

            self.current_pose['x'] = self.base_map_pose['x'] + dx_map
            self.current_pose['y'] = self.base_map_pose['y'] + dy_map
            self.current_pose['yaw'] = base_yaw + dyaw

    def set_initial_pose(self, x, y, yaw):
        self.base_map_pose = {'x': x, 'y': y, 'yaw': yaw}
        self.base_odom_pose = None
        self.current_pose['x'] = x
        self.current_pose['y'] = y
        self.current_pose['yaw'] = yaw

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        # Set stamp to 0 so TF uses the latest available transform without future extrapolation error
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.01
        self.pub_initial_pose.publish(msg)
        self.get_logger().info(f"📍 [Web] 초기 위치 보정 적용: ({x:.2f}, {y:.2f}, yaw={yaw:.2f} rad)")

    def send_goal_pose(self, x, y, yaw):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_goal.publish(goal)
        self.get_logger().info(f"🎯 [Web] 목표 전송: ({x:.2f}, {y:.2f}, yaw={yaw:.2f} rad)")

    def navigate_to_waypoint(self, wp_id):
        if wp_id == 'patrol':
            # Run patrol in background thread
            threading.Thread(target=self._run_patrol, daemon=True).start()
            return

        if wp_id in WAYPOINTS:
            x, y, yaw_deg = WAYPOINTS[wp_id]
            self.send_goal_pose(x, y, math.radians(yaw_deg))
            self.get_logger().info(f"📍 [Web Nav] 웨이포인트 이동: {wp_id} ({x}, {y})")

    def _run_patrol(self):
        self.get_logger().info("🚨 [Web Nav] 전체 순찰 미션 시작 (A -> B -> C -> Home)")
        for wp in ['point_a', 'point_b', 'point_c', 'home']:
            x, y, yaw_deg = WAYPOINTS[wp]
            self.send_goal_pose(x, y, math.radians(yaw_deg))
            # Wait 15 seconds per leg or until next
            time.sleep(12.0)

    def execute_action(self, action):
        if action == 'round_trip':
            threading.Thread(target=self._run_round_trip, daemon=True).start()

    def _run_round_trip(self):
        self.get_logger().info("🔄 [Web Action] 2초 직진 + 180도 회전 복귀 시작")
        # Step 1: Forward 2.0s
        self.update_cmd(0.20, 0.0)
        time.sleep(2.0)
        self.stop_robot()
        time.sleep(0.3)
        # Step 2: Rotate 180 deg (increased rotation time to 5.2s @ 0.80 rad/s)
        self.update_cmd(0.0, 0.80)
        time.sleep(5.2)
        self.stop_robot()
        time.sleep(0.3)
        # Step 3: Forward 2.0s back to start
        self.update_cmd(0.20, 0.0)
        time.sleep(2.0)
        self.stop_robot()
        self.get_logger().info("🔄 [Web Action] 2초 왕복 테스트 완료")

    def get_local_ip(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "127.0.0.1"

    def print_welcome_banner(self):
        local_ip = self.get_local_ip()
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"📱 [AMR Control Center] 통합 관제 웹 대시보드 구동 완료!")
        self.get_logger().info(f"👉 브라우저(스마트폰/PC)에서 아래 주소로 접속하세요:")
        self.get_logger().info(f"   🌐 http://{local_ip}:{self.port}")
        self.get_logger().info("=" * 60)

    def run_http_server(self):
        try:
            self.http_server = ThreadingHTTPServer(('0.0.0.0', self.port), WebTeleopRequestHandler)
            self.http_server.ros_node = self
            self.http_server.daemon_threads = True
            self.http_server.serve_forever()
        except Exception as e:
            self.get_logger().error(f"HTTP 서버 시작 실패: {e}")

    def update_cmd(self, linear, angular):
        self.target_linear = linear
        self.target_angular = angular
        self.last_cmd_time = time.time()
        # Immediate zero-latency publish
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub_cmd_vel.publish(msg)

    def stop_robot(self):
        self.target_linear = 0.0
        self.target_angular = 0.0
        msg = Twist()
        for _ in range(3):
            self.pub_cmd_vel.publish(msg)
            time.sleep(0.01)
        self.get_logger().info("🛑 [Web Nav] 로봇 긴급 정지 완료")

    def timer_callback(self):
        if (self.target_linear != 0.0 or self.target_angular != 0.0) and (time.time() - self.last_cmd_time > 0.8):
            self.target_linear = 0.0
            self.target_angular = 0.0

        msg = Twist()
        msg.linear.x = self.target_linear
        msg.angular.z = self.target_angular
        self.pub_cmd_vel.publish(msg)

    def destroy_node(self):
        if self.http_server:
            self.http_server.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebTeleopNode()
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
