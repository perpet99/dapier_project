#!/usr/bin/env python3
"""
AMR 자율주행 통합 웹 관제 서버 (amr_web_server.py)
------------------------------------------------------------------
- REST API, SSE 실시간 이벤트, Nav2 및 조이스틱 cmd_vel 완벽 연동
- TF / Odom / AMCL 실시간 구독을 통한 웹 지도 로봇 위치 실시간 동기화
"""

import os
import sys
import json
import time
import math
import asyncio
from aiohttp import web

# 모듈 경로 추가
_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _root not in sys.path:
    sys.path.insert(0, _root)

from bot_modules.config import get_waypoints_dict
from bot_modules.ros2_runner import stop_robot_helper, execute_coords_move
from bot_modules.mission_queue import run_action_queue

# 전역 로봇 상태
current_robot_pose = {'x': 2.70, 'y': 2.00, 'yaw': -2.033}
current_velocity = {'linear': 0.0, 'angular': 0.0}

# ROS2 노드 및 퍼블리셔/서브스크립션 초기화
_ros_node = None
_pub_cmd_vel = None
_pub_goal = None
_pub_init_pose = None
_tf_buffer = None
_tf_listener = None

try:
    import rclpy
    from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    import tf2_ros

    if not rclpy.ok():
        rclpy.init()
    _ros_node = rclpy.create_node('amr_unified_web_server')
    _pub_cmd_vel = _ros_node.create_publisher(Twist, '/cmd_vel', 10)
    _pub_goal = _ros_node.create_publisher(PoseStamped, '/goal_pose', 10)
    _pub_init_pose = _ros_node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    # TF 리스너 (map -> base_footprint 위치 추적)
    _tf_buffer = tf2_ros.Buffer()
    _tf_listener = tf2_ros.TransformListener(_tf_buffer, _ros_node)

    def _odom_cb(msg: Odometry):
        global current_velocity
        current_velocity['linear'] = msg.twist.twist.linear.x
        current_velocity['angular'] = msg.twist.twist.angular.z

    def _amcl_cb(msg: PoseWithCovarianceStamped):
        global current_robot_pose
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        current_robot_pose = {
            'x': float(msg.pose.pose.position.x),
            'y': float(msg.pose.pose.position.y),
            'yaw': float(yaw)
        }

    _sub_odom = _ros_node.create_subscription(Odometry, '/odom', _odom_cb, 10)
    _sub_amcl = _ros_node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', _amcl_cb, 10)

except Exception as e:
    print(f"⚠️ ROS2 Python 라이브러리 연동 경고: {e}")

routes = web.RouteTableDef()
WEB_APP_DIR = os.path.join(_root, "web_app")
MAPS_DIR = os.path.join(_root, "maps")
sse_clients = set()


def update_pose_from_tf():
    """TF (map -> base_footprint)에서 최신 좌표 가져오기"""
    global current_robot_pose
    if not _tf_buffer:
        return
    try:
        t = _tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        q = t.transform.rotation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        current_robot_pose = {
            'x': float(t.transform.translation.x),
            'y': float(t.transform.translation.y),
            'yaw': float(yaw)
        }
    except Exception:
        # map이 없으면 odom -> base_footprint 시도
        try:
            t = _tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
            q = t.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            current_robot_pose = {
                'x': float(t.transform.translation.x),
                'y': float(t.transform.translation.y),
                'yaw': float(yaw)
            }
        except Exception:
            pass


class WebChannel:
    """터미널 출력 및 SSE 클라이언트 실시간 브로드캐스트 채널"""
    async def send(self, content=None, embed=None):
        if not content:
            return
        text = content.replace("**", "").replace("`", "")
        print(f"🌐 [AMR Server]: {text}")

        status_type, state = "info", "idle"
        if "대기열 추가" in text or "대기 순번" in text:
            state = "queued"
        elif "도착 완료" in text or "완주했습니다" in text:
            status_type, state = "success", "arrived"
        elif any(k in text for k in ["실패", "중단", "취소", "❌", "⚠️"]):
            status_type, state = "error", "failed"
        elif any(k in text for k in ["출발", "이동 중", "시작", "주행", "다음 미션"]):
            status_type, state = "moving", "moving"

        payload = f"event: mission_update\ndata: {json.dumps({'message': text, 'status_type': status_type, 'state': state, 'timestamp': time.strftime('%H:%M:%S')}, ensure_ascii=False)}\n\n"
        for q in list(sse_clients):
            await q.put(payload)


web_channel = WebChannel()


@routes.get('/')
async def handle_index(request):
    return web.FileResponse(os.path.join(WEB_APP_DIR, "index.html"))


@routes.get('/api/waypoints')
async def handle_waypoints(request):
    return web.json_response(get_waypoints_dict())


@routes.get('/api/status')
@routes.get('/api/robot/status')
async def handle_status(request):
    if _ros_node:
        rclpy.spin_once(_ros_node, timeout_sec=0.001)
    update_pose_from_tf()

    return web.json_response({
        "status": "ok",
        "state": "idle",
        "message": "온라인 정상 연결됨",
        "pose": current_robot_pose,
        "linear": current_velocity['linear'],
        "angular": current_velocity['angular'],
        "battery": 98
    })


async def _blast_zero_cmd_vel():
    """Nav2 잔여 궤적 명령을 덮어쓰기 위해 0.5초 동안 연속으로 0 속도 송출"""
    for _ in range(25):
        if _pub_cmd_vel and _ros_node:
            _pub_cmd_vel.publish(Twist())
            rclpy.spin_once(_ros_node, timeout_sec=0.005)
        await asyncio.sleep(0.02)


@routes.post('/api/stop')
async def handle_stop(request):
    print("🛑 [웹 요청] 로봇 즉시 비상 정지!")
    await stop_robot_helper(web_channel)
    asyncio.create_task(_blast_zero_cmd_vel())
    return web.json_response({"success": True, "message": "Stopped"})


@routes.post('/api/navigate')
@routes.post('/api/move')
async def handle_navigate(request):
    try:
        data = await request.json()
        target = data.get("waypoint") or data.get("target", "")
        target = str(target).strip()
    except Exception:
        return web.json_response({"success": False, "error": "Invalid JSON"}, status=400)

    wps = get_waypoints_dict()
    if not target or target not in wps:
        print(f"❌ [웹 요청 오류] 등록되지 않은 장소: '{target}'")
        return web.json_response({"success": False, "error": f"등록되지 않은 장소: '{target}'"}, status=400)

    print(f"🚀 [웹 요청] '{target}' ({wps[target].get('name', target)}) 장소로 이동 시작")
    asyncio.create_task(run_action_queue(web_channel, [{"type": "move", "target": target}], wps))
    return web.json_response({"success": True, "target": target, "name": wps[target].get("name", target)})


@routes.post('/api/goal_pose')
async def handle_goal_pose(request):
    try:
        data = await request.json()
        x = float(data.get('x', 0.0))
        y = float(data.get('y', 0.0))
        yaw = float(data.get('yaw', 0.0))
    except Exception as e:
        return web.json_response({"success": False, "error": str(e)}, status=400)

    print(f"🎯 [웹 요청] 임의 목표 좌표 전송: X={x:.2f}, Y={y:.2f}, Yaw={yaw:.2f} rad")

    if _pub_goal and _ros_node:
        msg = PoseStamped()
        msg.header.stamp = _ros_node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        _pub_goal.publish(msg)
        rclpy.spin_once(_ros_node, timeout_sec=0.01)

    asyncio.create_task(execute_coords_move(web_channel, x, y, yaw))
    return web.json_response({"success": True, "x": x, "y": y, "yaw": yaw})


@routes.post('/api/initial_pose')
async def handle_initial_pose(request):
    global current_robot_pose
    try:
        data = await request.json()
        x = float(data.get('x', 0.0))
        y = float(data.get('y', 0.0))
        yaw = float(data.get('yaw', 0.0))
    except Exception as e:
        return web.json_response({"success": False, "error": str(e)}, status=400)

    print(f"📍 [웹 요청] 초기 위치 설정: X={x:.2f}, Y={y:.2f}, Yaw={yaw:.2f} rad")
    current_robot_pose = {'x': x, 'y': y, 'yaw': yaw}

    if _pub_init_pose and _ros_node:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = _ros_node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        _pub_init_pose.publish(msg)
        rclpy.spin_once(_ros_node, timeout_sec=0.01)

    return web.json_response({"success": True, "x": x, "y": y, "yaw": yaw})


@routes.post('/api/cmd_vel')
@routes.post('/cmd')
async def handle_cmd_vel(request):
    try:
        data = await request.json()
        linear = float(data.get('linear', 0.0))
        angular = float(data.get('angular', 0.0))
    except Exception:
        linear = 0.0
        angular = 0.0

    if _pub_cmd_vel and _ros_node:
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        _pub_cmd_vel.publish(msg)
        rclpy.spin_once(_ros_node, timeout_sec=0.005)

    return web.json_response({"status": "ok"})


@routes.post('/api/action')
@routes.post('/api/{action:(?:patrol|round_trip|straight_run)}')
async def handle_preset_actions(request):
    action = request.match_info.get('action')
    if not action:
        try:
            data = await request.json()
            action = data.get('action')
        except Exception:
            action = 'patrol'

    wps = get_waypoints_dict()
    print(f"⚡ [웹 요청] 프리셋 액션 실행: {action}")
    asyncio.create_task(run_action_queue(web_channel, [{"type": action}], wps))
    return web.json_response({"success": True, "action": action})


@routes.get('/api/map_image')
async def handle_map_image(request):
    for candidate in [
        os.path.join(MAPS_DIR, "my_map.png"),
        os.path.join(MAPS_DIR, "my_map.pgm"),
        os.path.join(os.path.expanduser("~"), "my_map.png"),
        os.path.join(os.path.expanduser("~"), "my_map.pgm")
    ]:
        if os.path.exists(candidate):
            return web.FileResponse(candidate)
    return web.Response(status=404, text="Map image not found")


@routes.get('/api/events')
async def handle_events_sse(request):
    res = web.StreamResponse(
        status=200, reason='OK',
        headers={'Content-Type': 'text/event-stream', 'Cache-Control': 'no-cache', 'Connection': 'keep-alive', 'Access-Control-Allow-Origin': '*'}
    )
    await res.prepare(request)
    queue = asyncio.Queue()
    sse_clients.add(queue)
    await res.write(b"event: ping\ndata: {}\n\n")

    try:
        while True:
            await res.write((await queue.get()).encode('utf-8'))
    except asyncio.CancelledError:
        pass
    finally:
        sse_clients.discard(queue)
    return res


def main():
    app = web.Application()
    app.add_routes(routes)
    if os.path.exists(WEB_APP_DIR):
        app.router.add_static('/', WEB_APP_DIR)

    port = 5000
    print("=" * 60)
    print(" 🚀 AMR 자율주행 관제 통합 웹 서버 가동")
    print(f" 🌐 접속 주소: http://localhost:{port}  (또는 http://<로봇IP>:{port})")
    print("=" * 60)
    web.run_app(app, host='0.0.0.0', port=port)


if __name__ == '__main__':
    main()
