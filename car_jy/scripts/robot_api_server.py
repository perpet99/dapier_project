#!/usr/bin/env python3
"""
robot_api_server.py
--------------------
LLM 비의존형 순수 AMR 로봇 자율주행 REST API 관제 서버
- 포트 5000번 가동
- 순수 REST API (/api/robot/move, /api/robot/stop, /api/waypoints 등) 제공
- 향후 어떤 외부 LLM/에이전트든 이 REST API를 즉시 호출하여 로봇을 조종 가능
"""

import os
import sys
import time
import asyncio
from aiohttp import web

# bot_modules 임포트 경로 보장
_cur_dir = os.path.dirname(os.path.abspath(__file__))
_root_dir = os.path.dirname(_cur_dir) if os.path.basename(_cur_dir) == 'scripts' else _cur_dir
for _p in [_root_dir, os.path.expanduser("~/ros2_amr_ws")]:
    if os.path.exists(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from bot_modules.config import get_waypoints_dict
from bot_modules.ros2_runner import (
    stop_robot_helper,
    execute_single_move,
    execute_coords_move,
    execute_round_trip
)

WEB_APP_DIR = os.path.join(_root_dir, "web_app_rest")

# 실시간 로봇 주행 상태
robot_state = {
    "state": "idle",       # "idle", "moving", "stopped"
    "target": None,
    "message": "온라인 대기 중",
    "updated_at": time.time()
}


class ConsoleChannel:
    """터미널 및 로봇 상태 동기화 헬퍼"""
    async def send(self, content=None, embed=None):
        if content:
            clean = content.replace("**", "").replace("`", "")
            print(f"🤖 [Robot Core]: {clean}")
            robot_state["message"] = clean
            robot_state["updated_at"] = time.time()


channel = ConsoleChannel()


# --- REST API 엔드포인트 핸들러 ---

async def handle_index(request):
    """관제 대시보드 메인 HTML 서빙"""
    html_path = os.path.join(WEB_APP_DIR, "index.html")
    if os.path.exists(html_path):
        return web.FileResponse(html_path)
    return web.Response(text="index.html not found", status=404)


async def handle_get_waypoints(request):
    """등록된 웨이포인트(장소) 목록 조회 API"""
    wps = get_waypoints_dict()
    return web.json_response(wps)


async def handle_get_status(request):
    """현재 로봇 상태 조회 API"""
    return web.json_response(robot_state)


async def handle_stop(request):
    """긴급 정지 API (POST /api/robot/stop)"""
    robot_state["state"] = "stopped"
    robot_state["message"] = "🛑 긴급 정지 신호 전송됨"
    robot_state["target"] = None
    robot_state["updated_at"] = time.time()

    await stop_robot_helper(channel)
    return web.json_response({"success": True, "message": "로봇을 정지했습니다."})


async def handle_move(request):
    """
    지점 또는 임의 좌표 이동 명령 API (POST /api/robot/move)
    - Body 예시 1 (웨이포인트 ID): {"target": "point_a"}
    - Body 예시 2 (직접 좌표): {"x": 1.5, "y": -0.8, "yaw": 0.0}
    """
    try:
        data = await request.json()
    except Exception:
        return web.json_response({"success": False, "error": "유효한 JSON 바디가 필요합니다."}, status=400)

    target_id = data.get("target")
    wps = get_waypoints_dict()

    # 1. 단일 웨이포인트 ID 이동
    if target_id:
        target_clean = str(target_id).strip()
        if target_clean not in wps:
            # 이름으로 역추적 검색 지원
            found_id = None
            for wid, winfo in wps.items():
                if target_clean.lower() in winfo.get("name", "").lower():
                    found_id = wid
                    break
            if found_id:
                target_clean = found_id
            else:
                return web.json_response({
                    "success": False,
                    "error": f"등록되지 않은 장소입니다: '{target_id}'"
                }, status=404)

        place_name = wps[target_clean]["name"]
        robot_state["state"] = "moving"
        robot_state["target"] = target_clean
        robot_state["message"] = f"[{place_name}] 지점으로 자율주행 중..."
        robot_state["updated_at"] = time.time()

        async def _move_task():
            success = await execute_single_move(channel, target_clean, wps)
            robot_state["state"] = "idle"
            robot_state["target"] = None
            robot_state["message"] = f"[{place_name}] 도착 완료" if success else "주행 중단/취소됨"
            robot_state["updated_at"] = time.time()

        asyncio.create_task(_move_task())
        return web.json_response({
            "success": True,
            "message": f"[{place_name}] 지점으로 주행을 시작했습니다.",
            "target": target_clean
        })

    # 2. 임의 X, Y, Yaw 좌표 이동
    if "x" in data and "y" in data:
        try:
            x = float(data["x"])
            y = float(data["y"])
            yaw = float(data.get("yaw", 0.0))
        except (ValueError, TypeError):
            return web.json_response({"success": False, "error": "x, y 좌표는 숫자여야 합니다."}, status=400)

        robot_state["state"] = "moving"
        robot_state["target"] = f"({x:.2f}, {y:.2f})"
        robot_state["message"] = f"좌표 ({x:.2f}, {y:.2f}) 로 자율주행 중..."
        robot_state["updated_at"] = time.time()

        async def _coords_task():
            success = await execute_coords_move(channel, x, y, yaw)
            robot_state["state"] = "idle"
            robot_state["target"] = None
            robot_state["message"] = f"좌표 ({x:.2f}, {y:.2f}) 도달 완료" if success else "주행 중단/취소됨"
            robot_state["updated_at"] = time.time()

        asyncio.create_task(_coords_task())
        return web.json_response({
            "success": True,
            "message": f"좌표 (X={x:.2f}, Y={y:.2f}) 로 주행을 시작했습니다."
        })

    return web.json_response({
        "success": False,
        "error": "이동할 'target' 또는 'x', 'y' 좌표를 지정하세요."
    }, status=400)


async def handle_patrol(request):
    """전체 순찰 주행 API (POST /api/robot/patrol)"""
    wps = get_waypoints_dict()
    if not wps:
        return web.json_response({"success": False, "error": "등록된 웨이포인트가 없습니다."}, status=400)

    robot_state["state"] = "moving"
    robot_state["target"] = "patrol"
    robot_state["message"] = f"총 {len(wps)}개 지점 전체 순찰 주행 중..."
    robot_state["updated_at"] = time.time()

    async def _patrol_task():
        for wp_id, info in wps.items():
            if robot_state["state"] != "moving":
                break
            await execute_single_move(channel, wp_id, wps)
            await asyncio.sleep(info.get("wait_time", 2.0))
        robot_state["state"] = "idle"
        robot_state["target"] = None
        robot_state["message"] = "전체 순찰 주행 완료"
        robot_state["updated_at"] = time.time()

    asyncio.create_task(_patrol_task())
    return web.json_response({"success": True, "message": "전체 순찰 주행을 시작했습니다."})


async def handle_round_trip(request):
    """1초 직진 후 180도 회전 복귀 테스트 API (POST /api/robot/round_trip)"""
    robot_state["state"] = "moving"
    robot_state["target"] = "round_trip"
    robot_state["message"] = "1초 직진 왕복 테스트 주행 중..."
    robot_state["updated_at"] = time.time()

    async def _test_task():
        await execute_round_trip(channel)
        robot_state["state"] = "idle"
        robot_state["target"] = None
        robot_state["message"] = "왕복 테스트 주행 완료"
        robot_state["updated_at"] = time.time()

    asyncio.create_task(_test_task())
    return web.json_response({"success": True, "message": "왕복 테스트 주행을 시작했습니다."})


# --- 서버 인스턴스 생성 ---

def create_app():
    app = web.Application()
    # 웹 페이지 서빙
    app.router.add_get('/', handle_index)

    # 순수 로봇 REST API 라우팅
    app.router.add_get('/api/waypoints', handle_get_waypoints)
    app.router.add_get('/api/robot/status', handle_get_status)
    app.router.add_post('/api/robot/move', handle_move)
    app.router.add_post('/api/robot/stop', handle_stop)
    app.router.add_post('/api/robot/patrol', handle_patrol)
    app.router.add_post('/api/robot/round_trip', handle_round_trip)

    # 정적 리소스 (style.css, app.js) 서빙
    if os.path.exists(WEB_APP_DIR):
        app.router.add_static('/', WEB_APP_DIR)

    return app


def main():
    port = 5000
    print("=" * 68)
    print(" 🤖 AMR 로봇 관제 REST API 서버 가동 (LLM 비의존형 독립 구동)")
    print(f" 🌐 대시보드 접속 주소: http://localhost:{port}  (또는 http://<로봇IP>:{port})")
    print(" 📡 제공 API: /api/robot/move, /api/robot/stop, /api/waypoints 등")
    print("=" * 68)

    app = create_app()
    web.run_app(app, host='0.0.0.0', port=port)


if __name__ == '__main__':
    main()
