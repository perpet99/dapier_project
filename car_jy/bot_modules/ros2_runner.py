import sys
import asyncio
import subprocess
from bot_modules.config import RUNNER_PATH, COORDS_PATH, TEST_ROUND_TRIP_PATH

# 현재 진행 중인 주행 서브프로세스 및 큐 취소 플래그
current_process = None
cancel_requested = False


import os
import signal

def emergency_stop_cmd_vel():
    """ROS2 /cmd_vel 강제 0 정지 신호 전송, Nav2 액션 취소 및 잔류 프로세스 초고속 정리"""
    # 1. 실행 중인 네비게이션/테스트 파이썬 스크립트 즉시 강제 종료 (SIGKILL)
    for target in ["run_xml_waypoints.py", "navigate_to_coords.py", "test_round_trip.py", "test_straight_run.py"]:
        try:
            subprocess.run(["pkill", "-9", "-f", target], capture_output=True, timeout=0.5)
        except Exception:
            pass

    # 2. Nav2 액션 취소 CLI 및 /cmd_vel 0 신호 다량 전송
    try:
        # Nav2 Goal Cancel
        subprocess.run(
            ["bash", "-c", "source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null; ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{}' --feedback >/dev/null 2>&1 &"],
            timeout=0.2
        )
    except Exception:
        pass



async def stop_robot_helper(channel):
    """긴급 정지 헬퍼 함수"""
    global current_process, cancel_requested
    cancel_requested = True
    
    # 대기 큐 전체 삭제
    try:
        from bot_modules.mission_queue import clear_mission_queue
        clear_mission_queue()
    except Exception:
        pass

    # 실행 중인 current_process 정지 신호 전송
    if current_process:
        try:
            current_process.terminate()
            await asyncio.sleep(0.05)
            if current_process.returncode is None:
                current_process.kill()
        except Exception:
            pass
        current_process = None

    emergency_stop_cmd_vel()
    await channel.send("🛑 **로봇 정지 신호를 전송하고 대기 중인 모든 미션을 중단했습니다!**")


async def execute_single_move(channel, wp_id: str, wps: dict) -> bool:
    """단일 지점 이동 수행"""
    global current_process, cancel_requested
    if cancel_requested:
        return False

    if wp_id not in wps:
        await channel.send(f"❌ 등록되지 않은 장소 ID입니다: `{wp_id}`")
        return False

    place_name = wps[wp_id]['name']
    await channel.send(f"🚀 **[{place_name}]** (`{wp_id}`) 지점으로 자율주행을 출발합니다! 🚗💨")

    current_process = await asyncio.create_subprocess_exec(
        sys.executable, RUNNER_PATH, wp_id,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE
    )

    stdout, stderr = await current_process.communicate()

    if cancel_requested:
        return False

    if current_process.returncode == 0:
        await channel.send(f"🎉 **[{place_name}]** 도착 완료! 🏁")
        return True
    else:
        await channel.send(f"⚠️ **[{place_name}]** 주행이 중단되었거나 취소되었습니다.")
        return False


async def execute_coords_move(channel, x: float, y: float, yaw: float) -> bool:
    """임의 좌표 이동 수행"""
    global current_process, cancel_requested
    if cancel_requested:
        return False

    await channel.send(f"📍 좌표 (X={x:.2f}, Y={y:.2f}, Yaw={yaw:.1f}°)로 출발합니다! 🚗💨")

    current_process = await asyncio.create_subprocess_exec(
        sys.executable, COORDS_PATH, str(x), str(y), str(yaw),
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE
    )

    stdout, stderr = await current_process.communicate()

    if cancel_requested:
        return False

    if current_process.returncode == 0:
        await channel.send(f"🎉 목표 좌표 (X={x:.2f}, Y={y:.2f}) 도달 완료! 🏁")
        return True
    else:
        await channel.send("⚠️ 좌표 이동 주행이 중단되었거나 취소되었습니다.")
        return False


async def execute_patrol(channel) -> bool:
    """순찰 주행 수행"""
    global current_process, cancel_requested
    if cancel_requested:
        return False

    await channel.send("🚨 **[전체 경유지 순찰 주행]**을 시작합니다! 🚗💨")

    current_process = await asyncio.create_subprocess_exec(
        sys.executable, RUNNER_PATH, "patrol",
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE
    )

    stdout, stderr = await current_process.communicate()

    if cancel_requested:
        return False

    if current_process.returncode == 0:
        await channel.send("🎉 **전체 순찰 코스를 무사히 완주했습니다!** 🏁")
        return True
    else:
        await channel.send("⚠️ 순찰 주행이 중단되었거나 취소되었습니다.")
        return False


async def execute_round_trip(channel) -> bool:
    """왕복 테스트 주행 수행 (1초 직진 -> 180도 회전 -> 복귀)"""
    global current_process, cancel_requested
    if cancel_requested:
        return False

    await channel.send("🔄 **[왕복 테스트 주행]**을 시작합니다! (1초 직진 ➔ 180도 회전 ➔ 원위치 복귀) 🚗💨")

    current_process = await asyncio.create_subprocess_exec(
        sys.executable, TEST_ROUND_TRIP_PATH,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE
    )

    stdout, stderr = await current_process.communicate()

    if cancel_requested:
        return False

    if current_process.returncode == 0:
        await channel.send("🎉 **왕복 테스트 주행을 성공적으로 완료했습니다!** 🏁")
        return True
    else:
        await channel.send("⚠️ 왕복 테스트 주행이 중단되었거나 취소되었습니다.")
        return False


async def execute_straight_run(channel) -> bool:
    """순수 1초 직진 테스트 주행 수행"""
    global current_process, cancel_requested
    if cancel_requested:
        return False

    await channel.send("🏎️ **[1초 직진 테스트]**를 시작합니다! (1초 전진 후 즉시 정지) 🚗💨")

    current_process = await asyncio.create_subprocess_exec(
        sys.executable, TEST_ROUND_TRIP_PATH, "--straight-only",
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE
    )

    stdout, stderr = await current_process.communicate()

    if cancel_requested:
        return False

    if current_process.returncode == 0:
        await channel.send("🎉 **1초 직진 테스트 주행을 완료했습니다!** 🏁")
        return True
    else:
        await channel.send("⚠️ 직진 테스트 주행이 중단되었거나 취소되었습니다.")
        return False
