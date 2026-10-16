import asyncio
import bot_modules.ros2_runner as runner

# 글로벌 미션 대기 큐 및 실행 상태
_mission_queue = asyncio.Queue()
_pending_missions = []
_worker_task = None
_is_processing = False


def get_pending_count() -> int:
    """현재 대기 중인 미션 개수 반환"""
    return len(_pending_missions)


def clear_mission_queue():
    """대기 중인 모든 미션 큐 비우기 (긴급 정지 시 호출)"""
    global _pending_missions
    while not _mission_queue.empty():
        try:
            _mission_queue.get_nowait()
            _mission_queue.task_done()
        except Exception:
            pass
    _pending_missions.clear()


async def _execute_action_list(channel, actions: list, wps: dict):
    """단일 미션에 포함된 액션 리스트 순차 실행"""
    total_steps = len(actions)
    for idx, act in enumerate(actions, start=1):
        if runner.cancel_requested:
            break

        act_type = act.get("type")

        if act_type == "move":
            target = act.get("target")
            place_name = wps.get(target, {}).get("name", target)
            await channel.send(f"📋 **[미션 진행 {idx}/{total_steps}]** [{place_name}] 지점으로 이동합니다...")
            success = await runner.execute_single_move(channel, target, wps)
            if not success:
                return False

        elif act_type == "coords":
            x = float(act.get("x", 0.0))
            y = float(act.get("y", 0.0))
            yaw = float(act.get("yaw", 0.0))
            await channel.send(f"📋 **[미션 진행 {idx}/{total_steps}]** 좌표 ({x:.2f}, {y:.2f})로 이동합니다...")
            success = await runner.execute_coords_move(channel, x, y, yaw)
            if not success:
                return False

        elif act_type == "patrol":
            await channel.send(f"📋 **[미션 진행 {idx}/{total_steps}]** 전체 순찰을 시작합니다...")
            success = await runner.execute_patrol(channel)
            if not success:
                return False

        elif act_type == "round_trip":
            await channel.send(f"📋 **[미션 진행 {idx}/{total_steps}]** 왕복 테스트를 시작합니다...")
            success = await runner.execute_round_trip(channel)
            if not success:
                return False

        elif act_type == "straight_run":
            await channel.send(f"📋 **[미션 진행 {idx}/{total_steps}]** 1초 직진 테스트를 시작합니다...")
            success = await runner.execute_straight_run(channel)
            if not success:
                return False

        elif act_type == "wait":
            sec = float(act.get("seconds", 3.0))
            await channel.send(f"⏱️ **[정차 대기]** {sec}초 동안 대기합니다...")
            await asyncio.sleep(sec)

        elif act_type == "stop":
            await runner.stop_robot_helper(channel)
            return False

    return True


async def _mission_worker():
    """백그라운드에서 큐에 쌓인 미션을 하나씩 꺼내어 순차 실행하는 워커"""
    global _is_processing, _worker_task
    _is_processing = True

    try:
        while not _mission_queue.empty():
            if runner.cancel_requested:
                clear_mission_queue()
                break

            channel, actions, wps, desc = await _mission_queue.get()
            
            # 대기 목록에서 제거
            if _pending_missions:
                _pending_missions.pop(0)

            # 미션 실행
            runner.cancel_requested = False
            await _execute_action_list(channel, actions, wps)
            _mission_queue.task_done()

            # 긴급 정지가 들어왔다면 즉시 중단 및 큐 삭제
            if runner.cancel_requested:
                clear_mission_queue()
                break

            # 다음 미션이 남아있다면 안내
            if not _mission_queue.empty():
                next_count = len(_pending_missions)
                await channel.send(f"➡️ **[다음 미션 시작]** 대기 중이던 다음 미션을 이어 진행합니다. (남은 대기 미션: {next_count}개)")
                await asyncio.sleep(0.5)

    finally:
        _is_processing = False
        _worker_task = None


async def run_action_queue(channel, actions: list, wps: dict):
    """
    미션을 큐에 등록하고 워커를 통해 순차 실행.
    이미 다른 미션이 진행 중이면 큐에 자동 등록(Enqueue).
    """
    global _worker_task

    # 미션 설명 문자열 생성
    first_act = actions[0] if actions else {}
    target_id = first_act.get("target", "")
    target_name = wps.get(target_id, {}).get("name", target_id) if target_id else first_act.get("type", "미션")

    # 1. 큐에 미션 등록
    _pending_missions.append(target_name)
    await _mission_queue.put((channel, actions, wps, target_name))

    # 2. 이미 주행 중인 경우 대기열 안내 메시지 전송
    if _is_processing or (runner.current_process and runner.current_process.returncode is None):
        queue_len = len(_pending_missions)
        await channel.send(f"📥 **[대기열 추가]** `{target_name}` 미션이 큐에 등록되었습니다. 현재 미션 완료 후 자동 실행됩니다. (대기 순번: {queue_len}번째)")
        return

    # 3. 워커가 실행 중이 아니면 워커 시작
    if _worker_task is None or _worker_task.done():
        _worker_task = asyncio.create_task(_mission_worker())
