---
name: navigate-amr
description: >-
  Commands the autonomous mobile robot (AMR) to navigate to specific named waypoints
  or coordinates, or execute test movement patterns (e.g. 1-second drive straight & 180-degree return).
  Use this skill whenever the user asks to move the robot, navigate to a location, go home, return to base,
  visit point A/B/C/D, run patrol missions, or test movements.
---

# AMR 자율주행 명령 및 웨이포인트 이동 스킬

이 스킬은 사용자가 자연어로 로봇에게 이동 명령을 내렸을 때, `run_xml_waypoints.py`, `navigate_to_coords.py`, 또는 `test_round_trip.py`를 실행하여 로봇을 목표 좌표로 자율주행시키거나 테스트 동작을 수행하는 절차를 정의합니다.

## 웨이포인트 목록 (`waypoints.xml`)

| ID | 장소 이름 | X (m) | Y (m) | Yaw (도) |
| :--- | :--- | :---: | :---: | :---: |
| `home` | 충전 거치대 / 홈 스테이션 | 2.50 | 2.00 | -116.5 |
| `point_a` | 지점 A | 1.00 | 1.00 | 0.0 |
| `point_b` | 지점 B | 0.00 | 2.00 | 90.0 |
| `point_c` | 지점 C | -2.00 | 3.50 | 0.0 |
| `origin` | 지도 절대 원점 | 0.00 | 0.00 | 0.0 |

## 실행 절차

### 1. 등록된 장소/ID로 이동 요청 시
사용자가 "지점 C로 가줘", "홈으로 가", "point_a로 이동해" 등 특정 장소를 언급한 경우:

```bash
python3 /home/juyeon/ros2_amr_ws/tools/run_xml_waypoints.py <ID>
```
* 예시: `python3 /home/juyeon/ros2_amr_ws/tools/run_xml_waypoints.py point_c`
* 예시: `python3 /home/juyeon/ros2_amr_ws/tools/run_xml_waypoints.py home`

### 1-1. Nav2 자율주행 후 AprilTag 마커로 최종 정밀 도킹 요청 시
사용자가 "홈으로 가서 도킹해줘", "충전하러 가", "Nav2로 이동 후 마커 도킹해"라고 한 경우:

```bash
python3 /home/juyeon/ros2_amr_ws/tools/navigate_and_dock.py <ID>
# 또는
python3 /home/juyeon/ros2_amr_ws/tools/run_xml_waypoints.py <ID> --dock
```
* 예시: `python3 /home/juyeon/ros2_amr_ws/tools/navigate_and_dock.py home`

### 1-2. 도킹 해제 및 안전 후진 (언도킹 / Undock) 요청 시
사용자가 "언도킹해줘", "도킹 해제해", "뒤로 빠져나와", "충전소에서 나와"라고 한 경우:

```bash
python3 /home/juyeon/ros2_amr_ws/tools/undock_amr.py
# 또는 180도 회전까지 포함
python3 /home/juyeon/ros2_amr_ws/tools/undock_amr.py --turn180
# 또는 ROS 2 노드 실행
ros2 run amr_bringup undock_node
```

### 2. 특정 임의 좌표 (X, Y)로 직접 이동 요청 시
사용자가 "X=1.2, Y=2.0으로 가줘"처럼 직접 좌표를 말한 경우:

```bash
python3 /home/juyeon/ros2_amr_ws/tools/navigate_to_coords.py <X> <Y> <Yaw>
```
* 예시: `python3 /home/juyeon/ros2_amr_ws/tools/navigate_to_coords.py 1.2 2.0 0`

### 3. 전체 순찰(Patrol) 요청 시
사용자가 "한 바퀴 돌고 와", "순찰해줘"라고 한 경우:

```bash
python3 /home/juyeon/ros2_amr_ws/tools/run_xml_waypoints.py patrol
```

### 4. 커스텀 스킬 왕복 테스트 (1초 직진 ➔ 180도 회전 ➔ 원위치 복귀) 요청 시
사용자가 "1초 직진했다 돌아와", "왕복 테스트해줘", "스킬 테스트 동작 실행해"라고 한 경우:

```bash
python3 /home/juyeon/ros2_amr_ws/tools/test_round_trip.py
```
