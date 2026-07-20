# Nav2 + SLAM Toolbox Tutorial Checklist (WSL, ROS 2 Jazzy)

아래 순서대로 그대로 실행하면 Nav2 튜토리얼의 SLAM 흐름을 재현할 수 있습니다.

## 0) 사전 확인

- [x] Ubuntu 24.04 (WSL)
- [x] ROS 2 Jazzy 설치
- [x] SLAM Toolbox 설치
- [x] Nav2 / TurtleBot3 시뮬레이션 패키지 설치

설치 확인 명령:

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg list | grep -E 'slam_toolbox|nav2_bringup|turtlebot3_gazebo|turtlebot3_navigation2|turtlebot3_teleop'
```

## 1) 기존 실행 정리 (권장)

- [ ] 기존에 실행 중인 nav2/gz 프로세스 정리

```bash
pkill -f tb3_simulation_launch.py || true
pkill -f slam_toolbox || true
pkill -f gz || true
```

## 2) Nav2 + SLAM 시뮬레이션 실행

- [ ] 터미널 A에서 시뮬레이션 실행

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True headless:=True use_rviz:=False
```

GUI를 쓰려면:

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True headless:=False use_rviz:=True
```

성공 기준:

- [ ] 로그에 lifecycle_manager_navigation의 Managed nodes are active 표시

## 3) 텔레옵으로 맵 생성

- [ ] 터미널 B를 새로 열고 텔레옵 실행

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

- [ ] 로봇을 맵 전역으로 충분히 이동 (복도/방 경계까지)

기본 키:

- [ ] i / j / l / , 로 이동/회전
- [ ] k 로 정지

## 4) 맵 저장

- [ ] 터미널 C에서 맵 저장

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

성공 기준:

- [ ] ~/maps/my_map.yaml 생성
- [ ] ~/maps/my_map.pgm 생성

확인:

```bash
ls -lh ~/maps/my_map.yaml ~/maps/my_map.pgm
```

## 5) SLAM 모드 종료 후 저장 맵으로 내비게이션 실행

- [ ] 터미널 A에서 Ctrl+C 로 종료
- [ ] 저장 맵 기반으로 재실행

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch nav2_bringup tb3_simulation_launch.py slam:=False map:=$HOME/maps/my_map.yaml headless:=True use_rviz:=False
```

GUI 사용 시:

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch nav2_bringup tb3_simulation_launch.py slam:=False map:=$HOME/maps/my_map.yaml headless:=False use_rviz:=True
```

## 6) 내비게이션 테스트

- [ ] (RViz 사용 시) 2D Pose Estimate로 초기 자세 설정
- [ ] Nav2 Goal로 목표점 지정
- [ ] 로봇이 장애물 회피하며 목표점까지 도달하는지 확인

## 7) 트러블슈팅

- [ ] map 토픽이 비어 있으면: 텔레옵으로 더 이동해서 스캔 누적
- [ ] tf 경고가 잠깐 보이는 것은 초기화 구간에서 흔함
- [ ] GUI가 안 뜨면 headless 모드로 먼저 동작 검증

