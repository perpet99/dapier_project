# TurtleBot3 Nav2 Mapping & Navigation Scripts (WSL)

이 구성은 Nav2 Getting Started 문서 흐름을 기준으로 만들었습니다.

- 맵 수집: SLAM 실행 + 수동 주행 후 맵 저장
- 맵 기반 이동: 저장된 맵으로 Nav2 실행 + 목표점 자동 순차 이동

## 1) WSL 준비

WSL(Ubuntu)에서 아래처럼 ROS 환경을 준비합니다.

```bash
export ROS_DISTRO=jazzy
source /opt/ros/$ROS_DISTRO/setup.bash
```

패키지 설치:

```bash
cd ~/.../myrobot_ws
chmod +x scripts/*.sh
./scripts/setup_wsl_nav2_tb3.sh
```

### setup_wsl_nav2_tb3.sh 사용 예제

기본 실행 예제:

```bash
cd ~/.../myrobot_ws
export ROS_DISTRO=jazzy
source /opt/ros/$ROS_DISTRO/setup.bash
./scripts/setup_wsl_nav2_tb3.sh
```

다른 배포판 예제(humble):

```bash
cd ~/.../myrobot_ws
export ROS_DISTRO=humble
source /opt/ros/$ROS_DISTRO/setup.bash
./scripts/setup_wsl_nav2_tb3.sh
```

설치 후 패키지 확인 예제:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 pkg list | grep -E "nav2|turtlebot3|slam_toolbox" | head
```

실행 중 `ROS_DISTRO is not set` 오류가 나면 아래를 먼저 실행하세요:

```bash
export ROS_DISTRO=jazzy
source /opt/ros/$ROS_DISTRO/setup.bash
```

## 2) 맵 수집

터미널 A:

```bash
cd ~/.../myrobot_ws
export ROS_DISTRO=jazzy
source /opt/ros/$ROS_DISTRO/setup.bash
export TURTLEBOT3_MODEL=waffle
./scripts/tb3_collect_map.sh start
```

터미널 B (주행):

```bash
cd ~/.../myrobot_ws
export ROS_DISTRO=jazzy
./scripts/tb3_teleop.sh
```

맵 저장 (터미널 C):

```bash
cd ~/.../myrobot_ws
source /opt/ros/$ROS_DISTRO/setup.bash
./scripts/tb3_collect_map.sh save maps/tb3_map
```

결과 파일:
- `maps/tb3_map.yaml`
- `maps/tb3_map.pgm`

## 3) 저장된 맵 기반 네비게이션 실행

터미널 A (Nav2 + map):

```bash
cd ~/.../myrobot_ws
export ROS_DISTRO=jazzy
source /opt/ros/$ROS_DISTRO/setup.bash
export TURTLEBOT3_MODEL=waffle
./scripts/tb3_navigate_map.sh start maps/tb3_map.yaml
```

RViz에서 초기 위치를 맞추거나, `config/nav_goals.yaml`의 `initial_pose`를 사용합니다.

터미널 B (자동 목표점 이동):

```bash
cd ~/.../myrobot_ws
source /opt/ros/$ROS_DISTRO/setup.bash
./scripts/tb3_navigate_map.sh run-goals config/nav_goals.yaml
```

## 4) 목표점 파일 형식

`config/nav_goals.yaml` 예시:

```yaml
frame_id: map
initial_pose:
  x: 0.0
  y: 0.0
  yaw: 0.0
goals:
  - x: 1.0
    y: 0.0
    yaw: 0.0
  - x: 1.0
    y: 1.0
    yaw: 1.57
```

## 5) WSL GUI 참고

- Windows 11 + WSLg면 Gazebo/RViz GUI가 기본적으로 표시됩니다.
- GUI가 안 뜨면 X 서버/WSLg 설정 상태를 먼저 확인하세요.

## 6) 트러블슈팅

`Failed to spin map subscription` 오류가 나오면 아래 순서로 점검하세요.

1. 맵 저장 전에 반드시 매핑 세션이 실행 중이어야 합니다.

```bash
./scripts/tb3_collect_map.sh start
```

2. 다른 터미널에서 로봇을 조금 이동시켜 실제 맵이 생성되도록 합니다.

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

3. 맵 토픽 데이터가 나오는지 확인합니다.

```bash
ros2 topic echo /map --once
```

4. 그 다음 저장합니다.

```bash
./scripts/tb3_collect_map.sh save maps/tb3_map
```

필요하면 토픽/시뮬레이션 시간 옵션을 지정할 수 있습니다.

```bash
MAP_TOPIC=/map USE_SIM_TIME=True ./scripts/tb3_collect_map.sh save maps/tb3_map
```

teleop 실행 시 `KeyError: 'TURTLEBOT3_MODEL'` 오류가 나오면 환경변수가 없는 상태입니다.
아래 중 하나로 해결하세요.

```bash
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

또는 helper 스크립트 사용:

```bash
export ROS_DISTRO=jazzy
./scripts/tb3_teleop.sh
```

키 입력이 되는데 로봇이 안 움직이면 아래를 점검하세요.

1. Gazebo가 일시정지 상태인지 확인 (상단 Play 버튼으로 재개)
2. teleop 실행 터미널 창에 포커스를 둔 상태에서 키 입력
3. `/cmd_vel` subscriber 수 확인

```bash
ros2 topic info /cmd_vel
```

`Subscriber count: 0`이면 시뮬레이터/컨트롤러 쪽이 안 붙은 상태입니다.
매핑 런치를 다시 실행하세요.

```bash
./scripts/tb3_collect_map.sh start
```

간단 점검 + teleop 실행은 helper 스크립트를 권장합니다.

```bash
export ROS_DISTRO=jazzy
./scripts/tb3_teleop.sh
```
