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

## 3-1) 목표 위치 이동 서비스 (go_to_pose)

`x, y, yaw`를 파라미터로 받아 Nav2로 해당 위치까지 이동시키는 ROS 2 서비스 노드입니다.
`myrobot_interfaces` (커스텀 `GoToPose.srv`)와 `myrobot_nav_service` (서비스 노드) 두 패키지로 구성되어 있으며,
다른 스크립트들과 달리 최초 1회 colcon build가 필요합니다.

### 빌드 (최초 1회, 또는 코드 변경 후)

```bash
cd ~/.../myrobot_ws
export ROS_DISTRO=jazzy
./scripts/build_ws.sh
```

### 서비스 노드 실행

Nav2가 이미 실행 중이어야 합니다 (`./scripts/tb3_navigate_map.sh start maps/tb3_map.yaml`).

`go_to_pose_service`는 시작할 때 AMCL 초기 위치(`/initialpose`)를 자동으로 퍼블리시합니다
(기본값 `x=0.0, y=0.0, yaw=0.0`, 0.5초 간격 3회 재전송). 초기 위치를 설정하지 않으면
`map → base_link` TF를 구할 수 없어 `go_to_pose` 호출 시
`success=False message=Goal rejected by navigate_to_pose`가 반환되므로, 로봇의 실제 시작 위치가
기본값과 다르면 파라미터로 지정하세요.

터미널 D:

```bash
cd ~/.../myrobot_ws
export ROS_DISTRO=jazzy
./scripts/tb3_goto_pose_service.sh
# 로봇 시작 위치가 원점이 아니면:
# ./scripts/tb3_goto_pose_service.sh --ros-args -p initial_pose_x:=1.0 -p initial_pose_y:=0.5 -p initial_pose_yaw:=1.57
```

이미 RViz "2D Pose Estimate"로 초기 위치를 맞췄다면 자동 퍼블리시를 꺼도 됩니다.

```bash
./scripts/tb3_goto_pose_service.sh --ros-args -p set_initial_pose:=false
```

### 목표 위치로 이동 요청

터미널 E:

```bash
cd ~/.../myrobot_ws
export ROS_DISTRO=jazzy
./scripts/tb3_goto_pose.sh 1.0 0.5 1.57
```

`ros2 service call`로 직접 호출할 수도 있습니다.

```bash
ros2 service call /go_to_pose myrobot_interfaces/srv/GoToPose "{x: 1.0, y: 0.5, yaw: 1.57}"
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

## 5) 실행 중인 프로세스 종료

시뮬레이션/Nav2/teleop 등을 백그라운드로 여러 터미널에서 띄운 뒤 한 번에 정리하려면 아래 스크립트를 사용합니다.

```bash
./scripts/tb3_kill.sh
```

`gzserver`, `gzclient`, `rviz2`, `nav2_*`, `slam_toolbox`, `turtlebot3_teleop`, `nav_goal_runner.py` 등 관련 프로세스를 찾아 먼저 SIGINT로 정상 종료를 시도하고, 2초 후에도 남아있으면 SIGKILL로 강제 종료합니다.

## 6) WSL GUI 참고

- Windows 11 + WSLg면 Gazebo/RViz GUI가 기본적으로 표시됩니다.
- GUI가 안 뜨면 X 서버/WSLg 설정 상태를 먼저 확인하세요.

## 7) 트러블슈팅

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

4. Subscriber count가 0이 아닌데도 안 움직이면 메시지 타입 불일치를 의심하세요.

`turtlebot3_teleop`의 `teleop_keyboard`는 `ROS_DISTRO`가 정확히 `humble`이 아니면 `/cmd_vel`에 `TwistStamped`를 발행합니다. 이 Nav2/Gazebo 시뮬레이션의 브리지는 `/cmd_vel`을 순수 `Twist` 타입으로 구독하므로, 타입이 다르면 연결 자체가 안 되어 키를 눌러도 로봇이 조용히 움직이지 않습니다. `tb3_teleop.sh`는 이를 우회하기 위해 teleop 실행 시에만 `ROS_DISTRO=humble`로 오버라이드해서 `Twist` 타입으로 발행하도록 되어 있습니다. 직접 `ros2 run turtlebot3_teleop teleop_keyboard`로 실행했다면 아래처럼 실행하세요.

```bash
ROS_DISTRO=humble ros2 run turtlebot3_teleop teleop_keyboard
```

간단 점검 + teleop 실행은 helper 스크립트를 권장합니다.

```bash
export ROS_DISTRO=jazzy
./scripts/tb3_teleop.sh
```

`./scripts/tb3_goto_pose.sh` 실행 시 `success=False message=Goal rejected by navigate_to_pose`가 나오면,
AMCL 초기 위치가 로봇의 실제 시작 위치와 맞지 않아 `map → base_link` TF를 구하지 못해 Nav2가 목표를 즉시 거부한 것입니다.
`go_to_pose_service`는 시작 시 `initial_pose_x/y/yaw` 파라미터(기본 0,0,0)로 `/initialpose`를 자동 퍼블리시하므로,
로봇 시작 위치가 원점이 아니라면 서비스 노드 실행 시 파라미터로 맞는 값을 넘기거나 RViz "2D Pose Estimate"로 다시 지정한 뒤 재시도하세요.

```bash
./scripts/tb3_goto_pose_service.sh --ros-args -p initial_pose_x:=1.0 -p initial_pose_y:=0.5 -p initial_pose_yaw:=1.57
```
