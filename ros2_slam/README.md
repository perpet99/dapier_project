# ros2_slam — car2 + Depth 카메라 SLAM / Nav2 (ROS 2 Jazzy)

`car2`(ESP32 + ST3215 서보 4개, 스키드 스티어) 실물 로봇에 Orbbec Astra Pro
depth 카메라를 얹어 ROS 2 Jazzy로 맵을 수집/저장하고, 저장된 맵으로 Nav2
내비게이션을 돌리기 위한 워크스페이스입니다. 시뮬레이션이 아니라 **실제
하드웨어** 대상입니다.

## 구성

```
ros2_slam/
  src/
    car2_driver/     # cmd_vel -> car2 시리얼 프로토콜, 서보 피드백 -> odom/TF
    car2_bringup/     # launch(bringup/mapping/navigation) + slam_toolbox/Nav2 설정
  maps/                # map_saver_cli로 저장한 맵 (yaml + pgm)
  scripts/             # build_ws.sh, run_mapping.sh, save_map.sh, run_navigation.sh
```

### 파이프라인

```
depth 카메라(OpenNI2) --depth image--> depthimage_to_laserscan --/scan-->
  slam_toolbox (매핑) 또는 nav2 AMCL (내비게이션)
car2_driver: /cmd_vel -> 시리얼(D,<left>,<right>) / STAT 피드백 -> /odom, odom->base_link TF
```

- `car2_driver`가 발행하는 `/odom` + `base_to_camera_tf`(정적) + depth 카메라가
  발행하는 `camera_link` 하위 TF들이 합쳐져 `map -> odom -> base_link ->
  camera_link -> ... -> camera_depth_frame`까지 이어지고, 여기서 `/scan`이
  나옵니다. 라이다 없이 depth 카메라만으로 2D SLAM/Nav2를 돌리는 표준적인
  방식입니다.

## 카메라 드라이버: Orbbec Astra Pro는 Orbbec 자체 OpenNI2 런타임이 필요함

Ubuntu apt의 `ros-jazzy-openni2-camera` + `libopenni2-0`(PrimeSense 계열
PS1080 드라이버)만으로는 이 카메라(`lsusb`상 `2bc5:060f` Depth Sensor +
`2bc5:050f` USB 2.0 Camera — RGB가 UVC로 분리 노출되는 Astra Pro 구조)가
**인식되지 않습니다** (`list_devices` → 0 devices). Ubuntu가 배포하는
드라이버가 Orbbec의 OEM 벤더 ID를 지원하지 않기 때문이며
(`hand_eye/test4.py`의 Windows 쪽 메모에 있는 "Orbbec SDK v2는 이 모델을
지원하지 않는다"와 같은 종류의 벤더 호환성 문제), udev 권한 문제가 아닙니다.

**해결: Orbbec이 공식 배포하는 OpenNI2 Linux SDK**
(https://www.orbbec.com/developers/openni-sdk/ → `dl.orbbec3d.com`)에는
Orbbec 벤더 ID가 포함된 자체 `liborbbec.so` 드라이버 + `libOpenNI2.so`
코어가 들어있고, 이 카메라를 정상적으로 찾아 스트리밍합니다. 아래 스크립트가
필요한 부분(코어 라이브러리 + 드라이버 .so, 약 2.5MB)만 내려받아
`third_party/orbbec_openni2/`에 설치하고, 공식 udev 규칙도 등록합니다.

```bash
cd ros2_slam
./scripts/install_orbbec_openni2.sh
```

`bringup.launch.py`는 `openni2_lib_override_dir` 인자(기본값이 바로 이
`third_party/orbbec_openni2/` 경로)로 카메라 프로세스의 `LD_LIBRARY_PATH`
맨 앞에 이 런타임을 끼워 넣어서, `ros-jazzy-openni2-camera` 패키지를
재빌드하지 않고도 Orbbec의 라이브러리로 카메라를 인식하게 합니다. 스크립트를
아직 안 돌렸으면 이 경로가 없으므로 조용히 시스템 기본 라이브러리로
폴백합니다(= 다시 0 devices).

Orbbec의 공식 ROS2 저장소(`orbbec/ros_astra_camera`의 `ros2-development`
브랜치)는 README에 "not stable, please do not use it"이라고 명시되어 있어
사용하지 않았습니다. 대신 기존 `ros-jazzy-openni2-camera` 패키지(ROS
표준 `sensor_msgs/Image` 토픽, `depthimage_to_laserscan`과 바로 호환)에
Orbbec의 런타임만 얹는 방식을 택했습니다.

또한 이 드라이버 버전은 `depth_registration=True`일 때 Astra Pro의 UVC
컬러 스트림을 "640x480@30Hz RGB888"로 열려고 시도하는데, 이 모드를 카메라가
지원하지 않아 컬러 스트림이 실패합니다(로그에 `Unsupported color video
mode` 에러가 찍히지만 무해합니다 — 컬러/뎁스 모드 설정은 각각 독립적으로
처리되어 뎁스에는 영향 없음). 2D 라이다 대체용으로는 컬러 정렬이 필요
없으므로 `bringup.launch.py`는 `depth_registration=False`로 카메라를
직접 구성하고, 미등록 뎁스 스트림 토픽 `depth_raw/image` +
`depth_raw/camera_info`를 구독합니다(등록된 `depth/image`가 아님 — 이
드라이버 버전에는 `depth/image_raw`라는 이름의 토픽 자체가 없습니다).

## 사전 준비

```bash
export ROS_DISTRO=jazzy
sudo apt update
sudo apt install -y ros-jazzy-openni2-camera ros-jazzy-depthimage-to-laserscan \
  ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-teleop-twist-keyboard python3-serial
```

Orbbec 카메라 런타임 + udev 규칙 설치(이미 이 환경엔 적용됨, 새 PC에서
다시 하려면):

```bash
cd ros2_slam
./scripts/install_orbbec_openni2.sh
```

car2 ESP32는 USB(Type-C, CP2102)로 이 PC에 연결합니다. 보통
`/dev/ttyUSB0`로 잡히고, `dialout` 그룹 소속이어야 포트를 열 수 있습니다
(`sudo usermod -aG dialout $USER` 후 재로그인).

빌드:

```bash
cd ros2_slam
./scripts/build_ws.sh
```

## ⚠️ 반드시 실측 후 조정해야 하는 값

이 리포에는 car2의 실제 치수(바퀴 반지름, 좌우 바퀴 간격, 카메라 장착
위치)가 없어서 아래 값들은 **전부 플레이스홀더**입니다. 실측 없이 그대로
쓰면 odometry가 틀리고 Nav2가 엉뚱하게 움직입니다.

| 값 | 위치 | 기본값(placeholder) | 비고 |
|---|---|---|---|
| `wheel_radius_m` | launch 인자 / `car2_driver` 파라미터 | 0.032 | 바퀴 반지름 실측 |
| `wheel_separation_m` | launch 인자 | 0.18 | 좌/우 바퀴 중심 간 거리 실측 |
| `base_to_camera_*` (x,y,z,roll,pitch,yaw) | `bringup.launch.py` 인자 | x=0.08, z=0.15, 나머지 0 | base_link(바퀴 중심 지면 투영점) -> 카메라 광학 중심 실측 |
| `robot_radius` | `config/nav2_params.yaml` | 0.15 | 로봇 풋프린트 반지름 실측 |
| 속도 제한(vx_max 등) | `config/nav2_params.yaml` | 보수적으로 낮게 설정 | 실측 후 올리기 |

`steps_per_rev`(4096)와 `max_step_speed`(3400)는 `car2/car2/Car.h`의
ST3215 스펙에서 그대로 가져온 값이라 바꿀 필요 없습니다.

`car2_driver`는 `STAT` 명령으로 받은 서보 실측 속도(spd)를 이용해
odometry를 적분합니다(위치값 pos는 회전마다 랩어라운드되는 원시 인코더
값이라 그대로 적분할 수 없어서, 속도 기반 dead-reckoning을 사용).
`car2/car2/Car.cpp`의 `INVERT_DEFAULT`가 바뀌면 `car2_driver/car2_serial_node.py`
상단의 `INVERT` 딕셔너리도 같이 맞춰야 합니다.

## 사용법

### 1) 맵 수집

터미널 A:
```bash
cd ros2_slam
./scripts/run_mapping.sh /dev/ttyUSB0
```

터미널 B (주행):
```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

터미널 C (맵이 충분히 쌓이면 저장):
```bash
cd ros2_slam
./scripts/save_map.sh my_map
```
결과: `maps/my_map.yaml`, `maps/my_map.pgm`

### 2) 저장된 맵으로 내비게이션

```bash
cd ros2_slam
./scripts/run_navigation.sh maps/my_map.yaml /dev/ttyUSB0
```

RViz에서:
1. **2D Pose Estimate**로 로봇의 실제 시작 위치/방향을 지정.
2. **Nav2 Goal**로 목적지를 지정하면 자동 주행.

`myrobot_ws`의 `go_to_pose_service` 패턴처럼 좌표를 코드/서비스로 넘기고
싶다면 `nav2_simple_commander`(이미 설치됨)를 사용하는 작은 스크립트를
추가하면 됩니다 (`myrobot_ws/src/myrobot_nav_service` 참고).

## 트러블슈팅

- **`Failed to find a free participant index for domain N` / 노드 생성 실패**:
  `run_mapping.sh`/`run_navigation.sh`/`save_map.sh`는 기본적으로
  `ROS_DOMAIN_ID=42`를 씁니다(기본 도메인 0이나, 이전 세션이 정리 안 되고
  남은 도메인과의 충돌을 피하기 위함). 그래도 이 에러가 나면 뭔가가 42번도
  이미 채워놓은 것이므로, 다른 값으로 바꾸세요: 세 스크립트 모두 실행 전에
  `export ROS_DOMAIN_ID=123` 처럼 원하는 값을 먼저 export하면 그 값을
  우선 사용합니다 (단, 같은 실행 흐름 안의 모든 터미널에 동일하게 적용해야
  서로 통신됩니다 - teleop 터미널 포함). 좀비 프로세스가 쌓인 게
  의심되면 `pkill -9 -f "component_container|ros2-daemon"`로 정리하세요.
- **`ros2 run openni2_camera list_devices`가 0 devices**: `./scripts/install_orbbec_openni2.sh`를
  실행했는지 확인하세요. 실행 후에도 0이면
  `LD_LIBRARY_PATH=ros2_slam/third_party/orbbec_openni2 ros2 run openni2_camera list_devices`로
  override가 실제로 적용됐는지 직접 확인해보세요.
- **`/scan`이 비어 있는데 `component_container`는 떠 있음**: 카메라가 실제로
  depth 이미지를 발행하는지 `ros2 topic hz /camera/depth_raw/image`로 먼저
  확인하세요. USB 허브 체인이나 다른 OpenNI2 프로세스와의 경합으로 이 카메라는
  가끔 첫 시도에 조용히 응답하지 않을 수 있습니다 — 그럴 땐
  `ros2 run openni2_camera usb_reset /dev/bus/usb/<bus>/<dev>`로 리셋
  (`lsusb`에서 Orbbec Depth Sensor의 버스/디바이스 번호 확인) 후 재실행하세요.
- **로봇이 타임아웃으로 멈춤(`TIMEOUT,STOP`)**: `car2_driver`가 살아있는지,
  `/cmd_vel` 구독자가 붙어 있는지(`ros2 topic info /cmd_vel`) 확인하세요.
  car2.ino는 마지막 명령 후 500ms 안에 새 명령이 없으면 자동 정지합니다.
- **포트 권한 오류(`/dev/ttyUSB0` Permission denied)**: `sudo usermod -aG
  dialout $USER` 후 재로그인.
- **Nav2가 목표를 바로 거부**: AMCL 초기 위치가 안 맞아 `map -> base_link`
  TF가 없는 상태입니다. RViz "2D Pose Estimate"로 먼저 위치를 맞추세요.
