# 주연테크 프로젝트 — 말로 시키는 이동형 집기 로봇

음성이나 글로 "책상으로 가서 컵을 집어"라고 지시하면 로봇이 움직입니다. 2D 라이다 지도로 목적지까지 자율 주행하고, depth 카메라로 물체를 찾아 SO-101 로봇팔로 집습니다.

이 문서는 처음 합류한 사람이 **하드웨어를 꾸미고, 소프트웨어를 설치하고, 전체를 실행**하기까지의 순서를 담습니다. 세부 내용은 하위 폴더 문서로 연결합니다.

| 폴더 | 내용 | 상세 문서 |
|---|---|---|
| [car2/](car2/) | 4륜 주행 베이스 펌웨어(ESP32 + ST3215 서보 4개)와 단독 테스트 도구 | [car2/readme.md](car2/readme.md) |
| [ros2_slam/](ros2_slam/) | ROS 2 Jazzy 워크스페이스: 차량 드라이버, 라이다 드라이버, SLAM, Nav2, 지도 웹 UI | [ros2_slam/README.md](ros2_slam/README.md) |
| [hand_eye/](hand_eye/) | SO-101 팔 캘리브레이션, 클릭 집기(`robot_arm.py`), Gemini 음성/글 지시 클라이언트(`llm_client.py`) | [hand_eye/readme.md](hand_eye/readme.md) |
| [car_jy/](car_jy/) | 별도 AMR 프로젝트(다른 팀원 작업). 이 문서의 실행 흐름에는 쓰이지 않음 | [car_jy/README.md](car_jy/README.md) |

---

## 1. 전체 구조

계산이 무거운 SLAM과 Nav2는 **노트북**에서 돌리고, 센서와 모터는 로봇 위의 **라즈베리파이**가 맡습니다. 둘은 같은 Wi-Fi에서 ROS 2 토픽(CycloneDDS)과 HTTP로 통신합니다.

```
┌──────────────────────── 노트북 (Ubuntu 24.04 + ROS 2 Jazzy) ────────────────────────┐
│  run_lidar_navigation.sh : map_server + AMCL + Nav2  ──/cmd_vel──┐                  │
│  run_lidar_mapping.sh    : slam_toolbox (지도 만들 때)            │                  │
│  RViz                                                            │                  │
└──────────────────────────────────────────────────────────────────┼──────────────────┘
                     ▲ /scan, /odom, TF        Wi-Fi (ROS_DOMAIN_ID=42)│
┌────────────────────┴─────── 라즈베리파이 4 (jdrobot, 192.168.0.115) ─▼──────────────┐
│  [tmux car2_lidar]  robot_lidar.launch.py                                           │
│     car2_serial_node : /cmd_vel → ESP32(/dev/ttyAMA0) → 바퀴 4개, 서보 속도 → /odom  │
│     ld_lidar_node    : LD 계열 2D 라이다(USB CP2102) → /scan                        │
│  [tmux map_web]     map_web.py  :8081  지도·위치 웹 UI + 이동 REST API (라벨, Go to) │
│  [tmux robot_arm]   robot_arm.py :8765 depth 카메라 + SO-101 클릭 집기 + REST API    │
│  [tmux llm_client]  llm_client.py :8770 Gemini 지시 웹 UI → 8081(이동) / 8765(팔)   │
└─────────────────────────────────────────────────────────────────────────────────────┘
```

**지시가 처리되는 흐름**: `llm_client`가 지시를 Gemini에 보냅니다. Gemini가 등록된 "스킬" 가운데 하나를 고르면, `llm_client`가 그 스킬에 맞춰 이동 API(`map_web` :8081, 지도 라벨로 이동)나 팔 API(`robot_arm` :8765, 화면 좌표로 집기/놓기)를 호출합니다.

### 포트 / 주소 한눈에 보기

| 서비스 | 위치 | 주소 |
|---|---|---|
| SSH | 파이 | `ssh user@192.168.0.115` (Wi-Fi 고정 IP. 유선 eth0은 DHCP라 쓰지 않음) |
| 지도 웹 UI · 이동 API | 파이 `map_web.py` | http://192.168.0.115:8081 ([API](ros2_slam/test/map_web_api.md)) |
| 팔 웹 UI · 집기 API | 파이 `robot_arm.py` | http://192.168.0.115:8765 |
| LLM 지시 웹 UI | 파이 `llm_client.py` | http://192.168.0.115:8770, 마이크를 쓰려면 https://192.168.0.115:8771 |
| 카메라 웹 UI (RTAB-Map용) | 파이 `camera_test.py` | http://192.168.0.115:8080 |
| ROS 없이 차량 조종 | 파이 `car2_web.py` | http://192.168.0.115:8766 ([API](car2/test/car2_web_api.md)) |

---

## 2. 하드웨어 구성

### 2.1 부품

| 부품 | 모델 | 연결 | 비고 |
|---|---|---|---|
| 메인 컴퓨터 | Raspberry Pi 4 Model B | — | Ubuntu 24.04 + ROS 2 Jazzy |
| 차량 제어 보드 | Waveshare General Driver for Robots (ESP32) | 파이 40핀 헤더 GPIO14/15 → `/dev/ttyAMA0`, 115200bps | 펌웨어 `car2/car2/car2.ino` |
| 바퀴 서보 | Feetech ST3215 × 4 | 보드 서보 버스(데이지 체인, 1Mbps) | ID 1=FL, 2=FR, 3=RL, 4=RR. 12V 권장 |
| 2D 라이다 | LDROBOT LD06/LD19 계열 프로토콜 | USB(CP2102), 230400bps | **USB 소켓 1.2에 꽂음** (아래 주의 참고) |
| depth 카메라 | Orbbec Astra Pro (`2bc5:060f`) | USB | Orbbec OpenNI2 런타임 필요. 최소 측정거리 약 60cm |
| 로봇팔 | SO-101 (Feetech STS3215 × 6) | USB(CH34x), 1Mbps | 12V 전원 별도 |
| 노트북 | Ubuntu 24.04 + ROS 2 Jazzy | Wi-Fi `aicampus_286` | SLAM / Nav2 / RViz |

### 2.2 차량 치수 (실측·보정값)

```
        앞
  [1 FL]    [2 FR]
  [3 RL]    [4 RR]
        뒤
```
- 실제 윤거 0.43m, 축거 0.20m. 바퀴가 미끄러지는 스키드 스티어라서 주행 계산에는 **유효 윤거 0.53m**를 씁니다.
- 바퀴 반지름 0.0329m: 0.87m를 직진시키며 라이다로 잰 값입니다.
- 라이다는 바퀴 사이 정중앙에 달려 있습니다. 0° 방향이 정면에서 약 4.5° 왼쪽이라 `lidar_yaw=0.0785`로 보정합니다.
- 로봇 반경 0.26m. 로봇 기둥 4개는 라이다 `self_mask_deg`로 가립니다.
- 라이다를 다시 달거나 바퀴를 바꾸면 위 값을 다시 재야 합니다. 측정 방법은 [ros2_slam/README.md §5~6](ros2_slam/README.md)을 보세요.

### 2.3 조립 순서

1. **서보 ID 설정**: 새 ST3215는 ID가 모두 1입니다. **한 개씩만** 연결해서 `servo_id_setup.ino`로 1~4를 지정합니다. 방법은 [car2/readme.md §2](car2/readme.md)에 있어요.
2. **차량 펌웨어 업로드**: Arduino IDE에서 보드를 `ESP32 Dev Module`로 고르고 SCServo 라이브러리를 설치한 뒤 `car2/car2/car2.ino`를 올립니다. 시리얼 모니터에 `SERVOS,4/4`와 `READY`가 나오면 정상이에요. ([car2/readme.md §1·§3](car2/readme.md))
3. **드라이버 보드를 파이 40핀 헤더에 장착**하고 파이 UART를 켭니다. `/boot/firmware/config.txt`에 아래 두 줄이 있어야 해요. 넣은 뒤 재부팅합니다.
   ```
   enable_uart=1
   dtoverlay=disable-bt     # 블루투스를 끄고 PL011 UART를 GPIO14/15에 연결 → /dev/ttyAMA0
   ```
4. **라이다를 파이 USB 소켓 1.2에 연결**합니다. CP2102 장치는 시리얼 번호가 모두 `0001`로 같아서, 라이다를 꽂힌 소켓 위치(`/dev/serial/by-path/...usb-0:1.2:1.0-port0`)로 찾기 때문이에요. 다른 소켓에 꽂았으면 파이에서 `ls -l /dev/serial/by-path/`로 경로를 확인하고 `lidar_port:=<경로>`로 지정합니다.
5. **Astra Pro 카메라와 SO-101 팔을 USB로 연결**하고, 팔에 12V를 공급합니다.
6. **전원**: 서보는 USB 전원만으로는 돌지 않습니다. 보드 DC 입력(7~13V, 12V 권장)을 반드시 연결하세요.

---

## 3. 소프트웨어 설치 (최초 1회)

### 3.1 공통 (노트북·파이 모두)

1. Ubuntu 24.04와 [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)를 설치합니다.
2. `~/.bashrc`에 아래를 추가합니다. **두 기계의 `ROS_DOMAIN_ID`가 같아야** 서로 토픽이 보입니다.
   ```bash
   source /opt/ros/jazzy/setup.bash
   export ROS_DOMAIN_ID=42
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   ```
3. ROS 패키지를 설치합니다.
   ```bash
   sudo apt update
   sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-slam-toolbox \
     ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-teleop-twist-keyboard \
     ros-jazzy-openni2-camera ros-jazzy-depthimage-to-laserscan python3-serial
   # RTAB-Map(카메라 3D 지도)도 쓸 때
   sudo apt install -y ros-jazzy-rtabmap-ros ros-jazzy-image-transport-plugins ros-jazzy-usb-cam
   ```
4. 시리얼 포트 권한을 줍니다. 적용하려면 다시 로그인해야 해요.
   ```bash
   sudo usermod -aG dialout $USER
   ```

### 3.2 노트북

```bash
git clone https://github.com/perpet99/dapier_project.git ~/git/dapier_project
cd ~/git/dapier_project/ros2_slam
./scripts/build_ws.sh                     # colcon build (car2_driver, car2_bringup)
./scripts/install_orbbec_openni2.sh       # 노트북에 카메라를 꽂아 쓸 때만
```

파이에 비밀번호 없이 접속하도록 SSH 키를 등록해 두면 원격 스크립트가 매번 비밀번호를 묻지 않습니다.
```bash
ssh-copy-id user@192.168.0.115
```

### 3.3 라즈베리파이

1. **네트워크**: Wi-Fi `aicampus_286`을 netplan(`/etc/netplan/50-cloud-init.yaml`)으로 **192.168.0.115 고정**으로 설정합니다. 공유기에서도 이 주소를 파이 MAC에 예약해 두면 다른 기기와 IP가 겹치지 않아요.
2. **ROS 패키지 배포·빌드**: 노트북에서 실행합니다. 파이의 `~/ros2_slam_robot`에 코드를 복사해 빌드하고, 처음 배포할 때는 Orbbec OpenNI2 arm64 런타임도 설치합니다. **코드를 고칠 때마다 다시 실행**하세요.
   ```bash
   cd ~/git/dapier_project/ros2_slam
   ./scripts/deploy_to_robot.sh
   ```
3. **hand_eye 파이썬 환경**: 파이에서 실행합니다.
   ```bash
   git clone https://github.com/perpet99/dapier_project.git ~/dapier_project
   cd ~/dapier_project
   python3 -m venv .venv && .venv/bin/pip install -r hand_eye/requirements.txt primesense
   sudo apt install -y fonts-nanum           # 웹 UI 한글 표시
   ```
   `robot_arm.py`는 카메라에 접근할 때 `/opt/openni2-orbbec`에 설치된 Orbbec OpenNI2 런타임을 씁니다.
4. **Gemini API 키**: `hand_eye/.env.example`을 복사해 `hand_eye/.env`를 만들고 `GEMINI_API_KEY=`를 채웁니다. 키는 저장소에 커밋하지 마세요(`.gitignore`에 들어 있음).

### 3.4 캘리브레이션 (최초 1회, 하드웨어를 바꿨을 때 다시)

| 대상 | 방법 | 결과 파일 |
|---|---|---|
| SO-101 팔 영점·관절 리밋·방향 | `python hand_eye/calibrate_arm.py` → `python hand_eye/verify_arm.py`로 검증 | `hand_eye/arm_calib.json` |
| 카메라→팔 좌표 변환 (hand-eye) | `./hand_eye/remote_robot_arm.sh start --calib` 실행 후 `attach`해서, 실제로 집을 수 있는 위치 4곳 이상 등록 | `hand_eye/handeye.json`, `handeye_points.json` |
| 파이에서 만든 캘리브레이션 가져오기 | `./hand_eye/remote_robot_arm.sh pull-calib` | 위 파일들 |

자세한 내용은 [hand_eye/readme.md](hand_eye/readme.md)를 보세요. `arm_calib.json`이 없으면 팔이 mock(가짜)으로 동작해서 실제로는 움직이지 않습니다.

---

## 4. 실행 방법

모든 명령은 **노트북**에서 실행합니다. `remote_*.sh` 스크립트는 SSH로 파이에 접속해 tmux 세션 안에서 프로그램을 띄웁니다. 모두 `start | stop | restart | status | log | attach` 명령을 지원합니다.

### 4.1 처음: 지도 만들기

`maps/` 폴더의 지도 파일은 git에 올리지 않으므로, 새로 clone하면 지도가 없습니다. 처음 한 번은 지도를 만들어야 해요.

```bash
cd ~/git/dapier_project/ros2_slam
./scripts/remote_car2_lidar.sh start      # 파이: 차량 드라이버 + 라이다
./scripts/remote_map_web.sh start         # 파이: 지도 웹 UI (주행 패드로 조종)
./scripts/run_lidar_mapping.sh            # 노트북: slam_toolbox + RViz
```
http://192.168.0.115:8081 의 주행 패드(또는 `./scripts/run_teleop.sh`)로 공간을 천천히 한 바퀴 돕니다. 지도가 다 채워지면 **매핑 중에** 저장합니다.
```bash
./scripts/save_lidar_map.sh lidarmap      # → maps/lidarmap.yaml / .pgm / .posegraph / .data
```

### 4.2 평소: 전체 실행 순서

```bash
cd ~/git/dapier_project/ros2_slam
# 1) 이동 로봇
./scripts/remote_car2_lidar.sh start                       # 파이: 차량 드라이버 + 라이다
./scripts/remote_map_web.sh start                          # 파이: 지도 웹 UI · 이동 API (:8081)
./scripts/run_lidar_navigation.sh ./maps/lidarmap.yaml     # 노트북: AMCL + Nav2 + RViz (이 터미널은 계속 열어 둠)

# 2) 다른 터미널에서 팔 + LLM
cd ~/git/dapier_project
./hand_eye/remote_robot_arm.sh start                       # 파이: 카메라 + 팔 (:8765)
./hand_eye/remote_llm_client.sh start                      # 파이: Gemini 지시 웹 UI (:8770)
```

1. **위치 맞추기**: 내비게이션은 지도를 만들기 시작한 자리를 출발점으로 가정합니다. 로봇이 다른 곳에 있으면 지도 웹 UI의 "현재 위치 자동 다시 잡기"나 RViz의 **2D Pose Estimate**로 위치를 맞추세요.
2. **위치 라벨 등록**: 지도 웹 UI에서 로봇을 목적지로 옮긴 뒤 이름("책상", "주방" 등)을 넣고 "현재 위치 라벨 추가"를 누릅니다. LLM이 "책상으로 가"라는 지시를 받으면 이 라벨로 이동해요.
3. **지시하기**: http://192.168.0.115:8770 에 글로 지시를 입력합니다. 음성으로 지시하려면 https://192.168.0.115:8771 로 들어가세요. 자체 서명 인증서라 브라우저 경고가 나오면 "계속"을 누르면 돼요.

**끄기**는 띄운 순서의 반대로 합니다. 각 스크립트에 `stop`을 주고, 내비게이션 터미널은 Ctrl-C로 끕니다. `robot_arm`은 `stop`할 때 팔 토크를 풀어서, **팔이 아래로 처질 수 있으니 손으로 받쳐 주세요**.

### 4.3 부분만 시험하기

| 하고 싶은 것 | 명령 |
|---|---|
| ROS 없이 차량만 조종 | `./ros2_slam/scripts/remote_car2_web.sh start` → http://192.168.0.115:8766 |
| 파이에서 차량 시리얼 직접 테스트 | `python car2/test/car2_test.py --port /dev/ttyAMA0` |
| 팔만 (클릭 집기) | `./hand_eye/remote_robot_arm.sh start` → http://192.168.0.115:8765 에서 클릭 |
| 팔만 LLM으로 (이동 제외) | 파이에서 `llm_client.py --no-base` |
| 카메라 3D 지도(RTAB-Map) | [ros2_slam/README.md §3~4](ros2_slam/README.md) |
| 실행 상태 확인 | 각 스크립트에 `status`, 로그는 `log`, 콘솔은 `attach` (빠져나오기 `Ctrl-b d`) |

---

## 5. 꼭 지켜야 할 규칙

- **차량 시리얼(`/dev/ttyAMA0`)은 한 프로그램만** 써야 합니다. `car2_lidar`, `robot_rgbd`, `car2_driver`, `car2_web` 중 하나만 띄우세요. 둘이 동시에 쓰면 응답이 섞여 `/odom`이 끊깁니다. 스크립트가 중복 실행을 막긴 하지만, 수동으로 띄울 때는 직접 확인해야 해요.
- **Astra 카메라도 한 프로그램만** 쓸 수 있습니다. `robot_arm.py`는 카메라를 ROS(`remote_robot_rgbd.sh`, `remote_car2_lidar.sh --camera`)가 쓰고 있으면 시작하지 않아요.
- **SLAM·위치추정 노드는 하나만** 띄웁니다. 매핑(`run_lidar_mapping.sh`)과 내비게이션(`run_lidar_navigation.sh`)을 동시에 띄우지 마세요.
- **안전 정지**: 차량은 `/cmd_vel`이 1초 끊기면 스스로 멈추고, 펌웨어는 시리얼 명령이 500ms 끊기면 멈춥니다. 그래도 처음 주행할 때는 로봇 옆에서 지켜보세요.
- **웹 API에는 인증이 없습니다.** 내부망에서만 쓰세요.
- **커밋하지 말 것**: `ros2_slam/maps/*`(지도, RTAB-Map DB는 수백 MB), `hand_eye/.env`(API 키), `hand_eye/llm_tls_*.pem`(개인키), `hand_eye/llm_trace.jsonl`·`llm_usage.json`(실행 로그). 모두 `.gitignore`에 들어 있습니다. GitHub는 100MB가 넘는 파일이 든 커밋을 거부해요.

---

## 6. 문제 해결

| 증상 | 확인할 것 |
|---|---|
| 파이에 SSH가 안 됨 | 노트북이 같은 Wi-Fi(`aicampus_286`)에 있는지 확인. `ping 192.168.0.115`가 안 되면 파이 전원과 Wi-Fi를 점검 |
| 노트북에서 `/scan`, `/odom` 토픽이 안 보임 | 두 기계의 `ROS_DOMAIN_ID`(42)와 `RMW_IMPLEMENTATION`(cyclonedds)이 같은지. `ros2 topic hz /scan` |
| 라이다가 안 잡힘 | 라이다가 USB 소켓 1.2에 꽂혀 있는지. `./ros2_slam/scripts/remote_car2_lidar.sh log` |
| 차량이 안 움직이거나 `TIMEOUT,STOP` | 12V 서보 전원, 드라이버 중복 실행, `ros2 topic info /cmd_vel`로 구독자 확인 |
| Nav2가 목표를 바로 거부하거나 "도달 불가" | 초기 위치가 틀렸을 수 있음 → 위치 다시 잡기. 지도에 가짜 장애물이 있으면 지도를 새로 만들기 |
| 팔이 안 움직임 (mock) | `arm_calib.json`이 있는지, 팔 USB(CH34x)·12V 연결. `./hand_eye/remote_robot_arm.sh status` |
| 카메라 0 devices | Orbbec OpenNI2 런타임 설치 여부, 다른 프로그램이 카메라를 쓰는지 |
| 집는 위치가 어긋남 | hand-eye를 다시 캘리브레이션 (`remote_robot_arm.sh start --calib`) |
| LLM이 응답 없음 | `.env`의 `GEMINI_API_KEY`, `./hand_eye/remote_llm_client.sh log` |

더 자세한 문제 해결은 [ros2_slam/README.md 트러블슈팅](ros2_slam/README.md#트러블슈팅)과 [hand_eye/readme.md](hand_eye/readme.md)에 있습니다.
