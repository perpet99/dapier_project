# 🤖 ROS 2 기반 지능형 자율이동로봇 (AMR) 프로젝트

Feetech STS3215 스마트 버스 서보 모터, LD14P 2D 라이다, ESP32 오도메트리 브릿지, UVC 카메라, REST API 웹 관제 대시보드 및 LLM 연동을 지원하는 차동 구동형 자율이동로봇(AMR) 프로젝트입니다.

---

## 🚀 빠른 시작 가이드 (실행 순서)

### 0단계: 최초 빌드 및 시리얼 포트 권한 설정

```bash
cd ~/ros2_amr_ws

# 1. 시리얼 디바이스 권한 부여 (모터 / 라이다)
sudo chmod 666 /dev/ttyACM0 /dev/ttyUSB0 2>/dev/null

# 2. ROS 기본 환경 소싱
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash

# 3. src 폴더 기준 전체 패키지 빌드
colcon build --base-paths src --symlink-install

# 4. 워크스페이스 환경 소싱
source install/setup.bash
```

---

### 1단계: 로봇 하드웨어 구동 (터미널 1)
모터 드라이버, 라이다 센서, URDF 모델 및 기본 브링업을 실행합니다:
```bash
cd ~/ros2_amr_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch amr_bringup full_bringup.launch.py
```

---

### 2단계: Nav2 자율주행 스택 구동 (터미널 2)

#### 🔹 [권장] 라이다 스캔 매칭 기반 내비게이션 (오도메트리 미끄러짐 방지)
회전 시 라이다 스캔 포인트가 벽에서 벗어나지 않고 완벽하게 일치합니다:
```bash
cd ~/ros2_amr_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch amr_bringup navigation_slam.launch.py
```

#### 🔹 일반 AMCL 기반 내비게이션
```bash
ros2 launch amr_bringup navigation.launch.py
```

---

### 3단계: 관제 웹 대시보드 서버 구동 (터미널 3)
REST API, 실시간 TF/Odom 위치 동기화 및 퀵 컨트롤 웹 서버를 실행합니다:
```bash
cd ~/ros2_amr_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 scripts/amr_web_server.py
```

* **접속 주소**: `http://localhost:5000` (또는 `http://<로봇IP>:5000`)
* **웹 주요 기능**:
  * 📍 **실시간 지도 관제**: 로봇의 실시간 좌표(X, Y, Yaw) 및 주행 궤적 표시
  * 🎯 **원터치 목적지 이동**: 등록된 장소 카드 클릭 시 즉시 자율주행 출발
  * 🕹️ **가상 조이스틱 / 키보드(WASD)**: 브라우저에서 직접 수동 원격 조종
  * 🛑 **초고속 비상 정지**: 0.1초 이내 모터 브레이크 및 주행 미션 즉시 취소

---

## 🌐 Gazebo 3D 시뮬레이션 및 AprilTag 도킹

실제 로봇 하드웨어 없이도 사용자가 만든 2D SLAM 맵을 3D Gazebo World로 자동 변환하여 자율주행 및 AprilTag 도킹을 완벽하게 시뮬레이션할 수 있습니다.

### 1. 2D SLAM 맵을 Gazebo 3D 월드로 자동 변환
```bash
python3 tools/map_to_gazebo_world.py --yaml maps/map_1787911624.yaml --out src/amr_bringup/worlds/my_map.sdf
```
* SLAM으로 생성된 2D 점유 격자 지도의 벽면을 1.0m 높이의 3D 벽면 블록(155개)으로 자동 변환합니다.
* 홈/도킹 스테이션 위치에 **AprilTag #1 3D 마커 모델**이 자동으로 배치됩니다.

### 2. Gazebo 시뮬레이션 실행 (로봇 스폰 및 ROS 2 브릿지)
```bash
ros2 launch amr_bringup simulation_docking.launch.py
```
* Gazebo Harmonic 3D 월드가 열리고 가상 AMR 로봇이 스폰됩니다.
* 가상 센서 토픽(`/camera/image_raw`, `/scan`, `/odom`, `/cmd_vel`)이 ROS 2로 실시간 브릿지됩니다.

### 3. 가상 AprilTag 정밀 도킹 실행
```bash
python3 tools/apriltag_docking.py --ros-camera --target-dist 0.10
```
* Gazebo 내 가상 카메라 영상을 구독하여 S-Curve 궤적 주행 및 10cm 정밀 도킹을 시뮬레이션합니다.

---

## 🕹️ 로봇 수동 조종 (Teleop) 가이드

상황에 따라 터미널 키보드 또는 웹 UI를 통해 로봇을 수동 조종할 수 있습니다:

### 1. 터미널 키보드 텔레옵 (ROS 2 기본 툴)
새 터미널을 열고 아래 명령어를 실행합니다:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
> **키 조작법**:
> * `i`: 전진 / `k`: 정지 / `,`: 후진
> * `j`: 좌회전 / `l`: 우회전
> * `u` / `o` / `m` / `.`: 대각선 이동
> * `q` / `z`: 최대 속도 10% 증가 / 감소

### 2. 웹 대시보드 기반 조이스틱 / WASD 조종 (권장)
`scripts/amr_web_server.py` 실행 후 브라우저(`http://localhost:5000`)에 접속:
* 화면 하단의 **가상 조이스틱**을 마우스로 드래그
* 키보드 `W`(전진), `S`(후진), `A`(좌회전), `D`(우회전) 키로 손쉽게 조종
* 스페이스바(`Space`): 즉시 브레이크 정지

---

## 🤖 LLM / 외부 시스템 연동 REST API 명세

어떤 LLM(ChatGPT, Claude, Gemini, Ollama)이나 외부 관제 시스템을 붙이더라도 아래 REST API 규격으로 바로 호출할 수 있습니다:

| 기능 | Method | URL | Request Body (JSON) | 설명 |
| :--- | :---: | :--- | :--- | :--- |
| **특정 장소 이동** | `POST` | `/api/navigate` | `{"waypoint": "home"}` | 등록된 장소(`home`, `point_a` 등)로 이동 |
| **임의 좌표 이동** | `POST` | `/api/goal_pose` | `{"x": 1.5, "y": 2.0, "yaw": 0.0}` | 특정 (X, Y) 좌표로 이동 |
| **비상 / 즉시 정지** | `POST` | `/api/stop` | `{}` | 로봇 즉각 브레이크 및 미션 취소 |
| **프리셋 미션** | `POST` | `/api/action` | `{"action": "patrol"}` | `patrol` (전체 순찰), `round_trip` (왕복) |
| **수동 속도 제어** | `POST` | `/api/cmd_vel` | `{"linear": 0.15, "angular": 0.0}` | 로봇 직진/회전 속도 수동 제어 |
| **로봇 상태 조회** | `GET` | `/api/status` | 없음 | 실시간 좌표, 속도, 배터리 정보 반환 |
| **장소 목록 조회** | `GET` | `/api/waypoints` | 없음 | XML에 등록된 모든 목적지 목록 반환 |

### Python LLM Tool Call 예시
```python
import requests

SERVER_URL = "http://localhost:5000"

def move_robot(waypoint_id: str):
    return requests.post(f"{SERVER_URL}/api/navigate", json={"waypoint": waypoint_id}).json()

def stop_robot():
    return requests.post(f"{SERVER_URL}/api/stop").json()
```

---

## 📁 디렉터리 구조

```text
ros2_amr_ws/
├── src/                         # ROS 2 핵심 패키지
│   ├── amr_bringup/             # 런치(Launch), 파라미터(Config), URDF 로봇 모델
│   ├── amr_servo/               # STS3215 서보 모터 직렬 제어 노드
│   ├── esp32_bridge/            # ESP32 휠 엔코더/오도메트리 브릿지
│   └── ldlidar_sl_ros2/         # LD14P 2D 라이다 드라이버
│
├── web_app/                     # 관제 웹 대시보드 프론트엔드 (HTML/CSS/JS)
│   ├── index.html               # 통합 대시보드 UI
│   ├── app.js                   # 캔버스 맵 엔진, 실시간 조이스틱 & REST 통신
│   └── style.css                # 다크 테마 스타일시트
│
├── bot_modules/                 # 로봇 제어 핵심 모듈
│   ├── config.py                # 설정 로드 및 waypoints.xml 파싱
│   ├── ros2_runner.py           # 초고속 비상정지 및 서브프로세스 주행 제어
│   └── mission_queue.py         # 다단계 액션 큐 관리자
│
├── scripts/                     # 서버 및 관제 스크립트
│   ├── amr_web_server.py        # REST API & 실시간 TF/Odom 동기화 웹 서버 (포트 5000)
│   ├── discord_amr_bot.py       # AI 자연어 디스코드 관제 봇
│   └── collision_warning_listener.py # 실시간 충돌 방지 안전 경고 리스너
│
├── config/                      # 설정 및 목표 지점 데이터
│   └── waypoints.xml            # 자율주행 목표 위치 및 각도 XML
│
└── tools/                       # 테스트 및 주행/도킹 도구
    ├── apriltag_docking.py      # 카메라 비전 기반 초정밀 태그 자동 도킹
    ├── undock_amr.py            # 도킹 해제 (45cm 안전 후진 + 180도 회전)
    ├── navigate_and_dock.py     # Nav2 주행 + 비전 자동 도킹 통합 파이프라인
    ├── run_xml_waypoints.py     # XML 기반 단일 이동/순찰 실행기
    ├── navigate_to_coords.py    # 임의 좌표 다이렉트 주행기
    └── test_round_trip.py       # 1초 직진-180도 회전-복귀 테스트
```
