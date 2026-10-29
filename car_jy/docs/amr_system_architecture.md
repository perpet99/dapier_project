# 🤖 AMR 자율주행 시스템 & 하드웨어 통합 아키텍처 다이어그램

본 문서는 **물리적 하드웨어 센서/액추에이터**부터 **`amr_bringup` 드라이버 레이어**, **Nav2 자율주행 스택**, **웹 관제 서버 및 프론트엔드 UI**까지의 데이터 흐름을 한눈에 보여줍니다.

---

## 🏗️ 전체 하드웨어 & 소프트웨어 데이터 흐름도

```mermaid
%%{init: {'themeVariables': { 'fontSize': '19px', 'subGraphTitleFontSize': '21px' }}}%%
flowchart TD
    %% 1. 사용자 레이어
    subgraph Client ["🖥️ 1. 사용자 관제 레이어 (Web UI)"]
        UI["UI 대시보드 (index.html)"]
        JS["클라이언트 로직 (app.js)"]
    end

    %% 2. 관제 서버 레이어
    subgraph WebServer ["🌐 2. 웹 서버 & 미션 큐 (amr_web_server)"]
        API["REST API (/api/move, /api/stop)"]
        SSE["SSE 실시간 브로드캐스터 (/api/events)"]
        Queue["FIFO 미션 큐 (mission_queue.py)"]
        Runner["ROS 2 주행 실행기 (ros2_runner.py)"]
    end

    %% 3. 자율주행 스택 레이어
    subgraph NavStack ["🧠 3. Nav2 자율주행 스택 (navigation_slam)"]
        NavClient["Simple Commander (navigate_to_coords.py)"]
        GlobalPlanner["글로벌 경로 생성기 (NavFn / Smac)"]
        LocalController["로컬 경로 추종기 (DWB Controller)"]
        Costmap["코스트맵 (Global / Local Costmap)"]
        SLAM_Loc["SLAM Toolbox 위치 추정 (Localization)"]
        CmdVel["/cmd_vel (목표 속도 Twist)"]
    end

    %% 4. ROS 2 Bringup 드라이버 레이어
    subgraph Bringup ["🚀 4. ROS 2 Bringup 드라이버 (full_bringup)"]
        LiDAR_Node["LD14P 라이다 드라이버 (ldlidar_sl_ros2)"]
        Servo_Node["STS 서보 제어 노드 (amr_servo)"]
        RSP_Node["로봇 모델 & TF (robot_state_publisher)"]
        ScanTopic["/scan (LaserScan 거리 데이터)"]
        TFTopic["/tf, /tf_static (로봇 좌표계)"]
    end

    %% 5. 물리적 하드웨어 장치 레이어
    subgraph HW_Sensors ["📡 하드웨어 센서 & 제어보드"]
        HW_Lidar["LD14P 2D LiDAR 센서"]
        HW_Port_Lidar["USB 포트 (/dev/lidar_port)"]
        HW_Servo_Board["Feetech 서보 제어 보드 (1a86 칩셋 @ 1Mbps)"]
        HW_Port_Servo["USB 포트 (/dev/servo_port)"]
    end

    %% 6. 모터 레이어
    subgraph HW_Motors ["🛞 4WD 서보 모터 (Feetech STS3215)"]
        M1["앞바퀴 좌측 (ID 1)"]
        M2["앞바퀴 우측 (ID 2)"]
        M3["뒷바퀴 좌측 (ID 3)"]
        M4["뒷바퀴 우측 (ID 4)"]
    end

    %% 데이터 흐름 연결선
    UI -->|1. 목적지 클릭| JS
    JS -->|2. POST /api/move| API
    API -->|3. Enqueue| Queue
    Queue -->|4. 순차 실행| Runner
    Runner -.->|실시간 상태 업데이트| SSE
    SSE -.->|SSE 푸시 이벤트| JS

    Runner -->|5. GoToPose Subprocess| NavClient
    NavClient -->|6. 목표 좌표 Goal| GlobalPlanner
    GlobalPlanner -->|글로벌 경로| LocalController
    Costmap -->|장애물 회피 계산| LocalController
    SLAM_Loc -->|현재 위치 피드백| LocalController
    LocalController -->|7. 속도 명령 발행| CmdVel

    CmdVel -->|8. 속도 수신| Servo_Node
    ScanTopic -->|장애물 데이터| Costmap
    ScanTopic -->|스캔 매칭| SLAM_Loc
    TFTopic -->|좌표계 변환| NavStack

    LiDAR_Node -->|UART 통신| HW_Port_Lidar
    HW_Port_Lidar --> HW_Lidar
    HW_Lidar -->|거리 측정 패킷| ScanTopic

    Servo_Node -->|9. Sync Write 0x83| HW_Port_Servo
    HW_Port_Servo --> HW_Servo_Board
    HW_Servo_Board -->|단일 패킷 4개 동시 구동| M1
    HW_Servo_Board -->|단일 패킷 4개 동시 구동| M2
    HW_Servo_Board -->|단일 패킷 4개 동시 구동| M3
    HW_Servo_Board -->|단일 패킷 4개 동시 구동| M4
    RSP_Node --> TFTopic
```

---

## 📌 하드웨어와 소프트웨어 간의 주요 인터페이스 요약

| 하드웨어 장치 | 연결 포트 | 담당 ROS 2 노드 | 통신 프로토콜 / 토픽 |
| :--- | :--- | :--- | :--- |
| **📡 LD14P 2D 라이다** | `/dev/lidar_port` (CP2102) | `ldlidar_sl_ros2` | 230,400 bps 시리얼 ➔ **/scan** (LaserScan) |
| **🎛️ STS3215 서보 제어보드** | `/dev/servo_port` (1a86) | `amr_servo (sts_controller)` | 1,000,000 bps 시리얼 ➔ **Sync Write (`0x83`)** |
| **🛞 4개 STS3215 모터** | Daisy-Chain Bus | `amr_servo` | ID 1(좌앞), ID 2(우앞), ID 3(좌뒤), ID 4(우뒤) |
| **🤖 로봇 프레임 (Chassis)** | - | `robot_state_publisher` | URDF ➔ **/tf** (`base_footprint` ➔ `base_link` ➔ `laser_frame`) |
