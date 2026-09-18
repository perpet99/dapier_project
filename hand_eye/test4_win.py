"""
test4.py - Depth 기반 클릭 Pick & Place (SO-101)

동작:
  1) Depth 카메라(또는 파일/합성) 스트림을 UI로 출력한다.
  2) 마우스로 물체를 클릭하면 -> 그 지점을 SO-101이 집어서 들어올린다.
  3) 다음 위치를 클릭하면 -> 집고 있던 물건을 그 자리에 내려놓는다.

키:
  q  종료          h  홈 자세        o/c  그리퍼 열기/닫기
  r  상태 리셋     d  보기 전환(센서/depth/일반카메라)   s  현재 프레임 저장
  w  집기 가능 영역 표시 ON/OFF (붉은색 = 도달 불가)
  k  hand-eye 캘리브레이션 모드 토글    t  토크 ON/OFF (손으로 팔 움직이기)

이 PC 의 실기 구성 (실측 확인함):
  카메라  Orbbec Astra Pro (VID 0x2BC5 / PID 0x060F), 640x480 @ 27fps
          Orbbec SDK v2 는 이 모델을 지원하지 않는다(벤더 ob_enumerate 도 인식 못 함).
          OpenNI2 런타임(D:/OpenNI2/Redist) + primesense 바인딩으로 접근한다.
          최소 측정거리가 약 60cm 이므로 그보다 가까운 곳은 depth 가 비어 있다.
          기본은 depth 를 컬러 좌표계로 하드웨어 정렬(DEPTH_ALIGN)하고 컬러를
          화면으로 쓴다. 정렬이 안 되면 IR 로 되돌아간다 - IR 은 depth 와 같은
          센서라 픽셀이 1:1 대응한다.
  로봇    SO-101, Feetech STS3215 x6, COM18 @ 1Mbps, 12.2V
          lerobot 캘리브레이션의 관절 리밋으로 목표값을 하드 클램프한다.

실행 전제:
  실행하면 캘리브레이션 상태부터 확인한다. 없으면 지금 잡을지, 있으면 다시 잡을지
  물어본다. 따로 스크립트를 먼저 돌릴 필요가 없다.
    1/2  arm_calib.json  - 관절 min/max 리밋 + 기준자세(부호/영점). 없으면 mock 으로 내려간다.
    2/2  handeye.json    - 카메라->로봇 변환. 없으면 탁상 위 집을 수 있는 위치를
                           4곳 이상 등록하는 모드로 바로 들어간다.
  $env:SO101_PORT='COM18' 을 설정하면 포트 자동검출을 건너뛴다.
  팔 TCP 만 확인하려면: python test4.py --tcp

원격 제어 (REST):
  화면 픽셀 좌표로 마우스 클릭과 똑같이 집고 놓는다. 기본은 127.0.0.1:8765.
    GET|POST /pick   x,y      GET|POST /place  x,y      GET /status
  다른 PC 에서 부르려면 $env:PICK_API_HOST='0.0.0.0', 끄려면 PICK_API_PORT='0'.
"""

from __future__ import annotations

import atexit
import json
import math
import os
import queue
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse
from dataclasses import dataclass, field
from typing import Callable, Optional, Sequence

import cv2
import numpy as np

try:
    import msvcrt          # Windows 콘솔에서 논블로킹 키 입력 (없으면 시간제한으로 대체)
except ImportError:
    msvcrt = None

# Windows 콘솔 기본 코드페이지(cp949/cp1252)에서 한글 print 가 깨지지 않게 한다.
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass


# ---------------------------------------------------------------------------
# 설정
# ---------------------------------------------------------------------------

# 캘리브레이션 파일은 어디서 실행하든 스크립트 옆에서 찾는다. 상대경로로 두면
# 다른 폴더에서 실행했을 때 조용히 '파일 없음'이 되어 mock 으로 내려간다.
_HERE = os.path.dirname(os.path.abspath(__file__))
HANDEYE_PATH = os.path.join(_HERE, "handeye.json")
HANDEYE_POINTS_PATH = os.path.join(_HERE, "handeye_points.json")

# --- SO-101 링크 치수 [m] -----------------------------------------------------
# 출처: TheRobotStudio/SO-ARM100 공식 URDF (Simulation/SO101/so101_new_calib.urdf).
# 관절 origin 을 체인으로 곱해 '연속 회전축 사이 거리'를 뽑은 값이다.
# TCP 는 lerobot 이 쓰는 것과 같은 gripper_frame_link (그리퍼 파지점) 로 잡았다.
#
# 이 값들로 만든 아래 평면 3링크 모델은 URDF 전체 체인과 무작위 400자세에서
# 최대 0.02mm 차이다. 즉 단순화로 잃는 정확도는 없다.
#
# 주의: shoulder_lift 회전축은 pan 축 위에 있지 않고 30.4mm 앞에 있다.
#       이 오프셋은 pan 과 함께 돌기 때문에 hand-eye 변환으로 흡수되지 않는다.
L_SHOULDER_R = 0.03039   # pan 축 -> shoulder_lift 축 (수평 오프셋)
L0_BASE_H = 0.11660      # 베이스 바닥 -> shoulder_lift 회전축 높이
L1_UPPER = 0.11600       # shoulder_lift -> elbow_flex
L2_FORE = 0.13500        # elbow_flex -> wrist_flex
L3_WRIST = 0.15942       # wrist_flex -> 그리퍼 파지점(TCP), 평면 성분

# --- 영자세(모든 관절 0도) 정의 -----------------------------------------------
# 위팔 수직(90도) / 아래팔 수평 / 그리퍼는 아래팔과 일직선.
# 관절각으로 말하면 lift 90도, elbow 90도 꺾임, wrist 0도다.
# 직각자 하나로 확인되는 자세라서 사람이 재현할 수 있다. 링크를 쭉 편 수평
# 자세는 영자세가 아니다 - 그렇게 잡으면 관절마다 70도 넘게 어긋난다.
A1_ZERO = math.radians(90.0)    # 위팔 (shoulder_lift -> elbow_flex)
A2_ZERO = math.radians(0.0)     # 아래팔 (elbow_flex -> wrist_flex)
A3_ZERO = math.radians(0.0)     # 손목+그리퍼 (wrist_flex -> TCP)

# 서보 가동범위의 한가운데는 위 자세가 아니다. 공식 URDF(so101_new_calib)의
# 영자세 = 범위 중앙이고, 그 자세의 링크각은 76.03 / 2.21 / -2.84 도다.
# 범위 중앙으로 영점을 잡을 때는 우리 0도와의 차이를 빼줘야 한다.
MIDRANGE_Q = {"shoulder_pan": 0.0,
              "shoulder_lift": math.radians(-13.9677),
              "elbow_flex": math.radians(16.1752),
              "wrist_flex": math.radians(-5.0480),
              "wrist_roll": 0.0}

REACH = L1_UPPER + L2_FORE   # 어깨에서 손목중심까지의 최대 거리 = 0.2509 m
IK_MARGIN = 0.008            # 완전신전(특이점) 근처를 피하기 위한 여유

# --- 작업 영역 제한 [m] (로봇 베이스 기준) ------------------------------------
# 이건 거친 안전 박스일 뿐이다. 실제 도달 가능 여부는 IK 가 판정한다.
WS_R_MIN, WS_R_MAX = 0.08, 0.25   # 베이스 축으로부터의 수평 거리
WS_Z_MIN, WS_Z_MAX = 0.005, 0.24  # 테이블면 위 높이

# 캘리브레이션 중에만 z 를 이만큼 더 허용한다 [m].
# 영점이 조금 틀어져 있으면 그리퍼를 테이블에 댔을 때 FK 가 z 를 음수로 보고한다.
# 그 값을 거부해 버리면 정작 영점을 고치는 데 필요한 점을 하나도 못 모은다.
# 실제 집기(pick/place)에는 적용하지 않는다 - 거기서는 테이블을 파고들면 안 된다.
CALIB_Z_TOL = 0.030
CALIB_Z_MIN = -CALIB_Z_TOL        # 테이블면(z=0) 기준으로 이만큼 아래까지 허용

# --- 파지 파라미터 [m] --------------------------------------------------------
# 접근/상승 높이는 "희망값"이다. 반경이 클수록 팔이 위로 뻗을 여유가 줄기 때문에
# 실제 높이는 max_z_at() 으로 도달 가능한 범위까지만 올린다.
APPROACH_H = 0.06    # 목표 위에서 정렬하고 싶은 높이
GRASP_SINK = 0.012   # 표면에서 얼마나 더 내려가 잡을지
LIFT_H = 0.08        # 파지 후 들어올리고 싶은 높이
PLACE_CLEAR = 0.050  # 놓을 때 표면에서 띄울 높이 - 여기서 그리퍼를 연다

# 그리퍼를 "연다"고 할 때 얼마나 벌릴지 (0=닫힘, 1=기계적 최대).
# 최대로 벌리면 필요 이상으로 크게 벌어져 주변 물체를 건드리고 여닫는 시간도 길다.
# set_gripper(1.0) 은 여전히 완전 개방을 뜻하고, 여기서는 기본 동작만 바꾼다.
GRIPPER_OPEN = 0.5

TOPDOWN_PITCH = math.radians(-90.0)  # TCP 접근각: -90deg = 수직 하강

DEPTH_PATCH = 5      # 클릭 지점 주변 median 창 크기 (depth 홀 보정)
DEPTH_FRONT_BAND = 0.020  # '앞면' 으로 볼 깊이 폭 [m] (얇은 물체용 near 샘플링)
DEPTH_FILL_MAX_PX = 24    # depth 구멍을 가장자리에서 몇 화소까지 메울지

# --- 컬러 카메라 / depth 정렬 (환경변수로 조정) ---
COLOR_CAM_INDEX = os.environ.get("COLOR_CAM_INDEX")
ORBBEC_VID = "VID_2BC5"

# depth 를 컬러 좌표계로 하드웨어 정렬한다 (기본 ON, 끄려면 $env:DEPTH_ALIGN='0')
# 켜면 화면이 컬러가 되고 IR 은 쓰지 않는다. 컬러에서 클릭한 픽셀이 곧 depth
# 화소라 별도 정렬 작업이 필요 없다. 컬러가 안 들어오면 자동으로 IR 로 되돌아간다.
DEPTH_ALIGN = os.environ.get("DEPTH_ALIGN", "1").lower() in ("1", "true", "yes", "on")
# 컬러 렌즈의 초점거리[px, 640 폭 기준]. 등록 워프 스케일 0.9432 에서 역산했다.
COLOR_FX = float(os.environ.get("COLOR_FX", "615.0"))
# 후보 카메라를 하나씩 열어보며 센서와 같은 장면인지 볼지 ($env:COLOR_SCAN='1')
COLOR_SCAN = os.environ.get("COLOR_SCAN", "").lower() in ("1", "true", "yes", "on")
# 정렬 모드에서 컬러 첫 프레임을 이만큼 기다린다. 안 오면 정렬을 끄고 IR 로 돌아간다.
ALIGN_COLOR_TIMEOUT = float(os.environ.get("ALIGN_COLOR_TIMEOUT", "20"))


# ---------------------------------------------------------------------------
# 1. Depth 소스
# ---------------------------------------------------------------------------

@dataclass
class Intrinsics:
    """핀홀 카메라 내부 파라미터."""
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


class DepthSource:
    """color(BGR uint8) + depth(float32, meter) 를 정렬된 상태로 제공한다."""

    intr: Intrinsics

    def read(self) -> tuple[np.ndarray, np.ndarray]:
        raise NotImplementedError

    def pause(self) -> None:
        """스트림을 잠시 멈춘다(대역 양보). 지원하지 않으면 아무것도 안 한다."""

    def resume(self) -> None:
        """pause 한 스트림을 되살린다."""

    def close(self) -> None:
        pass


class RealSenseSource(DepthSource):
    """Intel RealSense (D405/D435 등). depth 를 color 프레임에 align 한다."""

    def __init__(self, width: int = 640, height: int = 480, fps: int = 30):
        import pyrealsense2 as rs

        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        profile = self.pipeline.start(cfg)

        self.align = rs.align(rs.stream.color)
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

        ci = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.intr = Intrinsics(ci.fx, ci.fy, ci.ppx, ci.ppy, ci.width, ci.height)

        # 홀 채우기 / 노이즈 억제
        self.spatial = rs.spatial_filter()
        self.hole = rs.hole_filling_filter()

    def read(self) -> tuple[np.ndarray, np.ndarray]:
        frames = self.align.process(self.pipeline.wait_for_frames())
        cf, df = frames.get_color_frame(), frames.get_depth_frame()
        if not cf or not df:
            raise RuntimeError("RealSense 프레임 수신 실패")
        df = self.hole.process(self.spatial.process(df))
        color = np.asanyarray(cf.get_data())
        depth = np.asanyarray(df.get_data()).astype(np.float32) * self.depth_scale
        return color, depth

    def close(self) -> None:
        self.pipeline.stop()


class OrbbecSource(DepthSource):
    """Orbbec Astra 계열 (VID 0x2BC5). pyorbbecsdk 로 depth+color 를 받는다.

    이 장비의 depth 인터페이스는 Orbbec 전용 드라이버가 점유하고 있어서
    OpenCV 의 CAP_OBSENSOR(UVC 기반) 로는 컬러만 나오고 depth 는 오지 않는다.
    따라서 벤더 SDK 가 필수다.
    """

    def __init__(self, width: int = 640, height: int = 480, fps: int = 30):
        from pyorbbecsdk import (Config, Context, OBSensorType, OBFormat, Pipeline,
                                 AlignFilter, OBStreamType)

        # 장치가 없을 때 Pipeline() 을 그냥 만들면 파이썬 예외가 아니라 프로세스가
        # 통째로 죽는다(exit 127). 반드시 먼저 열거해서 확인한다.
        if Context().query_devices().get_count() == 0:
            raise RuntimeError("Orbbec SDK 가 인식하는 장치 없음 "
                               "(Astra Pro 계열은 SDK v2 미지원 - OpenNI2 를 쓸 것)")

        self.pipeline = Pipeline()
        cfg = Config()

        dev_info = self.pipeline.get_device().get_device_info()
        print(f"[source] Orbbec: {dev_info.get_name()} SN={dev_info.get_serial_number()}")

        cprofiles = self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        cfg.enable_stream(cprofiles.get_video_stream_profile(width, height, OBFormat.RGB, fps))
        dprofiles = self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        dprof = dprofiles.get_video_stream_profile(width, height, OBFormat.Y16, fps)
        cfg.enable_stream(dprof)

        self.pipeline.start(cfg)
        self.align = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)

        i = dprof.get_intrinsic()
        self.intr = Intrinsics(i.fx, i.fy, i.cx, i.cy, width, height)
        self.depth_scale = 0.001  # Orbbec Y16 은 mm 단위

    def read(self) -> tuple[np.ndarray, np.ndarray]:
        frames = self.pipeline.wait_for_frames(200)
        if frames is None:
            raise RuntimeError("Orbbec 프레임 타임아웃")
        frames = self.align.process(frames)
        cf, df = frames.get_color_frame(), frames.get_depth_frame()
        if cf is None or df is None:
            raise RuntimeError("Orbbec color/depth 프레임 없음")

        h, w = cf.get_height(), cf.get_width()
        rgb = np.frombuffer(cf.get_data(), np.uint8).reshape(h, w, 3)
        color = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        dh, dw = df.get_height(), df.get_width()
        depth = (np.frombuffer(df.get_data(), np.uint16)
                 .reshape(dh, dw).astype(np.float32) * self.depth_scale)
        if (dh, dw) != (h, w):
            depth = cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)
        return color, depth

    def close(self) -> None:
        self.pipeline.stop()


class OpenNI2Source(DepthSource):
    """Astra Pro 계열 (VID 0x2BC5 / PID 0x060F) 용 OpenNI2 백엔드.

    이 장비는 Orbbec SDK v2 지원 대상이 아니다(벤더 ob_enumerate 도 인식 못 함).
    depth 는 obdrv4 드라이버 + OpenNI2 로만 접근된다.

    정렬에 관한 중요한 점:
      Astra Pro 의 컬러는 별개의 UVC 카메라라서 depth 와 하드웨어 정렬이 안 된다.
      그래서 여기서는 IR 을 화면용 영상으로 쓴다. IR 은 depth 와 같은 센서에서
      나오므로 픽셀이 1:1로 대응한다. 즉 화면에서 클릭한 픽셀의 depth 가
      바로 그 지점의 깊이다 - 별도 정렬이 필요 없다.

    OpenNI2 런타임 DLL 이 필요하다. 환경변수로 위치를 알려준다:
      $env:OPENNI2_REDIST='C:\\path\\to\\OpenNI2\\Redist'
    """

    # OpenNI2 런타임(OpenNI2.dll + OpenNI2/Drivers/orbbec.dll)을 찾을 위치
    REDIST_CANDIDATES = ("D:/OpenNI2/Redist", "C:/OpenNI2/Redist",
                         "C:/Program Files/OpenNI2/Redist")

    @classmethod
    def find_redist(cls) -> Optional[str]:
        env = os.environ.get("OPENNI2_REDIST")
        for p in ((env,) if env else ()) + cls.REDIST_CANDIDATES:
            if p and os.path.exists(os.path.join(p, "OpenNI2.dll")):
                return p
        return None

    def __init__(self, width: int = 640, height: int = 480, fps: int = 30,
                 depth_align: bool = DEPTH_ALIGN):
        from primesense import openni2

        redist = self.find_redist()
        if redist is None:
            raise RuntimeError(
                "OpenNI2 런타임을 찾을 수 없다. OPENNI2_REDIST 로 경로를 지정하거나 "
                f"다음 중 하나에 설치할 것: {', '.join(self.REDIST_CANDIDATES)}")
        openni2.initialize(redist)
        self._openni2 = openni2

        self.dev = openni2.Device.open_any()
        self.depth_aligned = False
        self.color_cam: Optional[ColorCamera] = None
        self._last_rgb: Optional[np.ndarray] = None    # 컬러가 끊겼을 때 보여줄 직전 프레임
        self.depth_stream = self.dev.create_depth_stream()
        self.depth_stream.start()
        self._set_mirror(self.depth_stream, "depth")

        ref_ir = None
        if depth_align:
            # 장면으로 카메라를 고를 때만 IR 을 한 장 받는다. 그 스트림을 열었다
            # 닫는 것만으로도 depth 쪽 대역/할당이 흔들려서, 뒤이은 depth 읽기가
            # 통째로 막히는 일이 있었다(실측: read() 가 2분 넘게 안 돌아옴).
            if COLOR_SCAN:
                ref_ir = self._grab_ir_once()
            self._enable_depth_align(openni2)

        if self.depth_aligned:
            # depth 가 컬러 좌표계로 왔으므로 화면도 컬러여야 한다. IR 은 정렬
            # 대상이 아니라서 켜 두면 어긋나고, USB2.0 대역만 잡아먹는다.
            self.ir_stream = None
            self.color_cam = ColorCamera()
            self.color_cam.start(ref=ref_ir)
            if not self._wait_color(ALIGN_COLOR_TIMEOUT):
                # 컬러가 안 오면 정렬 모드는 무의미하다. 그대로 두면 화면에
                # depth 컬러맵만 뜨고, 최악의 경우 depth 읽기까지 막혀 멈춘다.
                print(f"[source] {ALIGN_COLOR_TIMEOUT:.0f}초 안에 컬러 프레임이 오지 않았다 "
                      "-> 정렬을 끄고 IR 기준으로 되돌린다")
                print("        (Astra Pro 는 depth 와 컬러가 USB 대역을 나눠 쓴다. "
                      "다른 USB 컨트롤러 포트에 꽂으면 같이 흐를 수 있다)")
                self._disable_depth_align(openni2)
        else:
            # IR 은 depth 와 같은 센서 -> 완벽히 정렬됨. 동시 스트리밍이 안 되는
            # 모델도 있으므로 실패하면 depth 컬러맵으로 대체한다.
            try:
                self.ir_stream = self.dev.create_ir_stream()
                self.ir_stream.start()
                self._set_mirror(self.ir_stream, "ir")
            except Exception as e:
                print(f"[source] IR 스트림 사용 불가 ({e}) -> depth 컬러맵을 화면으로 사용")
                self.ir_stream = None

        vm = self.depth_stream.get_video_mode()
        w, h = vm.resolutionX, vm.resolutionY
        hfov = self.depth_stream.get_horizontal_fov()
        vfov = self.depth_stream.get_vertical_fov()
        fx = (w / 2.0) / math.tan(hfov / 2.0)
        fy = (h / 2.0) / math.tan(vfov / 2.0)
        if self.depth_aligned:
            # 등록을 켜도 드라이버가 FOV 는 depth 렌즈 값 그대로 보고한다. 역투영은
            # 컬러 렌즈 기준으로 해야 하므로 실측한 초점거리를 쓴다.
            # (등록 워프의 스케일 0.9432 에서 역산: 579.87/0.9432 = 615)
            fx, fy = COLOR_FX, COLOR_FX * (fy / fx if fx else 1.0)
            print(f"[source] depth_align: 역투영을 컬러 렌즈 기준으로 한다 "
                  f"(fx={fx:.1f}, COLOR_FX 로 조정 가능)")
        self.intr = Intrinsics(fx=fx, fy=fy, cx=w / 2.0, cy=h / 2.0, width=w, height=h)
        print(f"[source] OpenNI2 depth {w}x{h}, fx={self.intr.fx:.1f} fy={self.intr.fy:.1f}"
              + ("  [depth->color 정렬됨]" if self.depth_aligned else ""))

    def _grab_ir_once(self) -> Optional[np.ndarray]:
        """IR 을 잠깐 켜서 한 장만 받는다. 컬러 카메라를 고르는 기준으로 쓴다."""
        try:
            st = self.dev.create_ir_stream()
            st.start()
            self._set_mirror(st, "ir")
            img = None
            for _ in range(6):
                f = st.read_frame()
                a = np.frombuffer(f.get_buffer_as_uint16(), np.uint16).reshape(
                    f.height, f.width).astype(np.float32)
                img = cv2.cvtColor(
                    cv2.normalize(a, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8),
                    cv2.COLOR_GRAY2BGR)
            st.stop()
            try:
                st.close()          # stop 만으로는 센서 할당이 남는다
            except Exception:
                pass
            return img
        except Exception as e:
            print(f"[source] 기준 IR 영상 확보 실패({e}) - 컬러는 번호로만 고른다")
            return None

    def _wait_color(self, timeout: float) -> bool:
        """컬러 첫 프레임을 기다린다. depth 는 읽지 않는다(읽으면 막힐 수 있다)."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.color_cam is not None and self.color_cam.latest((64, 48)) is not None:
                print(f"[source] 컬러 첫 프레임 {time.time()-t0:.1f}s")
                return True
            if self.color_cam is None or self.color_cam.failed:
                return False
            time.sleep(0.1)
        return False

    def _disable_depth_align(self, openni2) -> None:
        """정렬을 끄고 원래(IR 기준) 구성으로 되돌린다."""
        try:
            self.dev.set_image_registration_mode(openni2.IMAGE_REGISTRATION_OFF)
        except Exception as e:
            print(f"[source] 정렬 해제 실패: {e}")
        self.depth_aligned = False
        if self.color_cam is not None:
            self.color_cam.stop()
            self.color_cam = None
        try:
            self.ir_stream = self.dev.create_ir_stream()
            self.ir_stream.start()
            self._set_mirror(self.ir_stream, "ir")
        except Exception as e:
            print(f"[source] IR 스트림 사용 불가 ({e}) -> depth 컬러맵을 화면으로 사용")
            self.ir_stream = None

    def _enable_depth_align(self, openni2) -> None:
        """depth 를 컬러 좌표계로 하드웨어 정렬한다 (ros_astra_camera 의 depth_align).

        Astra Pro 는 컬러가 OpenNI 밖의 UVC 장치지만, 드라이버가 공장 D2C
        파라미터를 갖고 있어 컬러 스트림 없이도 depth 를 컬러 격자로 투영해 준다.
        실측: 워프 스케일 0.943, 중앙 15px / 모서리 32px 이동.
        """
        mode = openni2.IMAGE_REGISTRATION_DEPTH_TO_COLOR
        try:
            if not self.dev.is_image_registration_mode_supported(mode):
                print("[source] 이 장치는 depth->color 정렬을 지원하지 않는다")
                return
            self.dev.set_image_registration_mode(mode)
            self.depth_aligned = True
            print("[source] depth->color 정렬 ON")
        except Exception as e:
            print(f"[source] depth->color 정렬 실패: {e} - IR 기준으로 계속한다")

    @staticmethod
    def _set_mirror(stream, name: str, enabled: bool = False) -> None:
        """미러링을 끈다.

        Astra 는 OpenNI2 기본값이 미러 ON 이라 좌우가 뒤집힌 영상이 나온다.
        보기 불편한 문제로 끝나지 않는다. deproject() 는 u 가 오른쪽으로 증가하는
        표준 핀홀 기하를 가정하므로, 좌우가 뒤집히면 카메라 X 좌표의 부호가 반대가
        된다. 그러면 카메라-로봇 대응이 반사 관계가 되는데 Kabsch/PnP 는 정상
        회전(det=+1)만 낼 수 있어서 해가 통째로 망가진다.
        """
        try:
            if stream.get_mirroring_enabled() != enabled:
                stream.set_mirroring_enabled(enabled)
            got = stream.get_mirroring_enabled()
            if got != enabled:
                print(f"[source] 경고: {name} 미러링을 {enabled} 로 바꾸지 못했다 (현재 {got}). "
                      f"좌표가 좌우로 뒤집혀 캘리브레이션이 틀어진다.")
        except Exception as e:
            print(f"[source] 경고: {name} 미러링 제어 실패 ({e}). 좌우 반전 여부를 직접 확인할 것.")

    def read(self) -> tuple[np.ndarray, np.ndarray]:
        df = self.depth_stream.read_frame()
        w, h = df.width, df.height
        depth = (np.frombuffer(df.get_buffer_as_uint16(), np.uint16)
                 .reshape(h, w).astype(np.float32) * 0.001)   # mm -> m

        if self.color_cam is not None:
            # 정렬 모드: depth 가 컬러 격자에 실려 오므로 화면도 컬러다.
            # 아직 프레임이 없으면 직전 컬러를, 그것도 없으면 검은 화면을 준다.
            # depth 컬러맵으로 대신 채우면 DEPTH 뷰와 구분이 안 돼 헷갈린다.
            rgb = self.color_cam.latest((w, h))
            if rgb is not None:
                self._last_rgb = rgb
            elif self._last_rgb is None:
                rgb = np.zeros((h, w, 3), np.uint8)
            else:
                rgb = self._last_rgb
            return rgb, depth

        if self.ir_stream is not None:
            irf = self.ir_stream.read_frame()
            ir = np.frombuffer(irf.get_buffer_as_uint16(), np.uint16).reshape(
                irf.height, irf.width).astype(np.float32)
            ir = cv2.normalize(ir, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            if ir.shape != (h, w):
                ir = cv2.resize(ir, (w, h), interpolation=cv2.INTER_NEAREST)
            color = cv2.cvtColor(ir, cv2.COLOR_GRAY2BGR)
        else:
            color = colorize_depth(depth)[0]
        return color, depth

    def pause(self) -> None:
        """depth/IR 스트림을 멈춘다.

        Astra 는 depth/IR/컬러를 USB2.0 한 대역으로 보낸다. 컬러를 켜는 동안
        이쪽을 멈추지 않으면 둘 다 프레임이 밀린다. 멈추기만 하고 장치는
        열어 둔 채라 되살리는 것은 start() 한 번이다.
        """
        if self.color_cam is not None:
            self.color_cam.stop()        # 정렬 모드에서는 컬러도 같이 재운다
        for st, name in ((self.ir_stream, "ir"), (self.depth_stream, "depth")):
            if st is None:
                continue
            try:
                st.stop()
            except Exception as e:
                print(f"[source] {name} 스트림 정지 실패: {e}")

    def resume(self) -> None:
        if self.color_cam is not None:
            self.color_cam.start()
        for st, name in ((self.depth_stream, "depth"), (self.ir_stream, "ir")):
            if st is None:
                continue
            try:
                st.start()
            except Exception as e:
                print(f"[source] {name} 스트림 재시작 실패: {e} - 다시 만든다")
                try:                      # 마지막 수단: 스트림을 새로 만든다
                    new = (self.dev.create_depth_stream() if name == "depth"
                           else self.dev.create_ir_stream())
                    new.start()
                    self._set_mirror(new, name)
                    if name == "depth":
                        self.depth_stream = new
                    else:
                        self.ir_stream = new
                except Exception as e2:
                    print(f"[source] {name} 스트림 복구 실패: {e2}")

    def close(self) -> None:
        try:
            if self.color_cam is not None:
                self.color_cam.close()
            self.depth_stream.stop()
            if self.ir_stream is not None:
                self.ir_stream.stop()
        finally:
            self._openni2.unload()


class ObsensorColorSource(DepthSource):
    """OpenCV 내장 OBSENSOR 백엔드. 이 장비에서는 컬러만 나온다.

    depth 가 전혀 오지 않으므로 클릭-투-픽에는 쓸 수 없다. 카메라 화각 확인이나
    컬러 전용 디버깅에만 쓴다.
    """

    def __init__(self, index: int = 0):
        self.cap = cv2.VideoCapture(index, cv2.CAP_OBSENSOR)
        if not self.cap.isOpened():
            raise RuntimeError("OBSENSOR 장치를 열 수 없음")
        for _ in range(10):
            if self.cap.grab():
                break
        ok, bgr = self.cap.retrieve(flag=cv2.CAP_OBSENSOR_BGR_IMAGE)
        if not ok or bgr is None:
            self.cap.release()
            raise RuntimeError("OBSENSOR 컬러 프레임 수신 실패")
        h, w = bgr.shape[:2]
        self.intr = default_intrinsics(w, h)

    def read(self) -> tuple[np.ndarray, np.ndarray]:
        if not self.cap.grab():
            raise RuntimeError("OBSENSOR grab 실패")
        ok, bgr = self.cap.retrieve(flag=cv2.CAP_OBSENSOR_BGR_IMAGE)
        if not ok or bgr is None:
            raise RuntimeError("OBSENSOR 컬러 프레임 없음")
        return bgr, np.zeros(bgr.shape[:2], np.float32)   # depth 없음

    def close(self) -> None:
        self.cap.release()


# --- 일반(컬러) 카메라 --------------------------------------------------------
# depth 와 같은 Astra 장치의 컬러를 쓴다. Astra Pro 의 컬러는 depth 와 별개의
# UVC 인터페이스(VID 2BC5)로 붙어 있어서, 그냥 index 0 을 열면 노트북 내장
# 웹캠이 잡힌다. 그래서 장치 목록에서 Orbbec VID 를 찾아 그 번호를 연다.
#
# 컬러는 depth 와 픽셀이 대응하지 않는다(렌즈가 다른 별개 센서). 그래서 '보기'
# 전용이고 컬러 모드에서는 클릭을 막는다.
#
# 자동 검출이 틀리면 $env:COLOR_CAM_INDEX 로 번호를 직접 지정한다.



COLOR_CAM_PATH = os.path.join(_HERE, "color_cam.json")


def save_color_cam_index(index: int) -> None:
    """사람이 눈으로 확인한 카메라 번호를 남긴다.

    USB 열거 순서는 뽑았다 꽂으면 바뀌고, IR 과 컬러는 생김새가 달라 장면 매칭도
    잘 붙지 않는다. 결국 가장 확실한 건 '사람이 한 번 보고 고른 것'이다.
    """
    try:
        with open(COLOR_CAM_PATH, "w", encoding="utf-8") as f:
            json.dump({"index": int(index),
                       "saved_at": time.strftime("%Y-%m-%d %H:%M:%S")}, f, indent=2)
        print(f"[color] 카메라 번호 {index} 저장: {COLOR_CAM_PATH}")
    except OSError as e:
        print(f"[color] 카메라 번호 저장 실패: {e}")


def load_color_cam_index() -> Optional[int]:
    if not os.path.exists(COLOR_CAM_PATH):
        return None
    try:
        with open(COLOR_CAM_PATH, encoding="utf-8") as f:
            return int(json.load(f)["index"])
    except (ValueError, KeyError, OSError):
        return None


def find_uvc_index(vid: str = ORBBEC_VID) -> tuple[Optional[int], str]:
    """UVC 카메라 목록에서 그 VID 장치가 몇 번째인지. (인덱스, 이름)

    OpenCV 는 장치 이름을 알려주지 않고 번호만 받는다. usbvideo 드라이버를 쓰는
    장치를 순서대로 세면 DirectShow 가 매기는 번호와 같은 순서가 된다.
    """
    if os.name != "nt":
        return None, ""
    ps = ("Get-CimInstance Win32_PnPEntity -Filter \"Service='usbvideo'\" | "
          "ForEach-Object { $_.Name + '|' + $_.DeviceID }")
    try:
        out = subprocess.run(["powershell", "-NoProfile", "-Command", ps],
                             capture_output=True, text=True, timeout=15).stdout
    except (OSError, subprocess.SubprocessError):
        return None, ""
    for i, line in enumerate(l for l in out.splitlines() if l.strip()):
        name, _, dev = line.partition("|")
        if vid in dev.upper():
            return i, name.strip()
    return None, ""


class ColorCamera:
    """별도 스레드로 프레임을 받아 두는 컬러 카메라.

    UI 스레드에서 직접 열고 읽으면 화면이 멈춘다. 실제로 멈춘 이유가 셋이다:
      - 장치를 찾으려고 PowerShell 을 부른다 (수백 ms ~ 수 초)
      - DSHOW 초기화가 3초 넘게 걸린다
      - Astra 는 USB2.0 한 대역을 depth/IR 과 나눠 쓴다. 컬러까지 받으면
        read() 가 길게 막히고, 심하면 돌아오지 않는다
    그래서 여는 것도 읽는 것도 이 스레드에서만 한다. UI 는 '가장 최근 프레임'을
    잠깐 잠그고 가져갈 뿐이라 절대 기다리지 않는다.

    컬러 모드를 나가면 장치를 닫는다. 켜 둔 채로 두면 depth 가 쓸 대역을 계속
    빼앗는다.
    """

    STALL_SEC = 8.0          # 이 시간 동안 프레임이 없으면 포기한다

    def __init__(self, forced_index: Optional[str] = COLOR_CAM_INDEX):
        self.forced_index = int(forced_index) if forced_index not in (None, "") else None
        if self.forced_index is None:
            self.forced_index = load_color_cam_index()   # 사람이 골라 둔 번호
        self.index: Optional[int] = None                 # 지금 열려 있는 번호
        self.note = ""                       # 화면에 띄울 상태 문구
        self.failed = False
        self._frame: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._closing: Optional[threading.Thread] = None
        self._run = False
        self.ref_frame: Optional[np.ndarray] = None   # 같은 장면인지 볼 기준 영상
        self.opened = False                           # 장치가 실제로 열렸는가

    # --- UI 스레드에서 부르는 것들 ----------------------------------------
    def start(self, ref: Optional[np.ndarray] = None) -> None:
        """ref 를 주면 그 장면과 같은 것을 보는 카메라를 골라 연다.

        센서(IR) 영상을 넘기면 된다. 번호만 믿으면 노트북 웹캠이 잡힐 수 있다.
        """
        if self._thread is not None and self._thread.is_alive():
            return
        if ref is not None:
            self.ref_frame = ref.copy()
        self.failed = False
        self.note = "컬러 카메라 여는 중..."
        self._run = True
        prev, self._closing = self._closing, None   # 아직 닫히는 중인 이전 장치
        self._thread = threading.Thread(target=self._loop, args=(prev,), daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """즉시 돌아온다. read() 에 물려 있는 스레드는 스스로 빠져나가며 장치를 닫는다."""
        self._run = False
        t, self._thread = self._thread, None
        if t is not None:
            t.join(timeout=0.05)
            if t.is_alive():
                self._closing = t            # 다음에 켤 때 이것만 기다리면 된다
        with self._lock:
            self._frame = None
        self.note = ""

    def latest(self, size: tuple[int, int]) -> Optional[np.ndarray]:
        """가장 최근 프레임을 size=(w, h) 에 맞춰 돌려준다. 없으면 None."""
        with self._lock:
            bgr = None if self._frame is None else self._frame.copy()
        if bgr is None:
            return None
        if (bgr.shape[1], bgr.shape[0]) != size:
            bgr = cv2.resize(bgr, size, interpolation=cv2.INTER_AREA)
        return bgr

    def next_index(self) -> int:
        """다음 카메라 번호로 바꿔 다시 연다. 화면을 보고 고르라는 용도다."""
        # 열기에 실패했을 수도 있으므로 '요청한 번호' 를 우선으로 센다.
        # 열린 번호만 보면 실패한 자리에서 계속 맴돈다.
        cur = self.forced_index if self.forced_index is not None else self.index
        nxt = ((cur if cur is not None else -1) + 1) % 4
        self.forced_index = nxt
        # 저장은 실제로 열린 뒤에 한다. 없는 번호를 저장해 두면 다음 실행이
        # 그 번호로 시작해서 매번 실패한다.
        self.stop()
        self.start()
        return nxt

    def close(self) -> None:
        self.stop()

    # --- 카메라 스레드 ----------------------------------------------------
    def _loop(self, prev: Optional[threading.Thread] = None) -> None:
        # 같은 장치를 두 번 열지 않도록, 이전 스레드가 닫는 것을 여기서 기다린다.
        # UI 스레드에서 기다리면 모드를 바꿀 때마다 1초씩 멈춘다.
        if prev is not None and prev.is_alive():
            prev.join(timeout=3.0)
        if not self._run:
            return
        cap, obsensor = self._open()
        if cap is None:
            self.failed = True
            self.note = "컬러 카메라를 열 수 없습니다 (콘솔 참고)"
            return
        self.opened = True
        self.note = "첫 프레임 기다리는 중..."
        last = time.time()
        try:
            while self._run:
                if obsensor:
                    ok = cap.grab()
                    ok, bgr = (cap.retrieve(flag=cv2.CAP_OBSENSOR_BGR_IMAGE)
                               if ok else (False, None))
                else:
                    ok, bgr = cap.read()
                if ok and bgr is not None:
                    with self._lock:
                        self._frame = bgr
                    self.note = ""
                    last = time.time()
                elif time.time() - last > self.STALL_SEC:
                    self.failed = True
                    self.note = f"컬러 프레임이 {self.STALL_SEC:.0f}초간 오지 않습니다"
                    print(f"[color] {self.note} - 컬러를 끄고 센서 화면으로 돌아간다")
                    break
                else:
                    time.sleep(0.01)
        finally:
            self.opened = False
            cap.release()
            print("[color] 컬러 카메라 닫음")

    def _open(self) -> tuple[Optional[cv2.VideoCapture], bool]:
        if self.forced_index is not None:
            # 지정된 번호가 없는 장치일 수 있다(카메라 2대뿐인데 2번을 고른 경우).
            # 그러면 다음 번호로 넘어가며 열리는 것을 찾는다 - 사용자가 CAM 을
            # 눌러 빈 번호에 걸렸을 때 거기서 막히지 않게.
            for k in range(4):
                i = (self.forced_index + k) % 4
                cap = self._open_uvc(i, "지정된 번호" if k == 0 else "다음으로 열리는 번호")
                if cap is not None:
                    if i != self.forced_index:
                        print(f"[color] index={self.forced_index} 는 없어서 {i} 로 넘어갔다")
                    self.forced_index = i
                    save_color_cam_index(i)          # 실제로 열린 번호만 저장한다
                    return cap, False
            print("[color] 열 수 있는 카메라가 없다")
            return None, False
        cap = self._try_obsensor()
        if cap is not None:
            return cap, True

        idx, name = find_uvc_index()
        why = f"Orbbec {ORBBEC_VID} = '{name}'" if idx is not None else ""
        # 후보를 하나씩 열어보는 스캔은 기본으로 하지 않는다. 카메라를 열고 닫는
        # 데만 장치당 몇 초가 걸리는데, 정작 IR<->컬러 장면 매칭은 내부점이 4점
        # 대 0점 수준이라 그 시간을 들일 만큼 믿을 수 없다. 틀린 카메라가 잡히면
        # 화면의 CAM 버튼 한 번이 훨씬 빠르고 확실하다.
        if self.ref_frame is None or not COLOR_SCAN:
            if idx is None:
                print(f"[color] Orbbec({ORBBEC_VID}) 컬러 카메라를 찾지 못했다. "
                      f"$env:COLOR_CAM_INDEX 로 번호를 지정할 것")
                return None, False
            return self._open_uvc(idx, why), False

        # 장면으로 확인한다. USB 열거 순서는 뽑았다 꽂으면 바뀌어서, VID 로 센
        # 번호가 노트북 웹캠을 가리키는 일이 실제로 생긴다. depth 센서와 같은
        # 장면을 보는 카메라가 우리가 찾는 것이다.
        order = ([idx] if idx is not None else []) + [i for i in range(4) if i != idx]
        scores: list[tuple[int, int]] = []
        for i in order:
            frame = self._peek(i)
            if frame is None:
                continue
            score = scene_match_score(self.ref_frame, frame)
            print(f"[color] index={i} 장면 일치 {score}점"
                  + (f"  ({why})" if i == idx else ""))
            scores.append((score, i))
        scores.sort(reverse=True)
        if scores and scores[0][0] >= SCENE_MATCH_MIN:
            top, second = scores[0], (scores[1] if len(scores) > 1 else (0, None))
            if top[0] > second[0]:
                return self._open_uvc(top[1], f"센서와 같은 장면 {top[0]}점"), False
            print(f"[color] 후보가 비슷하다({scores[:2]}) - 번호로 고른다")
        if idx is not None:
            print(f"[color] 장면으로 못 고름(최고 {scores[0][0] if scores else 0}점) - "
                  f"VID 로 찾은 index={idx} 를 쓴다. 화면이 엉뚱하면 CAM 버튼으로 바꿀 것")
            return self._open_uvc(idx, why), False
        print("[color] 쓸 만한 컬러 카메라를 찾지 못했다")
        return None, False

    def _peek(self, index: int) -> Optional[np.ndarray]:
        """그 번호를 잠깐 열어 한 장만 본다. 확인용이라 바로 닫는다."""
        backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY
        cap = cv2.VideoCapture(index, backend)
        try:
            if not cap.isOpened():
                return None
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            frame = None
            for _ in range(3):                 # 처음 몇 장은 검은 화면일 수 있다
                ok, f = cap.read()
                if ok and f is not None:
                    frame = f
            return frame
        finally:
            cap.release()

    def _try_obsensor(self) -> Optional[cv2.VideoCapture]:
        """OpenCV 가 OBSENSOR 백엔드를 갖고 있으면 그게 제일 깔끔하다.
        (빌드에 따라 없다. 없으면 조용히 UVC 로 간다.)"""
        try:
            if cv2.CAP_OBSENSOR not in cv2.videoio_registry.getCameraBackends():
                return None
        except Exception:
            return None
        cap = cv2.VideoCapture(0, cv2.CAP_OBSENSOR)
        if cap.isOpened() and cap.grab():
            ok, bgr = cap.retrieve(flag=cv2.CAP_OBSENSOR_BGR_IMAGE)
            if ok and bgr is not None:
                print(f"[color] Astra 컬러 열림 (OBSENSOR {bgr.shape[1]}x{bgr.shape[0]})")
                return cap
        cap.release()
        return None

    def _open_uvc(self, index: int, why: str) -> Optional[cv2.VideoCapture]:
        backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY
        t0 = time.time()
        cap = cv2.VideoCapture(index, backend)
        if not cap.isOpened():
            cap.release()
            print(f"[color] 카메라 index={index} 를 열 수 없다 ({why})")
            return None
        # MJPEG 를 먼저 요청한다. 기본값 YUY2 는 640x480x30 에 약 18MB/s 를 쓰는데,
        # Astra 는 그 USB2.0 대역을 depth 와 나눠 쓴다. 압축 포맷이면 2~3MB/s 라
        # depth 와 같이 흘러도 서로 굶지 않는다. (ros_astra_camera 도 mjpeg 기본)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        # 해상도를 명시한다. 기본이 1280x720 인 장치면 대역을 더 먹는다.
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
        got = "".join(chr((fourcc >> 8 * i) & 0xFF) for i in range(4)) if fourcc else "?"
        print(f"[color] 포맷 {got} {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}"
              f"x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
        self.index = index
        print(f"[color] 컬러 카메라 열림: index={index} ({why}, {time.time()-t0:.1f}s)")
        print("        화면이 엉뚱한 카메라면 화면의 CAM 버튼을 눌러 바꿀 것 "
              "(선택은 color_cam.json 에 저장된다)")
        return cap


def scene_match_score(ref_bgr: np.ndarray, cand_bgr: np.ndarray) -> int:
    """두 영상이 '같은 장면'인지 점수(내부점 수)로 답한다.

    장치 번호로 카메라를 고르는 것은 못 믿는다 - USB 열거 순서는 뽑았다 꽂으면
    바뀐다. depth 센서가 보는 장면과 같은 것을 보는 카메라가 우리가 찾는 것이다.
    노트북 내장 웹캠은 사람 얼굴을 보고 있으니 점수가 바닥이다.
    """
    if ref_bgr is None or cand_bgr is None:
        return 0
    if ref_bgr.shape[:2] != cand_bgr.shape[:2]:
        cand_bgr = cv2.resize(cand_bgr, (ref_bgr.shape[1], ref_bgr.shape[0]))
    clahe = cv2.createCLAHE(2.0, (8, 8))
    g1 = clahe.apply(cv2.cvtColor(cv2.resize(ref_bgr, None, fx=.5, fy=.5), cv2.COLOR_BGR2GRAY))
    g2 = clahe.apply(cv2.cvtColor(cv2.resize(cand_bgr, None, fx=.5, fy=.5), cv2.COLOR_BGR2GRAY))
    orb = cv2.ORB_create(1200)
    k1, d1 = orb.detectAndCompute(g1, None)
    k2, d2 = orb.detectAndCompute(g2, None)
    if d1 is None or d2 is None or len(k1) < 20 or len(k2) < 20:
        return 0
    pairs = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(d1, d2, k=2)
    good = [a for a, b in pairs if len(pairs[0]) == 2 and a.distance < 0.75 * b.distance]
    if len(good) < 10:
        return 0
    src = np.float32([k1[a.queryIdx].pt for a in good]).reshape(-1, 1, 2)
    dst = np.float32([k2[a.trainIdx].pt for a in good]).reshape(-1, 1, 2)
    _, mask = cv2.findHomography(src, dst, cv2.RANSAC, 4.0)
    return 0 if mask is None else int(mask.sum())


# IR 과 컬러는 밝기 특성이 달라서 같은 장면이라도 내부점이 10개 안팎으로 적게
# 나온다. 절대 기준을 높게 잡으면 정작 맞는 카메라를 떨어뜨린다. 그래서 문턱은
# 낮게 두고 '후보 중 최고' 를 고른다.
SCENE_MATCH_MIN = 5


class FileSource(DepthSource):
    """저장된 color + depth 파일 재생. depth 는 16bit PNG(mm) 또는 .npy(m)."""

    def __init__(self, color_path: str, depth_path: str, intr: Optional[Intrinsics] = None):
        color = cv2.imread(color_path, cv2.IMREAD_COLOR)
        if color is None:
            raise FileNotFoundError(color_path)

        if depth_path.endswith(".npy"):
            depth = np.load(depth_path).astype(np.float32)
        else:
            raw = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
            if raw is None:
                raise FileNotFoundError(depth_path)
            depth = raw.astype(np.float32) / 1000.0  # mm -> m

        if depth.shape[:2] != color.shape[:2]:
            depth = cv2.resize(depth, (color.shape[1], color.shape[0]),
                               interpolation=cv2.INTER_NEAREST)

        self._color, self._depth = color, depth
        h, w = color.shape[:2]
        self.intr = intr or default_intrinsics(w, h)

    def read(self) -> tuple[np.ndarray, np.ndarray]:
        return self._color.copy(), self._depth.copy()


class SyntheticSource(DepthSource):
    """하드웨어가 없을 때의 데모용. 기울어진 테이블 평면 + 가짜 물체 융기."""

    def __init__(self, color_path: Optional[str] = None, width: int = 640, height: int = 480):
        color = cv2.imread(color_path, cv2.IMREAD_COLOR) if color_path else None
        if color is None:
            color = np.full((height, width, 3), 60, np.uint8)
            for cx, cy, r, bgr in [(200, 250, 40, (60, 90, 220)),
                                   (400, 200, 35, (90, 200, 90)),
                                   (320, 350, 30, (220, 170, 60))]:
                cv2.circle(color, (cx, cy), r, bgr, -1)
        else:
            color = cv2.resize(color, (width, height))

        self._color = color
        h, w = color.shape[:2]
        self.intr = default_intrinsics(w, h)

        # 카메라에서 0.35m 떨어진, 아래로 갈수록 멀어지는 테이블면
        vv, _uu = np.mgrid[0:h, 0:w].astype(np.float32)
        depth = 0.35 + (vv / h) * 0.10
        # 밝은 영역을 "물체"로 보고 2cm 솟아오르게 한다
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY).astype(np.float32)
        bump = cv2.GaussianBlur((gray > 110).astype(np.float32), (31, 31), 0)
        self._depth = depth - bump * 0.02

    def read(self) -> tuple[np.ndarray, np.ndarray]:
        return self._color.copy(), self._depth.copy()


def default_intrinsics(w: int, h: int) -> Intrinsics:
    """대략 60도 수평 화각 가정."""
    fx = fy = w / (2.0 * math.tan(math.radians(60.0) / 2.0))
    return Intrinsics(fx, fy, w / 2.0, h / 2.0, w, h)


def open_depth_source() -> DepthSource:
    """OpenNI2(Astra Pro) -> Orbbec SDK -> RealSense -> 파일 -> 합성 순으로 연다."""
    try:
        src = OpenNI2Source()
        print("[source] OpenNI2(Astra) 연결됨")
        return src
    except ImportError:
        print("[source] primesense 미설치 - OpenNI2 경로 사용 불가")
    except Exception as e:
        print(f"[source] OpenNI2 사용 불가 ({e.__class__.__name__}: {e})")
        if not os.environ.get("OPENNI2_REDIST"):
            print("         OPENNI2_REDIST 환경변수로 OpenNI2 Redist 폴더를 지정해야 한다.")

    try:
        src = OrbbecSource()
        print("[source] Orbbec 연결됨")
        return src
    except ImportError:
        print("[source] pyorbbecsdk 미설치 - Orbbec depth 사용 불가")
    except Exception as e:
        print(f"[source] Orbbec 사용 불가 ({e.__class__.__name__}: {e})")

    try:
        src = RealSenseSource()
        print("[source] RealSense 연결됨")
        return src
    except Exception as e:
        print(f"[source] RealSense 사용 불가 ({e.__class__.__name__}: {e})")

    if os.path.exists("depth.npy") and os.path.exists("color.png"):
        print("[source] color.png + depth.npy 재생")
        return FileSource("color.png", "depth.npy")

    print("[source] 합성 depth 데모 모드")
    return SyntheticSource("color.png" if os.path.exists("color.png") else None)


# ---------------------------------------------------------------------------
# 2. 역투영: 픽셀 + depth -> 카메라 3D 좌표
# ---------------------------------------------------------------------------

def sample_depth(depth: np.ndarray, u: int, v: int, patch: int = DEPTH_PATCH,
                 near: bool = False) -> float:
    """클릭 지점 주변의 유효 depth. 홀(0)은 제외한다.

    near=True 는 창 안에서 '가장 앞면' 무리만 골라 그 median 을 쓴다.
    그리퍼 끝처럼 얇은 것을 클릭하면 5x5 창의 대부분이 뒤쪽 테이블이라 그냥
    median 을 쓰면 그리퍼가 아니라 테이블 거리가 나온다. 그러면 허공에 있던
    점이 테이블 위로 무너져서 캘리브레이션이 통째로 틀어진다.
    """
    h, w = depth.shape[:2]
    if not (0 <= u < w and 0 <= v < h):
        return 0.0
    r = patch // 2
    win = depth[max(0, v - r):min(h, v + r + 1), max(0, u - r):min(w, u + r + 1)]
    valid = win[(win > 1e-4) & np.isfinite(win)]
    if not valid.size:
        return 0.0
    if near:
        valid = valid[valid <= valid.min() + DEPTH_FRONT_BAND]
    return float(np.median(valid))


def fill_depth_holes(depth: np.ndarray,
                     max_px: int = DEPTH_FILL_MAX_PX) -> tuple[np.ndarray, np.ndarray]:
    """depth 구멍을 주변 유효값으로 메운다. (채운 depth, 채운 자리 마스크)

    구멍은 대개 경계면/반사/최소거리 미만에서 생긴다. 그 화소를 그냥 버리면
    물체 가장자리를 클릭했을 때 "depth 가 없습니다" 로 거부되는데, 정작 집고
    싶은 곳이 그 가장자리다.

    유효 이웃의 평균으로 한 겹씩 안쪽으로 번져 들어간다(9x9 창으로 4화소씩).
    큰 구멍의 한가운데까지 지어내지는 않는다 - 가장자리에서 max_px 화소까지만
    닿고 그 안쪽은 여전히 무효로 남는다. 없는 데이터를 넓게 지어내면 로봇이
    엉뚱한 곳으로 가기 때문이다.

    채운 값은 '추정'이다. 어디가 추정인지 마스크로 돌려주므로, 표시할 때
    구분하고 클릭할 때 경고할 수 있다.
    """
    valid = (depth > 1e-4) & np.isfinite(depth)
    if valid.all() or not valid.any():
        return depth, np.zeros(depth.shape, bool)

    out = np.where(valid, depth, 0.0).astype(np.float32)
    m = valid.astype(np.float32)
    k = 9                                   # 한 번에 4화소씩 번진다
    for _ in range(max(1, -(-max_px // (k // 2)))):
        num = cv2.boxFilter(out, -1, (k, k), normalize=False, borderType=cv2.BORDER_ISOLATED)
        den = cv2.boxFilter(m, -1, (k, k), normalize=False, borderType=cv2.BORDER_ISOLATED)
        hole = (m < 0.5) & (den > 0.5)
        if not hole.any():
            break
        out[hole] = num[hole] / den[hole]
        m[hole] = 1.0

    filled = (m > 0.5) & ~valid
    out[m < 0.5] = 0.0                  # 못 채운 곳은 그대로 '없음'
    return out, filled


def deproject(intr: Intrinsics, u: float, v: float, z: float) -> np.ndarray:
    """픽셀 + 깊이 -> 카메라 좌표계 3D 점 [m]. (+X 오른쪽, +Y 아래, +Z 전방)"""
    return np.array([(u - intr.cx) * z / intr.fx,
                     (v - intr.cy) * z / intr.fy,
                     z], dtype=np.float64)


def project(intr: Intrinsics, p_cam: Sequence[float]) -> Optional[tuple[float, float]]:
    """카메라 3D 점 -> 픽셀. deproject 의 역.

    uv 를 남기지 않은 옛 점 기록도 화면에 되돌려 그릴 수 있어야 한다.
    """
    x, y, z = (float(v) for v in p_cam[:3])
    if z <= 1e-6:                      # 카메라 뒤/원점의 점은 투영되지 않는다
        return None
    return (x * intr.fx / z + intr.cx, y * intr.fy / z + intr.cy)


# ---------------------------------------------------------------------------
# 3. Hand-eye 변환 (카메라 -> 로봇 베이스)
# ---------------------------------------------------------------------------

class HandEye:
    """4x4 강체변환. 로봇좌표 p_base = R @ p_cam + t"""

    def __init__(self, T: Optional[np.ndarray] = None):
        self.T = np.eye(4) if T is None else np.asarray(T, dtype=np.float64)
        self.calibrated = T is not None

    def cam_to_base(self, p_cam: Sequence[float]) -> np.ndarray:
        p = np.asarray(p_cam, dtype=np.float64)
        return self.T[:3, :3] @ p + self.T[:3, 3]

    def save(self, path: str = HANDEYE_PATH) -> None:
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"T_base_cam": self.T.tolist()}, f, indent=2)
        print(f"[handeye] 저장: {path}")

    @classmethod
    def load(cls, path: str = HANDEYE_PATH) -> "HandEye":
        if not os.path.exists(path):
            print(f"[handeye] {path} 없음 -> 캘리브레이션 필요 (UI에서 'k')")
            return cls(None)
        with open(path, encoding="utf-8") as f:
            T = np.array(json.load(f)["T_base_cam"], dtype=np.float64)
        print(f"[handeye] 로드: {path}")
        return cls(T)


def euler_to_R(rx: float, ry: float, rz: float) -> np.ndarray:
    """도 단위 오일러각(ZYX) -> 회전행렬. 손으로 조절하기엔 회전벡터보다 직관적이다."""
    a, b, c = math.radians(rx), math.radians(ry), math.radians(rz)
    Rx = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    Ry = np.array([[math.cos(b), 0, math.sin(b)], [0, 1, 0], [-math.sin(b), 0, math.cos(b)]])
    Rz = np.array([[math.cos(c), -math.sin(c), 0], [math.sin(c), math.cos(c), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def R_to_euler(R: np.ndarray) -> tuple[float, float, float]:
    """회전행렬 -> 도 단위 오일러각(ZYX). euler_to_R 의 역변환."""
    sy = -R[2, 0]
    if abs(sy) < 0.999999:
        rx = math.atan2(R[2, 1], R[2, 2])
        ry = math.asin(float(np.clip(sy, -1.0, 1.0)))
        rz = math.atan2(R[1, 0], R[0, 0])
    else:                                   # 짐벌락
        rx = math.atan2(-R[1, 2], R[1, 1])
        ry = math.pi / 2 * math.copysign(1.0, sy)
        rz = 0.0
    return math.degrees(rx), math.degrees(ry), math.degrees(rz)


def solve_rigid_transform(cam_pts: np.ndarray, base_pts: np.ndarray) -> tuple[np.ndarray, float]:
    """Kabsch/SVD 로 대응점쌍에서 최적 강체변환을 구한다. (>=3점, 실용상 >=4점)

    반환: (4x4 변환, RMS 잔차[m])
    """
    A = np.asarray(cam_pts, dtype=np.float64)
    B = np.asarray(base_pts, dtype=np.float64)
    if A.shape != B.shape or len(A) < 3:
        raise ValueError("대응점이 3쌍 이상 필요하고 개수가 같아야 한다")

    ca, cb = A.mean(0), B.mean(0)
    H = (A - ca).T @ (B - cb)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    t = cb - R @ ca

    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    rms = float(np.sqrt((((A @ R.T + t) - B) ** 2).sum(1).mean()))
    return T, rms


# --- 점 데이터 자체의 품질 (변환을 풀기 전에 본다) ---------------------------
GAP_TOL = 0.010      # 점쌍 거리 불일치 허용치 [m]
FLAT_TOL = 0.015     # 점들이 한 평면에 눌려 있다고 볼 두께 [m]


def pair_gap_matrix(pairs: Sequence[tuple[np.ndarray, np.ndarray]]) -> np.ndarray:
    """점쌍 사이의 |카메라 거리 - 로봇 거리| [m].

    강체변환은 거리를 보존한다. 그래서 이 값은 해를 구하기 전에, 어떤 알고리즘을
    쓰든 상관없이 '점 데이터가 서로 모순되는가' 만을 말해준다. 여기가 크면
    Kabsch 든 PnP 든 맞출 수 없다 - 고칠 것은 알고리즘이 아니라 점이다.
    """
    n = len(pairs)
    G = np.zeros((n, n))
    for i in range(n):
        for j in range(i + 1, n):
            dc = float(np.linalg.norm(pairs[i][0] - pairs[j][0]))
            db = float(np.linalg.norm(pairs[i][1] - pairs[j][1]))
            G[i, j] = G[j, i] = abs(dc - db)
    return G


def plane_thickness(pts: np.ndarray) -> float:
    """점들이 놓인 평면의 두께 [m] (최소 특이값). 0 에 가까우면 전부 한 평면 위다."""
    X = np.asarray(pts, float)
    if len(X) < 3:
        return 0.0
    return float(np.linalg.svd(X - X.mean(0), compute_uv=False)[-1])


def point_set_quality(pairs: Sequence[tuple[np.ndarray, np.ndarray]]) -> dict:
    """점 집합의 상태를 한 번에 진단한다."""
    n = len(pairs)
    if n < 2:
        return {"n": n, "gap_max": 0.0, "gap_mean": 0.0, "worst": -1,
                "flat_cam": 0.0, "flat_base": 0.0}
    G = pair_gap_matrix(pairs)
    iu = np.triu_indices(n, 1)
    per_point = G.max(1)                       # 점마다 가장 심한 불일치
    return {"n": n,
            "gap_max": float(G[iu].max()),
            "gap_mean": float(G[iu].mean()),
            "worst": int(np.argmax(per_point)),
            "per_point": per_point,
            "flat_cam": plane_thickness(np.array([a for a, _ in pairs])),
            "flat_base": plane_thickness(np.array([b for _, b in pairs]))}


def quality_report(pairs: Sequence[tuple[np.ndarray, np.ndarray]]) -> list[str]:
    """사람이 읽을 진단문. 문제가 있으면 무엇을 다시 찍어야 하는지까지 말한다."""
    q = point_set_quality(pairs)
    if q["n"] < 2:
        return ["점이 2개 미만 - 진단할 것이 없다"]
    out = [f"점 {q['n']}개: 거리 불일치 평균 {q['gap_mean']*1000:.1f}mm "
           f"최대 {q['gap_max']*1000:.1f}mm (0 이어야 정상)"]
    if q["gap_max"] > GAP_TOL:
        out.append(f"  -> {q['worst']+1}번 점이 가장 어긋난다 "
                   f"({q['per_point'][q['worst']]*1000:.1f}mm). 그 점을 다시 찍거나 버릴 것")
        out.append("     원인 1: Enter 를 누른 순간 그리퍼 끝이 클릭한 그 지점에 없었다 "
                   "(다른 자세로 옮겼거나 허공에 떠 있었다)")
        out.append("     원인 2: 클릭한 픽셀의 depth 가 그리퍼가 아니라 뒤쪽 테이블에서 잡혔다")
    if q["flat_cam"] < FLAT_TOL and q["flat_base"] > FLAT_TOL:
        out.append(f"  -> 카메라 점들은 두께 {q['flat_cam']*1000:.1f}mm 의 한 평면에 눌려 있는데 "
                   f"로봇 점들은 {q['flat_base']*1000:.1f}mm 로 퍼져 있다")
        out.append("     = 클릭한 지점들은 모두 테이블 위인데 팔은 높이가 서로 달랐다는 뜻. "
                   "그리퍼를 그 지점에 정확히 대고 확정해야 한다")
    elif q["flat_cam"] < FLAT_TOL:
        out.append(f"  -> 점들이 한 평면 위에 있다 (두께 {q['flat_cam']*1000:.1f}mm). "
                   "높이가 다른 점을 섞어야 변환이 안정된다")
    return out


# ---------------------------------------------------------------------------
# 4. SO-101 역기구학 (5-DOF, 접근각 구속)
# ---------------------------------------------------------------------------

class IKError(RuntimeError):
    pass


def so101_ik(x: float, y: float, z: float,
             pitch: float = TOPDOWN_PITCH,
             roll: float = 0.0,
             elbow_up: bool = True) -> dict[str, float]:
    """로봇 베이스 기준 TCP 위치 -> 관절각[rad].

    5-DOF 이므로 자세를 완전히 지정할 수 없다. yaw 는 베이스 pan 에 종속되고,
    pitch(접근각)와 roll(그리퍼 회전)만 자유롭게 지정한다.
    """
    r = math.hypot(x, y)
    if r < 1e-6:
        raise IKError("베이스 회전축 위의 특이점")

    q_pan = math.atan2(y, x)

    # 손목 중심을 TCP 에서 접근각 반대 방향으로 L3 만큼 뒤로 뺀다
    rw = r - L3_WRIST * math.cos(pitch) - L_SHOULDER_R
    zw = (z - L0_BASE_H) - L3_WRIST * math.sin(pitch)

    d2 = rw * rw + zw * zw
    reach = L1_UPPER + L2_FORE
    if d2 > reach * reach or d2 < (L1_UPPER - L2_FORE) ** 2:
        raise IKError(f"도달 불가: 손목중심 거리 {math.sqrt(d2):.3f}m (최대 {reach:.3f}m)")

    cos_e = (d2 - L1_UPPER ** 2 - L2_FORE ** 2) / (2 * L1_UPPER * L2_FORE)
    bend = math.acos(float(np.clip(cos_e, -1.0, 1.0)))     # 위팔과 아래팔 사이 각
    if elbow_up:
        bend = -bend

    # 먼저 각 링크가 수평과 이루는 각(a1, a2, a3)을 구한다. 관절각은 그 다음이다.
    a1 = math.atan2(zw, rw) - math.atan2(L2_FORE * math.sin(bend),
                                         L1_UPPER + L2_FORE * math.cos(bend))
    a2 = a1 + bend
    a3 = pitch

    # 관절각 = 영자세로부터의 회전량. 영자세는 일직선이 아니므로 그 각을 빼준다.
    q_lift = a1 - A1_ZERO
    q_elbow = bend - (A2_ZERO - A1_ZERO)
    q_wrist = (a3 - a2) - (A3_ZERO - A2_ZERO)

    return {"shoulder_pan": q_pan, "shoulder_lift": q_lift, "elbow_flex": q_elbow,
            "wrist_flex": q_wrist, "wrist_roll": roll}


def so101_fk(q: dict[str, float]) -> np.ndarray:
    """IK 검증용 정기구학. 관절각[rad] -> TCP 위치 [m]."""
    a1 = A1_ZERO + q["shoulder_lift"]
    a2 = A2_ZERO + q["shoulder_lift"] + q["elbow_flex"]
    a3 = A3_ZERO + q["shoulder_lift"] + q["elbow_flex"] + q["wrist_flex"]
    r = (L_SHOULDER_R + L1_UPPER * math.cos(a1) + L2_FORE * math.cos(a2)
         + L3_WRIST * math.cos(a3))
    z = L0_BASE_H + L1_UPPER * math.sin(a1) + L2_FORE * math.sin(a2) + L3_WRIST * math.sin(a3)
    pan = q["shoulder_pan"]
    return np.array([r * math.cos(pan), r * math.sin(pan), z])


def max_z_at(x: float, y: float, pitch: float = TOPDOWN_PITCH) -> float:
    """(x, y) 위에서 주어진 접근각으로 도달 가능한 최대 TCP 높이 [m].

    반경이 클수록 팔이 이미 뻗어 있어 위로 올릴 여유가 줄어든다.
    도달 불가한 반경이면 -inf.
    """
    rw = math.hypot(x, y) - L3_WRIST * math.cos(pitch) - L_SHOULDER_R
    s = REACH - IK_MARGIN
    if abs(rw) >= s:
        return float("-inf")
    return math.sqrt(s * s - rw * rw) + L0_BASE_H + L3_WRIST * math.sin(pitch)


def clamp_z(x: float, y: float, z_wanted: float, z_floor: float,
            pitch: float = TOPDOWN_PITCH) -> float:
    """올리고 싶은 높이를 실제 도달 가능한 범위로 낮춘다."""
    return max(z_floor, min(z_wanted, max_z_at(x, y, pitch), WS_Z_MAX))


def tcp_sanity(tcp: Optional[Sequence[float]]) -> Optional[str]:
    """팔이 스스로 보고한 위치가 물리적으로 가능한가. 가능하면 None.

    팔이 테이블 위에 놓여 있으면 그리퍼가 테이블면(z=0) 아래로 갈 수 없다.
    그런 값이 나온다면 카메라가 아니라 관절 영점(arm_calib.json)이 틀린 것이다.
    이걸 모르고 hand-eye 를 아무리 다시 잡아도 맞지 않는다 - FK 가 자세마다
    다르게 틀리면 그 오차는 하나의 강체변환으로 흡수되지 않기 때문이다.
    """
    if tcp is None:
        return None
    z = float(tcp[2])
    if z < CALIB_Z_MIN:
        return (f"팔이 보고한 높이 z={z*1000:.0f}mm 가 테이블면보다 "
                f"{CALIB_Z_TOL*1000:.0f}mm 넘게 아래다. "
                f"관절 영점이 틀렸다 - calibrate_arm.py 를 다시 돌릴 것")
    return None


def in_safety_box(x: float, y: float, z: float,
                  z_min: float = WS_Z_MIN) -> tuple[bool, str]:
    """거친 사전 필터. 통과해도 IK 가 도달 불가를 낼 수 있다.

    z_min 을 낮추는 것은 캘리브레이션 검증처럼 '이미 그 자세에 있었던 것을
    재현하는' 경우에만 허용한다. 집기 동작은 기본값을 그대로 쓴다.
    """
    r = math.hypot(x, y)
    if not (WS_R_MIN <= r <= WS_R_MAX):
        return False, f"수평거리 {r*100:.1f}cm 가 [{WS_R_MIN*100:.0f}, {WS_R_MAX*100:.0f}]cm 밖"
    if not (z_min <= z <= WS_Z_MAX):
        return False, f"높이 {z*100:.1f}cm 가 [{z_min*100:.1f}, {WS_Z_MAX*100:.0f}]cm 밖"
    return True, ""


def plan_pose(x: float, y: float, z: float,
              pitch: float = TOPDOWN_PITCH, roll: float = 0.0,
              z_min: float = WS_Z_MIN) -> dict[str, float]:
    """안전박스 + IK 를 모두 통과한 관절각을 돌려준다. 실패하면 IKError."""
    ok, why = in_safety_box(x, y, z, z_min)
    if not ok:
        raise IKError(why)
    return so101_ik(x, y, z, pitch, roll)


# ---------------------------------------------------------------------------
# 5. 로봇 백엔드
# ---------------------------------------------------------------------------

JOINT_ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# 관절을 한 개씩 움직일 때의 순서.
# 나갈 때는 베이스부터(pan -> lift -> elbow -> wrist) 펴고, 돌아올 때는 반대로 접는다.
# 손끝이 직선으로 가지는 않지만, 어느 관절이 언제 움직이는지가 눈에 보인다.
ORDER_OUT = tuple(JOINT_ORDER)
ORDER_BACK = tuple(reversed(JOINT_ORDER))

# 실기 서보의 부호/영점은 조립마다 다르다. lerobot 캘리브레이션 후 여기서 보정한다.
JOINT_SIGN = {n: 1.0 for n in JOINT_ORDER}
JOINT_OFFSET_DEG = {n: 0.0 for n in JOINT_ORDER}

HOME_POSE = (0.16, 0.0, 0.12)  # 홈에서의 TCP 위치 [m] (도달 가능한 중립 자세)


class RobotArm:
    """제어 인터페이스. 상위 시퀀스는 이 API 만 사용한다."""

    def connect(self) -> None:
        pass

    def disconnect(self) -> None:
        pass

    def send_joints(self, q: dict[str, float], settle: float = 0.8,
                    order: Optional[Sequence[str]] = None) -> None:
        raise NotImplementedError

    def set_gripper(self, opening: float, settle: float = 0.5) -> None:
        """opening: 0.0(닫힘) ~ 1.0(완전히 열림)"""
        raise NotImplementedError

    def get_tcp(self) -> Optional[np.ndarray]:
        """현재 TCP 위치[m]를 실측으로 돌려준다. 알 수 없으면 None.

        hand-eye 캘리브레이션에서 이 값을 쓰면 사용자가 로봇 좌표를 손으로
        입력할 필요가 없다.
        """
        return None

    def get_joint_counts(self) -> Optional[dict[str, int]]:
        """현재 서보 카운트. 캘리브레이션 점을 찍을 때의 '팔 자세'를 남기는 용도."""
        return None

    def set_torque(self, on: bool) -> None:
        """캘리브레이션 중 팔을 손으로 움직이려면 토크를 꺼야 한다."""
        raise NotImplementedError("이 백엔드는 토크 제어를 지원하지 않는다")

    # --- 파생 동작 ---------------------------------------------------------
    def move_to(self, x: float, y: float, z: float,
                pitch: float = TOPDOWN_PITCH, roll: float = 0.0,
                settle: float = 0.8, z_min: float = WS_Z_MIN,
                order: Optional[Sequence[str]] = None) -> None:
        self.send_joints(plan_pose(x, y, z, pitch, roll, z_min),
                         settle=settle, order=order)

    def open_gripper(self) -> None:
        self.set_gripper(GRIPPER_OPEN)

    def close_gripper(self) -> None:
        self.set_gripper(0.0, settle=0.8)

    def home(self) -> None:
        """홈은 언제나 '돌아오는' 동작이므로 접는 순서로 간다."""
        self.move_to(*HOME_POSE, settle=1.2, order=ORDER_BACK)


class MockArm(RobotArm):
    """하드웨어 없이 전체 파이프라인을 검증하기 위한 백엔드."""

    def __init__(self, speed: float = 1.0):
        self.speed = speed
        self.last_q: dict[str, float] = {}
        self.gripper = 1.0

    def connect(self) -> None:
        print("[arm] MockArm 연결 (실제 동작 없음)")

    def send_joints(self, q: dict[str, float], settle: float = 0.8,
                    order: Optional[Sequence[str]] = None) -> None:
        deg = {k: math.degrees(v) for k, v in q.items()}
        seq = f" [{'>'.join(SHORT_NAME[n] for n in order)} 순서]" if order else ""
        print("[arm] joints deg: " + "  ".join(f"{k}={deg[k]:+7.2f}" for k in JOINT_ORDER) + seq)
        self.last_q = q
        time.sleep(settle / self.speed)

    def set_gripper(self, opening: float, settle: float = 0.5) -> None:
        print(f"[arm] gripper -> {opening:.2f}")
        self.gripper = opening
        time.sleep(settle / self.speed)


class LeRobotSO101Arm(RobotArm):
    """lerobot 의 SO101Follower 를 통한 실제 제어."""

    def __init__(self, port: str, robot_id: str = "so101",
                 gripper_range_deg: tuple[float, float] = (2.0, 35.0)):
        self.port, self.robot_id = port, robot_id
        self.gmin, self.gmax = gripper_range_deg
        self.robot = None

    def connect(self) -> None:
        from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig

        self.robot = SO101Follower(SO101FollowerConfig(port=self.port, id=self.robot_id))
        self.robot.connect()
        print(f"[arm] SO-101 연결: {self.port}")

    def disconnect(self) -> None:
        if self.robot is not None:
            self.robot.disconnect()
            self.robot = None

    def _send(self, action: dict[str, float], settle: float) -> None:
        if self.robot is None:
            raise RuntimeError("로봇이 연결되어 있지 않다")
        self.robot.send_action(action)
        time.sleep(settle)

    def send_joints(self, q: dict[str, float], settle: float = 0.8,
                    order: Optional[Sequence[str]] = None) -> None:
        # lerobot 백엔드는 전 관절 동시 전송만 지원한다 - order 는 쓰지 않는다.
        action = {}
        for name in JOINT_ORDER:
            deg = math.degrees(q[name]) * JOINT_SIGN[name] + JOINT_OFFSET_DEG[name]
            action[f"{name}.pos"] = deg
        self._send(action, settle)

    def set_gripper(self, opening: float, settle: float = 0.5) -> None:
        opening = float(np.clip(opening, 0.0, 1.0))
        self._send({"gripper.pos": self.gmin + (self.gmax - self.gmin) * opening}, settle)


# --- Feetech STS3215 (SO-101 실기) -------------------------------------------
# lerobot 없이 pyserial 만으로 제어한다. lerobot 은 torch 까지 끌고 오는데
# 우리는 IK 를 이미 갖고 있어서 필요한 건 위치 명령뿐이다.

LEROBOT_CALIB = os.path.expanduser(
    "~/.cache/huggingface/lerobot/calibration/robots/so_follower/None.json")
ARM_CALIB_PATH = os.path.join(_HERE, "arm_calib.json")

STS_ADDR = {"torque_enable": 40, "acceleration": 41, "goal_position": 42,
            "goal_speed": 46, "present_position": 56, "present_load": 60, "present_voltage": 62,
            "present_temp": 63, "homing_offset": 31}

SERVO_IDS = {"shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3,
             "wrist_flex": 4, "wrist_roll": 5, "gripper": 6}

COUNTS_PER_DEG = 4096.0 / 360.0
CENTER_COUNT = 2047


class FeetechBus:
    """STS3215 시리얼 프로토콜 최소 구현 (ping / read / write / sync write)."""

    def __init__(self, port: str, baud: int = 1_000_000, timeout: float = 0.06):
        import serial
        self.ser = serial.Serial(port, baud, timeout=timeout)
        # UI 스레드(위치 읽기)와 워커 스레드(동작 명령)가 같은 버스를 쓴다.
        # 락이 없으면 패킷이 서로 끼어들어 양쪽 다 깨진다.
        self._lock = threading.Lock()

    @staticmethod
    def _cks(body: Sequence[int]) -> int:
        return (~sum(body)) & 0xFF

    def _txrx(self, sid: int, instr: int, params: Sequence[int] = (), rxlen: int = 0):
        body = [sid, len(params) + 2, instr, *params]
        with self._lock:
            self.ser.reset_input_buffer()
            self.ser.write(bytes([0xFF, 0xFF, *body, self._cks(body)]))
            resp = self.ser.read(6 + rxlen)
        if len(resp) < 6 + rxlen or resp[0] != 0xFF or resp[1] != 0xFF:
            return None
        return resp

    def ping(self, sid: int) -> bool:
        return self._txrx(sid, 0x01) is not None

    # 시리얼은 가끔 한 패킷을 흘린다. 그때마다 프로그램이 죽으면 안 되므로
    # 몇 번 다시 시도하고, 그래도 안 되면 None 을 돌려 호출측이 판단하게 한다.
    def read_u16(self, sid: int, addr: int, retries: int = 2) -> Optional[int]:
        for _ in range(retries + 1):
            r = self._txrx(sid, 0x02, (addr, 2), rxlen=2)
            if r is not None:
                return r[5] | (r[6] << 8)                   # STS = little endian
        return None

    def read_u8(self, sid: int, addr: int, retries: int = 2) -> Optional[int]:
        for _ in range(retries + 1):
            r = self._txrx(sid, 0x02, (addr, 1), rxlen=1)
            if r is not None:
                return r[5]
        return None

    def write_u8(self, sid: int, addr: int, val: int) -> None:
        self._txrx(sid, 0x03, (addr, val & 0xFF))

    def write_u16(self, sid: int, addr: int, val: int) -> None:
        v = int(val) & 0xFFFF
        self._txrx(sid, 0x03, (addr, v & 0xFF, (v >> 8) & 0xFF))

    def sync_write_positions(self, id_counts: dict[int, int]) -> None:
        """모든 관절을 한 패킷으로 동시에 보낸다 (관절별 시차 방지)."""
        params = [STS_ADDR["goal_position"], 2]
        for sid, cnt in id_counts.items():
            c = int(np.clip(cnt, 0, 4095))
            params += [sid, c & 0xFF, (c >> 8) & 0xFF]
        body = [0xFE, len(params) + 2, 0x83, *params]
        with self._lock:
            self.ser.write(bytes([0xFF, 0xFF, *body, self._cks(body)]))

    def close(self) -> None:
        self.ser.close()


def load_lerobot_calibration(path: str = LEROBOT_CALIB) -> dict:
    """lerobot 캘리브레이션(관절별 id / range_min / range_max)을 읽는다."""
    with open(path, encoding="utf-8") as f:
        return json.load(f)


class ArmCalibration:
    """IK 각도 -> 서보 각도 매핑. 관절마다 부호와 오프셋을 갖는다.

    lerobot 의 homing offset 은 이미 서보 레지스터에 기록되어 있어서 서보가
    보고하는 위치는 '캘리브레이션 좌표계'다. 하지만 그 좌표계의 0도가 우리 IK
    convention(모든 관절 0 = 팔이 수평으로 쭉 뻗은 자세)의 0도와 같다는 보장은
    없다. 그 차이를 실측으로 구한 값이 이 파일이다.
    """

    def __init__(self, mapping: dict[str, dict], ranges: dict[str, dict]):
        self.mapping = mapping
        self.ranges = ranges

    @classmethod
    def load(cls, path: str = ARM_CALIB_PATH) -> Optional["ArmCalibration"]:
        if not os.path.exists(path):
            return None
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
        return cls(d["mapping"], d["ranges"])

    def save(self, path: str = ARM_CALIB_PATH) -> None:
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"mapping": self.mapping, "ranges": self.ranges}, f, indent=2)
        print(f"[arm] 캘리브레이션 저장: {path}")

    def ik_to_count(self, joint: str, angle_rad: float) -> int:
        m = self.mapping[joint]
        servo_deg = math.degrees(angle_rad) * m["sign"] + m["offset_deg"]
        count = CENTER_COUNT + servo_deg * COUNTS_PER_DEG
        r = self.ranges[joint]
        return int(np.clip(round(count), r["range_min"], r["range_max"]))

    def counts_to_ik(self, counts: dict[str, int]) -> dict[str, float]:
        """서보 카운트 -> IK 좌표계 각도[rad]. ik_to_count 의 역변환(클램프 제외).

        캘리브레이션 도구도 이 변환이 필요하다. 식을 두 군데 적어두면 한쪽만
        고쳐져서, 정작 영점을 검증하는 도구가 다른 좌표계를 보게 된다.
        """
        out = {}
        for n in JOINT_ORDER:
            servo_deg = (counts[n] - CENTER_COUNT) / COUNTS_PER_DEG
            m = self.mapping[n]
            out[n] = math.radians((servo_deg - m["offset_deg"]) / m["sign"])
        return out

    def tcp_from_counts(self, counts: dict[str, int]) -> np.ndarray:
        """서보 카운트 -> TCP 위치[m]. 영점이 맞는지 보는 가장 직접적인 창."""
        return so101_fk(self.counts_to_ik(counts))


class FeetechSO101Arm(RobotArm):
    """SO-101 실기 제어. 관절 리밋은 캘리브레이션 range 로 하드 클램프한다."""

    def __init__(self, port: str, calib: ArmCalibration,
                 speed: int = 500, accel: int = 20,
                 max_step_deg: float = 8.0, step_pause: float = 0.06,
                 load_limit: int = 700):
        self.port, self.calib = port, calib
        self.speed, self.accel = speed, accel
        self.load_limit = load_limit
        self.max_step_counts = max_step_deg * COUNTS_PER_DEG
        self.step_pause = step_pause
        self.bus: Optional[FeetechBus] = None

    def connect(self) -> None:
        self.bus = FeetechBus(self.port)
        missing = [n for n, sid in SERVO_IDS.items() if not self.bus.ping(sid)]
        if missing:
            self.bus.close()
            self.bus = None
            raise RuntimeError(f"응답 없는 서보: {missing}")

        # 속도/가속도를 반드시 먼저 제한한 뒤에 토크를 켠다.
        # STS3215 는 goal_speed=0 이 '무제한 최고속'이라, 이걸 빼먹으면 첫 명령에
        # 팔이 최고속으로 튄다. acceleration 기본값도 254(최대)다.
        for sid in SERVO_IDS.values():
            self.bus.write_u16(sid, STS_ADDR["goal_speed"], self.speed)
            self.bus.write_u8(sid, STS_ADDR["acceleration"], self.accel)

        applied = {n: self.bus.read_u16(sid, STS_ADDR["goal_speed"])
                   for n, sid in SERVO_IDS.items()}
        if any(v != self.speed for v in applied.values()):
            self.bus.close()
            self.bus = None
            raise RuntimeError(f"속도 제한 적용 실패: {applied} (기대값 {self.speed}). "
                               f"토크를 켜지 않고 중단한다.")

        # 서보의 Goal_Position 에는 지난 세션의 값이 남아 있다. 그대로 토크를 켜면
        # 그 낡은 목표로 즉시 달려간다(실측 최대 55도). 반드시 먼저 현재 위치로
        # 목표를 덮어써서 "제자리 유지"로 만들어 놓고 토크를 켠다.
        stale = 0.0
        for name, sid in SERVO_IDS.items():
            present = self.bus.read_u16(sid, STS_ADDR["present_position"])
            if present is None:
                self.bus.close()
                self.bus = None
                raise RuntimeError(f"{name} 위치 읽기 실패 - 토크를 켜지 않는다")
            goal = self.bus.read_u16(sid, STS_ADDR["goal_position"])
            if goal is not None:
                stale = max(stale, abs(goal - present) / COUNTS_PER_DEG)
            self.bus.write_u16(sid, STS_ADDR["goal_position"], present)

        for name, sid in SERVO_IDS.items():          # 반영 확인
            g = self.bus.read_u16(sid, STS_ADDR["goal_position"])
            p = self.bus.read_u16(sid, STS_ADDR["present_position"])
            if g is None or abs(g - p) / COUNTS_PER_DEG > 2.0:
                self.bus.close()
                self.bus = None
                raise RuntimeError(f"{name} 목표위치 동기화 실패 (goal={g}, present={p}). "
                                   f"토크를 켜지 않고 중단한다.")

        if stale > 2.0:
            print(f"[arm] 낡은 목표위치 {stale:.1f}deg 발견 -> 현재 위치로 덮어씀")

        for sid in SERVO_IDS.values():
            self.bus.write_u8(sid, STS_ADDR["torque_enable"], 1)

        # 프로세스가 예외/강제종료로 죽으면 finally 가 안 돌아 토크가 켜진 채 남는다.
        # 실제로 그런 일이 있었으므로 인터프리터 종료 훅으로 한 번 더 보장한다.
        atexit.register(self._emergency_torque_off)

        print(f"[arm] SO-101 연결: {self.port} "
              f"(speed={self.speed}, accel={self.accel}, 토크 ON, 제자리 유지)")

    def _emergency_torque_off(self) -> None:
        if self.bus is None:
            return
        try:
            for sid in SERVO_IDS.values():
                self.bus.write_u8(sid, STS_ADDR["torque_enable"], 0)
            print("[arm] (비상) 토크 OFF")
        except Exception:
            pass

    def disconnect(self) -> None:
        if self.bus is None:
            return
        for sid in SERVO_IDS.values():                       # 팔을 부드럽게 놓아준다
            self.bus.write_u8(sid, STS_ADDR["torque_enable"], 0)
        self.bus.close()
        self.bus = None
        try:
            atexit.unregister(self._emergency_torque_off)
        except Exception:
            pass
        print("[arm] 토크 OFF, 연결 해제")

    def read_positions_deg(self) -> dict[str, float]:
        assert self.bus is not None
        out = {}
        for name, sid in SERVO_IDS.items():
            raw = self.bus.read_u16(sid, STS_ADDR["present_position"])
            if raw is not None:
                out[name] = (raw - CENTER_COUNT) / COUNTS_PER_DEG
        return out

    def read_counts(self) -> dict[str, int]:
        assert self.bus is not None
        out = {}
        for name, sid in SERVO_IDS.items():
            raw = self.bus.read_u16(sid, STS_ADDR["present_position"])
            if raw is None:
                raise RuntimeError(f"{name} 위치 읽기 실패")
            out[name] = raw
        return out

    def _ramp_to(self, target: dict[str, int], settle: float) -> None:
        """현재 위치에서 target 카운트까지 잘게 나눠 보낸다 (관절 동시 이동).

        서보에 목표를 한 번에 던지면 속도 제한이 있어도 관절마다 도달 시간이 달라서
        경로가 예측 불가능해진다. 중간점을 넣으면 관절들이 함께 움직여 경로가
        직선에 가까워지고, 첫 동작이 갑자기 튀지 않는다.
        """
        start = self.read_counts()
        max_delta = max(abs(target[n] - start[n]) for n in JOINT_ORDER)
        steps = max(1, int(math.ceil(max_delta / self.max_step_counts)))

        for i in range(1, steps + 1):
            t = i / steps
            self.bus.sync_write_positions({
                SERVO_IDS[n]: int(round(start[n] + (target[n] - start[n]) * t))
                for n in JOINT_ORDER})
            time.sleep(self.step_pause if i < steps else settle)

            # 이동 도중에 부하를 본다. 다 끝난 뒤에 확인하면 이미 밀어붙인 뒤다.
            if self.load_limit:
                who, mag = self.worst_load()
                if mag > self.load_limit:
                    self.bus.sync_write_positions(self.read_counts())  # 그 자리에 정지
                    raise RuntimeError(
                        f"{who} 부하 {mag} > {self.load_limit} - 충돌 의심, 정지했다")

    def send_joints(self, q: dict[str, float], settle: float = 0.8,
                    order: Optional[Sequence[str]] = None) -> None:
        """order 를 주면 그 순서로 한 관절씩, 아니면 전 관절을 함께 움직인다."""
        if self.bus is None:
            raise RuntimeError("로봇이 연결되어 있지 않다")

        target = {n: self.calib.ik_to_count(n, q[n]) for n in JOINT_ORDER}
        start = self.read_counts()
        max_delta = max(abs(target[n] - start[n]) for n in JOINT_ORDER)
        if max_delta > self.max_step_counts:
            print(f"[arm] 최대 {max_delta/COUNTS_PER_DEG:.0f}deg 이동"
                  + (f" -> {'/'.join(SHORT_NAME[n] for n in order)} 순서로"
                     if order else " -> 동시"))

        if not order:
            self._ramp_to(target, settle)
            return

        # 한 관절씩 목표까지 보내고 다음 관절로 넘어간다. 나머지 관절은 지금
        # 위치를 그대로 목표로 줘서 움직이지 않게 고정한다.
        held = dict(start)
        last = order[-1]
        for n in order:
            if abs(target[n] - held[n]) < 1:
                held[n] = target[n]
                continue
            held[n] = target[n]
            self._ramp_to(dict(held), settle if n == last else 0.15)

    def worst_load(self) -> tuple[str, int]:
        """가장 큰 서보 부하. STS3215 부하는 하위 10비트가 크기, 상위가 방향."""
        worst, who = 0, ""
        for name, sid in SERVO_IDS.items():
            raw = self.bus.read_u16(sid, STS_ADDR["present_load"])
            mag = (raw & 0x3FF) if raw is not None else 0
            if mag > worst:
                worst, who = mag, name
        return who, worst

    def counts_to_ik(self, counts: dict[str, int]) -> dict[str, float]:
        return self.calib.counts_to_ik(counts)

    def get_tcp(self) -> Optional[np.ndarray]:
        """읽기에 실패해도 None 만 돌려준다.

        이건 화면 표시용이라, 시리얼이 한 번 흔들렸다고 프로그램이 죽으면 안 된다.
        (동작 명령 쪽의 read_counts 는 그대로 예외를 던진다 - 거기서는 위치를
         모르는 채로 움직이면 위험하기 때문이다.)
        """
        if self.bus is None:
            return None
        try:
            return so101_fk(self.counts_to_ik(self.read_counts()))
        except (RuntimeError, OSError):
            return None

    def get_joint_counts(self) -> Optional[dict[str, int]]:
        if self.bus is None:
            return None
        try:
            return self.read_counts()
        except (RuntimeError, OSError):
            return None

    def set_torque(self, on: bool) -> None:
        if self.bus is None:
            raise RuntimeError("로봇이 연결되어 있지 않다")
        for sid in SERVO_IDS.values():
            self.bus.write_u8(sid, STS_ADDR["torque_enable"], 1 if on else 0)
        print(f"[arm] 토크 {'ON' if on else 'OFF'}")

    def set_gripper(self, opening: float, settle: float = 0.5) -> None:
        if self.bus is None:
            raise RuntimeError("로봇이 연결되어 있지 않다")
        r = self.calib.ranges["gripper"]
        opening = float(np.clip(opening, 0.0, 1.0))
        count = int(r["range_min"] + (r["range_max"] - r["range_min"]) * opening)
        self.bus.sync_write_positions({SERVO_IDS["gripper"]: count})
        time.sleep(settle)


def find_so101_port() -> Optional[str]:
    """SO-101 의 USB 시리얼 어댑터(CH343, VID 1A86 / PID 55D3)를 자동으로 찾는다.

    환경변수를 깜빡하면 조용히 mock 으로 내려가서, 왜 팔이 안 움직이는지
    한참 헤매게 된다. 포트는 기계적으로 찾을 수 있으니 찾아 준다.
    """
    try:
        from serial.tools import list_ports
    except ImportError:
        return None

    ports = list(list_ports.comports())
    for p in ports:                       # CH343 을 VID/PID 로 특정
        if (p.vid, p.pid) == (0x1A86, 0x55D3):
            print(f"[arm] SO-101 어댑터 자동 검출: {p.device} ({p.description})")
            return p.device
    for p in ports:                       # 같은 제조사의 다른 CH34x
        if p.vid == 0x1A86:
            print(f"[arm] CH34x 어댑터 검출: {p.device} ({p.description})")
            return p.device
    return None


# ---------------------------------------------------------------------------
# 5b. 팔 캘리브레이션 (관절 리밋 + 기준자세). calibrate_arm.py 가 이걸 부른다.
# ---------------------------------------------------------------------------

COUNT_MIN, COUNT_MAX = 0, 4095
SAFETY_MARGIN_DEG = 2.0     # 실측한 끝단에서 안쪽으로 물러설 여유
MIN_SPAN_DEG = 10.0         # 이보다 좁으면 측정이 잘못된 것으로 본다
SWEEP_SECONDS = 12.0        # msvcrt 가 없을 때의 고정 시간

POSITIVE_HINT = {
    "shoulder_pan":  "베이스를 위에서 봤을 때 반시계 방향(로봇 기준 왼쪽)으로",
    "shoulder_lift": "위팔을 위로 들어올리는 방향으로",
    "elbow_flex":    "아래팔을 위로 펴올리는 방향으로",
    "wrist_flex":    "손목을 위로 젖히는 방향으로",
    "wrist_roll":    "그리퍼를 반시계 방향으로 회전",
}

# ---------------------------------------------------------------------------
# 유틸
# ---------------------------------------------------------------------------

def c2d(count: float) -> float:
    """서보 카운트 -> 각도[deg] (캘리브레이션 좌표계)"""
    return (count - CENTER_COUNT) / COUNTS_PER_DEG


def d2c(deg: float) -> int:
    return int(round(CENTER_COUNT + deg * COUNTS_PER_DEG))


def read_pos(bus: FeetechBus, sid: int, name: str = "") -> int:
    raw = bus.read_u16(sid, STS_ADDR["present_position"])
    if raw is None:
        raise RuntimeError(f"{name}(ID{sid}) 위치 읽기 실패")
    return raw


def read_servo_deg(bus: FeetechBus) -> dict[str, float]:
    return {n: c2d(read_pos(bus, sid, n)) for n, sid in SERVO_IDS.items()}


def read_homing_offset(bus: FeetechBus, sid: int) -> int:
    v = bus.read_u16(sid, STS_ADDR["homing_offset"])
    if v is None:
        return 0
    return -(v & 0x7FF) if (v & 0x800) else v


class CalibAborted(RuntimeError):
    """캘리브레이션을 계속할 수 없다 (입력 불가 등). 저장하지 않고 빠져나간다."""


def ask(prompt: str, default: str = "") -> str:
    try:
        s = input(prompt).strip()
    except EOFError:
        return default
    return s or default


def wait_enter(prompt: str) -> None:
    """Enter 를 기다린다. 입력을 받을 수 없는 환경이면 중단한다.

    파이프로 돌리는 등 stdin 이 없을 때 EOFError 로 죽으면 팔이 토크 OFF 인 채
    프로그램만 사라진다. 정상 경로로 빠져나가 토크 정리까지 하게 만든다.
    """
    try:
        input(prompt)
    except EOFError:
        raise CalibAborted("입력을 받을 수 없는 환경입니다")


# ---------------------------------------------------------------------------
# 리밋 측정
# ---------------------------------------------------------------------------

def sweep_joint(bus: FeetechBus, name: str, sid: int) -> tuple[int, int]:
    """관절을 손으로 끝에서 끝까지 움직이는 동안 최소/최대 카운트를 기록한다.

    토크가 꺼져 있으므로 사람이 미는 대로만 움직인다. 기계적 스토퍼에 살짝
    닿을 때까지 양쪽 끝을 훑으면 된다.
    """
    lo = hi = read_pos(bus, sid, name)
    print(f"    지금부터 [{name}] 을 양쪽 끝까지 천천히 움직이세요.")
    if msvcrt is not None:
        print("    다 하면 Enter. (실시간으로 최소/최대를 기록합니다)")
    else:
        print(f"    {SWEEP_SECONDS:.0f}초 동안 기록합니다.")

    t0 = time.time()
    last = 0.0
    while True:
        p = read_pos(bus, sid, name)
        lo, hi = min(lo, p), max(hi, p)

        now = time.time()
        if now - last > 0.08:
            last = now
            print(f"\r      현재 {p:>4} ({c2d(p):+7.2f}deg)   "
                  f"측정범위 [{lo:>4}, {hi:>4}] = [{c2d(lo):+7.2f}, {c2d(hi):+7.2f}]deg "
                  f"폭 {c2d(hi)-c2d(lo):6.2f}deg   ", end="", flush=True)

        if msvcrt is not None:
            if msvcrt.kbhit() and msvcrt.getch() in (b"\r", b"\n"):
                break
        elif now - t0 > SWEEP_SECONDS:
            break
        time.sleep(0.01)

    print()
    return lo, hi


def apply_margin(lo: int, hi: int) -> tuple[int, int]:
    m = SAFETY_MARGIN_DEG * COUNTS_PER_DEG
    a, b = int(round(lo + m)), int(round(hi - m))
    if b <= a:                      # 범위가 너무 좁으면 여유를 포기한다
        a, b = lo, hi
    return max(COUNT_MIN, a), min(COUNT_MAX, b)


def set_ranges(bus: FeetechBus, base: dict) -> dict:
    """관절별로 리밋을 유지/측정/직접입력 중에 고른다."""
    print("\n" + "=" * 66)
    print("1단계: 관절 min/max 리밋")
    print("=" * 66)
    print("  이 값이 test4.py 의 하드 클램프가 됩니다. 팔은 절대 이 범위를 넘지 않습니다.")
    print("  관절마다: [k]유지  [s]손으로 훑어 측정  [t]각도 직접입력\n")

    out: dict[str, dict] = {}
    for name, sid in SERVO_IDS.items():
        cur = base.get(name, {})
        lo = cur.get("range_min", COUNT_MIN)
        hi = cur.get("range_max", COUNT_MAX)
        pos = read_pos(bus, sid, name)
        print(f"[{name}] ID{sid}  현재 {pos} ({c2d(pos):+.2f}deg)")
        print(f"    기존 리밋 [{lo}, {hi}] = [{c2d(lo):+.2f}, {c2d(hi):+.2f}]deg "
              f"(폭 {c2d(hi)-c2d(lo):.1f}deg)")

        while True:
            m = ask("    선택 [k/s/t] (기본 k) > ", "k").lower()
            if m == "k":
                nlo, nhi = lo, hi
                break
            if m == "s":
                rlo, rhi = sweep_joint(bus, name, sid)
                span = c2d(rhi) - c2d(rlo)
                if span < MIN_SPAN_DEG:
                    print(f"    측정 폭이 {span:.1f}deg 로 너무 좁습니다. 다시 하세요.")
                    continue
                nlo, nhi = apply_margin(rlo, rhi)
                print(f"    실측 [{rlo}, {rhi}] -> 여유 {SAFETY_MARGIN_DEG}deg 적용 "
                      f"[{nlo}, {nhi}] = [{c2d(nlo):+.2f}, {c2d(nhi):+.2f}]deg")
                break
            if m == "t":
                s = ask("    min max [deg] 를 공백으로 구분해 입력 > ")
                try:
                    a, b = (float(v) for v in s.replace(",", " ").split())
                except ValueError:
                    print("    숫자 두 개를 입력하세요.")
                    continue
                if b <= a:
                    print("    max 가 min 보다 커야 합니다.")
                    continue
                nlo, nhi = max(COUNT_MIN, d2c(a)), min(COUNT_MAX, d2c(b))
                print(f"    -> [{nlo}, {nhi}] 카운트")
                break
            print("    k, s, t 중에 고르세요.")

        if not (nlo <= pos <= nhi):
            print(f"    주의: 현재 위치 {pos} 가 새 리밋 밖입니다. "
                  f"이 관절은 클램프 때문에 즉시 끌려갑니다.")

        out[name] = {"id": sid,
                     "drive_mode": cur.get("drive_mode", 0),
                     "homing_offset": read_homing_offset(bus, sid),
                     "range_min": int(nlo), "range_max": int(nhi)}
        print()
    return out

ZERO_POSE_DESC = """
[영자세] 모든 관절 0도 = lift 90도 / elbow 90도 / wrist 0도

  - 위팔(shoulder_lift -> elbow)  : 수직으로 곧게 선다
  - 아래팔(elbow -> wrist)        : 수평. 위팔과 직각
  - 손목+그리퍼(wrist -> TCP)     : 아래팔과 일직선, 그대로 수평
  - 팔은 정면(pan 중앙), 손목 회전(wrist_roll)은 중립

  직각자를 위팔에 대고 아래팔이 직각인지 보면 된다. 링크를 앞으로 쭉 편
  수평 자세가 아니다 - 그렇게 잡으면 관절마다 70도 넘게 어긋난다.
  이 자세의 TCP 는 pan 축에서 324.8mm 앞, 테이블에서 232.6mm 위다.
"""


def set_mapping(bus: FeetechBus, ranges: dict) -> dict:
    """관절별 부호와 영점(offset_deg)을 정한다.

    부호를 먼저 잡는다. 범위 중앙으로 영점을 계산하려면 부호가 있어야 하고,
    자세를 손으로 잡는 경우에도 부호 확인 중에 팔을 흔들게 되므로 자세는
    맨 마지막에 잡는 편이 낫다.

    영점 방식:
      [r] 가동범위의 한가운데에서 계산 - 사람이 자세를 잡지 않는다. 대신
          1단계에서 관절을 양쪽 끝까지 제대로 훑었어야 한다.
      [p] 영자세를 손으로 잡아 실측 - 범위를 다 훑지 못했을 때의 대안.
    """
    print("\n" + "=" * 66)
    print("2단계: 부호 + 영점")
    print("=" * 66)
    print(ZERO_POSE_DESC)

    while True:
        how = ask("  영점 [r]가동범위 중앙에서 계산(권장) / [p]자세를 손으로 잡아 측정 > ",
                  "r").lower()
        if how in ("r", "p"):
            break
        print("  r 또는 p 를 고르세요.")

    print("\n  먼저 관절별 '+방향'을 확인합니다. 안내 방향으로 15도 이상 움직인 뒤 Enter.\n")
    sign: dict[str, float] = {}
    for n in JOINT_ORDER:
        base = read_servo_deg(bus)[n]
        while True:
            wait_enter(f"  [{n}] {POSITIVE_HINT[n]} 움직이고 Enter > ")
            delta = read_servo_deg(bus)[n] - base
            if abs(delta) >= 5.0:
                break
            print(f"    변화가 {delta:+.1f}도로 너무 작습니다. 더 크게 움직여 주세요.")
        sign[n] = 1.0 if delta > 0 else -1.0
        print(f"    delta={delta:+7.2f}deg -> sign={sign[n]:+.0f}")

    print()
    zero: dict[str, float] = {}
    if how == "r":
        for n in JOINT_ORDER:
            rg = ranges[n]
            mid = (rg["range_min"] + rg["range_max"]) / 2.0
            # 범위 중앙은 URDF 영자세다. 우리 0도(90/90/0)와의 차이를 빼준다.
            zero[n] = c2d(mid) - sign[n] * math.degrees(MIDRANGE_Q[n])
            span = c2d(rg["range_max"]) - c2d(rg["range_min"])
            warn = "  <-- 범위가 좁다. 1단계를 다시 훑을 것" if span < 60 else ""
            print(f"    {n:<15} 중앙 {mid:7.1f}카운트 = {c2d(mid):+7.2f}deg"
                  f"  -> 영점 {zero[n]:+7.2f}deg  (폭 {span:.1f}deg){warn}")
    else:
        wait_enter("  팔을 영자세(위팔 수직 / 아래팔 수평)로 잡은 채로 Enter > ")
        zero = read_servo_deg(bus)
        print("\n  영자세 서보각:")
        for n in JOINT_ORDER:
            print(f"    {n:<15}{zero[n]:+8.2f} deg")

    return {n: {"sign": sign[n], "offset_deg": zero[n]} for n in JOINT_ORDER}


# ---------------------------------------------------------------------------
# TCP 확인
# ---------------------------------------------------------------------------
#
# 영점(offset_deg)이 맞았는지는 서보각만 봐서는 알 수 없다. 링크 길이까지
# 통과한 TCP 좌표라야 "그리퍼가 테이블에 닿아 있는데 z 가 -116mm 라고 한다"
# 같은 모순이 눈에 보인다. 그 모순을 여기서 못 잡으면 hand-eye 캘리브레이션까지
# 끌고 가서, 카메라 탓을 하며 몇 시간을 버리게 된다.

TABLE_Z_TOL = 0.010     # 테이블에 댔을 때 TCP z 가 0 에서 벗어나도 되는 폭 [m]


def read_all_counts(bus: FeetechBus) -> dict[str, int]:
    return {n: read_pos(bus, sid, n) for n, sid in SERVO_IDS.items()}


def tcp_of(calib: ArmCalibration, counts: dict[str, int]):
    """카운트 -> (IK 관절각[rad], TCP[m])"""
    q = calib.counts_to_ik(counts)
    return q, so101_fk(q)


SHORT_NAME = {"shoulder_pan": "pan", "shoulder_lift": "lift", "elbow_flex": "elbow",
              "wrist_flex": "wflex", "wrist_roll": "wroll"}


def tcp_text(calib: ArmCalibration, counts: dict[str, int]) -> str:
    q, p = tcp_of(calib, counts)
    r = math.hypot(p[0], p[1])
    joints = "  ".join(f"{SHORT_NAME[n]} {math.degrees(q[n]):+6.1f}" for n in JOINT_ORDER)
    return (f"TCP  x{p[0]*1000:+7.1f}  y{p[1]*1000:+7.1f}  z{p[2]*1000:+7.1f} mm"
            f"   반경 {r*1000:6.1f}   |  {joints}")


def watch_tcp(bus: FeetechBus, calib: ArmCalibration, note: str = "") -> dict[str, int]:
    """Enter 를 누를 때까지 TCP 를 실시간으로 보여주고, 그 순간의 카운트를 돌려준다."""
    if note:
        print(f"  {note}")
    if msvcrt is not None:
        print("  움직여 보면서 값을 확인하세요. 다 보면 Enter.")
    else:
        print(f"  {SWEEP_SECONDS:.0f}초 동안 표시합니다.")

    counts = read_all_counts(bus)
    t0 = last = time.time()
    while True:
        counts = read_all_counts(bus)
        now = time.time()
        if now - last > 0.1:
            last = now
            print("\r  " + tcp_text(calib, counts) + "   ", end="", flush=True)
        if msvcrt is not None:
            if msvcrt.kbhit() and msvcrt.getch() in (b"\r", b"\n"):
                break
        elif now - t0 > SWEEP_SECONDS:
            break
        time.sleep(0.02)
    print()
    return counts


def table_check(bus: FeetechBus, calib: ArmCalibration) -> None:
    """그리퍼 끝을 테이블에 댄 자세에서 TCP z 가 0 인지 본다.

    팔이 테이블 위에 놓여 있다면 이건 자를 대지 않고도 쓸 수 있는 유일한
    절대 기준이다. 여기서 z 가 0 이 아니면 그만큼 기준자세가 틀린 것이고,
    그 오차는 자세마다 다르게 나타나서 나중에 어떤 보정으로도 흡수되지 않는다.
    """
    print("\n" + "=" * 66)
    print("4단계: 테이블 접촉 검사 (영점이 실제로 맞는지)")
    print("=" * 66)
    print("  그리퍼 끝을 테이블면에 살짝 대세요. 팔이 테이블 위에 놓여 있다면")
    print("  그 순간 TCP z 는 0 근처여야 합니다.")
    counts = watch_tcp(bus, calib, "그리퍼 끝을 테이블에 댄 채로 Enter")

    q, p = tcp_of(calib, counts)
    z, r = float(p[2]), math.hypot(p[0], p[1])
    print(f"\n  접촉 지점의 TCP: z = {z*1000:+.1f}mm  (반경 {r*1000:.1f}mm)")
    if abs(z) <= TABLE_Z_TOL:
        print(f"  -> 정상. 영점이 맞습니다 (허용 +-{TABLE_Z_TOL*1000:.0f}mm)")
        return

    print(f"  -> 어긋남 {z*1000:+.1f}mm. 기준자세(2단계)가 그만큼 틀렸습니다.")
    if r > 1e-3:
        deg = math.degrees(math.atan2(-z, r))
        print(f"     이 반경에서는 팔 전체가 약 {deg:+.1f}도 기울어진 것과 같습니다.")
        print(f"     (shoulder_lift offset_deg 를 {deg*(-calib.mapping['shoulder_lift']['sign']):+.1f}도 "
              f"움직이면 이 자세에서는 z 가 0 이 됩니다. 다만 한 자세만 맞춘 것이라")
        print("      다른 자세에서 또 틀어집니다 - 되도록 2단계를 다시 하세요.)")
    print("     기준자세는 세 링크가 한 직선이 되게 펴고 그 직선이 바닥과 평행해야 합니다.")
    print("     자나 테이블 모서리에 대고 맞추면 눈대중보다 훨씬 정확합니다.")


# ---------------------------------------------------------------------------

def verify_home_reach(calib: ArmCalibration) -> bool:
    print("\n" + "=" * 66)
    print("5단계: 검증 (전송하지 않고 계산만)")
    print("=" * 66)
    q = so101_ik(*HOME_POSE)
    ok = True
    counts = {}
    for n in JOINT_ORDER:
        r = calib.ranges[n]
        m = calib.mapping[n]
        servo_deg = math.degrees(q[n]) * m["sign"] + m["offset_deg"]
        raw = CENTER_COUNT + servo_deg * COUNTS_PER_DEG
        clamped = calib.ik_to_count(n, q[n])
        counts[n] = clamped
        hit = "" if abs(raw - clamped) < 1 else "  <-- 리밋에 걸림"
        if hit:
            ok = False
        print(f"  {n:<15} ik={math.degrees(q[n]):+7.2f}deg  servo={servo_deg:+7.2f}deg  "
              f"count={raw:7.0f}  범위[{r['range_min']},{r['range_max']}]{hit}")

    # 왕복 검사: 목표 -> 카운트 -> 다시 TCP. 리밋에 걸리면 여기서 어긋난다.
    reached = so101_fk(calib.counts_to_ik(counts))
    want = HOME_POSE
    d = math.dist(reached, want)
    print(f"\n  HOME 목표    x{want[0]*1000:+7.1f} y{want[1]*1000:+7.1f} z{want[2]*1000:+7.1f} mm")
    print(f"  클램프 후 TCP x{reached[0]*1000:+7.1f} y{reached[1]*1000:+7.1f} "
          f"z{reached[2]*1000:+7.1f} mm   차이 {d*1000:.1f}mm")
    if d > 0.002:
        print("  -> 리밋 때문에 목표에 못 간다. 리밋이나 기준자세를 다시 볼 것")
        ok = False
    return ok


def zero_pose_tcp():
    """기준자세(모든 관절 0도)에서 나와야 하는 TCP. 화면에서 바로 대조할 기준값."""
    return so101_fk({n: 0.0 for n in JOINT_ORDER})


def monitor_tcp(port: str) -> None:
    """저장된 arm_calib.json 으로 TCP 만 실시간으로 본다. 아무것도 바꾸지 않는다.

    영점이 의심스러울 때 캘리브레이션 전체를 다시 하지 않고 확인만 하는 길.
    """
    calib = ArmCalibration.load()
    if calib is None:
        print(f"[calib] {ARM_CALIB_PATH} 가 없다 - 먼저 캘리브레이션할 것")
        return
    bus = FeetechBus(port)
    try:
        missing = [n for n, sid in SERVO_IDS.items() if not bus.ping(sid)]
        if missing:
            print(f"[calib] 응답 없는 서보: {missing}")
            return
        for sid in SERVO_IDS.values():
            bus.write_u8(sid, STS_ADDR["torque_enable"], 0)
        z = zero_pose_tcp()
        print(f"[calib] {port} 연결, 토크 OFF (손으로 움직입니다)")
        print(f"        기준자세라면 TCP 는 x{z[0]*1000:+.0f} y{z[1]*1000:+.0f} "
              f"z{z[2]*1000:+.0f}mm 여야 합니다.")
        watch_tcp(bus, calib)
        table_check(bus, calib)
    except KeyboardInterrupt:
        print("\n[calib] 중단")
    finally:
        for sid in SERVO_IDS.values():
            try:
                bus.write_u8(sid, STS_ADDR["torque_enable"], 0)
            except Exception:
                pass
        bus.close()


def run_arm_calibration(port: str) -> Optional[ArmCalibration]:
    """관절 리밋 -> 기준자세 -> TCP 확인 -> 테이블 검사 -> 저장. 실패/중단이면 None.

    전 과정 토크 OFF 다. 서보에 목표위치를 쓰지 않으므로 팔은 스스로 움직이지
    않는다. 사람이 손으로 자세를 잡는 방식이고, 팔은 중력으로 처지니 받쳐야 한다.
    """
    try:
        base = load_lerobot_calibration(LEROBOT_CALIB)
        print(f"[calib] 기존 리밋 로드: {LEROBOT_CALIB}")
    except Exception as e:
        print(f"[calib] 기존 리밋 없음 ({e.__class__.__name__}) - 전부 새로 측정해야 한다")
        base = {}

    bus = FeetechBus(port)
    try:
        missing = [n for n, sid in SERVO_IDS.items() if not bus.ping(sid)]
        if missing:
            print(f"[calib] 응답 없는 서보: {missing} - 캘리브레이션 중단")
            return None

        for sid in SERVO_IDS.values():          # 안전: 전 관절 토크 OFF 보장
            bus.write_u8(sid, STS_ADDR["torque_enable"], 0)
        print(f"[calib] {port} 연결, 전 관절 토크 OFF (팔이 손으로 움직입니다)")

        ranges = set_ranges(bus, base)
        mapping = set_mapping(bus, ranges)
        calib = ArmCalibration(mapping, ranges)

        print("\n" + "=" * 66)
        print("3단계: 실시간 TCP 확인")
        print("=" * 66)
        z = zero_pose_tcp()
        print("  이제 팔을 손으로 움직이면 그 자세의 TCP 가 실시간으로 보입니다.")
        print(f"  기준자세로 되돌리면 x{z[0]*1000:+.0f} y{z[1]*1000:+.0f} z{z[2]*1000:+.0f}mm "
              "근처여야 합니다.")
        watch_tcp(bus, calib)

        table_check(bus, calib)

        if not verify_home_reach(calib):
            print("\n[calib] 경고: HOME 자세가 리밋에 걸립니다.")
            print("        기준자세가 부정확했거나 리밋이 너무 좁습니다.")
            if ask("        그래도 저장할까요? (y/N) > ").lower() != "y":
                print("[calib] 저장하지 않고 종료")
                return None

        calib.save(ARM_CALIB_PATH)
        print(f"[calib] 완료. 이제 {ARM_CALIB_PATH} 로 실기를 제어합니다.")
        return calib
    except KeyboardInterrupt:
        print("\n[calib] 사용자 중단 - 저장하지 않음")
        return None
    except CalibAborted as e:
        print(f"[calib] 중단: {e} - 저장하지 않음")
        return None
    finally:
        for sid in SERVO_IDS.values():
            try:
                bus.write_u8(sid, STS_ADDR["torque_enable"], 0)
            except Exception:
                pass
        bus.close()


# ---------------------------------------------------------------------------
# 시작 마법사: 없는 캘리브레이션은 만들고, 있는 것은 다시 할지 묻는다
# ---------------------------------------------------------------------------

def ask_yes(question: str, default: bool = True) -> bool:
    """예/아니오. 입력이 불가능한 환경(파이프 실행)에서는 기본값을 쓴다."""
    tail = "(Y/n)" if default else "(y/N)"
    try:
        s = input(f"{question} {tail} > ").strip().lower()
    except EOFError:
        print(f"  (입력 없음 -> 기본값 {'예' if default else '아니오'})")
        return default
    if not s:
        return default
    return s[0] in ("y", "1")


def setup_arm() -> RobotArm:
    """팔 캘리브레이션을 확인/실행한 뒤 실기(또는 mock)를 돌려준다.

    파일이 없으면 지금 만들지 묻고, 있으면 다시 잡을지 묻는다. 캘리브레이션
    없이는 실기를 움직이지 않는다 - 리밋과 부호를 모르는 채로 명령하면
    팔이 테이블로 내리꽂힌다.
    """
    port = os.environ.get("SO101_PORT") or find_so101_port()
    if not port:
        print("[arm] SO-101 포트를 찾지 못함 -> mock 사용 "
              "(수동 지정: $env:SO101_PORT='COM18')")
        arm = MockArm()
        arm.connect()
        return arm

    calib = ArmCalibration.load()
    print()
    if calib is None:
        print(f"[1/2] 팔 캘리브레이션: {ARM_CALIB_PATH} 이 없습니다.")
        print("      각 관절의 min/max 리밋과 기준자세(부호+영점)를 지금 설정합니다.")
        print("      (전 과정 토크 OFF - 팔을 손으로 잡고 진행합니다)")
        if ask_yes("      지금 설정할까요?", True):
            calib = run_arm_calibration(port)
    else:
        print(f"[1/2] 팔 캘리브레이션: {ARM_CALIB_PATH} 이 이미 있습니다.")
        _print_calib_summary(calib)
        if ask_yes("      다시 설정할까요?", False):
            new = run_arm_calibration(port)
            calib = new or calib

    if calib is None:
        print("[arm] 팔 캘리브레이션 없음 -> mock 사용 (화면과 카메라는 그대로 동작합니다)")
    else:
        try:
            arm = FeetechSO101Arm(port, calib)
            arm.connect()
            return arm
        except Exception as e:
            print(f"[arm] 실기 연결 실패 ({e.__class__.__name__}: {e}) -> mock 사용")

    arm = MockArm()
    arm.connect()
    return arm


def _print_calib_summary(calib: ArmCalibration) -> None:
    for n in JOINT_ORDER:
        r = calib.ranges[n]
        m = calib.mapping[n]
        print(f"        {n:<15} 리밋 [{r['range_min']:>4}, {r['range_max']:>4}]  "
              f"부호 {m['sign']:+.0f}  영점 {m['offset_deg']:+7.2f}deg")


HANDEYE_MIN_POINTS = 4


def want_handeye_calib(arm: Optional[RobotArm] = None) -> bool:
    """카메라-로봇 캘리브레이션을 지금 잡을지 묻는다."""
    print()
    if arm is not None and arm.get_tcp() is None:
        print("      주의: 팔이 로봇 좌표를 보고하지 못합니다(mock 또는 미보정). "
              "점마다 xyz 를 손으로 입력해야 합니다.")
    if not os.path.exists(HANDEYE_PATH):
        print(f"[2/2] 카메라-로봇 캘리브레이션: {HANDEYE_PATH} 이 없습니다.")
        print(f"      탁상 위에서 실제로 집을 수 있는 위치 {HANDEYE_MIN_POINTS}곳 이상을 "
              "등록해야 합니다.")
        return ask_yes("      지금 잡을까요?", True)
    print(f"[2/2] 카메라-로봇 캘리브레이션: {HANDEYE_PATH} 이 이미 있습니다.")
    return ask_yes("      다시 잡을까요?", False)


# ---------------------------------------------------------------------------
# 6. Pick / Place 시퀀스
# ---------------------------------------------------------------------------

# 시퀀스는 (종류, 값...) 스텝의 리스트로 표현한다. 이렇게 두면 "실행"과
# "사전 도달성 검사"가 같은 정의를 공유하므로, 물건을 문 채로 중간에
# 도달 불가로 멈추는 일이 생기지 않는다.
Step = tuple

def pick_steps(target: Sequence[float], roll: float = 0.0) -> list[Step]:
    x, y, z = (float(v) for v in target)
    z_grasp = max(WS_Z_MIN, z - GRASP_SINK)
    z_appr = clamp_z(x, y, z + APPROACH_H, z_grasp)
    z_lift = clamp_z(x, y, z + LIFT_H, z_grasp)
    return [
        ("grip", GRIPPER_OPEN, 0.5),                       # 1. 그리퍼 열기
        ("move", x, y, z_appr, roll, 0.8, ORDER_OUT),      # 2. 목표 위에서 정렬
        ("move", x, y, z_grasp, roll, 1.0, ORDER_OUT),     # 3. 하강
        ("grip", 0.0, 0.8),                                # 4. 파지
        ("move", x, y, z_lift, roll, 1.0, ORDER_BACK),     # 5. 들어올림
        # 홈으로 물러난다. 집은 자리 위에 팔이 서 있으면 카메라가 그 지점을
        # 가려서 다음 클릭을 할 수 없고, 물체를 문 채로 팔이 뻗어 있으면
        # 서보에 계속 부하가 걸린다. 들어올린 다음에 옮겨야 물체를 끌지 않는다.
        ("move", *HOME_POSE, roll, 1.2, ORDER_BACK),       # 6. 홈 복귀 (문 채로)
    ]


def place_steps(target: Sequence[float], roll: float = 0.0) -> list[Step]:
    x, y, z = (float(v) for v in target)
    z_down = max(WS_Z_MIN, z + PLACE_CLEAR)          # 여기서 놓는다 (표면 위 50mm)
    # 접근 높이는 '놓는 높이' 위로 잡는다. 표면 기준으로 잡으면 놓는 높이가
    # 접근 높이에 가까워져서 정렬 구간이 사라진다.
    z_appr = clamp_z(x, y, z_down + APPROACH_H, z_down)
    return [
        ("move", x, y, z_appr, roll, 0.8, ORDER_OUT),      # 1. 놓을 위치 위로 이동
        ("move", x, y, z_down, roll, 1.0, ORDER_OUT),      # 2. 놓는 높이까지 하강
        ("grip", GRIPPER_OPEN, 0.5),                       # 3. 놓기 (표면 위 50mm)
        ("move", x, y, z_appr, roll, 0.8, ORDER_BACK),     # 4. 후퇴
        # 닫은 채로 이동한다. 벌린 손가락은 이동 중에 주변 물체를 걸기 쉽다.
        # 놓은 물체를 밀지 않도록 후퇴한 뒤에 닫는다.
        ("grip", 0.0, 0.5),                                # 5. 그리퍼 닫기
        ("move", *HOME_POSE, 0.0, 1.2, ORDER_BACK),        # 6. 홈 복귀
    ]


def check_steps(steps: Sequence[Step], z_min: float = WS_Z_MIN) -> tuple[bool, str]:
    """모든 move 스텝이 도달 가능한지 미리 검사한다."""
    for i, s in enumerate(steps):
        if s[0] != "move":
            continue
        x, y, z, roll = s[1], s[2], s[3], s[4]
        try:
            plan_pose(x, y, z, roll=roll, z_min=z_min)
        except IKError as e:
            return False, f"{i+1}번 경유점 z={z*100:.1f}cm: {e}"
    return True, ""


def _reach_ok_vec(x: np.ndarray, y: np.ndarray, z: np.ndarray) -> np.ndarray:
    """top-down 파지에서 (x,y,z)가 도달 가능한가. plan_pose 를 벡터로 푼 것.

    pitch=-90 이면 cos(pitch)=0, sin(pitch)=-1 이라 so101_ik 의 조건이
    닫힌 형태로 정리된다. 화면 전체를 매 프레임 판정해야 해서 반복문으로는 못 한다.
    """
    r = np.hypot(x, y)
    box = ((r >= WS_R_MIN) & (r <= WS_R_MAX) &
           (z >= WS_Z_MIN) & (z <= WS_Z_MAX))
    zw = z - L0_BASE_H + L3_WRIST          # 손목중심 높이 (pitch=-90)
    rw = r - L_SHOULDER_R                  # 어깨축은 pan 축보다 앞에 있다
    d2 = rw * rw + zw * zw
    reach = ((d2 <= REACH * REACH) & (d2 >= (L1_UPPER - L2_FORE) ** 2) & (r > 1e-6))
    return box & reach


def _max_z_vec(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """max_z_at 의 벡터판 (pitch=-90 고정)."""
    rw = np.hypot(x, y) - L_SHOULDER_R
    s = REACH - IK_MARGIN
    out = np.full(rw.shape, -np.inf)
    ok = np.abs(rw) < s
    out[ok] = np.sqrt(s * s - rw[ok] ** 2) + L0_BASE_H - L3_WRIST
    return out


def pick_reachable_mask(base_xyz: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """각 화소의 로봇좌표에 대해 pick 시퀀스 전체가 수행 가능한지.

    pick_steps 와 같은 경유점(접근/파지/상승)을 그대로 검사한다. 목표점만 보면
    물건을 문 채 중간에 멈추는 경우를 놓친다.
    """
    x, y, z = base_xyz[..., 0], base_xyz[..., 1], base_xyz[..., 2]
    zmax = _max_z_vec(x, y)

    z_grasp = np.maximum(WS_Z_MIN, z - GRASP_SINK)
    z_appr = np.maximum(z_grasp, np.minimum(np.minimum(z + APPROACH_H, zmax), WS_Z_MAX))
    z_lift = np.maximum(z_grasp, np.minimum(np.minimum(z + LIFT_H, zmax), WS_Z_MAX))

    ok = valid.copy()
    for zz in (z_appr, z_grasp, z_lift):
        ok &= _reach_ok_vec(x, y, zz)
    return ok


def cam_to_base_grid(intr: Intrinsics, depth: np.ndarray, he: HandEye,
                     stride: int = 1, _cache: dict = {}) -> tuple[np.ndarray, np.ndarray]:
    """depth 를 한 번에 로봇좌표로 변환한다. (base_xyz, 유효마스크)

    stride 를 주면 그만큼 성기게 계산한다. 도달 가능/불가 경계는 매끄러워서
    화면 표시용으로는 성긴 격자로 충분하고, 전체 해상도로 하면 44ms 가 걸려
    UI 프레임레이트를 반토막 낸다.
    """
    d = depth[::stride, ::stride]
    h, w = d.shape[:2]
    key = (w, h, stride, intr.fx, intr.fy, intr.cx, intr.cy)
    if _cache.get("key") != key:                 # 픽셀 격자는 매번 만들 필요 없다
        vv, uu = np.mgrid[0:h, 0:w].astype(np.float32)
        _cache.update(key=key,
                      ux=(uu * stride - intr.cx) / intr.fx,
                      uy=(vv * stride - intr.cy) / intr.fy)
    valid = d > 1e-4
    cam = np.stack([_cache["ux"] * d, _cache["uy"] * d, d], axis=-1)
    base = cam @ he.T[:3, :3].T + he.T[:3, 3]
    return base, valid


def fit_table_plane(intr: Intrinsics, depth: np.ndarray, stride: int = 6,
                    max_pts: int = 1500, iters: int = 120,
                    tol: float = 0.012) -> Optional[tuple[np.ndarray, np.ndarray]]:
    """장면에서 가장 큰 평면(테이블면)을 카메라 좌표계에서 찾는다.

    hand-eye 와 무관한 순수 기하다. 로봇이 그 테이블 위에 놓여 있으므로,
    올바른 hand-eye 라면 이 평면을 로봇계에서 z=0 의 수평면으로 보내야 한다.
    카메라 높이보다 이 기울기가 훨씬 예민한 검증 지표다.

    반환: (법선[카메라계], 평면 위 점들 표본) 또는 None
    """
    base, valid = cam_to_base_grid(intr, depth, HandEye(), stride)   # 항등 = 카메라계
    pts = base[valid].astype(np.float64)
    if len(pts) < 200:
        return None

    rng = np.random.default_rng(0)
    best_n, best_inl = None, 0
    for _ in range(iters):
        p = pts[rng.choice(len(pts), 3, replace=False)]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n = n / nn
        cnt = int((np.abs(pts @ n + (-n @ p[0])) < tol).sum())
        if cnt > best_inl:
            best_n, best_inl = (n, -n @ p[0]), cnt

    if best_n is None or best_inl < 0.25 * len(pts):
        return None

    n, d0 = best_n
    inl = pts[np.abs(pts @ n + d0) < tol]
    c = inl.mean(0)
    _, _, Vt = np.linalg.svd(inl - c, full_matrices=False)   # 내부점으로 재적합
    n = Vt[2]
    if n[2] < 0:
        n = -n
    if len(inl) > max_pts:
        inl = inl[rng.choice(len(inl), max_pts, replace=False)]
    return n, inl


def plane_camera_pose(n_cam: np.ndarray, pts_cam: np.ndarray) -> tuple[float, float]:
    """평면으로부터 카메라의 (수직높이[mm], 광축 기울기[deg])를 구한다.

    depth 만으로 얻는 값이라 hand-eye 와 무관하다. 화면 중앙의 depth 값은 광선을
    따라 잰 경사거리라서 수직높이가 아니다 - 기울어져 있으면 크게 다르다.
    """
    c = pts_cam.mean(0)
    height = abs(float(n_cam @ c)) * 1000.0
    tilt = math.degrees(math.acos(float(np.clip(abs(n_cam[2]), 0.0, 1.0))))
    return height, tilt


def align_to_plane(he: HandEye, n_cam: np.ndarray, pts_cam: np.ndarray) -> np.ndarray:
    """테이블면이 로봇계에서 z=0 수평면이 되도록 roll/pitch/tz 를 맞춘 변환.

    로봇이 테이블 위에 있다는 사실만으로 6자유도 중 3개가 정해진다.
    남는 자유도는 yaw(평면 안에서의 회전)와 tx, ty 뿐이고, 그건 지금 값을 유지한다.
    """
    # 테이블의 '위' 방향은 카메라 쪽이다. fit_table_plane 은 법선을 n_z>=0 으로
    # 정규화하는데 그건 카메라에서 멀어지는 방향이라, 그대로 쓰면 로봇 위아래가
    # 뒤집혀 카메라가 테이블 밑으로 간다.
    z_ax = n_cam / np.linalg.norm(n_cam)
    if z_ax[2] > 0:
        z_ax = -z_ax
    seed = np.array([0.0, 1.0, 0.0]) if abs(z_ax[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
    x_ax = np.cross(seed, z_ax)
    x_ax /= np.linalg.norm(x_ax)
    R0 = np.vstack([x_ax, np.cross(z_ax, x_ax), z_ax])      # R0 @ n_cam = [0,0,1]

    M = he.T[:3, :3] @ R0.T                                  # 남은 회전 = 대략 yaw
    yaw = math.atan2(M[1, 0], M[0, 0])
    cy, sy = math.cos(yaw), math.sin(yaw)
    R = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ R0

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = he.T[:3, 3].copy()
    T[2, 3] = -float((R @ pts_cam.mean(0))[2])               # 평면이 z=0 이 되도록
    return T


def plane_vs_robot(he: HandEye, n_cam: np.ndarray,
                   pts_cam: np.ndarray) -> tuple[float, float, float]:
    """평면을 로봇계로 옮겼을 때 (기울기[deg], z평균[mm], z편차[mm]).

    정상이면 기울기 0, z평균 0, z편차 작음. 회전은 매 프레임 바뀌지만 평면 적합은
    비싸므로, 적합 결과를 재사용하고 여기서 변환만 다시 적용한다.
    """
    nb = he.T[:3, :3] @ n_cam
    tilt = math.degrees(math.acos(float(np.clip(abs(nb[2]), 0.0, 1.0))))
    z = (pts_cam @ he.T[:3, :3].T + he.T[:3, 3])[:, 2]
    return tilt, float(z.mean() * 1000), float((z.max() - z.min()) * 1000)


REACH_STRIDE = 4


def reachable_mask(intr: Intrinsics, depth: np.ndarray, he: HandEye) -> np.ndarray:
    """화면 크기의 '여기를 클릭하면 집을 수 있다' 마스크."""
    base, valid = cam_to_base_grid(intr, depth, he, REACH_STRIDE)
    small = pick_reachable_mask(base, valid)
    return cv2.resize(small.astype(np.uint8), (depth.shape[1], depth.shape[0]),
                      interpolation=cv2.INTER_NEAREST).astype(bool)


def draw_reachability(frame: np.ndarray, depth: np.ndarray, ok: np.ndarray,
                     est: Optional[np.ndarray] = None) -> tuple[int, int]:
    """집을 수 없는 곳을 붉게 덮는다. depth 가 없는 곳은 더 어둡게 구분한다.

    est 는 주변값으로 메운(추정) 화소다. 쓸 수는 있지만 실측이 아니라는 것을
    알아야 하므로 다른 색으로 구분해 준다.
    """
    nodepth = depth <= 1e-4
    unreach = (~ok) & (~nodepth)

    tint = frame.copy()
    tint[unreach] = (0, 0, 190)        # 도달 불가 (BGR: 빨강)
    if est is not None:
        tint[est & ok] = (200, 160, 0)  # 추정 depth (BGR: 청록)
    tint[nodepth] = (40, 40, 40)       # depth 없음 - 애초에 클릭이 거부됨
    cv2.addWeighted(tint, 0.38, frame, 0.62, 0, dst=frame)

    # 경계선을 그려 어디까지가 작업영역인지 또렷하게
    edges = cv2.morphologyEx(ok.astype(np.uint8), cv2.MORPH_GRADIENT,
                             np.ones((3, 3), np.uint8)).astype(bool)
    frame[edges] = (120, 255, 120)
    return int(ok.sum()), int(unreach.sum())


def run_steps(arm: RobotArm, steps: Sequence[Step], tag: str = "seq",
              z_min: float = WS_Z_MIN) -> None:
    for s in steps:
        if s[0] == "move":
            _, x, y, z, roll, settle = s[:6]
            order = s[6] if len(s) > 6 else None      # 없으면 전 관절 동시
            arm.move_to(x, y, z, roll=roll, settle=settle, z_min=z_min, order=order)
        else:
            _, opening, settle = s
            arm.set_gripper(opening, settle=settle)
    print(f"[{tag}] 완료")


def do_pick(arm: RobotArm, target: np.ndarray, roll: float = 0.0) -> None:
    """target: 물체 윗면의 로봇 베이스 좌표 [m]"""
    print(f"[pick] target=({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})")
    run_steps(arm, pick_steps(target, roll), "pick")


def do_place(arm: RobotArm, target: np.ndarray, roll: float = 0.0) -> None:
    """target: 내려놓을 지점(표면)의 로봇 베이스 좌표 [m]"""
    print(f"[place] target=({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})")
    run_steps(arm, place_steps(target, roll), "place")


# ---------------------------------------------------------------------------
# 7. UI / 상태머신
# ---------------------------------------------------------------------------

WINDOW = "SO-101 Depth Pick & Place"

STATE_IDLE = "IDLE"        # 다음 클릭 = pick
STATE_HOLDING = "HOLDING"  # 다음 클릭 = place
STATE_BUSY = "BUSY"        # 로봇 동작 중, 클릭 무시


@dataclass
class Marker:
    uv: tuple[int, int]
    label: str
    color: tuple[int, int, int]


@dataclass
class Job:
    """워커에게 넘기는 로봇 작업. done_mode 가 None 이면 이전 상태로 복귀한다."""
    fn: Callable[..., None]
    args: tuple
    done_mode: Optional[str] = None
    done_msg: str = ""
    clear_markers: bool = False
    kind: str = "job"          # pick / place / home / gripper / verify - 상태 보고용


@dataclass
class AppState:
    mode: str = STATE_IDLE
    status: str = "물체를 클릭하면 집습니다"
    hover: tuple[int, int] = (0, 0)
    view: str = "sensor"      # sensor(IR/센서 영상) / depth(컬러맵) / color(순수 컬러)
    view_note: str = ""       # 컬러 카메라 준비 상태 (여는 중 / 실패 사유)
    color_readonly: bool = False   # color 뷰가 비정렬 별개 렌즈라 클릭 불가인가
    depth_live: bool = True   # depth 를 지금도 읽고 있는가 (컬러 모드에서는 멈춘다)
    color_display: bool = False   # 지금 화면이 컬러 카메라 영상인가
    cam_index: Optional[int] = None   # 그 컬러 카메라의 번호
    show_reach: bool = True
    calibrating: bool = False
    markers: list[Marker] = field(default_factory=list)
    calib_cam: list[np.ndarray] = field(default_factory=list)
    calib_base: list[np.ndarray] = field(default_factory=list)


def colorize_depth(depth: np.ndarray,
                   lo: Optional[float] = None,
                   hi: Optional[float] = None) -> tuple[np.ndarray, float, float]:
    """depth -> 컬러맵. lo/hi 를 주지 않으면 유효 화소의 5~95 퍼센타일로 자동 스케일한다.

    고정 범위를 쓰면 테이블처럼 깊이 폭이 좁은 장면이 단색으로 뭉개져서
    물체 높이차가 눈에 보이지 않는다.
    """
    valid = depth[(depth > 1e-4) & np.isfinite(depth)]
    if lo is None or hi is None:
        if valid.size:
            lo, hi = (float(v) for v in np.percentile(valid, [5, 95]))
        else:
            lo, hi = 0.2, 1.0
        if hi - lo < 0.02:          # 너무 평평하면 최소 폭을 준다
            mid = (lo + hi) / 2
            lo, hi = mid - 0.01, mid + 0.01

    d = np.clip((depth - lo) / max(hi - lo, 1e-6), 0.0, 1.0)
    vis = cv2.applyColorMap((d * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    vis[depth <= 1e-4] = (0, 0, 0)  # depth 홀은 검정
    return vis, lo, hi


def draw_depth_scale(img: np.ndarray, lo: float, hi: float) -> None:
    """우측 하단에 컬러바와 근/원 거리 라벨을 그린다."""
    h, w = img.shape[:2]
    x0, y0, bw, bh = w - 150, h - 60, 130, 12
    bar = cv2.applyColorMap(
        np.tile(np.linspace(0, 255, bw, dtype=np.uint8), (bh, 1)), cv2.COLORMAP_TURBO)
    img[y0:y0 + bh, x0:x0 + bw] = bar
    cv2.rectangle(img, (x0, y0), (x0 + bw, y0 + bh), (255, 255, 255), 1)
    for txt, ax in ((f"{lo*100:.0f}cm", x0 - 2), (f"{hi*100:.0f}cm", x0 + bw - 34)):
        cv2.putText(img, txt, (ax, y0 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                    (255, 255, 255), 1, cv2.LINE_AA)


VIEW_MODES = ("sensor", "depth", "color")
VIEW_LABEL = {"sensor": "VIEW: SENSOR", "depth": "VIEW: DEPTH", "color": "VIEW: COLOR"}


def view_button_rect(w: int) -> tuple[int, int, int, int]:
    """화면 우상단의 보기 모드 버튼. 배너(높이 52) 바로 아래에 둔다."""
    return w - 128, 60, 118, 26


def cam_button_rect(w: int) -> tuple[int, int, int, int]:
    """컬러 카메라 번호를 바꾸는 버튼. 화면을 보고 고르라는 용도."""
    return w - 128, 124, 118, 26


def draw_view_button(frame: np.ndarray, st: AppState) -> None:
    x, y, bw, bh = view_button_rect(frame.shape[1])
    col = {"sensor": (70, 70, 78), "depth": (96, 76, 58),
           "color": (58, 96, 76)}[st.view]
    cv2.rectangle(frame, (x, y), (x + bw, y + bh), col, -1)
    cv2.rectangle(frame, (x, y), (x + bw, y + bh), (200, 200, 205), 1)
    cv2.putText(frame, VIEW_LABEL[st.view], (x + 8, y + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (240, 240, 245), 1, cv2.LINE_AA)


def draw_overlay(frame: np.ndarray, st: AppState, depth: np.ndarray,
                 intr: Intrinsics, he: HandEye,
                 est: Optional[np.ndarray] = None) -> np.ndarray:
    out = frame.copy()
    h, w = out.shape[:2]

    for m in st.markers:
        cv2.drawMarker(out, m.uv, m.color, cv2.MARKER_CROSS, 20, 2)
        cv2.circle(out, m.uv, 12, m.color, 2)
        cv2.putText(out, m.label, (m.uv[0] + 15, m.uv[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, m.color, 2, cv2.LINE_AA)

    # 커서 아래 지점의 3D 정보
    u, v = st.hover
    if 0 <= u < w and 0 <= v < h:
        z = sample_depth(depth, u, v)
        cv2.line(out, (u - 12, v), (u + 12, v), (255, 255, 255), 1)
        cv2.line(out, (u, v - 12), (u, v + 12), (255, 255, 255), 1)
        if z > 0:
            pc = deproject(intr, u, v, z)
            mark = " [추정]" if est is not None and est[v, u] else ""
            txt = f"({u},{v}){mark} d={z*1000:.0f}mm  cam=({pc[0]:+.3f},{pc[1]:+.3f},{pc[2]:.3f})"
            if he.calibrated:
                pb = he.cam_to_base(pc)
                txt += f"  base=({pb[0]:+.3f},{pb[1]:+.3f},{pb[2]:+.3f})"
        else:
            txt = f"({u},{v}) depth 없음"
        cv2.putText(out, txt, (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 1, cv2.LINE_AA)

    # 상태 배너
    banner = {STATE_IDLE: (60, 140, 60), STATE_HOLDING: (30, 140, 220),
              STATE_BUSY: (40, 40, 200)}[st.mode]
    if st.calibrating:
        banner = (150, 60, 150)
    cv2.rectangle(out, (0, 0), (w, 52), banner, -1)
    head = "CALIB" if st.calibrating else st.mode
    cv2.putText(out, head, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(out, st.status, (10, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1, cv2.LINE_AA)
    if not he.calibrated and not st.calibrating:
        cv2.putText(out, "hand-eye not calibrated: press 'k'", (w - 320, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

    draw_view_button(out, st)
    if st.color_display:
        cx, cy, cw, ch = cam_button_rect(w)
        cv2.rectangle(out, (cx, cy), (cx + cw, cy + ch), (70, 62, 58), -1)
        cv2.rectangle(out, (cx, cy), (cx + cw, cy + ch), (200, 200, 205), 1)
        cv2.putText(out, f"CAM {st.cam_index}" if st.cam_index is not None else "CAM ?",
                    (cx + 8, cy + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    (240, 240, 245), 1, cv2.LINE_AA)
    if st.view_note:
        cv2.putText(out, st.view_note, (10, h - 56), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (60, 200, 255), 1, cv2.LINE_AA)
    if st.view == "color" and st.color_readonly:
        # 비정렬 모드의 color 는 depth 와 안 맞는 별개 렌즈다. 여기서 클릭하면 엉뚱한 곳을 집는다.
        cv2.putText(out, "color view only - use DEPTH_ALIGN mode to click", (10, h - 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 200, 255), 1, cv2.LINE_AA)
    return out


PANEL_W = 340

# 패널 색 (BGR)
P_BG = (28, 28, 30)
P_FG = (232, 232, 236)
P_DIM = (130, 130, 138)
P_OK = (110, 210, 120)
P_BAD = (70, 70, 235)
P_ACC = (60, 168, 235)


class PanelButton:
    def __init__(self, x, y, w, h, label, action, col=None, scale=0.4):
        self.rect, self.label, self.action = (x, y, w, h), label, action
        self.col, self.scale = col, scale

    def hit(self, x, y) -> bool:
        rx, ry, rw, rh = self.rect
        return rx <= x <= rx + rw and ry <= y <= ry + rh

    def draw(self, img):
        x, y, w, h = self.rect
        cv2.rectangle(img, (x, y), (x + w, y + h), self.col or (62, 62, 70), -1)
        cv2.rectangle(img, (x, y), (x + w, y + h), (96, 96, 104), 1)
        (tw, th), _ = cv2.getTextSize(self.label, cv2.FONT_HERSHEY_SIMPLEX, self.scale, 1)
        cv2.putText(img, self.label, (x + (w - tw) // 2, y + (h + th) // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, self.scale, P_FG, 1, cv2.LINE_AA)


class HandEyePanel:
    """카메라->로봇 변환을 손으로 조절하고 저장하는 옆 패널.

    핵심은 오차 피드백이다. 그리퍼를 어떤 지점에 두고 화면에서 그 지점을 클릭하면
    '변환이 말하는 로봇좌표'와 '팔이 실제로 있는 좌표(FK)'가 나란히 뜨고 그 차이가
    바로 보인다. 그 값을 줄이는 방향으로 6개 파라미터를 누르면 된다.

    점을 여러 개 모아 두면 자동으로 풀 수도 있다(Kabsch).
    """

    TRANS_STEPS = [1.0, 5.0, 20.0]      # mm
    ROT_STEPS = [0.2, 1.0, 5.0]         # deg

    def __init__(self, app: "App"):
        self.app = app
        self.step_i = 1
        self.buttons: list[PanelButton] = []
        self.msg, self.msg_col = "그리퍼를 놓은 지점을 클릭하세요", P_DIM
        self.pairs: list[tuple[np.ndarray, np.ndarray]] = []
        self.last_cam: Optional[np.ndarray] = None
        self.last_uv: Optional[tuple[int, int]] = None
        self.last_split = 0.0               # 클릭 창의 앞뒤 깊이 차 (얇은 물체 경고용)
        self.records: list[dict] = []
        self.show_points = True             # 수집/불러온 점을 화면에 겹쳐 그린다
        self._tcp: Optional[np.ndarray] = None
        self._tcp_at = 0.0
        self._plane: Optional[tuple[np.ndarray, np.ndarray]] = None
        self._plane_at = 0.0
        self.set_from_T(app.he.T)

    # --- 파라미터 <-> 행렬 -------------------------------------------------
    def set_from_T(self, T: np.ndarray) -> None:
        self.t = (T[:3, 3] * 1000.0).astype(float)          # mm
        self.r = np.array(R_to_euler(T[:3, :3]), float)      # deg

    def to_T(self) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = euler_to_R(*self.r)
        T[:3, 3] = self.t / 1000.0
        return T

    def apply(self) -> None:
        self.app.he = HandEye(self.to_T())

    def bump(self, idx: int, sign: int) -> None:
        if idx < 3:
            self.t[idx] += sign * self.TRANS_STEPS[self.step_i]
        else:
            self.r[idx - 3] += sign * self.ROT_STEPS[self.step_i]
        self.apply()
        self.msg, self.msg_col = "조절됨 - 오차를 보며 계속 맞추세요", P_FG

    # --- 점 수집 / 자동 풀기 ----------------------------------------------
    def plane_stats(self) -> Optional[tuple[float, float, float]]:
        """테이블면이 로봇계에서 수평인가. 캘리브레이션 품질의 가장 예민한 지표다.

        평면 적합은 18ms 라 2초에 한 번만 하고, 변환 재적용(0.02ms)은 매 프레임 한다.
        그래야 파라미터를 누르는 즉시 기울기가 반응한다.
        """
        now = time.time()
        if self._plane is None or now - self._plane_at > 2.0:
            self._plane_at = now
            d = self.app.depth
            if d is not None and d.size:
                self._plane = fit_table_plane(self.app.src.intr, d)
        if self._plane is None:
            return None
        return plane_vs_robot(self.app.he, *self._plane)

    def tcp(self) -> Optional[np.ndarray]:
        """매 프레임 서보 6개를 읽으면 버스가 포화된다. 10Hz 로만 갱신하고 캐시한다."""
        now = time.time()
        if now - self._tcp_at >= 0.1:
            self._tcp_at = now
            self._tcp = self.app.arm.get_tcp()
        return self._tcp

    def add_pair(self) -> None:
        # 캐시가 아니라 지금 값을 읽는다. 점을 찍는 순간의 자세가 기록되어야 한다.
        tcp = self.app.arm.get_tcp()
        if self.last_cam is None:
            self.msg, self.msg_col = "먼저 화면에서 지점을 클릭하세요", P_BAD
            return
        if tcp is None:
            self.msg, self.msg_col = "팔이 연결되지 않아 로봇좌표를 읽을 수 없습니다", P_BAD
            return
        why = tcp_sanity(tcp)
        if why:
            self.msg, self.msg_col = why, P_BAD
            print(f"[calib] 경고: {why}")
            return          # 이 값으로 점을 모아봐야 맞지 않는다
        self.pairs.append((self.last_cam.copy(), np.asarray(tcp, float)))
        self.records.append({
            "uv": list(self.last_uv) if self.last_uv else None,
            "cam": [float(v) for v in self.last_cam],
            "tcp": [float(v) for v in tcp],
            "counts": self.app.arm.get_joint_counts(),   # 그 순간의 팔 자세
        })
        self.msg, self.msg_col = f"점 {len(self.pairs)}개 수집됨", P_OK
        self.warn_new_point()

    def save_points_only(self) -> None:
        """변환은 건드리지 않고 점만 저장한다.

        변환이 아직 맞지 않아도 힘들게 모은 점은 남길 수 있어야 한다.
        점 저장 때문에 잘못된 handeye.json 을 덮어쓰게 만들면 안 된다.
        """
        if not self.records:
            self.msg, self.msg_col = "저장할 점이 없습니다", P_BAD
            return
        self.save_points()
        self.msg, self.msg_col = f"점 {len(self.records)}개 저장됨 (변환은 그대로)", P_OK

    def save_points(self) -> None:
        """캘리브레이션에 쓴 점들을 남긴다. 나중에 재현성을 검증할 근거가 된다."""
        if not self.records:
            return
        with open(HANDEYE_POINTS_PATH, "w", encoding="utf-8") as f:
            json.dump({"saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                       "T_base_cam": self.to_T().tolist(),
                       "points": self.records}, f, indent=2, ensure_ascii=False)
        print(f"[calib] 점 {len(self.records)}개 저장: {HANDEYE_POINTS_PATH}")

    def arm_calib(self) -> Optional[ArmCalibration]:
        """지금 팔을 움직이고 있는 캘리브레이션. 실기가 없으면 파일에서 읽는다."""
        calib = getattr(self.app.arm, "calib", None)
        return calib if calib is not None else ArmCalibration.load()

    def refresh_tcp_from_counts(self, recs: list[dict]) -> tuple[int, float]:
        """기록된 서보 카운트로 로봇좌표를 다시 계산한다.

        점 파일에서 실측인 것은 uv/cam 과 counts 다. tcp 는 그 counts 를 그때의
        기구학으로 푼 결과일 뿐이라, 링크 상수나 영점이 바뀌면 같이 바뀌어야 한다.
        그러지 않으면 기구학을 고칠 때마다 점을 처음부터 다시 찍어야 한다.

        반환: (다시 계산한 점 수, 가장 많이 움직인 거리[m])
        """
        calib = self.arm_calib()
        if calib is None:
            return 0, 0.0
        n, worst = 0, 0.0
        for rec in recs:
            counts = rec.get("counts")
            if not counts:
                continue
            try:
                tcp = calib.tcp_from_counts(counts)
            except (KeyError, TypeError, ValueError):
                continue
            old = np.array(rec["tcp"], float)
            worst = max(worst, float(np.linalg.norm(tcp - old)))
            rec["tcp"] = [float(v) for v in tcp]
            n += 1
        return n, worst

    def load_points(self) -> None:
        """저장된 점을 되살리고 곧바로 화면에 띄운다.

        숫자만 '3개 불러옴' 이라고 하면 어떤 점이 들어왔는지 알 수 없다.
        점이 화면 어디에 찍혔고 현재 변환에서 얼마나 틀리는지가 같이 보여야
        지울 점과 남길 점을 판단할 수 있다.
        """
        if not os.path.exists(HANDEYE_POINTS_PATH):
            self.msg, self.msg_col = f"{os.path.basename(HANDEYE_POINTS_PATH)} 없음", P_BAD
            return
        try:
            with open(HANDEYE_POINTS_PATH, encoding="utf-8") as f:
                d = json.load(f)
            recs = [r for r in d.get("points", [])
                    if r.get("cam") is not None and r.get("tcp") is not None]
        except Exception as e:
            self.msg, self.msg_col = f"점 읽기 실패: {e}", P_BAD
            return
        if not recs:
            self.msg, self.msg_col = (f"{os.path.basename(HANDEYE_POINTS_PATH)} 에 "
                                      "쓸 수 있는 점이 없습니다"), P_BAD
            return

        # counts 가 남아 있으면 지금의 기구학으로 로봇좌표를 다시 푼다.
        n_re, worst = self.refresh_tcp_from_counts(recs)

        self.records = recs
        self.pairs = [(np.array(r["cam"], float), np.array(r["tcp"], float))
                      for r in recs]
        self.show_points = True             # 읽었으면 바로 보여야 확인이 된다

        errs = [float(np.linalg.norm(self.app.he.cam_to_base(c) - b))
                for c, b in self.pairs]
        a = float(np.mean(errs))
        tag = " (좌표 재계산됨)" if n_re else ""
        self.msg = f"점 {len(recs)}개 불러옴{tag} - 현재 변환 오차 평균 {a*1000:.1f}mm"
        self.msg_col = P_OK if a < CALIB_Z_TOL else P_BAD
        if n_re and a >= CALIB_Z_TOL:
            # 좌표가 바뀌었으면 예전 변환은 그 좌표로 푼 것이 아니다.
            self.msg = (f"점 {len(recs)}개 재계산됨 - '자동풀기' 로 변환을 다시 푸세요 "
                        f"(현재 오차 {a*1000:.0f}mm)")
        print(f"[calib] 점 {len(recs)}개 로드: {HANDEYE_POINTS_PATH}")
        if n_re:
            print(f"[calib] 그중 {n_re}개는 저장된 서보 카운트로 로봇좌표를 다시 계산했다 "
                  f"(최대 {worst*1000:.1f}mm 이동). 지금의 링크 상수/영점 기준이다.")
        elif any(r.get("counts") for r in recs):
            print("[calib] 서보 카운트는 있지만 arm_calib.json 이 없어 다시 계산하지 못했다")
        self.print_quality()
        for i, (rec, e) in enumerate(zip(recs, errs), 1):
            uv = rec.get("uv")
            where = f"uv={tuple(uv)}" if uv else "uv 없음(카메라좌표에서 투영)"
            print(f"  [{i}] {where}  오차 {e*1000:6.1f}mm")

    # --- 점 품질 진단 -----------------------------------------------------
    def warn_new_point(self) -> None:
        """방금 넣은 점이 기존 점들과 어긋나면 그 자리에서 알린다.

        나중에 RMS 로 알게 되면 어느 점이 문제였는지 기억나지 않는다. 찍은
        직후여야 다시 찍을 수 있다.
        """
        if len(self.pairs) < 2:
            return
        G = pair_gap_matrix(self.pairs)
        i = len(self.pairs) - 1
        j = int(np.argmax(G[i]))
        gap = float(G[i, j])
        if gap > GAP_TOL:
            self.msg = (f"{i+1}번 점 의심: {j+1}번과의 거리가 카메라/로봇에서 "
                        f"{gap*1000:.0f}mm 다릅니다")
            self.msg_col = P_BAD
            print(f"[calib] 경고: {i+1}번 점이 {j+1}번과 {gap*1000:.1f}mm 어긋난다 "
                  "(카메라가 잰 거리와 팔이 잰 거리가 다르다). 그리퍼 끝이 클릭한 그 지점에 "
                  "정확히 있었는지, depth 가 뒤쪽 테이블에서 잡히지 않았는지 확인하고 다시 찍을 것")
        if self.last_split > 0.015:
            print(f"[calib] 경고: 클릭한 창 안에 앞뒤 {self.last_split*1000:.0f}mm 차이의 "
                  f"두 면이 있다. 그리퍼가 허공에 떠 있으면 뒤쪽 면을 잴 위험이 크다")

    def print_quality(self) -> None:
        for line in quality_report(self.pairs):
            print(f"[calib] {line}")

    def quality_line(self) -> Optional[tuple[str, tuple[int, int, int]]]:
        """패널 한 줄 요약. 풀기 전에 데이터가 성한지 보여준다."""
        if len(self.pairs) < 2:
            return None
        q = point_set_quality(self.pairs)
        bad = q["gap_max"] > GAP_TOL
        flat = q["flat_cam"] < FLAT_TOL
        txt = (f"불일치 최대 {q['gap_max']*1000:.0f}mm"
               + (f" ({q['worst']+1}번)" if bad else "")
               + f" / 평면두께 {q['flat_cam']*1000:.0f}mm")
        return txt, (P_BAD if (bad or flat) else P_OK)

    def drop_worst(self) -> None:
        """가장 어긋난 점 하나를 버린다. 나쁜 점 하나가 전체를 망친다."""
        if len(self.pairs) < 3:
            self.msg, self.msg_col = "버릴 만큼 점이 많지 않습니다", P_BAD
            return
        q = point_set_quality(self.pairs)
        i = q["worst"]
        gap = float(q["per_point"][i])
        del self.pairs[i]
        if i < len(self.records):
            del self.records[i]
        self.msg = f"{i+1}번 점 버림 (불일치 {gap*1000:.0f}mm), {len(self.pairs)}개 남음"
        self.msg_col = P_FG
        self.print_quality()

    def toggle_points(self) -> None:
        self.show_points = not self.show_points
        self.msg = ("점을 화면에 표시합니다" if self.show_points
                    else "점 표시를 껐습니다")
        self.msg_col = P_FG

    # --- 화면 위 점 표시 ---------------------------------------------------
    def point_uv(self, rec: dict, intr: Intrinsics) -> Optional[tuple[int, int]]:
        """기록된 점의 픽셀 위치. uv 가 없으면 카메라좌표를 되투영한다."""
        uv = rec.get("uv")
        if uv is None:
            uv = project(intr, np.array(rec["cam"], float))
        if uv is None:
            return None
        return int(round(uv[0])), int(round(uv[1]))

    def draw_points(self, frame: np.ndarray, intr: Intrinsics) -> None:
        """수집/불러온 점을 색으로 오차를 알려주며 겹쳐 그린다.

        초록/파랑/빨강은 그 점 하나의 현재 오차다. 빨간 점만 지우면
        나머지로 다시 풀 수 있다.
        """
        if not self.show_points or not self.records:
            return
        h, w = frame.shape[:2]
        for i, rec in enumerate(self.records, 1):
            uv = self.point_uv(rec, intr)
            if uv is None or not (0 <= uv[0] < w and 0 <= uv[1] < h):
                continue
            cam = np.array(rec["cam"], float)
            err = float(np.linalg.norm(self.app.he.cam_to_base(cam)
                                       - np.array(rec["tcp"], float)))
            col = P_OK if err < 0.01 else (P_ACC if err < 0.03 else (60, 60, 255))
            cv2.drawMarker(frame, uv, col, cv2.MARKER_TILTED_CROSS, 14, 2)
            cv2.circle(frame, uv, 9, col, 1)
            cv2.putText(frame, f"{i} {err*1000:.0f}mm", (uv[0] + 11, uv[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv2.LINE_AA)

    # --- 재현성 검증 -------------------------------------------------------
    def verify_points(self) -> None:
        """저장된 각 점으로 실제 이동해 '그때 그 자세'로 돌아가는지 확인한다.

        캘리브레이션이 맞다면, 카메라가 본 점을 변환한 위치로 팔을 보냈을 때
        그 점을 찍을 당시의 자세가 재현되어야 한다. 순수 계산 오차와 달리
        기구학/캘리브레이션/서보를 한 번에 통과하는 검증이다.

        그리퍼는 닫지 않는다. 물체를 실제로 집으면 놓친 경우 넘어뜨리거나
        엉뚱한 것을 물 수 있어서, 위치 재현성만 본다.
        """
        if not self.records:
            self.msg, self.msg_col = "검증할 점이 없습니다", P_BAD
            return
        if self.app.arm.get_tcp() is None:
            self.msg, self.msg_col = "팔이 연결되어 있지 않습니다", P_BAD
            return

        # 기록된 자세 자체가 안전상자 밖이면 캘리브레이션이 아니라 상자가 틀렸다.
        # 이 경우 모든 점이 '이동 불가' 로 걸러져 검증이 통째로 실패하는데,
        # 원인을 말해주지 않으면 캘리브레이션을 계속 다시 하게 된다.
        zs = [r["tcp"][2] for r in self.records]
        outside = [z for z in zs if not (CALIB_Z_MIN <= z <= WS_Z_MAX)]
        if len(outside) == len(zs):
            self.msg = (f"기록된 점이 전부 허용범위 밖 (z {min(zs)*1000:.0f}~{max(zs)*1000:.0f}mm, "
                        f"허용 {CALIB_Z_MIN*1000:.0f}~{WS_Z_MAX*1000:.0f}mm)")
            self.msg_col = P_BAD
            print("[검증] 시작도 못 한다. 점을 찍을 때 팔이 보고한 높이가 "
                  f"z={min(zs)*1000:.0f}~{max(zs)*1000:.0f}mm 인데 "
                  f"캘리브레이션 허용범위는 {CALIB_Z_MIN*1000:.0f}~{WS_Z_MAX*1000:.0f}mm 다 "
                  f"(테이블 아래로 {CALIB_Z_TOL*1000:.0f}mm 까지 봐준 값).")
            print("       팔이 테이블 위에 놓여 있다면 테이블면(z=0) 아래는 물리적으로 불가능하다.")
            print("       -> 카메라가 아니라 관절 영점(arm_calib.json)이 틀린 것이다. "
                  "calibrate_arm.py 로 영점을 다시 잡고, 그리퍼를 테이블에 댔을 때 "
                  "TCP z 가 0 근처로 나오는지 확인한 뒤 점을 다시 찍을 것.")
            print("       (베이스를 일부러 받침대 위에 올린 설치라면 그때는 WS_Z_MIN 쪽을 고친다)")
            return

        self.app._submit(Job(self._verify_job, (), done_msg="", kind="verify"))
        self.msg, self.msg_col = "검증 중... 콘솔을 보세요", P_ACC

    def joint_compare(self, rec: dict, pred: np.ndarray) -> Optional[dict]:
        """그 점에서 관절별로 얼마나 어긋나는지 카운트/도 단위로 본다.

        TCP 거리(mm)는 어느 관절이 범인인지 알려주지 않는다. 서보 카운트는 가장
        날것의 측정값이라 기구학을 거치지 않고 바로 비교할 수 있다.

          기록  = 점을 찍을 때 실제로 있던 카운트
          계산  = 지금 캘리브레이션이 '거기로 가려면 이 카운트' 라고 말하는 값
          실제  = 그 명령을 보내고 실제로 멈춘 카운트

        기록↔계산 차이는 캘리브레이션/기구학 오차, 계산↔실제 차이는 서보가
        목표에 못 간 양(부하, 리밋 클램프)이다.
        """
        want = rec.get("counts")
        calib = getattr(self.app.arm, "calib", None)
        if not want or calib is None:
            return None
        try:
            # 기록된 자세의 접근각/롤로 푼다. IK 는 기본이 수직 하강(-90도)인데,
            # 사람이 손으로 잡아 찍은 자세는 대개 몇 도 다르다. 그 차이를 그대로
            # 두면 같은 점을 다른 관절 조합으로 가는 것이라 관절 차이가 실제
            # 캘리브레이션 오차보다 5~10배 크게 보인다(실측: 10도 vs 1도).
            q0 = calib.counts_to_ik(want)
            pitch = (A3_ZERO + q0["shoulder_lift"] + q0["elbow_flex"]
                     + q0["wrist_flex"])
            q = so101_ik(*pred, pitch=pitch, roll=q0["wrist_roll"])
            calc = {n: calib.ik_to_count(n, q[n]) for n in JOINT_ORDER}
        except (IKError, KeyError):
            return None
        got = self.app.arm.get_joint_counts() or {}
        out = {"_pitch_deg": math.degrees(pitch)}
        for n in JOINT_ORDER:
            if n not in want:
                continue
            out[n] = {"want": int(want[n]), "calc": int(calc[n]),
                      "got": (int(got[n]) if n in got else None)}
        return out

    @staticmethod
    def _fmt_joint_row(name: str, d: dict) -> str:
        deg = COUNTS_PER_DEG
        calc_d = (d["calc"] - d["want"]) / deg
        row = (f"        {SHORT_NAME.get(name, name):<6} 기록 {d['want']:>4}"
               f" | 계산 {d['calc']:>4} ({d['calc']-d['want']:+4d}, {calc_d:+6.2f}deg)")
        if d["got"] is not None:
            got_d = (d["got"] - d["calc"]) / deg
            row += (f" | 실제 {d['got']:>4} ({d['got']-d['calc']:+4d},"
                    f" {got_d:+6.2f}deg)")
        return row

    def _verify_job(self) -> None:
        arm = self.app.arm
        errs, moved, skipped = [], 0, 0
        jerr: dict[str, list[float]] = {n: [] for n in JOINT_ORDER}   # 기록<->계산 [deg]
        jgot: dict[str, list[float]] = {n: [] for n in JOINT_ORDER}   # 계산<->실제 [deg]
        print(f"\n[검증] 저장된 {len(self.records)}개 점으로 위치 재현성 확인")
        print("       (그리퍼는 닫지 않는다 - 위치만 본다)")

        for i, rec in enumerate(self.records, 1):
            cam = np.array(rec["cam"], float)
            want = np.array(rec["tcp"], float)          # 점을 찍을 때 팔이 있던 곳
            pred = self.app.he.cam_to_base(cam)         # 카메라가 말하는 곳
            math_err = float(np.linalg.norm(pred - want))

            # 목표점 자체로 간다. pick 의 파지 깊이(GRASP_SINK)만큼 더 내려가면
            # 기록된 표면 위치와 그만큼 어긋나서, 완벽한 캘리브레이션도 12mm
            # 오차로 보인다. 여기서는 위치 재현성만 보므로 목표점을 그대로 쓴다.
            z_appr = clamp_z(pred[0], pred[1], pred[2] + APPROACH_H, pred[2])
            steps = [("move", pred[0], pred[1], z_appr, 0.0, 0.8),
                     ("move", pred[0], pred[1], pred[2], 0.0, 1.0)]
            # 이미 그 자세에 있었던 것을 재현하는 것이므로 캘리브레이션 여유를 준다.
            ok, why = check_steps(steps, z_min=CALIB_Z_MIN)
            if not ok:
                print(f"  [{i}] 예측 {np.round(pred*1000,1)}mm  계산오차 {math_err*1000:6.1f}mm"
                      f"  -> 이동 불가: {why}")
                skipped += 1
                continue
            try:
                run_steps(arm, steps, tag=f"verify{i}", z_min=CALIB_Z_MIN)
                got = arm.get_tcp()
                moved += 1
            except Exception as e:
                print(f"  [{i}] 이동 실패: {e}")
                skipped += 1
                continue

            real_err = float(np.linalg.norm(np.asarray(got) - want)) if got is not None else float("nan")
            errs.append(real_err)
            jc = self.joint_compare(rec, pred)
            print(f"  [{i}] 계산오차 {math_err*1000:6.1f}mm | 실제 도달 후 오차 {real_err*1000:6.1f}mm"
                  f"  (기록 {np.round(want*1000,0)} -> 실제 {np.round(np.asarray(got)*1000,0)})")
            if jc:
                print(f"        (기록 접근각 {jc['_pitch_deg']:+.1f}deg 기준으로 비교)")
                for n, dd in jc.items():
                    if n.startswith("_"):
                        continue
                    print(self._fmt_joint_row(n, dd))
                    jerr[n].append((dd["calc"] - dd["want"]) / COUNTS_PER_DEG)
                    if dd["got"] is not None:
                        jgot[n].append((dd["got"] - dd["calc"]) / COUNTS_PER_DEG)

        try:
            arm.home()
        except Exception:
            pass

        if errs:
            a = float(np.mean(errs))
            self.msg = f"검증: {moved}점 평균오차 {a*1000:.1f}mm" + (f", {skipped}점 불가" if skipped else "")
            self.msg_col = P_OK if a < CALIB_Z_TOL else P_BAD
            print(f"[검증] 평균 {a*1000:.1f}mm  최대 {max(errs)*1000:.1f}mm  "
                  f"({moved}점 이동, {skipped}점 건너뜀) - 합격선 {CALIB_Z_TOL*1000:.0f}mm")
            self._print_joint_summary(jerr, jgot)
        else:
            self.msg, self.msg_col = (f"검증 불가 - {skipped}점 모두 이동 불가 "
                                      f"(위 콘솔의 사유를 볼 것)"), P_BAD

    @staticmethod
    def _print_joint_summary(jerr: dict, jgot: dict) -> None:
        """관절별로 어디가 얼마나 어긋나는지 한눈에.

        한 관절만 유독 크면 그 관절의 영점(offset_deg)이 범인이다. 전 관절이
        고루 작으면 남은 것은 클릭/depth 잡음이라 캘리브레이션으로는 못 줄인다.
        """
        rows = [(n, v) for n, v in jerr.items() if v]
        if not rows:
            return
        print("[검증] 관절별 어긋남 (기록 자세 대비 계산 자세)")
        worst = max(rows, key=lambda kv: max(abs(x) for x in kv[1]))
        for n, v in rows:
            a = float(np.mean(np.abs(v)))
            mx = float(max(np.abs(v)))
            line = (f"       {SHORT_NAME.get(n, n):<6} 평균 {a:5.2f}deg  최대 {mx:5.2f}deg"
                    f"  ({a*COUNTS_PER_DEG:4.0f}카운트)")
            if jgot.get(n):
                line += f"  | 서보 추종오차 평균 {float(np.mean(np.abs(jgot[n]))):4.2f}deg"
            print(line + ("   <-- 가장 큼" if n == worst[0] else ""))
        print("       한 관절만 크면 그 관절 영점(offset_deg)을 의심할 것. "
              "고루 작으면 클릭/depth 잡음이다.")

    def auto_solve(self) -> None:
        if len(self.pairs) < 3:
            self.msg, self.msg_col = "점이 3개 이상 필요합니다", P_BAD
            return
        try:
            T, rms = solve_rigid_transform(np.array([a for a, _ in self.pairs]),
                                           np.array([b for _, b in self.pairs]))
        except Exception as e:
            self.msg, self.msg_col = f"풀기 실패: {e}", P_BAD
            return
        self.set_from_T(T)
        self.apply()
        # 푼 결과를 바로 파일에 남긴다. 예전에는 handeye.json 을 따로 '저장' 해야
        # 반영돼서, 자동풀기만 하고 검증을 누르면 옛 변환으로 계산된 큰 오차가
        # 나왔다. 점 파일과 변환 파일이 항상 같은 해를 가리키게 한다.
        self.app.he.save()
        self.save_points()          # 계산에 쓴 점도 그대로 남긴다
        self.msg = (f"{len(self.pairs)}점으로 풀어 저장했습니다 "
                    f"(RMS {rms*1000:.1f}mm)")
        self.msg_col = P_OK if rms < CALIB_Z_TOL else P_BAD
        self.print_quality()
        if rms >= CALIB_Z_TOL:
            q = point_set_quality(self.pairs)
            self.msg = (f"RMS {rms*1000:.0f}mm - 점 데이터가 서로 모순됩니다 "
                        f"({q['worst']+1}번 의심)")
            print("[calib] RMS 가 큰 것은 푸는 방법이 아니라 점이 문제다. "
                  "위 진단을 보고 의심되는 점을 다시 찍거나 버릴 것")

    def align_plane(self) -> None:
        if self._plane is None:
            self.msg, self.msg_col = "평면을 아직 찾지 못했습니다", P_BAD
            return
        n, pts = self._plane
        self.set_from_T(align_to_plane(self.app.he, n, pts))
        self.apply()
        h, tilt = plane_camera_pose(n, pts)
        self.msg = f"평면 정렬됨 (높이 {h:.0f}mm) - 이제 yaw/tx/ty 만 맞추면 됩니다"
        self.msg_col = P_OK

    def clear_pairs(self) -> None:
        self.pairs.clear()
        self.records.clear()
        self.msg, self.msg_col = "수집한 점을 비웠습니다", P_FG

    def save(self) -> None:
        self.apply()
        self.app.he.save()
        self.save_points()
        n = f" (+점 {len(self.records)}개)" if self.records else ""
        self.msg, self.msg_col = f"{os.path.basename(HANDEYE_PATH)} 저장됨{n}", P_OK

    def reload(self) -> None:
        he = HandEye.load()
        if not he.calibrated:
            self.msg, self.msg_col = "저장된 handeye.json 이 없습니다", P_BAD
            return
        self.app.he = he
        self.set_from_T(he.T)
        self.msg, self.msg_col = "파일에서 다시 읽었습니다", P_FG

    # --- 입력 -------------------------------------------------------------
    def on_click(self, x: int, y: int) -> None:
        for b in self.buttons:
            if b.hit(x, y):
                b.action()
                return

    # --- 그리기 -----------------------------------------------------------
    def draw(self, h: int) -> np.ndarray:
        img = np.full((h, PANEL_W, 3), P_BG, np.uint8)
        self.buttons = []

        def t(s, xy, sc=0.42, c=P_FG):
            cv2.putText(img, s, xy, cv2.FONT_HERSHEY_SIMPLEX, sc, c, 1, cv2.LINE_AA)

        t("hand-eye 조절", (12, 24), 0.6, P_FG)
        cal = self.app.he.calibrated
        t("보정됨" if cal else "미보정", (240, 24), 0.42, P_OK if cal else P_BAD)

        labels = ["tx", "ty", "tz", "roll", "pitch", "yaw"]
        units = ["mm", "mm", "mm", "deg", "deg", "deg"]
        y = 44
        for i, (lb, un) in enumerate(zip(labels, units)):
            val = self.t[i] if i < 3 else self.r[i - 3]
            t(lb, (12, y + 15), 0.44, P_DIM)
            t(f"{val:+9.2f} {un}", (56, y + 15), 0.46)
            bm = PanelButton(212, y, 30, 22, "-", lambda i=i: self.bump(i, -1))
            bp = PanelButton(248, y, 30, 22, "+", lambda i=i: self.bump(i, +1))
            self.buttons += [bm, bp]
            bm.draw(img)
            bp.draw(img)
            y += 22

        step = (f"{self.TRANS_STEPS[self.step_i]:g}mm / "
                f"{self.ROT_STEPS[self.step_i]:g}deg")
        t("스텝", (12, y + 16), 0.42, P_DIM)
        bs = PanelButton(56, y, 150, 22, step,
                         lambda: setattr(self, "step_i",
                                         (self.step_i + 1) % len(self.TRANS_STEPS)))
        self.buttons.append(bs)
        bs.draw(img)
        y += 26

        cv2.line(img, (10, y), (PANEL_W - 10, y), (60, 60, 66), 1)
        y += 16

        # --- 오차 피드백 ---------------------------------------------------
        if self.last_cam is not None:
            pb = self.app.he.cam_to_base(self.last_cam)
            t("클릭 지점 -> 로봇좌표 [mm]", (12, y), 0.38, P_DIM)
            t(f"{pb[0]*1000:+7.1f} {pb[1]*1000:+7.1f} {pb[2]*1000:+7.1f}", (12, y + 18), 0.44)
            tcp = self.tcp()
            if tcp is not None:
                sane = tcp_sanity(tcp)
                t("실제 팔 TCP [mm]" + ("" if sane is None else " (영점 이상!)"),
                  (12, y + 38), 0.38, P_DIM if sane is None else P_BAD)
                t(f"{tcp[0]*1000:+7.1f} {tcp[1]*1000:+7.1f} {tcp[2]*1000:+7.1f}",
                  (12, y + 56), 0.44, P_FG if sane is None else P_BAD)
                d = np.asarray(tcp, float) - pb
                e = float(np.linalg.norm(d))
                col = P_OK if e < 0.01 else (P_ACC if e < 0.03 else P_BAD)
                t(f"오차 {e*1000:6.1f} mm", (12, y + 78), 0.5, col)
                t(f"dx{d[0]*1000:+5.0f} dy{d[1]*1000:+5.0f} dz{d[2]*1000:+5.0f}",
                  (146, y + 78), 0.4, col)
            else:
                t("팔 미연결 - 오차 계산 불가", (12, y + 38), 0.38, P_DIM)
        else:
            t("화면에서 지점을 클릭하면", (12, y + 14), 0.4, P_DIM)
            t("여기에 오차가 표시됩니다", (12, y + 32), 0.4, P_DIM)
        y += 86

        # --- 테이블면 검증 -------------------------------------------------
        cv2.line(img, (10, y - 6), (PANEL_W - 10, y - 6), (60, 60, 66), 1)
        st = self.plane_stats()
        if st is None:
            t("테이블면 검증: 평면을 찾지 못함", (12, y + 12), 0.4, P_DIM)
        else:
            tilt, zm, zs = st
            good = tilt < 5.0 and abs(zm) < 20.0
            col = P_OK if good else P_BAD
            t("테이블면 검증 (0 에 가까워야 정상)", (12, y + 12), 0.38, P_DIM)
            t(f"기울기 {tilt:5.1f} deg", (12, y + 32), 0.46, col)
            t(f"z평균 {zm:+6.0f} mm", (150, y + 32), 0.46, col)
            if self._plane is not None:
                h, ct = plane_camera_pose(*self._plane)
                t(f"실측 카메라 높이 {h:.0f}mm  기울기 {ct:.0f}deg", (12, y + 50), 0.38, P_DIM)
            ba = PanelButton(228, y + 38, 100, 20, "평면정렬", self.align_plane,
                             (58, 76, 96), 0.38)
            self.buttons.append(ba)
            ba.draw(img)
        y += 58

        for x, w, lbl, act, col in [
            (12, 86, f"점추가 ({len(self.pairs)})", self.add_pair, (58, 76, 96)),
            (102, 72, "자동풀기", self.auto_solve, (96, 76, 58)),
            (178, 74, "이상점버림", self.drop_worst, (96, 58, 58)),
            (256, 72, "비우기", self.clear_pairs, (58, 58, 66)),
        ]:
            b = PanelButton(x, y, w, 26, lbl, act, col)
            self.buttons.append(b)
            b.draw(img)
        y += 29
        for x, w, lbl, act, col in [
            (12, 82, f"검증 ({len(self.records)})", self.verify_points, (96, 76, 58)),
            (98, 74, "점 저장", self.save_points_only, (48, 84, 56)),
            (176, 74, "점 읽기", self.load_points, (58, 58, 66)),
            (254, 74, "표시 ON" if self.show_points else "표시 OFF",
             self.toggle_points, (58, 76, 96) if self.show_points else (58, 58, 66)),
        ]:
            b = PanelButton(x, y, w, 26, lbl, act, col)
            self.buttons.append(b)
            b.draw(img)
        y += 29
        for x, w, lbl, act, col in [
            (12, 150, "저장 (handeye.json)", self.save, (48, 84, 56)),
            (170, 156, "파일에서 다시읽기", self.reload, (48, 62, 72)),
        ]:
            b = PanelButton(x, y, w, 26, lbl, act, col)
            self.buttons.append(b)
            b.draw(img)
        y += 32

        # 풀기 전에 데이터가 성한지부터 보인다. RMS 는 푼 뒤에야 나오지만
        # 거리 불일치는 점만 있으면 바로 알 수 있고, 원인도 그쪽에 있다.
        q = self.quality_line()
        if q is not None:
            t(q[0], (12, y), 0.36, q[1])
            y += 14
        for i, line in enumerate(_wrap(self.msg, 44)[:1]):
            t(line, (12, y + i * 16), 0.38, self.msg_col)
        return img


def _wrap(s: str, n: int) -> list[str]:
    out, cur = [], ""
    for wd in s.split():
        if len(cur) + len(wd) + 1 > n:
            out.append(cur)
            cur = wd
        else:
            cur = f"{cur} {wd}".strip()
    if cur:
        out.append(cur)
    return out or [""]


class App:
    def __init__(self, source: DepthSource, arm: RobotArm, handeye: HandEye):
        self.src, self.arm, self.he = source, arm, handeye
        self.st = AppState()
        self.jobs: "queue.Queue[Job]" = queue.Queue()
        self.depth = np.zeros((source.intr.height, source.intr.width), np.float32)
        self.depth_est = np.zeros(self.depth.shape, bool)   # 주변값으로 메운 화소
        # 소스가 이미 depth 를 컬러에 정렬해 주면, 화면(=컬러)과 depth 가 1:1 이라
        # 별도 컬러 뷰도 ALIGN 호모그래피도 필요 없다.
        self.depth_aligned = bool(getattr(source, "depth_aligned", False))
        self.color_cam = ColorCamera()                  # 보기용 별도 컬러 카메라
        self._src_paused = False                        # depth 스트림을 재웠는가
        self.current_job: Optional[str] = None          # 지금 워커가 도는 작업 종류
        self.last_error: Optional[str] = None           # 마지막 실패 사유
        self.last_color: Optional[np.ndarray] = None    # REST 로 넘길 원본 영상
        self.last_frame: Optional[np.ndarray] = None    # 오버레이까지 그린 영상
        self.running = True
        self.calib_on_start = False      # 시작하자마자 hand-eye 캘리브레이션으로 들어갈지
        self._cmd_lock = threading.Lock()   # 마우스와 REST 가 동시에 명령하는 것을 막는다
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.panel = HandEyePanel(self)

    # --- 로봇 동작 워커 (UI 프리즈 방지) ----------------------------------
    def _worker_loop(self) -> None:
        while self.running:
            try:
                job = self.jobs.get(timeout=0.2)
            except queue.Empty:
                continue

            prev_mode = self.st.mode
            self.st.mode = STATE_BUSY
            self.current_job = job.kind
            try:
                job.fn(*job.args)
                self.st.mode = job.done_mode or prev_mode
                self.st.status = job.done_msg or self.st.status
                if job.clear_markers:
                    self.st.markers = []
            except Exception as e:
                self.st.mode = prev_mode
                self.st.status = f"실패: {e}"
                self.last_error = str(e)
                print(f"[error] {e}")
            finally:
                self.current_job = None
                self.jobs.task_done()

    def _submit(self, job: Job) -> None:
        self.jobs.put(job)

    # --- 마우스 ------------------------------------------------------------
    def on_mouse(self, event: int, x: int, y: int, flags: int, param) -> None:
        # 오른쪽은 조절 패널이다. 카메라 영상 좌표와 섞이지 않게 먼저 갈라낸다.
        cam_w = self.src.intr.width
        if x >= cam_w:
            if event == cv2.EVENT_LBUTTONDOWN:
                self.panel.on_click(x - cam_w, y)
            return

        if event == cv2.EVENT_MOUSEMOVE:
            self.st.hover = (x, y)
            return
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        bx, by, bw, bh = view_button_rect(cam_w)         # 보기 모드 버튼이 먼저다
        if bx <= x <= bx + bw and by <= y <= by + bh:
            self.cycle_view()
            return

        cx, cy, cw, ch = cam_button_rect(cam_w)          # 컬러 카메라 번호 바꾸기
        cam = self.active_color_cam()
        if cam is not None and cx <= x <= cx + cw and cy <= y <= cy + ch:
            n = cam.next_index()
            self.st.status = (f"컬러 카메라 index={n} 로 바꿨습니다 "
                              "(맞는 화면이 나올 때까지 누르세요, 선택은 저장됩니다)")
            print(f"[color] 사용자가 index={n} 선택")
            return

        if self.st.view == "color" and not self.depth_aligned:
            # 이 화면은 depth 와 정렬되지 않은 별개 렌즈다. 클릭해서 집으려면
            # DEPTH_ALIGN 모드를 쓴다 - 거기서는 색 뷰 자체가 이미 정렬돼 있어 클릭이 된다.
            self.st.status = ("일반 카메라 화면에서는 클릭할 수 없습니다 "
                              "(DEPTH_ALIGN 모드를 쓰세요)")
            return

        z = sample_depth(self.depth, x, y)                       # 집기용: 창 전체
        z_near = sample_depth(self.depth, x, y, near=True)       # 캘리브용: 앞면만
        if z <= 0:
            self.st.status = "그 지점의 depth 가 없습니다 (다시 클릭)"
            return
        est = bool(self.depth_est[y, x])          # 주변값으로 메운 화소인가
        # 캘리브레이션 점은 그리퍼 끝(앞면)이어야 한다. 뒤쪽 테이블을 재면
        # 점이 평면으로 무너져서 어떤 알고리즘도 못 푸는 데이터가 된다.
        p_cam = deproject(self.src.intr, x, y, z_near)
        self.panel.last_cam = p_cam          # 패널이 오차를 계산할 기준점
        self.panel.last_uv = (x, y)
        self.panel.last_split = float(z - z_near)   # 창 안에 앞뒤 두 면이 있는가

        if self.st.calibrating:
            if est:
                # 캘리브레이션은 실측만 써야 한다. 추정값으로 점을 모으면
                # 그 오차가 변환에 그대로 박힌다.
                print("[calib] 경고: 이 화소의 depth 는 주변에서 메운 추정값이다. "
                      "가능하면 실측 depth 가 있는 지점을 클릭할 것")
            self._calib_click(x, y, p_cam)
            return
        # 손이 비었으면 집고, 물고 있으면 놓는다. 실제 판단과 실행은 command()
        # 한 곳에서만 한다 - REST 요청도 같은 함수를 부른다.
        action = "pick" if self.st.mode == STATE_IDLE else "place"
        ok, msg, _ = self.command(action, x, y)
        if not ok:
            self.st.status = msg

    def _pause_src(self) -> None:
        """컬러가 흐르기 시작했을 때만 depth 를 재운다 (USB 대역 양보)."""
        if self._src_paused:
            return
        try:
            self.src.pause()
            self._src_paused = True
            self.st.depth_live = False
        except Exception as e:
            print(f"[source] 일시정지 실패(무시): {e}")

    def _resume_src(self) -> None:
        if not self._src_paused:
            return
        try:
            self.src.resume()
        except Exception as e:
            print(f"[source] 재개 실패: {e}")
        self._src_paused = False

    def active_color_cam(self) -> Optional["ColorCamera"]:
        """지금 화면에 컬러를 대고 있는 카메라. 없으면 None.

        정렬 모드에서는 소스가 컬러를 들고 있고, 아니면 앱이 들고 있다.
        """
        if self.depth_aligned:
            return getattr(self.src, "color_cam", None)
        return self.color_cam if self.st.view == "color" else None

    def cycle_view(self) -> None:
        """센서 -> depth 컬러맵 -> 순수 컬러 순환.

        정렬 모드에서는 '센서' 화면이 이미 컬러라 'color' 모드도 같은 정렬된
        피드를 그대로 쓴다(카메라를 두 번 열지 않는다) - 도달가능영역 같은
        오버레이만 뺀 순수 화면이고, 정렬돼 있으니 그대로 클릭할 수 있다.
        비정렬 모드에서는 기존처럼 별개 UVC 렌즈를 열어 보기 전용으로 보여준다.
        """
        modes = VIEW_MODES
        self.st.view = modes[(modes.index(self.st.view) + 1) % len(modes)]             if self.st.view in modes else modes[0]
        legacy_color = self.st.view == "color" and not self.depth_aligned
        self.st.color_readonly = legacy_color
        # 컬러는 쓸 때만 연다. 켜 둔 채로 두면 depth 가 쓸 USB 대역을 계속 빼앗는다.
        if legacy_color:
            # 여기서 depth 를 재우지 않는다. 컬러 첫 프레임이 오기 전에 재우면
            # 화면이 정지 화면으로 굳어서 '멈춘 것' 처럼 보인다. 컬러가 실제로
            # 들어오는 순간에 루프가 재운다.
            self.color_cam.start(ref=self.last_color)
        else:
            self.color_cam.stop()
            self.st.view_note = ""
            self._resume_src()
        note = {"sensor": "센서 영상 (클릭 가능)",
                "depth": "depth 컬러맵 (클릭 가능)",
                "color": ("일반 카메라 - 보기 전용, depth 와 정렬되지 않아 클릭 불가"
                          if legacy_color else
                          "순수 컬러 - 오버레이 없음 (클릭 가능)")}
        self.st.status = note[self.st.view]
        print(f"[view] {self.st.view} - {note[self.st.view]}")

    # --- 컬러 화면에서 집기 -------------------------------------------------
    # --- 화면 좌표로 집기/놓기 (마우스와 REST 가 공유하는 한 경로) -----------
    def command(self, action: str, x: int, y: int) -> tuple[bool, str, dict]:
        """화면 좌표 (x, y) 로 pick 또는 place 를 건다.

        마우스 클릭과 REST 요청이 같은 함수를 쓴다. 경로가 갈라지면 한쪽에만
        검사가 빠지고, 그 빠진 검사가 팔을 테이블에 꽂는다.

        반환: (접수됨, 사람이 읽을 메시지, 상세 dict)
        """
        with self._cmd_lock:
            return self._command_locked(action, int(x), int(y))

    def _command_locked(self, action: str, x: int, y: int) -> tuple[bool, str, dict]:
        h, w = self.depth.shape[:2]
        info: dict = {"action": action, "uv": [int(x), int(y)]}
        if not (0 <= x < w and 0 <= y < h):
            return False, f"좌표 ({x},{y}) 가 화면({w}x{h}) 밖입니다", info
        if self.st.calibrating:
            return False, "캘리브레이션 중에는 집기/놓기를 하지 않습니다", info
        if self.st.mode == STATE_BUSY:
            return False, "동작 중입니다", info
        if not self.he.calibrated:
            return False, "hand-eye 미보정 - 'k' 로 캘리브레이션하세요", info
        if not self.st.depth_live:
            # 컬러 모드에서는 depth 가 멈춰 있다. 낡은 depth 로 집으면 그 사이
            # 물체가 움직였어도 옛 자리로 간다.
            return False, "컬러 모드라 depth 가 멈춰 있습니다 (SENSOR 로 전환하세요)", info

        # 상태 확인: 손이 비어야 집고, 물고 있어야 놓는다. 마우스 클릭이
        # IDLE/HOLDING 으로 갈리는 것과 같은 규칙이다.
        if action == "pick" and self.st.mode != STATE_IDLE:
            return False, "이미 물체를 물고 있습니다 (place 먼저)", info
        if action == "place" and self.st.mode != STATE_HOLDING:
            return False, "물고 있는 물체가 없습니다 (pick 먼저)", info

        z = sample_depth(self.depth, x, y)
        if z <= 0:
            return False, "그 지점의 depth 가 없습니다", info
        est = bool(self.depth_est[y, x])
        p_base = self.he.cam_to_base(deproject(self.src.intr, x, y, z))
        info.update(depth_m=round(float(z), 4), estimated_depth=est,
                    target_base_mm=[round(float(v) * 1000, 1) for v in p_base])

        steps = pick_steps(p_base) if action == "pick" else place_steps(p_base)
        ok, why = check_steps(steps)
        if not ok:
            return False, f"도달 불가: {why}", info

        pos = f"({p_base[0]:.3f},{p_base[1]:.3f},{p_base[2]:.3f})"
        if est:
            pos += " [추정 depth]"
        if action == "pick":
            self.st.markers = [Marker((x, y), "PICK", (0, 255, 0))]
            self.st.status = f"집는 중... {pos}"
            self._submit(Job(do_pick, (self.arm, p_base), STATE_HOLDING,
                             "집어서 홈으로 복귀했습니다. 내려놓을 위치를 클릭하세요",
                             kind="pick"))
        else:
            self.st.markers.append(Marker((x, y), "PLACE", (0, 165, 255)))
            self.st.status = f"내려놓는 중... {pos}"
            self._submit(Job(do_place, (self.arm, p_base), STATE_IDLE,
                             "물체를 클릭하면 집습니다", clear_markers=True, kind="place"))
        return True, self.st.status, info

    # --- 지금 무엇을 하고 있는가 -------------------------------------------
    # 내부 모드(IDLE/HOLDING/BUSY)만으로는 "잡는중"과 "놓는중"을 구분할 수 없다.
    # BUSY 는 그냥 '워커가 뭔가 하는 중'이라서, 무슨 작업인지는 Job.kind 로 안다.
    STATES = {
        "waiting_pick":   "잡기 기다림",
        "picking":        "잡는중",
        "waiting_place":  "놓기 기다림",
        "placing":        "놓는중",
        "homing":         "홈 복귀중",
        "busy":           "동작중",
        "calibrating":    "캘리브레이션중",
        "not_calibrated": "미보정",
    }

    def state(self) -> str:
        if self.st.calibrating:
            return "calibrating"
        if self.st.mode == STATE_BUSY:
            return {"pick": "picking", "place": "placing",
                    "home": "homing"}.get(self.current_job or "", "busy")
        if self.st.mode == STATE_HOLDING:
            return "waiting_place"
        if not self.he.calibrated:
            # 물고 있지 않은데 보정도 안 됐으면 집기 명령이 거부된다.
            # '잡기 기다림' 이라고 하면 원격에서 계속 pick 을 던지게 된다.
            return "not_calibrated"
        return "waiting_pick"

    def snapshot(self) -> dict:
        """원격에서 상태를 물어볼 때 돌려줄 것들."""
        tcp = self.arm.get_tcp()
        st = self.state()
        return {"state": st,
                "label": self.STATES[st],
                "mode": self.st.mode,
                "busy": self.st.mode == STATE_BUSY,
                "holding": self.st.mode == STATE_HOLDING,
                "calibrating": self.st.calibrating,
                "handeye_calibrated": self.he.calibrated,
                "depth_live": self.st.depth_live,
                "view": self.st.view,
                "status": self.st.status,
                "last_error": self.last_error,
                "points": len(self.panel.records),
                "image_size": [int(self.src.intr.width), int(self.src.intr.height)],
                "tcp_mm": None if tcp is None else [round(float(v) * 1000, 1) for v in tcp],
                "arm": type(self.arm).__name__}

    def latest_images(self) -> tuple[Optional[np.ndarray], Optional[np.ndarray],
                                     Optional[np.ndarray], Optional[np.ndarray]]:
        """원격에 넘길 최신 프레임. (원본 영상, 오버레이 영상, depth[m], 추정마스크)

        루프가 프레임마다 통째로 새 배열을 만들어 넣으므로, 여기서 읽는 쪽은
        '어느 한 프레임' 을 온전히 본다. 반쯤 그려진 화면을 볼 일은 없다.
        """
        return self.last_color, self.last_frame, self.depth, self.depth_est

    def point_info(self, x: int, y: int) -> tuple[bool, str, dict]:
        """한 화소의 depth 와 로봇좌표. 팔은 움직이지 않는다.

        원격에서 어디를 집을지 고르려면 그 지점이 실제로 어디인지, 도달은
        가능한지 먼저 알아야 한다. pick 을 던져 보고 409 로 알아내는 것보다 낫다.
        """
        h, w = self.depth.shape[:2]
        info: dict = {"uv": [int(x), int(y)]}
        if not (0 <= x < w and 0 <= y < h):
            return False, f"좌표 ({x},{y}) 가 화면({w}x{h}) 밖입니다", info
        z = sample_depth(self.depth, x, y)
        info["depth_m"] = round(float(z), 4)
        info["estimated_depth"] = bool(self.depth_est[y, x])
        info["depth_live"] = bool(self.st.depth_live)     # 컬러 모드면 낡은 값이다
        if z <= 0:
            return False, "그 지점의 depth 가 없습니다", info
        p_cam = deproject(self.src.intr, x, y, z)
        info["cam_mm"] = [round(float(v) * 1000, 1) for v in p_cam]
        if self.he.calibrated:
            p_base = self.he.cam_to_base(p_cam)
            info["target_base_mm"] = [round(float(v) * 1000, 1) for v in p_base]
            ok, why = check_steps(pick_steps(p_base))
            info["pickable"] = bool(ok)
            if not ok:
                info["reason"] = why
        return True, "ok", info

    # --- 캘리브레이션 ------------------------------------------------------
    def _calib_click(self, x: int, y: int, p_cam: np.ndarray) -> None:
        n = len(self.st.calib_cam) + 1
        print(f"\n[calib] {n}번 점 - 카메라 좌표 "
              f"({p_cam[0]:+.4f}, {p_cam[1]:+.4f}, {p_cam[2]:.4f})")

        # 팔이 자기 위치를 알면 사람이 좌표를 입력할 필요가 없다.
        can_read = self.arm.get_tcp() is not None
        if can_read:
            now = self.arm.get_tcp()
            print(f"        (지금 로봇 TCP: {now[0]:+.4f}, {now[1]:+.4f}, {now[2]:+.4f})")
            print("        그리퍼 끝을 방금 클릭한 그 지점에 맞춘 뒤 Enter 를 누르세요.")
            print("        지금 옮겨도 됩니다 - 로봇 좌표는 Enter 누르는 순간에 읽습니다.")
            print("        직접 입력하려면 x y z, 건너뛰려면 s")
            prompt = "        [Enter=지금 위치 사용] > "
        else:
            print("        로봇 TCP 를 같은 물리적 지점에 대고 그 x y z [m] 를 입력 (건너뛰려면 s)")
            prompt = "        x y z > "

        try:
            raw = input(prompt).strip()
        except EOFError:
            raw = "s"

        if raw.lower() == "s":
            print("[calib] 건너뜀")
            return
        if not raw:
            # 클릭한 뒤 팔을 옮겼을 수 있으므로 '확정하는 이 시점'에 다시 읽는다.
            tcp = self.arm.get_tcp()
            if tcp is None:
                print("[calib] 입력 없음 - 건너뜀")
                return
            p_base = np.asarray(tcp, dtype=np.float64)
            print(f"        확정 시점 TCP: ({p_base[0]:+.4f}, {p_base[1]:+.4f}, {p_base[2]:+.4f})")
        else:
            try:
                p_base = np.array([float(v) for v in raw.replace(",", " ").split()],
                                  dtype=np.float64)
                if p_base.size != 3:
                    raise ValueError
            except ValueError:
                print("[calib] 입력 형식 오류 - 숫자 세 개를 공백으로 구분")
                return

        why = tcp_sanity(p_base)
        if why:
            print(f"[calib] 경고: {why}")
            print("        이 점은 버린다. 영점을 먼저 잡지 않으면 몇 점을 모아도 맞지 않는다.")
            return

        self.st.calib_cam.append(p_cam)
        self.st.calib_base.append(p_base)
        self.st.markers.append(Marker((x, y), f"C{n}", (255, 0, 255)))

        # 캘리브레이션에 실제로 쓴 점이 곧 저장할 점이다. 따로 '점추가' 를 누를
        # 필요가 없고, 저장 파일이 항상 지금의 handeye.json 을 만든 그 점들이 된다.
        self.panel.pairs.append((np.array(p_cam, float), np.array(p_base, float)))
        self.panel.records.append({
            "uv": [int(x), int(y)],
            "cam": [float(v) for v in p_cam],
            "tcp": [float(v) for v in p_base],
            "counts": self.arm.get_joint_counts(),   # 그 순간의 팔 자세
        })
        self.panel.warn_new_point()      # 지금 아니면 어느 점이 나빴는지 알 수 없다

        m = len(self.st.calib_cam)
        self.st.status = f"캘리브레이션 점 {m}개 (4개 이상 권장, 'k' 로 확정)"
        print(f"[calib] 누적 {m}점")
        if m >= 3:
            try:
                _T, rms = solve_rigid_transform(np.array(self.st.calib_cam),
                                                np.array(self.st.calib_base))
                print(f"[calib] 현재 잔차 RMS = {rms*1000:.1f}mm")
            except Exception as e:
                print(f"[calib] {e}")

    def _finish_calib(self) -> None:
        """점이 모자라면 끝내지 않는다. 여기서 캘리브레이션 모드를 빠져나가면
        모은 점이 사라져서 처음부터 다시 찍어야 한다."""
        n = len(self.st.calib_cam)
        if n < HANDEYE_MIN_POINTS:
            self.st.status = (f"점 {n}개 - {HANDEYE_MIN_POINTS}곳 이상 필요 "
                              "(캘리브레이션 계속)")
            print(f"[calib] 점 {n}개로는 풀지 않는다. {HANDEYE_MIN_POINTS}곳 이상을 "
                  "서로 다른 위치/높이에 등록할 것")
            return

        T, rms = solve_rigid_transform(np.array(self.st.calib_cam),
                                       np.array(self.st.calib_base))
        self.he = HandEye(T)
        self.he.save()
        self.panel.set_from_T(T)
        # 변환과 그 변환을 만든 점을 함께 남긴다. 점이 없으면 나중에
        # '검증'(재현성)도 '자동풀기'(다시 풀기)도 할 수 없다.
        self.panel.save_points()
        self.st.status = (f"캘리브레이션 완료 (RMS {rms*1000:.1f}mm)"
                          f" - 점 {len(self.panel.records)}개 저장됨")
        print(f"[calib] 완료, RMS={rms*1000:.1f}mm")
        print(T)
        self.panel.print_quality()
        if rms >= CALIB_Z_TOL:
            print("[calib] 잔차가 크다. 점 데이터를 의심할 것 - 위 진단을 보고 "
                  "의심되는 점을 다시 찍거나 패널의 '이상점버림' 을 쓸 것")
        self.st.calib_cam.clear()
        self.st.calib_base.clear()
        self.st.calibrating = False
        self.st.markers.clear()

    def _start_calib(self) -> None:
        self.st.calibrating = True
        self.st.calib_cam.clear()
        self.st.calib_base.clear()
        self.st.markers.clear()
        # 이번에 찍는 점만 저장되도록 이전 점은 비운다. 지난 세션의 점이 섞이면
        # '이 변환을 만든 점' 이라는 파일의 의미가 사라진다.
        self.panel.pairs.clear()
        self.panel.records.clear()
        tcp0 = self.arm.get_tcp()
        why = tcp_sanity(tcp0)
        if why:
            self.st.status = f"영점 이상: {why}"
            print(f"[calib] 경고: {why}")
            print("        지금 자세에서 팔이 말하는 위치가 물리적으로 불가능하다. "
                  "hand-eye 를 잡기 전에 팔 영점부터 고칠 것.")
        auto = tcp0 is not None
        self.st.status = ("캘리브레이션: 그리퍼를 점에 대고 그 점을 클릭"
                          if auto else "캘리브레이션: 점 클릭 -> 콘솔에 로봇 xyz 입력")
        print("\n[calib] 시작. 탁상 위에서 실제로 집을 수 있는 위치를 "
              f"{HANDEYE_MIN_POINTS}곳 이상 등록한다.")
        print("        되도록 넓게 퍼뜨리고 높이도 섞을 것 "
              "(전부 같은 평면이면 변환이 불안정하다).")
        if auto:
            print("        't' 로 토크를 끄면 팔을 손으로 움직일 수 있다.")
            print("        절차: 그리퍼 끝을 목표 지점에 댄다 -> 화면에서 그 지점을 클릭")
            print("              -> 콘솔에서 Enter (로봇 좌표는 자동 실측)")

    # --- 메인 루프 ---------------------------------------------------------
    def run(self) -> None:
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(WINDOW, self.on_mouse)
        self.worker.start()
        print(__doc__)
        if self.depth_aligned:
            print("[align] depth 가 컬러 좌표계로 정렬되어 있다. 화면(컬러)에서 클릭한 "
                  "픽셀이 곧 depth 화소다 - ALIGN 호모그래피가 필요 없다.")
            print("        주의: 좌표계와 내부파라미터가 IR 기준과 다르다. "
                  "IR 로 잡아 둔 handeye.json 은 다시 잡아야 한다.")

        # 시작하자마자 홈 자세로 맞춘다. 팔이 어디에 있었는지 모르는 채로 첫
        # 클릭을 받으면 그 동작이 어떤 경로로 갈지 예측할 수 없다. send_joints 가
        # 8도씩 나눠 보내므로 여기서 갑자기 튀지 않는다.
        self._submit(Job(RobotArm.home, (self.arm,), STATE_IDLE,
                         "홈 자세 - 물체를 클릭하면 집습니다", kind="home"))

        if self.calib_on_start:
            self._start_calib()

        try:
            while self.running:
                legacy_color = self.st.view == "color" and not self.depth_aligned
                rgb_now = (self.color_cam.latest((self.src.intr.width,
                                                   self.src.intr.height))
                           if legacy_color else None)
                if legacy_color and self.color_cam.opened:
                    # 장치가 열린 뒤에 대역을 넘긴다. 여는 데 몇 초가 걸리는데 그
                    # 동안까지 재우면 화면이 굳어서 '멈춘 것' 처럼 보이고, 반대로
                    # 안 재우면 depth 가 대역을 다 먹어 컬러 프레임이 안 온다.
                    self._pause_src()

                if legacy_color and self.last_color is None and self._src_paused:
                    # 첫 프레임도 받기 전에 컬러로 바꾼 경우. 스트림이 멈춰 있어서
                    # 지금 읽으면 예외가 난다. 빈 화면으로 버틴다.
                    color = np.zeros((self.src.intr.height, self.src.intr.width, 3),
                                     np.uint8)
                    depth = self.depth
                    self.st.depth_live = False
                elif legacy_color and self._src_paused:
                    # 컬러 모드에서는 depth 를 아예 읽지 않는다.
                    # Astra 는 depth/IR/컬러를 USB2.0 한 대역으로 보낸다. 컬러가
                    # 도는 동안 depth read_frame() 이 오지 않는 프레임을 기다리며
                    # 통째로 막히고, 그게 화면이 멈추던 원인이다. 컬러 모드에서는
                    # 어차피 클릭도 막혀 있으니 마지막 depth 를 그대로 들고 있는다.
                    color, depth = self.last_color, self.depth
                    self.st.depth_live = False
                else:
                    try:
                        color, depth = self.src.read()
                    except Exception as e:
                        print(f"[source] 읽기 실패: {e}")
                        break
                    # 구멍을 주변 유효값으로 메운다. 물체 가장자리처럼 정작 집고
                    # 싶은 곳이 비어 있는 경우가 많아서, 메우지 않으면 클릭이 거부된다.
                    depth, self.depth_est = fill_depth_holes(depth)
                    self.depth = depth
                    self.last_color = color
                    self.st.depth_live = True

                lo = hi = None
                if self.st.view == "depth":
                    dvis, lo, hi = colorize_depth(depth)
                    base = cv2.addWeighted(dvis, 0.75, color, 0.25, 0)
                elif legacy_color:
                    # 절대 기다리지 않는다. 프레임이 아직 없으면 센서 영상을 쓴다.
                    if self.color_cam.failed:
                        self.st.status = self.color_cam.note or "일반 카메라를 쓸 수 없습니다"
                        self.color_cam.stop()
                        self._resume_src()
                        self.st.view = "sensor"
                        base = color
                    else:
                        base = color if rgb_now is None else rgb_now
                    self.st.view_note = self.color_cam.note
                else:
                    base = color   # sensor, 또는 정렬 모드의 순수 컬러(color) 뷰

                cam = self.active_color_cam()
                if self.depth_aligned and cam is not None:
                    # 컬러가 아직 안 나올 때 검은 화면만 보여주면 왜 그런지 모른다.
                    self.st.view_note = cam.note
                self.st.color_display = cam is not None
                self.st.cam_index = None if cam is None else cam.index
                frame = base.copy()
                reach_note = ""
                # 도달 가능 영역은 depth 화소에 그리는 것이라 컬러 화면에는 못 얹는다.
                if self.st.show_reach and self.he.calibrated and self.st.view != "color":
                    ok_m = reachable_mask(self.src.intr, depth, self.he)
                    n_ok, n_bad = draw_reachability(frame, depth, ok_m, self.depth_est)
                    tot = n_ok + n_bad
                    n_est = int(self.depth_est.sum())
                    reach_note = (f"집기 가능 {100*n_ok/tot:.0f}%" if tot else "집기 가능 영역 없음")
                    if n_est:
                        reach_note += f"  (추정 depth {100*n_est/self.depth_est.size:.0f}%)"
                frame = draw_overlay(frame, self.st, depth, self.src.intr, self.he,
                                     self.depth_est)
                self.panel.draw_points(frame, self.src.intr)
                if reach_note:
                    cv2.putText(frame, reach_note, (10, 74), cv2.FONT_HERSHEY_SIMPLEX,
                                0.45, (120, 255, 120), 1, cv2.LINE_AA)
                if lo is not None:
                    draw_depth_scale(frame, lo, hi)
                self.last_frame = frame
                cv2.imshow(WINDOW, np.hstack([frame, self.panel.draw(frame.shape[0])]))

                key = cv2.waitKey(1) & 0xFF
                if key == 255:
                    continue
                if key in (ord('q'), 27):
                    break
                if key == ord('d'):
                    self.cycle_view()
                elif key == ord('w'):
                    self.st.show_reach = not self.st.show_reach
                    self.st.status = ("도달 가능 영역 표시 ON" if self.st.show_reach
                                      else "도달 가능 영역 표시 OFF")
                elif key == ord('r'):
                    self.st.mode = STATE_IDLE
                    self.st.markers.clear()
                    self.st.status = "리셋됨 - 물체를 클릭하면 집습니다"
                elif key == ord('k'):
                    self._finish_calib() if self.st.calibrating else self._start_calib()
                elif key == ord('t'):
                    try:
                        self.torque_on = not getattr(self, "torque_on", True)
                        self.arm.set_torque(self.torque_on)
                        self.st.status = (f"토크 {'ON' if self.torque_on else 'OFF'}"
                                          + ("" if self.torque_on else " - 손으로 움직일 수 있음"))
                    except Exception as e:
                        self.st.status = f"토크 전환 불가: {e}"
                elif key == ord('s'):
                    ts = time.strftime("%Y%m%d_%H%M%S")
                    cv2.imwrite(f"frame_{ts}.png", color)
                    np.save(f"depth_{ts}.npy", depth)
                    self.st.status = f"저장: frame_{ts}.png / depth_{ts}.npy"
                elif self.st.mode != STATE_BUSY:
                    if key == ord('h'):
                        self._submit(Job(RobotArm.home, (self.arm,), done_msg="홈 자세", kind="home"))
                    elif key == ord('o'):
                        self._submit(Job(RobotArm.open_gripper, (self.arm,),
                                         done_msg="그리퍼 열림", kind="gripper"))
                    elif key == ord('c'):
                        self._submit(Job(RobotArm.close_gripper, (self.arm,),
                                         done_msg="그리퍼 닫힘", kind="gripper"))
        finally:
            # 팔부터 정리한다. 카메라 닫기가 실패해도 토크는 반드시 꺼져야 하므로
            # 안전에 직결된 것을 먼저, 각각 독립적으로 감싼다.
            self.running = False
            for what, fn in (("팔", self.arm.disconnect),
                             ("컬러 카메라", self.color_cam.close),
                             ("카메라", self.src.close),
                             ("창", cv2.destroyAllWindows)):
                try:
                    fn()
                except Exception as e:
                    print(f"[app] {what} 정리 실패: {e.__class__.__name__}: {e}")
            print("[app] 종료")


# ---------------------------------------------------------------------------
# 8. REST API - 원격에서 화면 좌표로 pick / place
# ---------------------------------------------------------------------------
#
# 마우스 클릭과 같은 개념이다. 화면 픽셀 (x, y) 를 주면 그 지점의 depth 로
# 로봇좌표를 만들어 집거나 놓는다. 판단과 실행은 App.command() 한 곳에서만
# 하므로 마우스와 REST 가 어긋날 일이 없다.
#
#   GET  /get_state               지금 상태 (잡기 기다림 / 잡는중 / 놓기 기다림 / ...)
#   GET  /pick?x=320&y=240        POST /pick   {"x": 320, "y": 240}
#   GET  /place?x=420&y=300       POST /place  {"x": 420, "y": 300}
#   GET  /get_rgb                 카메라 영상 (기본 jpg, ?format=png, ?overlay=1)
#   GET  /get_depth               depth 16bit PNG [mm] (?format=color 는 보기용)
#   GET  /get_depth?x=&y=         그 화소의 depth/카메라/로봇 좌표 (JSON, 팔 안 움직임)
#
# 좌표는 창 왼쪽의 카메라 영상 기준이다(오른쪽 패널 제외). 응답은 '접수' 까지고
# 동작 완료는 /status 로 확인한다 - 팔이 움직이는 몇 초 동안 HTTP 를 붙잡고
# 있으면 클라이언트가 타임아웃으로 재시도해 같은 동작을 두 번 걸게 된다.
#
# 기본은 127.0.0.1 만 듣는다. HTTP 한 번에 팔이 움직이므로 네트워크에 여는 것은
# 의도한 선택이어야 한다.
#   다른 PC 에서 부르려면:  $env:PICK_API_HOST='0.0.0.0'
#   끄려면:                $env:PICK_API_PORT='0'

API_HOST = os.environ.get("PICK_API_HOST", "127.0.0.1")
API_PORT = int(os.environ.get("PICK_API_PORT", "8765"))


class _ApiHandler(BaseHTTPRequestHandler):
    app: Optional["App"] = None
    server_version = "SO101PickApi/1"

    def log_message(self, fmt, *args) -> None:
        pass                     # 기본 액세스 로그는 콘솔을 덮는다. 필요한 건 직접 찍는다

    def _send(self, code: int, obj: dict) -> None:
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_bytes(self, code: int, ctype: str, data: bytes,
                    extra: Optional[dict] = None) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        for k, v in (extra or {}).items():
            self.send_header(k, str(v))
        self.end_headers()
        self.wfile.write(data)

    def _send_image(self, img, fmt: str, extra: Optional[dict] = None) -> None:
        ext = ".png" if fmt == "png" else ".jpg"
        ok, buf = cv2.imencode(ext, img)
        if not ok:
            self._send(500, {"ok": False, "error": "이미지 인코딩 실패"})
            return
        ctype = "image/png" if ext == ".png" else "image/jpeg"
        self._send_bytes(200, ctype, buf.tobytes(), extra)

    def do_GET(self) -> None:
        u = urlparse(self.path)
        self._route(u.path, {k: v[0] for k, v in parse_qs(u.query).items()})

    def do_POST(self) -> None:
        n = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(n) if n > 0 else b""
        try:
            params = json.loads(raw.decode("utf-8")) if raw else {}
        except (ValueError, UnicodeDecodeError):
            self._send(400, {"ok": False, "error": "본문이 JSON 이 아닙니다"})
            return
        if not isinstance(params, dict):
            params = {}
        u = urlparse(self.path)
        params.update({k: v[0] for k, v in parse_qs(u.query).items()})
        self._route(u.path, params)

    def _route(self, path: str, params: dict) -> None:
        app = _ApiHandler.app
        if app is None:
            self._send(503, {"ok": False, "error": "앱이 준비되지 않았습니다"})
            return
        path = path.rstrip("/") or "/"

        if path in ("/get_state", "/state", "/status"):
            self._send(200, {"ok": True, **app.snapshot(),
                             "states": app.STATES})
            return

        if path in ("/get_rgb", "/rgb"):
            color, frame, _, _ = app.latest_images()
            img = frame if params.get("overlay") in ("1", "true", "yes") else color
            if img is None:
                self._send(503, {"ok": False, "error": "아직 프레임이 없습니다"})
                return
            self._send_image(img, params.get("format", "jpg"),
                             {"X-Image-Size": f"{img.shape[1]}x{img.shape[0]}"})
            return

        if path in ("/get_depth", "/depth"):
            # x,y 를 주면 그 화소의 depth 와 로봇좌표를 JSON 으로 (팔은 안 움직인다)
            if "x" in params and "y" in params:
                try:
                    x = int(round(float(params["x"])))
                    y = int(round(float(params["y"])))
                except (TypeError, ValueError):
                    self._send(400, {"ok": False, "error": "x, y 를 숫자로 주세요"})
                    return
                ok, msg, info = app.point_info(x, y)
                self._send(200 if ok else 409, {"ok": ok, "message": msg, **info})
                return

            _, _, depth, est = app.latest_images()
            if depth is None or not depth.size:
                self._send(503, {"ok": False, "error": "아직 depth 가 없습니다"})
                return
            hdr = {"X-Image-Size": f"{depth.shape[1]}x{depth.shape[0]}",
                   "X-Depth-Unit": "mm", "X-Depth-Filled-Px": int(est.sum())}
            if params.get("format") == "color":
                # 사람이 보는 용도. 값을 읽을 수는 없다.
                vis, lo, hi = colorize_depth(depth)
                hdr.update({"X-Depth-Min-Mm": round(lo * 1000, 1),
                            "X-Depth-Max-Mm": round(hi * 1000, 1)})
                self._send_image(vis, "jpg", hdr)
                return
            # 기본은 16bit PNG(mm). 무손실이라 받은 쪽에서 그대로 값을 쓴다.
            mm = np.clip(depth * 1000.0, 0, 65535).astype(np.uint16)
            ok, buf = cv2.imencode(".png", mm)
            if not ok:
                self._send(500, {"ok": False, "error": "depth 인코딩 실패"})
                return
            self._send_bytes(200, "image/png", buf.tobytes(), hdr)
            return

        if path in ("/pick", "/place"):
            try:
                x = int(round(float(params["x"])))
                y = int(round(float(params["y"])))
            except (KeyError, TypeError, ValueError):
                self._send(400, {"ok": False,
                                 "error": "x, y 를 숫자로 주세요 (예: /pick?x=320&y=240)"})
                return
            action = path[1:]
            ok, msg, info = app.command(action, x, y)
            print(f"[api] {action}({x},{y}) -> {'접수' if ok else '거부'}: {msg}")
            self._send(200 if ok else 409, {"ok": ok, "message": msg, **info})
            return

        if path == "/":
            self._send(200, {"ok": True, "endpoints": [
                "GET /get_state  (= /status) 지금 무엇을 하는 중인지",
                "GET|POST /pick   x,y (화면 픽셀)",
                "GET|POST /place  x,y (화면 픽셀)",
                "GET /get_rgb    [?format=png] [?overlay=1]",
                "GET /get_depth  [?format=color] - 기본은 16bit PNG(mm)",
                "GET /get_depth?x=&y= - 그 화소의 depth/로봇좌표 (JSON)"]})
            return

        self._send(404, {"ok": False, "error": f"없는 경로: {path}"})


def start_api(app: "App") -> Optional[ThreadingHTTPServer]:
    """REST 서버를 데몬 스레드로 띄운다. 못 열어도 프로그램은 계속 간다."""
    if API_PORT <= 0:
        print("[api] PICK_API_PORT=0 - REST API 를 켜지 않는다")
        return None
    _ApiHandler.app = app
    try:
        srv = ThreadingHTTPServer((API_HOST, API_PORT), _ApiHandler)
    except OSError as e:
        print(f"[api] 포트를 열지 못했다 ({e}) - REST 없이 계속한다")
        return None
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"[api] REST 대기: http://{API_HOST}:{API_PORT}  "
          f"(/get_state /pick /place /get_rgb /get_depth)")
    if API_HOST in ("127.0.0.1", "localhost"):
        print("      다른 PC 에서 부르려면 $env:PICK_API_HOST='0.0.0.0' 로 실행할 것")
    return srv

# ---------------------------------------------------------------------------
# 진입점
# ---------------------------------------------------------------------------

def main() -> None:
    if "--tcp" in sys.argv:            # 팔 TCP 만 확인하는 모드 (카메라를 열지 않는다)
        monitor_tcp(os.environ.get("SO101_PORT") or find_so101_port() or "COM18")
        return

    source = open_depth_source()
    arm = setup_arm()                  # 1/2: 팔 캘리브레이션 확인/실행
    calib_now = want_handeye_calib(arm)   # 2/2: 카메라-로봇 캘리브레이션 확인/실행
    handeye = HandEye.load()
    app = App(source, arm, handeye)
    app.calib_on_start = calib_now
    api = start_api(app)
    try:
        app.run()
    finally:
        if api is not None:
            api.shutdown()
            print("[api] 종료")


if __name__ == "__main__":
    main()
