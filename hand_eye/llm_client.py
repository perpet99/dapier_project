"""
llm_client.py - Gemini Robotics-ER 이 test4.py 의 REST API 를 스킬로 쓰는 클라이언트.

무엇을 하나:
  1) /get_rgb 로 지금 카메라가 보는 화면을 받아 사용자에게 보여주고(웹 + 파일),
     모델에게도 보내 "무엇이 보이고 무엇을 할 수 있는지" 를 먼저 말하게 한다.
  2) 사용자가 시키면, 모델이 아래 스킬을 하나씩 호출하고 클라이언트가 REST 로
     실행한 뒤 결과를 다시 모델에 넣는다. 끝날 때까지 반복.

스킬 (= test4.py 의 REST API)
  get_state              GET /get_state          지금 무엇을 하는 중인지
  get_rgb                GET /get_rgb            화면을 다시 본다
  get_depth_at(y, x)     GET /get_depth?x=&y=    그 점의 깊이/로봇좌표/집기 가능 여부
  pick(y, x)             GET /pick?x=&y=         그 점의 물체를 집는다
  place(y, x)            GET /place?x=&y=        그 점에 내려놓는다
  wait(seconds)          (/get_state 폴링)       동작이 끝날 때까지 기다린다
  say(message)           -                       사용자에게 한마디
  done(message)          -                       일을 마쳤다고 알리고 종료

이동 로봇(car2) 스킬 (= ros2_slam/test/map_web.py 의 REST API, 문서 map_web_api.md)
  base_state             GET /api/state, /api/drive   위치·방향, 라벨 목록, Nav2/근접 상태
  goto_label(label)      POST /api/nav/goto           라벨 위치로 Nav2 자율 주행 (끝날 때까지 기다림)
  save_label(label)      POST /api/labels             지금 위치를 라벨로 저장
  base_move(meters)      POST /api/drive (반복)       직진/후진 (±1 m, 앞뒤 라이다 장애물이면 정지)
  base_turn(degrees)     POST /api/drive (반복)       제자리 회전 (±180°, + = 왼쪽)
  base_stop              /api/drive/stop + 취소        모든 이동 정지
  relocalize(global)     POST /api/relocalize         라이다로 지도 위 위치 다시 잡기
  wall_approach(gap)     POST /api/wall/*             정면 벽과 수직으로 맞추고 gap(m) 앞까지 전진
  depth_approach         POST /api/approach/*         깊이 카메라 근접선까지 전진
  주소는 기본으로 --api 와 같은 호스트의 8081 포트 (--map-api 로 지정).
  map_web 이 없으면(연결 실패 또는 --no-base) 이 스킬들은 빠진다.

좌표 규약은 이 프로젝트의 다른 VLM 코드(test3.py)와 같다 - **[y, x] 순서로
0~1000 정규화**. 클라이언트가 실제 픽셀로 바꿔서 REST 에 넘긴다. 모델이 픽셀
크기를 몰라도 되고, 화면 크기가 바뀌어도 프롬프트를 고칠 필요가 없다.

지시는 **웹으로 받는다**. 실행하면 http://127.0.0.1:8770 이 열리고, 거기서
현재 화면 / 진행 로그 / 로봇 상태를 보면서 명령을 넣는다. 터미널로 받고 싶으면
--cli 를 준다.

pick/place 는 팔이 실제로 움직인다. 묻지 않고 바로 실행하고, **실패했을 때만**
다시 할지 묻는다(--auto 를 주면 그것도 묻지 않고 그냥 넘어간다).

사용:
  hand_eye/.env 에 GEMINI_API_KEY=... 한 줄 (hand_eye/.env.example 참고)
  또는 $env:GEMINI_API_KEY='...'
  python hand_eye/llm_client.py                      # 웹 UI 로 지시
  python hand_eye/llm_client.py "주황색 통을 상자에 넣어"   # 첫 지시만 미리 주기
  python hand_eye/llm_client.py --cli                # 터미널로 지시
  python hand_eye/llm_client.py --auto "책상 위 물건 하나만 집어봐"   # 재시도도 안 묻기
  python hand_eye/llm_client.py --api http://192.168.0.10:8765 "..."
  python hand_eye/llm_client.py --map-api http://192.168.0.31:8081 "책상으로 가서 컵을 집어"
  python hand_eye/llm_client.py --no-base            # 이동 로봇 스킬 없이 팔만
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import os
import queue
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import List, Optional

import cv2
import numpy as np
from google import genai
from pydantic import BaseModel, Field

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass

DEFAULT_API = "http://192.168.0.31:8765"
MAP_WEB_PORT = 8081       # ros2_slam/test/map_web.py (이동 로봇). 주소는 --api 의 호스트를 따른다
BASE_MAX_MOVE = 1.0       # base_move 한 번의 최대 거리 (m)
BASE_SPEED = 0.08         # base_move 속도 (m/s)
BASE_TURN_SPEED = 0.4     # base_turn 각속도 (rad/s)
BASE_CLEAR = 0.10         # base_move: 차체 앞/뒤에서 이 거리 안에 라이다 점이 있으면 정지 (m)
NAV_TIMEOUT = 240.0       # goto_label 최대 대기 (s)
MODEL_ID = os.environ.get("GEMINI_MODEL", "gemini-robotics-er-2-preview")
VIEW_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "llm_view.jpg")
SKILLS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "llm_skills.json")  # 웹에서 끈 스킬
SEND_WIDTH = 800          # 모델에 보낼 이미지 폭. 토큰을 아끼되 점을 찍을 만큼은 크게
MAX_STEPS = 12            # 한 지시당 스킬 호출 상한 (무한루프 방지)
MAX_RETRY = 3             # pick/place 가 실패했을 때 다시 해볼 최대 횟수
WEB_PORT = 8770           # 웹 입력 UI 포트 (test4 의 REST 8765 와 다른 포트다)
ASK_TIMEOUT = 300.0       # 재시도 질문에 답이 없으면 이 시간 뒤 '아니오' 로 본다


# ---------------------------------------------------------------------------
# 1. REST 스킬
# ---------------------------------------------------------------------------

class Api:
    """test4.py 의 REST API. 오류도 예외 대신 본문으로 돌려준다 - 거부 사유가 정보다."""

    def __init__(self, base: str):
        self.base = base.rstrip("/")

    def _get(self, path: str, params: Optional[dict] = None) -> tuple[int, dict, bytes]:
        q = f"?{urllib.parse.urlencode(params)}" if params else ""
        try:
            with urllib.request.urlopen(f"{self.base}{path}{q}", timeout=10) as r:
                return r.status, dict(r.headers), r.read()
        except urllib.error.HTTPError as e:
            return e.code, dict(e.headers), e.read()

    def json(self, path: str, params: Optional[dict] = None) -> dict:
        code, _, body = self._get(path, params)
        try:
            out = json.loads(body.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            out = {"ok": False, "error": body[:200].decode("utf-8", "replace")}
        out["http"] = code
        return out

    def rgb(self) -> Optional[np.ndarray]:
        code, _, body = self._get("/get_rgb", {"format": "png"})
        if code != 200:
            return None
        return cv2.imdecode(np.frombuffer(body, np.uint8), cv2.IMREAD_COLOR)


class MapApi:
    """이동 로봇(car2)의 map_web.py REST API. 문서: ros2_slam/test/map_web_api.md

    동작 스킬은 '끝날 때까지 기다렸다가 결과 요약' 을 돌려준다. 모델은 한 번에
    스킬 하나만 부르므로, 요청만 넣고 바로 돌아오면 진행을 따라갈 수 없다.
    """

    def __init__(self, base: str, log=print, status=lambda s: None):
        self.base = base.rstrip("/")
        self.log, self.status = log, status

    # --- HTTP -------------------------------------------------------------
    def call(self, path: str, body: Optional[dict] = None, timeout: float = 10.0) -> dict:
        """GET (body None) 또는 POST JSON. 오류도 예외 대신 본문({'error':...})으로."""
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(f"{self.base}{path}", data=data,
                                     headers={"Content-Type": "application/json"} if data else {})
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                code, raw = r.status, r.read()
        except urllib.error.HTTPError as e:
            code, raw = e.code, e.read()
        except (urllib.error.URLError, OSError) as e:
            return {"ok": False, "error": f"map_web 연결 실패: {e}", "http": 0}
        try:
            out = json.loads(raw.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            out = {"error": raw[:200].decode("utf-8", "replace")}
        if out is None or not isinstance(out, dict):
            out = {"value": out}
        out["http"] = code
        return out

    def alive(self) -> bool:
        st = self.call("/api/drive")
        return st.get("http") == 200

    # --- 상태 -------------------------------------------------------------
    def state(self) -> dict:
        return self.call("/api/state")

    @staticmethod
    def _deg(rad) -> Optional[float]:
        return None if rad is None else round(math.degrees(rad), 1)

    def summary(self) -> dict:
        """모델에게 줄 짧은 상태 (스캔 점 같은 큰 배열은 뺀다)."""
        st, drv = self.state(), self.call("/api/drive")
        if st.get("http") != 200:
            return {"ok": False, "error": st.get("error", "상태를 못 읽음")}
        p, nav, wall = st.get("pose"), st.get("nav") or {}, st.get("wall") or {}
        w = wall.get("wall")
        return {
            "ok": True,
            "frame": st.get("frame"),          # map = 지도 위 위치, odom = 지도 없음(주행거리)
            "pose": None if not p else {"x": p["x"], "y": p["y"], "yaw_deg": self._deg(p["yaw"])},
            "driver_connected": drv.get("driver_connected"),
            "nav": {"state": nav.get("state"), "server": nav.get("server"),
                    "target": (nav.get("target") or {}).get("name"), "message": nav.get("message")},
            "labels": [{"name": lb["name"], "x": lb["x"], "y": lb["y"], "yaw_deg": self._deg(lb["yaw"]),
                        "reachable": lb.get("cost") is None or lb.get("cost") < 99,
                        "dist_m": (round(math.hypot(lb["x"] - p["x"], lb["y"] - p["y"]), 2)
                                   if p and st.get("frame") == "map" else None)}
                       for lb in st.get("labels", [])],
            "front_clear_m": wall.get("corridor"),   # 차체 폭 안 정면 가장 가까운 점 (로봇 중심 기준)
            "wall_ahead": None if not w else {"dist_m": w["dist"], "angle_err_deg": w["err_deg"]},
        }

    # --- Go to ------------------------------------------------------------
    def goto_label(self, name: str) -> dict:
        labels = self.call("/api/labels").get("labels", [])
        exact = [lb for lb in labels if lb["name"] == name]
        part = exact or [lb for lb in labels if name and name.lower() in lb["name"].lower()]
        if len(part) != 1:
            return {"ok": False, "message": ("그런 라벨이 없다" if not part else "여러 라벨이 맞는다")
                    + f" - 있는 라벨: {[lb['name'] for lb in labels]}"}
        lb = part[0]
        r = self.call("/api/nav/goto", {"id": lb["id"]})
        if r.get("http") != 200:
            return {"ok": False, "message": r.get("error", f"HTTP {r.get('http')}")}
        self.log(f"    >> goto '{lb['name']}' (x {lb['x']:.2f}, y {lb['y']:.2f})")
        t0, nav = time.time(), {}
        while time.time() - t0 < NAV_TIMEOUT:
            time.sleep(1.0)
            nav = self.state().get("nav") or {}
            if nav.get("state") not in ("backing", "sending", "active", "canceling"):
                break
            d = nav.get("distance")
            self.status(f"'{lb['name']}' 로 이동 중" + (f" - 남은 거리 {d:.1f} m" if d is not None else ""))
        else:
            self.call("/api/nav/cancel", {})
            return {"ok": False, "message": f"{NAV_TIMEOUT:.0f}초 안에 도착하지 못해 취소했다"}
        out = {"ok": nav.get("state") == "succeeded", "state": nav.get("state"),
               "message": nav.get("message"), "note": nav.get("note")}
        p = self.state().get("pose")
        if p:
            out["error_m"] = round(math.hypot(p["x"] - lb["x"], p["y"] - lb["y"]), 3)
        return out

    def save_label(self, name: str) -> dict:
        r = self.call("/api/labels", {"name": name})
        if r.get("http") != 200:
            return {"ok": False, "message": r.get("error", f"HTTP {r.get('http')}")}
        return {"ok": True, "label": {k: r.get(k) for k in ("name", "x", "y")},
                "yaw_deg": self._deg(r.get("yaw"))}

    # --- 직접 이동 (0.5초 유효한 /api/drive 를 0.2초마다 다시 보낸다) -------
    def _rear_clear(self, st: dict) -> Optional[float]:
        """스캔 점(지도/odom 좌표)을 로봇 좌표로 돌려 차체 뒤 가장 가까운 거리."""
        p = st.get("pose")
        if not p or not st.get("scan"):
            return None
        back = (st.get("wall") or {}).get("robot_front", 0.16)   # 뒤 길이도 앞과 같다고 본다
        half = st.get("robot_radius", 0.26)
        c, s_ = math.cos(-p["yaw"]), math.sin(-p["yaw"])
        best = None
        for wx, wy in st["scan"]:
            dx, dy = wx - p["x"], wy - p["y"]
            x, y = dx * c - dy * s_, dx * s_ + dy * c
            if x < -back and abs(y) < half:
                best = -x if best is None else min(best, -x)
        return best

    def _check_ready(self) -> Optional[dict]:
        st = self.state()
        if st.get("http") != 200 or not st.get("pose"):
            return {"ok": False, "message": "로봇 위치(TF)를 모른다 - 드라이버가 꺼져 있다"}
        if (st.get("nav") or {}).get("state") in ("backing", "sending", "active", "canceling"):
            return {"ok": False, "message": "Nav2 이동 중이다 - 끝나거나 base_stop 후에 하라"}
        if not self.call("/api/drive").get("driver_connected"):
            return {"ok": False, "message": "차량 드라이버가 /cmd_vel 을 받지 않는다"}
        return None

    def move(self, meters: float) -> dict:
        meters = max(-BASE_MAX_MOVE, min(BASE_MAX_MOVE, float(meters)))
        bad = self._check_ready()
        if bad:
            return bad
        st = self.state()
        p0, front = st["pose"], (st.get("wall") or {}).get("robot_front", 0.16)
        sign, moved, why, t0 = (1 if meters >= 0 else -1), 0.0, "", time.time()
        self.log(f"    >> base_move {meters:+.2f} m")
        try:
            while True:
                st = self.state()
                p = st.get("pose")
                if not p:
                    why = "위치(TF)를 잃었다"
                    break
                moved = math.hypot(p["x"] - p0["x"], p["y"] - p0["y"])
                if moved >= abs(meters) - 0.01:
                    break
                if sign > 0:
                    clear = (st.get("wall") or {}).get("corridor")
                    if clear is not None and clear < front + BASE_CLEAR:
                        why = f"앞 {clear:.2f} m(중심 기준)에 장애물"
                        break
                else:
                    clear = self._rear_clear(st)
                    if clear is not None and clear < front + BASE_CLEAR:
                        why = f"뒤 {clear:.2f} m(중심 기준)에 장애물"
                        break
                if time.time() - t0 > abs(meters) / BASE_SPEED * 2 + 5:
                    why = "시간 초과 (바퀴가 안 움직인다?)"
                    break
                remain = abs(meters) - moved
                self.call("/api/drive", {"linear": sign * max(0.03, min(BASE_SPEED, remain * 1.5)),
                                         "angular": 0.0})
                time.sleep(0.2)
        finally:
            self.call("/api/drive/stop", {})
        return {"ok": not why, "moved_m": round(sign * moved, 3), "message": why or "완료"}

    def turn(self, degrees: float) -> dict:
        degrees = max(-180.0, min(180.0, float(degrees)))
        bad = self._check_ready()
        if bad:
            return bad
        target = math.radians(degrees)
        last = self.state()["pose"]["yaw"]
        turned, why, t0 = 0.0, "", time.time()
        self.log(f"    >> base_turn {degrees:+.0f}°")
        try:
            while True:
                p = self.state().get("pose")
                if not p:
                    why = "위치(TF)를 잃었다"
                    break
                d = p["yaw"] - last
                turned += math.atan2(math.sin(d), math.cos(d))      # unwrap
                last = p["yaw"]
                remain = target - turned
                if abs(remain) < math.radians(2):
                    break
                if time.time() - t0 > abs(target) / BASE_TURN_SPEED * 2 + 5:
                    why = "시간 초과 (바퀴가 안 움직인다?)"
                    break
                w = max(0.15, min(BASE_TURN_SPEED, abs(remain) * 1.2))
                self.call("/api/drive", {"linear": 0.0, "angular": math.copysign(w, remain)})
                time.sleep(0.2)
        finally:
            self.call("/api/drive/stop", {})
        return {"ok": not why, "turned_deg": round(math.degrees(turned), 1), "message": why or "완료"}

    def stop(self) -> dict:
        self.call("/api/drive/stop", {})
        self.call("/api/nav/cancel", {})
        self.call("/api/approach/stop", {})
        self.call("/api/wall/stop", {})
        return {"ok": True, "message": "정지"}

    # --- 위치 다시 잡기 / 근접 이동 -------------------------------------------
    def relocalize(self, global_search: bool) -> dict:
        r = self.call("/api/relocalize", {"global": bool(global_search)})
        if r.get("http") != 200:
            return {"ok": False, "message": r.get("error", f"HTTP {r.get('http')}")}
        t0, rl = time.time(), {}
        time.sleep(0.5)
        while time.time() - t0 < 40:
            rl = self.state().get("reloc") or {}
            if rl.get("state") in ("done", "failed"):
                break
            self.status("위치 다시 잡는 중")
            time.sleep(1.0)
        res = rl.get("result") or {}
        return {"ok": rl.get("state") == "done", "message": rl.get("message"),
                "match": res.get("match"), "shift_m": res.get("shift"), "ambiguous": res.get("ambiguous")}

    def _approach(self, kind: str, timeout: float) -> dict:
        r = self.call(f"/api/{kind}/start", {})
        if r.get("http") != 200:
            return {"ok": False, "message": r.get("error", f"HTTP {r.get('http')}")}
        t0, a = time.time(), r
        while a.get("state") == "running" and time.time() - t0 < timeout:
            time.sleep(0.5)
            a = self.call(f"/api/{kind}")
            self.status(f"{kind} 근접 이동 중 - {a.get('message', '')}")
        if a.get("state") == "running":
            self.call(f"/api/{kind}/stop", {})
            return {"ok": False, "message": "시간 초과로 정지했다"}
        return {"ok": a.get("state") == "done", "state": a.get("state"), "message": a.get("message"),
                "moved_m": a.get("moved")}

    def wall_approach(self, gap: Optional[float]) -> dict:
        if gap is not None:
            c = self.call("/api/wall/config", {"gap": float(gap)})
            if c.get("http") != 200:
                return {"ok": False, "message": c.get("error", f"HTTP {c.get('http')}")}
        return self._approach("wall", 120.0)

    def depth_approach(self) -> dict:
        return self._approach("approach", 90.0)


BASE_SKILLS = ("base_state", "goto_label", "save_label", "base_move", "base_turn", "base_stop",
               "relocalize", "wall_approach", "depth_approach")
BASE_MOVING = ("goto_label", "base_move", "base_turn", "wall_approach", "depth_approach")

# 웹 UI 의 '등록된 스킬' 표 (이름, 인자, REST, 설명)
ARM_SKILL_LIST = [
    ("get_state", "", "GET /get_state", "지금 무엇을 하는 중인지"),
    ("get_rgb", "", "GET /get_rgb", "화면을 다시 본다"),
    ("get_depth_at", "point", "GET /get_depth", "그 점의 깊이/로봇좌표/집기 가능 여부"),
    ("pick", "point", "GET /pick", "그 점의 물체를 집는다"),
    ("place", "point", "GET /place", "그 점에 내려놓는다"),
    ("wait", "seconds", "/get_state 폴링", "동작이 끝날 때까지 기다린다"),
    ("say", "message", "-", "사용자에게 한마디"),
    ("done", "message", "-", "일을 마쳤다고 알리고 종료"),
]
BASE_SKILL_LIST = [
    ("base_state", "", "GET /api/state", "위치·방향, 라벨 목록, Nav2/근접 상태"),
    ("goto_label", "label", "POST /api/nav/goto", "라벨 위치로 Nav2 자율 주행"),
    ("save_label", "label", "POST /api/labels", "지금 위치를 라벨로 저장"),
    ("base_move", "meters", "POST /api/drive", "직진/후진 (±1 m, 장애물이면 정지)"),
    ("base_turn", "degrees", "POST /api/drive", "제자리 회전 (±180°, + = 왼쪽)"),
    ("base_stop", "", "/api/drive/stop + 취소", "모든 이동 정지"),
    ("relocalize", "global_search", "POST /api/relocalize", "라이다로 지도 위 위치 다시 잡기"),
    ("wall_approach", "gap", "POST /api/wall/*", "정면 벽과 수직으로 맞추고 gap(m) 앞까지 전진"),
    ("depth_approach", "", "POST /api/approach/*", "깊이 카메라 근접선까지 전진"),
]
LOCKED_SKILLS = ("done",)     # 끌 수 없다 - 이게 없으면 지시를 끝낼 방법이 없다


# ---------------------------------------------------------------------------
# 2. 모델이 낼 답의 형식
# ---------------------------------------------------------------------------

class SkillCall(BaseModel):
    skill: str = Field(description="get_state | get_rgb | get_depth_at | pick | place "
                                   "| wait | say | done | base_state | goto_label | save_label "
                                   "| base_move | base_turn | base_stop | relocalize "
                                   "| wall_approach | depth_approach 중 하나")
    point: Optional[List[int]] = Field(
        default=None,
        description="[y, x] 순서, 0~1000 정규화. get_depth_at/pick/place 에만 쓴다")
    seconds: Optional[float] = Field(default=None, description="wait 의 대기 시간(초)")
    message: Optional[str] = Field(default=None, description="say/done 에서 사용자에게 할 말")
    label: Optional[str] = Field(default=None, description="goto_label/save_label 의 라벨 이름")
    meters: Optional[float] = Field(default=None, description="base_move 거리 (m, + 전진, - 후진, ±1 이내)")
    degrees: Optional[float] = Field(default=None, description="base_turn 각도 (°, + 왼쪽, ±180 이내)")
    gap: Optional[float] = Field(default=None, description="wall_approach 에서 차체 앞과 벽 사이 남길 거리 (m)")
    global_search: Optional[bool] = Field(default=None, description="relocalize: 지도 전체 검색이면 true")
    reason: str = Field(description="왜 이 스킬을 지금 부르는지 한 문장")


SKILL_DOC = """너는 탁상 위 물체를 집어 옮기는 로봇 팔의 조종자다.
카메라 영상 한 장을 보고, 아래 스킬을 한 번에 하나씩 호출해 일을 끝낸다.

스킬:
  get_state            지금 로봇 상태. 다음이 있다:
                       waiting_pick(잡기 기다림) picking(잡는중)
                       waiting_place(놓기 기다림) placing(놓는중)
                       homing busy calibrating not_calibrated
  get_rgb              카메라 영상을 새로 본다 (물체가 움직였거나 결과를 확인할 때)
  get_depth_at(point)  그 점의 깊이와 로봇좌표, 집기 가능 여부를 미리 확인한다.
                       팔은 움직이지 않는다. pick 전에 확인하는 습관이 좋다.
  pick(point)          그 점의 물체를 집는다. waiting_pick 상태에서만 된다.
  place(point)         그 점에 내려놓는다. waiting_place 상태에서만 된다.
  wait(seconds)        동작이 끝날 때까지 기다린다 (picking/placing 이면 필요).
  say(message)         사용자에게 한마디 하고 계속한다.
  done(message)        일이 끝났거나 더 할 수 없을 때. 이유를 message 에 적는다.

규칙:
  - point 는 반드시 [y, x] 순서이고 0~1000 으로 정규화한 값이다.
  - 집을 물체의 '가운데 윗면' 을 가리켜라. 가장자리는 깊이가 튄다.
  - pick 은 손이 비었을 때만, place 는 물체를 물고 있을 때만 된다.
    확실하지 않으면 get_state 를 먼저 불러라.
  - pick/place 는 확인 없이 바로 실행된다. 실패하면 클라이언트가 사용자에게 물어
    최대 3번까지 스스로 다시 해 본다. 그래도 ok 가 false 로 오면 그 지점은 이미
    포기한 것이다 - 같은 지점을 다시 부르지 말고, 다른 지점을 고르거나 done 을
    불러 message 에 이유를 적어라.
  - 한 번에 스킬 하나만 낸다. 결과를 보고 다음을 정한다.
  - 할 일이 없거나 사용자의 지시가 끝나면 done 을 불러라."""

BASE_DOC = """
팔은 이동 로봇(car2) 위에 있다. 로봇 몸체를 움직이는 스킬 (끝날 때까지 기다렸다가 결과를 준다):
  base_state           위치(map 좌표 x,y m / 방향°), 저장된 라벨 목록(이름, reachable, 거리),
                       Nav2 상태, 정면 장애물 거리(front_clear_m), 정면 벽(wall_ahead).
  goto_label(label)    저장된 라벨 위치로 자율 주행 (Nav2). label 은 base_state 의 이름 그대로.
                       reachable 이 false 인 라벨은 갈 수 없다.
  save_label(label)    지금 위치를 새 라벨 이름으로 저장 (지도가 있을 때만).
  base_move(meters)    똑바로 직진(+)/후진(-), 한 번에 ±1 m 이내. 앞/뒤에 장애물이 보이면 멈춘다.
  base_turn(degrees)   제자리 회전, + 왼쪽(반시계) / - 오른쪽, ±180° 이내.
  base_stop            모든 이동을 즉시 멈춘다.
  relocalize           지도 위 위치가 틀린 것 같을 때 라이다로 다시 잡는다
                       (global_search true = 지도 전체에서 찾기). 로봇은 움직이지 않는다.
  wall_approach(gap)   정면 벽(책상 앞면 등)과 수직으로 맞추고 차체 앞이 gap m 남을 때까지 다가간다.
                       gap 생략 시 저장된 값. 물건을 집기 전 작업대에 붙을 때 쓴다.
  depth_approach       깊이 카메라 기준선까지 천천히 다가간다.

몸체 규칙:
  - 몸체가 움직이면 카메라 화면이 바뀐다. 움직인 뒤에는 클라이언트가 화면을 새로 받는다.
  - 팔이 물체를 들고 있는 동안(waiting_place) 몸체를 움직여도 되지만 천천히 짧게 움직여라.
  - 위치나 라벨 이름이 확실하지 않으면 base_state 를 먼저 불러라.
  - 이동 결과 ok 가 false 면 message 를 읽고, 같은 이동을 그대로 반복하지 마라."""


# ---------------------------------------------------------------------------
# 3. 모델 호출
# ---------------------------------------------------------------------------

class Vlm:
    """test3.py 와 같은 호출 방식(interactions + JSON 스키마)을 쓴다."""

    def __init__(self, api_key: str, log=None, model: str = MODEL_ID):
        self.client = genai.Client(api_key=api_key)
        self.model = model
        self.log = log or print

    @staticmethod
    def _b64(bgr: np.ndarray) -> tuple[str, int, int]:
        h, w = bgr.shape[:2]
        small = cv2.resize(bgr, (SEND_WIDTH, int(SEND_WIDTH * h / w)),
                           interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".png", small)
        if not ok:
            raise RuntimeError("이미지 인코딩 실패")
        return base64.b64encode(buf.tobytes()).decode(), w, h

    def ask(self, bgr: np.ndarray, prompt: str,
            schema: Optional[dict] = None) -> str:
        img_b64, _, _ = self._b64(bgr)
        body = {
            "model": self.model,
            "input": [{"type": "user_input", "content": [
                {"type": "image", "data": img_b64, "mime_type": "image/png"},
                {"type": "text", "text": prompt},
            ]}],
            "generation_config": {"thinking_level": "low"},
        }
        if schema is not None:
            body["response_format"] = {"type": "text",
                                       "mime_type": "application/json",
                                       "schema": schema}
        t0 = time.perf_counter()
        res = self.client.interactions.create(**body)
        self.log(f"      (모델 {time.perf_counter() - t0:.1f}s)")
        return res.output_text


# ---------------------------------------------------------------------------
# 4. 입력 - 웹(기본) 또는 터미널
#
#    둘 다 같은 것만 한다:
#      log(text)      진행 상황을 사람에게 보여준다
#      set_view(img)  방금 받은 화면을 사람에게 보여준다
#      set_status(s)  지금 무엇을 하는 중인지 한 줄
#      next_order()   다음 지시를 받는다 (None 이면 끝내자는 뜻)
#      ask(question)  실패했을 때만 묻는 예/아니오
# ---------------------------------------------------------------------------

class ConsoleUi:
    """예전 방식. --cli 일 때 쓴다."""

    def log(self, text: str = "") -> None:
        print(text)

    def set_view(self, bgr: np.ndarray) -> None:
        pass                      # 파일로는 이미 저장돼 있다

    def set_status(self, text: str) -> None:
        pass

    def next_order(self) -> Optional[str]:
        try:
            return input("> ")
        except (EOFError, KeyboardInterrupt):
            print()
            return None

    def ask(self, question: str) -> bool:
        try:
            ans = input(f"    >> {question} (Y/n) ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            return False
        return ans in ("", "y", "yes", "예")

    def close(self) -> None:
        pass


PAGE = """<!doctype html>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>로봇 조종 - llm_client</title>
<style>
  :root { color-scheme: dark; }
  body { margin: 0; background: #14161a; color: #e6e6e6;
         font: 14px/1.5 "Malgun Gothic", "맑은 고딕", system-ui, sans-serif; }
  .wrap { max-width: 1100px; margin: 0 auto; padding: 16px; }
  h1 { font-size: 18px; margin: 0 0 12px; display: flex; align-items: center; gap: 10px; }
  .chip { font-size: 13px; font-weight: normal; background: #24303c; color: #7ec8ff;
          border-radius: 999px; padding: 3px 12px; }
  .chip.busy { background: #3a2d18; color: #ffc46b; }
  .row { display: flex; gap: 14px; align-items: flex-start; flex-wrap: wrap; }
  img { width: 640px; max-width: 100%; border-radius: 8px; background: #000; display: block; }
  .side { flex: 1 1 320px; min-width: 280px; }
  pre { background: #0e1013; border: 1px solid #262a30; border-radius: 8px;
        padding: 10px; height: 420px; overflow: auto; white-space: pre-wrap;
        word-break: break-all; margin: 0; font-size: 12.5px; }
  .ask { background: #3a2d18; border: 1px solid #7a5c22; border-radius: 8px;
         padding: 12px; margin-bottom: 10px; }
  .ask div { margin-bottom: 8px; }
  form { display: flex; gap: 8px; margin-top: 14px; flex-wrap: wrap; }
  input { flex: 1 1 320px; padding: 10px 12px; border-radius: 8px;
          border: 1px solid #333a44; background: #0e1013; color: #e6e6e6; font-size: 15px; }
  button { padding: 10px 16px; border-radius: 8px; border: 1px solid #2f6fb0;
           background: #1d4f80; color: #fff; font-size: 14px; cursor: pointer; }
  button.no { background: #3a2027; border-color: #7a3646; }
  .hint { color: #8b939c; font-size: 12.5px; margin-top: 8px; }
  details { margin-top: 14px; background: #0e1013; border: 1px solid #262a30;
            border-radius: 8px; padding: 8px 12px; }
  summary { cursor: pointer; font-weight: bold; }
  .grp { margin: 10px 0 4px; color: #7ec8ff; font-size: 13px; }
  .grp.off { color: #8b939c; }
  .tbl { overflow-x: auto; }
  table { border-collapse: collapse; width: 100%; font-size: 12.5px; }
  td, th { text-align: left; padding: 4px 8px; border-top: 1px solid #262a30; vertical-align: top; }
  th { color: #8b939c; font-weight: normal; }
  td code { color: #ffc46b; }
  td.rest { color: #8b939c; white-space: nowrap; }
  tr.off td { opacity: .45; }
  tr.off td:first-child { opacity: 1; }
  .sw { position: relative; display: inline-block; width: 34px; height: 18px; }
  .sw input { opacity: 0; width: 0; height: 0; }
  .sw span { position: absolute; inset: 0; background: #3a2027; border-radius: 999px;
             cursor: pointer; transition: background .15s; }
  .sw span::before { content: ""; position: absolute; left: 2px; top: 2px; width: 14px;
                     height: 14px; border-radius: 50%; background: #e6e6e6; transition: transform .15s; }
  .sw input:checked + span { background: #1d4f80; }
  .sw input:checked + span::before { transform: translateX(16px); }
  .sw input:disabled + span { cursor: not-allowed; opacity: .6; }
  .grp button { padding: 2px 8px; font-size: 12px; margin-left: 6px; }
</style>
<div class="wrap">
  <h1>로봇 조종 <span id="state" class="chip">-</span>
      <span id="status" class="chip busy" hidden></span></h1>
  <div class="row">
    <img id="view" src="/view.jpg?v=0" alt="현재 화면">
    <div class="side">
      <div id="ask" class="ask" hidden>
        <div id="askText"></div>
        <button onclick="answer('y')">예, 다시 해봐</button>
        <button class="no" onclick="answer('n')">아니오</button>
      </div>
      <pre id="log"></pre>
    </div>
  </div>
  <form onsubmit="send(); return false;">
    <input id="order" autocomplete="off" autofocus
           placeholder="무엇을 시킬까요?  예) 주황색 통을 상자에 넣어">
    <button>보내기</button>
    <button type="button" onclick="quick('r')">화면 다시 보기</button>
    <button type="button" class="no" onclick="quick('q')">종료</button>
  </form>
  <div class="hint">지시 하나가 끝나면 화면을 새로 받고 다음 지시를 기다린다.
    pick/place 는 묻지 않고 바로 실행하고, 실패했을 때만 위에서 다시 할지 묻는다.</div>
  <details id="skills" open>
    <summary>등록된 스킬 <span id="skillCount"></span></summary>
    <div id="skillBody" class="hint">불러오는 중...</div>
  </details>
</div>
<script>
let since = 0, viewSeq = -1, asking = 0;
const $ = function (id) { return document.getElementById(id); };

async function poll() {
  try {
    const r = await fetch('/poll?since=' + since);
    const d = await r.json();
    since = d.seq;
    if (d.text) {
      const el = $('log');
      el.textContent += d.text;
      el.scrollTop = el.scrollHeight;
    }
    if (d.view !== viewSeq) {
      viewSeq = d.view;
      $('view').src = '/view.jpg?v=' + viewSeq;
    }
    $('state').textContent = d.state || '-';
    $('status').textContent = d.status || '';
    $('status').hidden = !d.status;
    if (d.ask) {
      asking = d.ask.id;
      $('askText').textContent = d.ask.text;
      $('ask').hidden = false;
    } else {
      asking = 0;
      $('ask').hidden = true;
    }
  } catch (e) {
    $('state').textContent = '연결 끊김';
  }
  setTimeout(poll, 600);
}

async function send() {
  const el = $('order');
  const v = el.value.trim();
  if (!v) return;
  el.value = '';
  await fetch('/order?text=' + encodeURIComponent(v));
}

function quick(v) { fetch('/order?text=' + encodeURIComponent(v)); }

function answer(v) {
  fetch('/answer?id=' + asking + '&v=' + v);
  $('ask').hidden = true;
}

function esc(s) {
  return String(s).replace(/[&<>"]/g, function (c) {
    return {'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;'}[c];
  });
}

function renderSkills(groups) {
  let html = '', on = 0, total = 0;
  groups.forEach(function (g, gi) {
    const k = g.skills.filter(function (s) { return s.on; }).length;
    html += '<div class="grp' + (g.enabled ? '' : ' off') + '">' + esc(g.name);
    if (g.enabled) {
      html += ' (' + k + '/' + g.skills.length + ' 켜짐)'
            + '<button type="button" onclick="toggleGroup(' + gi + ', true)">모두 켜기</button>'
            + '<button type="button" class="no" onclick="toggleGroup(' + gi + ', false)">모두 끄기</button>';
    } else {
      html += ' - ' + esc(g.note || '사용 안 함');
    }
    html += '</div>';
    if (!g.enabled) return;
    on += k; total += g.skills.length;
    html += '<div class="tbl"><table><tr><th>사용</th><th>스킬</th><th>인자</th><th>REST</th><th>설명</th></tr>';
    g.skills.forEach(function (s) {
      html += '<tr class="' + (s.on ? '' : 'off') + '"><td><label class="sw" title="'
            + (s.locked ? '끌 수 없다 (지시를 끝내는 스킬)' : '켜기/끄기') + '">'
            + '<input type="checkbox" data-skill="' + esc(s.name) + '"' + (s.on ? ' checked' : '')
            + (s.locked ? ' disabled' : '') + ' onchange="toggleSkill(this)"><span></span></label></td>'
            + '<td><code>' + esc(s.name) + '</code></td><td>' + esc(s.args || '-')
            + '</td><td class="rest">' + esc(s.rest) + '</td><td>' + esc(s.desc) + '</td></tr>';
    });
    html += '</table></div>';
  });
  html += '<div class="hint">끈 스킬은 모델에게 알려주지 않고, 불러도 거부한다. 진행 중인 지시에도 다음 단계부터 바로 적용되고, 설정은 hand_eye/llm_skills.json 에 저장돼 재시작해도 유지된다.</div>';
  skillGroups = groups;
  $('skillBody').className = '';
  $('skillBody').innerHTML = html;
  $('skillCount').textContent = '(' + on + '/' + total + ')';
}

let skillGroups = [];

async function setSkills(names, on) {
  try {
    const r = await fetch('/skill_toggle?on=' + (on ? 1 : 0) + '&names='
                          + encodeURIComponent(names.join(',')));
    const d = await r.json();
    renderSkills(d.groups);
  } catch (e) {
    loadSkills();
  }
}

function toggleSkill(el) { setSkills([el.dataset.skill], el.checked); }

function toggleGroup(gi, on) {
  setSkills(skillGroups[gi].skills.map(function (s) { return s.name; }), on);
}

async function loadSkills() {
  // 이동 로봇 연결 여부는 시작 직후에 정해지므로, 준비될 때까지 다시 묻는다
  try {
    const d = await (await fetch('/skills')).json();
    if (!d.ready) { setTimeout(loadSkills, 1000); return; }
    renderSkills(d.groups);
  } catch (e) {
    setTimeout(loadSkills, 2000);
  }
}

poll();
loadSkills();
</script>
"""


def _make_handler(ui: "WebUi"):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):        # 콘솔을 요청 로그로 더럽히지 않는다
            pass

        def _send(self, code: int, ctype: str, body: bytes) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
                pass

        def _json(self, obj: dict) -> None:
            self._send(200, "application/json; charset=utf-8",
                       json.dumps(obj, ensure_ascii=False).encode("utf-8"))

        @staticmethod
        def _int(q: dict, key: str) -> int:
            try:
                return int(q.get(key, ["0"])[0])
            except (ValueError, TypeError):
                return 0

        def do_GET(self):
            u = urllib.parse.urlparse(self.path)
            q = urllib.parse.parse_qs(u.query)
            if u.path in ("/", "/index.html"):
                self._send(200, "text/html; charset=utf-8", PAGE.encode("utf-8"))
            elif u.path == "/view.jpg":
                data = ui.view_bytes
                if data:
                    self._send(200, "image/jpeg", data)
                else:
                    self._send(404, "text/plain; charset=utf-8", b"no view yet")
            elif u.path == "/poll":
                self._json(ui.snapshot(self._int(q, "since")))
            elif u.path == "/skills":
                ready = ui.skills_fn is not None
                self._json({"ready": ready, "groups": ui.skills_fn() if ready else []})
            elif u.path == "/skill_toggle":
                if ui.skills_fn is None or ui.toggle_fn is None:
                    self._json({"ok": False, "groups": []})
                    return
                names = [n for n in (q.get("names", [""])[0] or "").split(",") if n]
                on = q.get("on", ["1"])[0] == "1"
                changed = ui.toggle_fn(names, on)
                self._json({"ok": True, "changed": changed, "groups": ui.skills_fn()})
            elif u.path == "/order":
                text = (q.get("text", [""])[0] or "").strip()
                if text:
                    ui.put_order(text)
                self._json({"ok": bool(text)})
            elif u.path == "/answer":
                ok = ui.answer(self._int(q, "id"), q.get("v", ["n"])[0] == "y")
                self._json({"ok": ok})
            else:
                self._send(404, "text/plain; charset=utf-8",
                           "없는 주소".encode("utf-8"))

    return Handler


class WebUi:
    """브라우저로 지시를 받고, 화면 / 로그 / 재시도 질문을 보여준다.

    에이전트는 메인 스레드에서 돌고(next_order 가 큐에서 막힌다), HTTP 는 데몬
    스레드에서 받는다. 둘 사이는 큐 하나와 Event 하나로만 만난다.
    """

    def __init__(self, port: int = WEB_PORT, host: str = "127.0.0.1",
                 state_fn=None, open_browser: bool = True):
        self.lock = threading.Lock()
        self.lines: List[str] = []
        self.orders: "queue.Queue[str]" = queue.Queue()
        self.view_bytes: bytes = b""
        self.view_seq = 0
        self.status_text = ""
        self.pending: Optional[dict] = None
        self._ask_id = 0
        self._answer = False
        self._answered = threading.Event()
        self.state_fn = state_fn
        self._state: dict = {}
        self._state_at = 0.0
        # Agent 가 만들어지면 채운다 (skill_groups / set_skills). 그 전엔 None
        self.skills_fn = None
        self.toggle_fn = None

        self.srv = ThreadingHTTPServer((host, port), _make_handler(self))
        self.srv.daemon_threads = True
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()
        self.url = f"http://{host}:{port}/"
        print(f"[web] 지시는 여기서 넣으세요: {self.url}")
        if open_browser:
            try:
                webbrowser.open(self.url)
            except Exception:
                pass

    # --- 사람에게 보여주는 쪽 ------------------------------------------------
    def log(self, text: str = "") -> None:
        print(text)
        with self.lock:
            self.lines.extend(str(text).split("\n"))

    def set_view(self, bgr: np.ndarray) -> None:
        ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not ok:
            return
        with self.lock:
            self.view_bytes = buf.tobytes()
            self.view_seq += 1

    def set_status(self, text: str) -> None:
        with self.lock:
            self.status_text = text

    def snapshot(self, since: int) -> dict:
        with self.lock:
            since = max(0, min(since, len(self.lines)))
            out = {
                "seq": len(self.lines),
                "text": "".join(line + "\n" for line in self.lines[since:]),
                "view": self.view_seq,
                "status": self.status_text,
                "ask": dict(self.pending) if self.pending else None,
            }
        out["state"] = self._robot_state()
        return out

    def _robot_state(self) -> str:
        """상태는 브라우저가 물어볼 때 test4 에 되물어 온다 (0.5초 캐시)."""
        now = time.time()
        if self.state_fn and now - self._state_at > 0.5:
            try:
                self._state = self.state_fn() or {}
            except Exception:
                self._state = {"label": "?"}
            self._state_at = now
        return self._state.get("label", "-")

    # --- 사람에게서 받는 쪽 --------------------------------------------------
    def put_order(self, text: str) -> None:
        self.orders.put(text)

    def next_order(self) -> Optional[str]:
        self.set_status("")
        try:
            return self.orders.get()
        except KeyboardInterrupt:
            return None

    def ask(self, question: str) -> bool:
        self._answered.clear()
        with self.lock:
            self._ask_id += 1
            self.pending = {"id": self._ask_id, "text": question}
        self.log(f"    ?? {question}  -> 웹에서 [예/아니오]")
        got = self._answered.wait(ASK_TIMEOUT)
        with self.lock:
            self.pending = None
            ans = self._answer if got else False
        self.log("    -> " + ("예" if ans else
                              ("아니오" if got else "답이 없어 아니오로 본다")))
        return ans

    def answer(self, ask_id: int, yes: bool) -> bool:
        with self.lock:
            if not self.pending or self.pending["id"] != ask_id:
                return False
            self._answer = yes
        self._answered.set()
        return True

    def close(self) -> None:
        try:
            self.srv.shutdown()
            self.srv.server_close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# 5. 에이전트 루프
# ---------------------------------------------------------------------------

class Agent:
    def __init__(self, api: Api, vlm: Vlm, ui, auto: bool = False, base: Optional[MapApi] = None):
        self.api, self.vlm, self.ui, self.auto, self.base = api, vlm, ui, auto, base
        self.frame: Optional[np.ndarray] = None
        self.disabled: set[str] = self._load_disabled()   # 웹 UI 에서 끈 스킬 (재시작해도 유지)
        if self.disabled:
            self.ui.log(f"[skill] 꺼 둔 스킬 (저장된 설정): {', '.join(sorted(self.disabled))}")

    # --- 스킬 켜기/끄기 설정 (SKILLS_PATH 에 저장) ----------------------------
    @staticmethod
    def _load_disabled() -> set[str]:
        try:
            with open(SKILLS_PATH, encoding="utf-8") as f:
                names = json.load(f).get("disabled", [])
        except FileNotFoundError:
            return set()
        except (OSError, ValueError, AttributeError) as e:
            print(f"[skill] {SKILLS_PATH} 를 읽지 못해 모든 스킬을 켠다: {e}")
            return set()
        known = {s[0] for s in ARM_SKILL_LIST + BASE_SKILL_LIST}
        return {n for n in names if n in known and n not in LOCKED_SKILLS}

    def _save_disabled(self) -> None:
        tmp = SKILLS_PATH + ".tmp"
        try:
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump({"disabled": sorted(self.disabled)}, f, ensure_ascii=False, indent=2)
            os.replace(tmp, SKILLS_PATH)       # 쓰다 죽어도 이전 파일은 멀쩡하게
        except OSError as e:
            self.ui.log(f"[skill] 설정 저장 실패 ({SKILLS_PATH}): {e}")

    def skill_doc(self) -> str:
        doc = SKILL_DOC + (BASE_DOC if self.base else "")
        off = sorted(self.disabled)
        if off:
            doc += ("\n\n사용자가 다음 스킬을 꺼 두었다 - 부르지 마라 (불러도 거부된다): "
                    + ", ".join(off) + "\n이 스킬 없이는 할 수 없는 일이면 done 으로 이유를 알려라.")
        return doc

    def set_skills(self, names: List[str], on: bool) -> List[str]:
        """스킬을 켜거나 끈다. 실제로 바뀐 이름만 돌려준다 (done 은 끌 수 없다)."""
        known = {s[0] for s in ARM_SKILL_LIST + BASE_SKILL_LIST}
        disabled, changed = set(self.disabled), []
        for n in names:
            if n not in known or n in LOCKED_SKILLS or (n in disabled) != on:
                continue
            (disabled.discard if on else disabled.add)(n)
            changed.append(n)
        self.disabled = disabled             # 통째로 바꿔서 에이전트 스레드가 반쯤 바뀐 걸 보지 않게
        if changed:
            self._save_disabled()
            self.ui.log(f"[skill] {'켬' if on else '끔'}: {', '.join(changed)}")
        return changed

    def skill_groups(self) -> list:
        """스킬 목록과 켜짐 여부 (웹 UI 표시용)."""
        off = self.disabled

        def rows(lst):
            return [{"name": n, "args": a, "rest": r, "desc": d,
                     "on": n not in off, "locked": n in LOCKED_SKILLS} for n, a, r, d in lst]
        return [
            {"name": f"로봇 팔 - {self.api.base}", "enabled": True, "skills": rows(ARM_SKILL_LIST)},
            {"name": "이동 로봇 (car2)" + (f" - {self.base.base}" if self.base else ""),
             "enabled": self.base is not None, "skills": rows(BASE_SKILL_LIST),
             "note": "map_web 미연결 또는 --no-base"},
        ]

    # --- 화면 -------------------------------------------------------------
    def look(self) -> bool:
        """지금 화면을 받아 저장한다. 사용자도 같은 그림을 볼 수 있어야 한다."""
        frame = self.api.rgb()
        if frame is None:
            self.ui.log("[error] /get_rgb 실패 - test4.py 가 떠 있는지 확인하세요")
            return False
        self.frame = frame
        cv2.imwrite(VIEW_PATH, frame)
        self.ui.set_view(frame)
        h, w = frame.shape[:2]
        self.ui.log(f"[view] 현재 화면 {w}x{h} (저장: {VIEW_PATH})")
        return True

    def to_pixel(self, point: List[int]) -> tuple[int, int]:
        """모델의 [y, x](0~1000) -> 실제 픽셀 (x, y)."""
        h, w = self.frame.shape[:2]
        y = int(round(point[0] / 1000.0 * (h - 1)))
        x = int(round(point[1] / 1000.0 * (w - 1)))
        return max(0, min(w - 1, x)), max(0, min(h - 1, y))

    # --- 첫 인사: 무엇이 보이고 무엇을 할 수 있는지 -------------------------
    def brief(self) -> str:
        state = self.api.json("/get_state")
        base = ""
        if self.base:
            bs = self.base.summary()
            names = [lb["name"] for lb in bs.get("labels", []) if lb.get("reachable")]
            base = ("이 팔은 이동 로봇 위에 있어서 저장된 장소로 이동할 수 있다"
                    + (f" (장소: {', '.join(names)})" if names else "") + ".\n")
        prompt = (
            "이 사진은 로봇 팔이 보고 있는 작업대다. "
            f"로봇 상태는 '{state.get('label', '?')}' 이다.\n" + base +
            "1) 보이는 물체를 짧게 나열하고, 2) 지금 이 로봇으로 할 수 있는 일을 "
            "두세 가지 제안해라. 각 제안은 한 줄로, 사용자가 그대로 지시할 수 있는 "
            "문장으로 써라. 좌표는 아직 말하지 마라."
        )
        return self.vlm.ask(self.frame, prompt)

    # --- 스킬 실행 ---------------------------------------------------------
    def run_skill(self, call: SkillCall) -> dict:
        s = call.skill
        if s in self.disabled:
            return {"ok": False, "message": f"{s} 는 사용자가 꺼 둔 스킬이다 - 다른 방법을 쓰거나 done"}
        if s in ("say", "done"):
            return {"ok": True, "message": call.message or ""}
        if s in BASE_SKILLS:
            return self.run_base(call)
        if s == "get_state":
            return self.api.json("/get_state")
        if s == "get_rgb":
            return {"ok": self.look(), "note": "새 화면을 받았다"}
        if s == "wait":
            return self.wait_idle(float(call.seconds or 3.0))
        if s in ("get_depth_at", "pick", "place"):
            if not call.point or len(call.point) != 2:
                return {"ok": False, "message": "point 가 [y, x] 형식이 아니다"}
            x, y = self.to_pixel(call.point)
            if s == "get_depth_at":
                return {**self.api.json("/get_depth", {"x": x, "y": y}), "uv": [x, y]}
            return self.act(s, x, y)           # 묻지 않고 바로 한다
        return {"ok": False, "message": f"모르는 스킬: {s}"}

    def run_base(self, call: SkillCall) -> dict:
        b, s = self.base, call.skill
        if b is None:
            return {"ok": False, "message": "이동 로봇(map_web)이 연결되지 않았다 - 몸체 스킬을 쓸 수 없다"}
        self.ui.set_status(f"{s} 실행 중")
        if s == "base_state":
            return b.summary()
        if s == "base_stop":
            return b.stop()
        if s in ("goto_label", "save_label"):
            if not call.label:
                return {"ok": False, "message": "label 이 필요하다"}
            return b.goto_label(call.label) if s == "goto_label" else b.save_label(call.label)
        if s == "base_move":
            if call.meters is None:
                return {"ok": False, "message": "meters 가 필요하다"}
            return b.move(call.meters)
        if s == "base_turn":
            if call.degrees is None:
                return {"ok": False, "message": "degrees 가 필요하다"}
            return b.turn(call.degrees)
        if s == "relocalize":
            return b.relocalize(bool(call.global_search))
        if s == "wall_approach":
            return b.wall_approach(call.gap)
        if s == "depth_approach":
            return b.depth_approach()
        return {"ok": False, "message": f"모르는 스킬: {s}"}

    def act(self, action: str, x: int, y: int) -> dict:
        """팔을 움직인다. 확인 없이 바로 하고, 실패했을 때만 다시 할지 묻는다."""
        out: dict = {}
        for attempt in range(1, MAX_RETRY + 1):
            self.ui.set_status(f"{action} 실행 중")
            self.ui.log(f"    >> {action}({x},{y}) 실행"
                        + (f" (재시도 {attempt - 1})" if attempt > 1 else ""))
            out = self.api.json(f"/{action}", {"x": x, "y": y})
            if out.get("ok"):                      # 접수됐으면 끝날 때까지 기다린다
                out["after_wait"] = self.wait_idle(20.0)
                if out["after_wait"].get("ok"):
                    return out
                why = out["after_wait"].get("message", "동작이 끝나지 않았다")
            else:
                why = out.get("message") or out.get("error") or f"HTTP {out.get('http')}"
            self.ui.log(f"    !! {action} 실패: {why}")
            if attempt >= MAX_RETRY or not self.ask_retry(action, x, y, why):
                break
        out["ok"] = False
        out["message"] = why
        out["tries"] = attempt
        return out

    def ask_retry(self, action: str, x: int, y: int, why: str) -> bool:
        """실패했을 때만 묻는다. --auto 면 묻지 않고 모델에게 실패를 알린다."""
        if self.auto:
            return False
        self.ui.set_status(f"{action} 실패 - 다시 할지 기다리는 중")
        return self.ui.ask(f"{action}({x},{y}) 실패: {why} / 다시 할까요?")

    def wait_idle(self, timeout: float) -> dict:
        """동작이 끝날 때까지 상태를 지켜본다."""
        t0 = time.time()
        last = {}
        while time.time() - t0 < timeout:
            last = self.api.json("/get_state")
            if last.get("state") not in ("picking", "placing", "homing", "busy"):
                return {"ok": True, "state": last.get("state"), "label": last.get("label")}
            time.sleep(0.5)
        return {"ok": False, "state": last.get("state"), "message": "동작이 아직 안 끝났다"}

    # --- 지시 하나를 끝까지 --------------------------------------------------
    def follow(self, order: str) -> None:
        history: list[str] = []
        for step in range(1, MAX_STEPS + 1):
            state = self.api.json("/get_state")
            self.ui.set_status(f"생각 중 ({step}/{MAX_STEPS})")
            prompt = (
                f"{self.skill_doc()}\n\n"
                f"사용자 지시: {order}\n"
                f"로봇 상태: {state.get('state')} ({state.get('label')})\n"
                + ("지금까지 한 일:\n" + "\n".join(history) + "\n" if history else "")
                + "다음에 부를 스킬 하나를 JSON 으로 내라."
            )
            try:
                raw = self.vlm.ask(self.frame, prompt, SkillCall.model_json_schema())
                call = SkillCall(**json.loads(raw))
            except Exception as e:
                self.ui.log(f"[error] 모델 응답을 읽지 못했다: {type(e).__name__}: {e}")
                return

            arg = "".join(f" {v}" for v in (call.point, call.label, call.meters, call.degrees, call.gap)
                          if v is not None)
            self.ui.log(f"  [{step}] {call.skill}{arg}  - {call.reason}")
            if call.message:
                self.ui.log(f"      말: {call.message}")

            result = self.run_skill(call)
            self.ui.log(f"      결과: {json.dumps(result, ensure_ascii=False)[:200]}")
            history.append(f"- {call.skill}{arg} -> "
                           f"{json.dumps(result, ensure_ascii=False)[:160]}")

            if call.skill == "done":
                return
            if call.skill in ("pick", "place") and result.get("ok"):
                self.look()          # 움직였으면 화면이 바뀐다
            if call.skill in BASE_MOVING and (result.get("moved_m") or result.get("turned_deg")
                                              or result.get("state") in ("succeeded", "done")):
                self.look()          # 몸체가 움직였으면 카메라가 보는 곳이 바뀐다
        self.ui.log("[warn] 스킬 호출 상한에 도달했다 - 지시를 더 구체적으로 주세요")


# ---------------------------------------------------------------------------
# 6. 실행
# ---------------------------------------------------------------------------

def run(agent: Agent, api: Api, ui, first_order: str = "") -> None:
    """지시 하나를 끝내면 화면을 새로 받고 다음 지시를 기다린다."""
    if not agent.look():
        return

    ui.set_status("화면 살펴보는 중")
    ui.log("[화면 설명]")
    ui.log(agent.brief())
    ui.log("")
    ui.log("무엇을 시킬까요? ('r' 화면 다시 보기, 'q' 종료)")

    pending = (first_order or "").strip()
    while True:
        if pending:
            order, pending = pending, ""
            ui.log("")
            ui.log(f"[지시] {order}")
            agent.follow(order)
            # 명령이 끝났으니 화면을 새로 보고 다음 명령을 기다린다
            ui.set_status("화면 갱신 중")
            agent.look()
            st = api.json("/get_state")
            ui.log("")
            ui.log(f"[완료] 로봇 상태: {st.get('label', '?')} - 다음 지시를 기다립니다")
            continue

        order = ui.next_order()
        if order is None:
            break
        order = order.strip()
        if not order or order.lower() in ("q", "quit", "exit", "종료"):
            break
        if order.lower() == "r":
            ui.set_status("화면 다시 보는 중")
            agent.look()
            ui.log(agent.brief())
            continue
        pending = order

    ui.log("[bye]")



ENV_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env")


def load_api_key() -> str:
    """환경변수 GEMINI_API_KEY 가 우선, 없으면 이 파일 옆의 .env 에서 읽는다.
    .env 는 git 에 올라가지 않는다 (.gitignore)."""
    key = os.environ.get("GEMINI_API_KEY", "").strip()
    if key or not os.path.exists(ENV_FILE):
        return key
    with open(ENV_FILE, encoding="utf-8") as f:
        for line in f:
            name, sep, value = line.strip().partition("=")
            if sep and name.strip() == "GEMINI_API_KEY":
                return value.strip().strip("'\"")
    return ""


def main() -> None:
    ap = argparse.ArgumentParser(description="Gemini + test4.py REST 로봇 조종")
    ap.add_argument("order", nargs="*", help="첫 지시 (없으면 웹/터미널에서 받는다)")
    ap.add_argument("--api", default=DEFAULT_API, help=f"REST 주소 (기본 {DEFAULT_API})")
    ap.add_argument("--auto", action="store_true",
                    help="실패해도 다시 할지 묻지 않는다 (기본은 실패 때만 묻는다)")
    ap.add_argument("--cli", action="store_true", help="웹 대신 터미널로 지시를 받는다")
    ap.add_argument("--port", type=int, default=WEB_PORT,
                    help=f"웹 입력 UI 포트 (기본 {WEB_PORT})")
    ap.add_argument("--host", default="127.0.0.1",
                    help="웹 입력 UI 바인드 주소 (기본 127.0.0.1)")
    ap.add_argument("--no-browser", action="store_true",
                    help="브라우저를 자동으로 열지 않는다")
    ap.add_argument("--map-api", default="",
                    help=f"이동 로봇 map_web REST 주소 (기본: --api 와 같은 호스트의 {MAP_WEB_PORT} 포트)")
    ap.add_argument("--no-base", action="store_true",
                    help="이동 로봇 스킬을 쓰지 않는다 (팔만)")
    args = ap.parse_args()

    key = load_api_key()
    if not key:
        print(f"GEMINI_API_KEY 가 없다. {ENV_FILE} 에 GEMINI_API_KEY=... 를 적거나 "
              "$env:GEMINI_API_KEY='...' 로 설정할 것 "
              "(이 저장소의 test3.py 가 쓰는 키와 같은 것을 쓰면 된다)")
        return

    api = Api(args.api)
    state = api.json("/get_state")
    if not state.get("ok"):
        print(f"[error] {args.api} 에 연결하지 못했다 - test4.py 를 먼저 실행하세요")
        return
    print(f"[api] {args.api} | 로봇 상태: {state.get('label')} ({state.get('state')})")
    if state.get("state") == "not_calibrated":
        print("      주의: hand-eye 미보정이라 pick 이 거부된다")

    if args.cli:
        ui = ConsoleUi()
    else:
        try:
            ui = WebUi(args.port, args.host,
                       state_fn=lambda: api.json("/get_state"),
                       open_browser=not args.no_browser)
        except OSError as e:
            print(f"[error] 웹 UI 포트 {args.port} 를 열지 못했다: {e}")
            print("        --port 로 다른 포트를 주거나, --cli 로 터미널을 쓰세요")
            return

    base = None
    if not args.no_base:
        if not args.map_api:          # 팔과 이동 로봇은 같은 파이에서 돈다
            host = urllib.parse.urlparse(args.api).hostname or "127.0.0.1"
            args.map_api = f"http://{host}:{MAP_WEB_PORT}"
        base = MapApi(args.map_api, log=ui.log, status=ui.set_status)
        if base.alive():
            print(f"[base] {args.map_api} | 이동 로봇 스킬 사용 (base_state, goto_label, base_move ...)")
        else:
            print(f"[base] {args.map_api} 에 연결하지 못했다 - 이동 로봇 스킬 없이 팔만 쓴다")
            base = None

    agent = Agent(api, Vlm(key, ui.log), ui, auto=args.auto, base=base)
    if isinstance(ui, WebUi):
        ui.toggle_fn = agent.set_skills
        ui.skills_fn = agent.skill_groups
    try:
        run(agent, api, ui, " ".join(args.order))
    except KeyboardInterrupt:
        print()
    finally:
        ui.close()


if __name__ == "__main__":
    main()
