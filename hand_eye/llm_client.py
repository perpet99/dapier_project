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
현재 화면 / 진행 로그 / 로봇 상태를 보면서 명령을 넣는다. 기본으로 0.0.0.0 에
바인드하므로 같은 네트워크의 다른 PC 에서도 http://<이 기계 IP>:8770 으로 들어올
수 있다 (시작할 때 주소를 찍는다. 이 기계에서만 쓰려면 --host 127.0.0.1).
인증이 없으니 접속할 수 있는 사람은 누구나 로봇을 움직일 수 있다는 점에 주의.
터미널로 받고 싶으면 --cli 를 준다.

웹에서는 [🎤 음성 명령] 으로 말로도 지시할 수 있다. 말하는 동안 브라우저가 16 kHz
PCM 을 0.2초마다 서버로 흘리고, 서버는 Gemini Live API(--live-model, 기본
gemini-3.5-transcribe-live)로 실시간 받아 적어 중간 결과를 입력칸에 바로 보여준다.
녹음이 끝나면 확정 문장이 곧바로 나온다 (말 끝나고 3초 무음 뒤 멈추므로 그 사이 확정된다).
Live 를 못 쓰면 쌓아 둔 음성을 --stt-model(기본 gemini-3.5-flash-lite)로 한 번에 받아 적는다.

음성 답변: 지시를 받으면 무엇을 할지, 끝나면 결과를 짧은 한 문장으로 소리 내 알려 준다.
모델이 스킬 JSON 의 speech 에 쓴 문장을 서버가 --tts-model(기본 gemini-3.8-flash-lite-tts)
로 음성으로 만들고, 웹 UI 가 차례로 재생한다 ('🔊 음성 답변 듣기' 로 끄고 켠다). TTS 가 안
되면 브라우저 내장 음성(ko-KR)으로 읽는다. --cli 에서는 글로 찍는다. 마이크는 브라우저 규칙상 https 또는 localhost(127.0.0.1) 에서만 열린다.
그래서 다른 PC 용으로 https 포트(--https-port, 기본 8771)를 자체 서명 인증서
(llm_tls_*.pem, 없으면 자동 생성)로 함께 연다 - https://<IP>:8771 로 들어가
처음 한 번 브라우저 경고에서 '계속' 을 누르면 마이크를 쓸 수 있다.

처리 단계 로그: 음성 입력(V1, V2 ...)과 AI 처리(지시 O1 ..., 화면 설명 B1 ...)의
단계마다 시각 / 걸린 시간 / 프롬프트·응답 원문을 남긴다. 웹 UI 의 '처리 단계 로그'
에서 진행 중인 단계를 보고, 전부는 hand_eye/llm_trace.jsonl 에 쌓인다 (5 MB 넘으면 .1 로).

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
import asyncio
import base64
import io
import json
import math
import os
import queue
import re
import socket
import ssl
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import wave
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import List, Optional

import cv2
import numpy as np
from google import genai
from google.genai import types as genai_types
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
# 음성 명령 받아쓰기. 실시간(Live API)이 기본이고, Live 가 안 되면 일괄 모델로 대신한다.
# 측정 (Pi, 6.4초 한국어): Live = 말 끝나고 ~2초 안에 확정 / flash-lite 일괄 4~6초 /
# 3.8-flash 일괄 12~158초.
LIVE_STT_MODEL_ID = os.environ.get("GEMINI_LIVE_STT_MODEL", "gemini-3.5-transcribe-live")
STT_MODEL_ID = os.environ.get("GEMINI_STT_MODEL", "gemini-3.5-flash-lite")
VOICE_RATE = 16000                  # 브라우저가 보내는 PCM (16bit 모노)
LIVE_END_WAIT = 5.0                 # 녹음이 끝난 뒤 확정 결과를 기다리는 최대 시간 (s)
LIVE_IDLE_SEC = 15.0                # 이만큼 브라우저 소식이 없으면 세션을 닫는다 (탭을 닫은 경우)
MAX_VOICE_BYTES = 4 * 1024 * 1024   # 음성 업로드 상한 (16 kHz 16bit 모노면 약 2분)
# 음성 답변 (지시를 받으면 할 일 / 끝나면 결과를 한 문장으로 들려준다). 측정: 한 문장 3~5초.
TTS_MODEL_ID = os.environ.get("GEMINI_TTS_MODEL", "gemini-3.8-flash-lite-tts")
TTS_VOICE = os.environ.get("GEMINI_TTS_VOICE", "Kore")
TTS_KEEP = 30                       # 서버에 남겨 둘 최근 음성 답변 수
# 모델 호출 시간 제한. API 가 답 없이 매달려도 에이전트가 멈추지 않게 (실측: 12분 넘게 무응답).
# 시간 초과나 일시 오류(5xx, 429, 연결)는 MODEL_TRIES 번까지 시도한다.
MODEL_TIMEOUT = 30.0                # 로봇 판단 / 화면 설명 한 번
STT_TIMEOUT = 30.0                  # 일괄 받아쓰기 한 번
TTS_TIMEOUT = 20.0                  # 음성 답변 한 번 (실패하면 브라우저 내장 음성)
MODEL_TRIES = 2
VOICE_MIN_PEAK = 800                # 이보다 작으면(16bit, 약 -32 dBFS) 말이 없다고 본다
TLS_CERT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "llm_tls_cert.pem")
TLS_KEY = os.path.join(os.path.dirname(os.path.abspath(__file__)), "llm_tls_key.pem")
VIEW_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "llm_view.jpg")
SKILLS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "llm_skills.json")  # 웹에서 끈 스킬
TRACE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "llm_trace.jsonl")   # 처리 단계 로그
TRACE_MAX_BYTES = 5 * 1024 * 1024   # 넘으면 llm_trace.jsonl.1 로 돌린다
TRACE_KEEP = 1000                   # 메모리(웹 UI)에 남길 줄 수
TRACE_DETAIL = 8000                 # 프롬프트/응답 원문은 이만큼만 남긴다
SEND_WIDTH = 800          # 모델에 보낼 이미지 폭. 토큰을 아끼되 점을 찍을 만큼은 크게
MAX_STEPS = 12            # 한 지시당 스킬 호출 상한 (무한루프 방지)
MAX_RETRY = 3             # pick/place 가 실패했을 때 다시 해볼 최대 횟수
WEB_PORT = 8770           # 웹 입력 UI 포트 (test4 의 REST 8765 와 다른 포트다)
ASK_TIMEOUT = 300.0       # 재시도 질문에 답이 없으면 이 시간 뒤 '아니오' 로 본다


# ---------------------------------------------------------------------------
# 0. 처리 단계 로그
# ---------------------------------------------------------------------------

class Trace:
    """음성 입력과 AI 처리의 단계별 내부 로그. 웹 UI 의 '처리 단계 로그' 가 읽는다.

    일 한 건(job)마다 id 를 붙인다 - V3 = 3번째 음성 입력, O2 = 2번째 지시,
    B1 = 화면 설명, S1 = 프로그램 시작. 단계(stage)마다 한 줄, 마지막 줄은 end=True.
    최근 TRACE_KEEP 줄은 메모리에, 전부는 TRACE_PATH 에 JSONL 로 쌓는다.
    """
    KINDS = {"V": "음성", "O": "지시", "B": "화면 설명", "S": "시스템"}

    def __init__(self, path: str = TRACE_PATH):
        self.path = path
        self.lock = threading.Lock()
        self.items: List[dict] = []
        self.seq = 0
        self._count: dict = {}
        self._start: dict = {}

    def new_job(self, prefix: str) -> str:
        with self.lock:
            n = self._count[prefix] = self._count.get(prefix, 0) + 1
        return f"{prefix}{n}"

    def add(self, job: Optional[str], stage: str, msg: str = "", level: str = "info",
            ms: Optional[float] = None, detail: Optional[str] = None, end: bool = False) -> None:
        """level: info | ok | warn | error. ms = 이 단계에 걸린 시간."""
        if not job:
            return
        now = time.time()
        with self.lock:
            self.seq += 1
            start = self._start.setdefault(job, now)
            e = {"seq": self.seq, "ts": round(now, 3),
                 "t": time.strftime("%H:%M:%S", time.localtime(now)) + f".{int(now % 1 * 1000):03d}",
                 "job": job, "kind": self.KINDS.get(job[:1], job[:1]), "stage": stage,
                 "msg": str(msg), "level": level, "el": round(now - start, 2)}
            if ms is not None:
                e["ms"] = int(ms)
            if detail:
                e["detail"] = str(detail)[:TRACE_DETAIL]
            if end:
                e["end"] = True
                self._start.pop(job, None)
            self.items.append(e)
            del self.items[:-TRACE_KEEP]
            self._write(e)

    def _write(self, e: dict) -> None:
        try:
            if os.path.exists(self.path) and os.path.getsize(self.path) > TRACE_MAX_BYTES:
                os.replace(self.path, self.path + ".1")
            with open(self.path, "a", encoding="utf-8") as f:
                f.write(json.dumps(e, ensure_ascii=False) + "\n")
        except OSError:
            pass                                  # 로그 때문에 로봇이 멈추면 안 된다

    def since(self, seq: int) -> tuple[List[dict], bool]:
        """seq 뒤의 줄들. 브라우저가 서버 재시작 전 번호를 들고 오면 처음부터 (reset=True)."""
        with self.lock:
            if seq > self.seq:
                return list(self.items), True
            return [e for e in self.items if e["seq"] > seq], False


TRACE = Trace()


def valid_job(job: str, prefixes: str = "VOBS") -> bool:
    """브라우저가 보낸 job id 검사 (V12 같은 모양만)."""
    return 2 <= len(job or "") <= 8 and job[0] in prefixes and job[1:].isdigit()


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
    speech: Optional[str] = Field(
        default=None,
        description="사용자에게 소리로 들려줄 아주 짧은 한국어 한 문장 (30자 안팎). "
                    "첫 스킬에서는 무엇을 할지, done 에서는 결과를 말한다. 다른 단계에서는 비운다")
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
  - 할 일이 없거나 사용자의 지시가 끝나면 done 을 불러라.
  - speech 는 사용자에게 소리로 들려준다. 지시를 받은 첫 스킬에서 무엇을 할지,
    done 에서 결과를 각각 짧은 한 문장(30자 안팎, 존댓말)으로 쓴다. 좌표나 스킬 이름은
    말하지 말고, 그 밖의 단계에서는 비워라."""

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

class ModelTimeout(Exception):
    """모델이 시간 제한 안에 답하지 않았다."""


def with_deadline(fn, seconds: float):
    """fn() 을 별도 스레드에서 돌려 seconds 까지만 기다린다. SDK 가 안에서 재시도하거나
    연결이 매달려도 부른 쪽은 반드시 돌아온다 (남은 스레드는 SDK 시간 제한으로 곧 끝난다)."""
    box: dict = {}

    def run():
        try:
            box["v"] = fn()
        except BaseException as e:            # 부른 쪽 스레드로 넘긴다
            box["e"] = e

    th = threading.Thread(target=run, daemon=True, name="model-call")
    th.start()
    th.join(seconds)
    if th.is_alive():
        raise ModelTimeout(f"{seconds:.0f}초 안에 응답이 없다")
    if "e" in box:
        raise box["e"]
    return box["v"]


def quota_message(e: BaseException) -> Optional[str]:
    """일일 할당량을 다 쓴 429 면 사람이 읽을 설명, 아니면 None.
    (무료 등급 gemini-robotics-er-2-preview 는 하루 20회 - 다시 해도 소용없다)"""
    text = str(e)
    if not ("429" in text or "RESOURCE_EXHAUSTED" in text or "too_many_requests" in text):
        return None
    if not re.search(r"per day|PerDay|quota", text, re.I):
        return None                             # 분당 한도 같은 짧은 제한은 다시 해 볼 만하다
    m = re.search(r"limit: (\d+)[^.]*?per day", text) or re.search(r"limit: (\d+)", text)
    wait = re.search(r"retry in ([0-9hms.]+)", text)
    return ("모델 일일 할당량을 다 썼다"
            + (f" (하루 {m.group(1)}회)" if m else "")
            + (f" - {re.sub(r'\.\d+s$', 's', wait.group(1))} 뒤에 다시 쓸 수 있다" if wait else ""))


def is_transient(e: BaseException) -> bool:
    """다시 해 볼 만한 오류인가 (시간 초과, 서버 5xx, 짧은 429, 연결)."""
    if isinstance(e, ModelTimeout):
        return True
    if quota_message(e):
        return False
    name = type(e).__name__
    if any(k in name for k in ("Timeout", "Connection", "Server", "RateLimit", "Unavailable")):
        return True
    code = getattr(e, "code", None) or getattr(e, "status_code", None)
    return isinstance(code, int) and (code == 429 or code >= 500)


class Vlm:
    """test3.py 와 같은 호출 방식(interactions + JSON 스키마)을 쓴다."""

    def __init__(self, api_key: str, log=None, model: str = MODEL_ID,
                 stt_model: str = STT_MODEL_ID, live_model: Optional[str] = LIVE_STT_MODEL_ID,
                 tts_model: Optional[str] = TTS_MODEL_ID):
        self.client = genai.Client(api_key=api_key)
        # interactions 클라이언트는 기본으로 429/5xx 를 최대 1시간 동안 조용히 재시도한다 -
        # 할당량 초과(429)가 '응답 없음' 으로 보이게 된다. 재시도는 _call 이 한 번만 한다.
        try:
            from google.genai._gaos.utils.retries import RetryConfig
            self.client.interactions.sdk_configuration.retry_config = RetryConfig("none", None, False)
        except Exception:                      # SDK 내부 구조가 바뀌면 그냥 기본값으로
            pass
        self.model = model
        self.stt_model = stt_model
        self.live_model = live_model          # None 이면 실시간 없이 일괄 받아쓰기만
        self.tts_model = tts_model            # None 이면 음성 답변은 브라우저 내장 음성으로
        self.log = log or print

    def live(self, job: str) -> "LiveStt":
        """음성 하나를 실시간으로 받아 적는 세션을 연다 (웹 녹음 시작 때)."""
        return LiveStt(self, job)

    def _call(self, fn, timeout: float, job: Optional[str], what: str, tries: int = MODEL_TRIES):
        """모델 호출 하나에 시간 제한과 재시도를 건다. fn(timeout) 은 SDK 에도 같은 제한을 넘긴다."""
        for attempt in range(1, tries + 1):
            try:
                return with_deadline(lambda: fn(timeout), timeout + 5)
            except Exception as e:
                if attempt >= tries or not is_transient(e):
                    raise
                why = "시간 초과" if isinstance(e, ModelTimeout) else type(e).__name__
                self.log(f"      ({what} {why} - 다시 시도 {attempt + 1}/{tries})")
                TRACE.add(job, f"{what} 다시 시도", f"{why}: {e}"[:200] + f" ({attempt + 1}/{tries})",
                          level="warn")
                time.sleep(1.0)

    def tts(self, text: str, job: Optional[str] = None) -> bytes:
        """음성 답변용 WAV. 재생은 브라우저가 한다."""
        TRACE.add(job, "음성 답변 만들기", f"{self.tts_model}: {text}")
        t0 = time.perf_counter()
        try:
            r = self._call(lambda t: self.client.models.generate_content(
                model=self.tts_model, contents=text,
                config=genai_types.GenerateContentConfig(
                    response_modalities=["AUDIO"],
                    http_options=genai_types.HttpOptions(timeout=int(t * 1000)),
                    speech_config=genai_types.SpeechConfig(voice_config=genai_types.VoiceConfig(
                        prebuilt_voice_config=genai_types.PrebuiltVoiceConfig(voice_name=TTS_VOICE))))),
                TTS_TIMEOUT, job, "음성 답변", tries=1)    # 실패하면 브라우저 내장 음성이 대신한다
            part = r.candidates[0].content.parts[0].inline_data
            data = part.data
            if data[:4] != b"RIFF":                # 날 PCM 이면 WAV 로 싼다 (24 kHz 16bit)
                rate = 24000
                for kv in (part.mime_type or "").split(";"):
                    if kv.strip().startswith("rate="):
                        rate = int(kv.split("=")[1])
                data = Vlm.pcm_to_wav(data, rate)
        except Exception as e:
            TRACE.add(job, "음성 답변 오류", f"{type(e).__name__}: {e}"[:300], level="warn",
                      ms=(time.perf_counter() - t0) * 1000)
            raise
        TRACE.add(job, "음성 답변 준비", f"WAV {len(data) / 1024:.0f} KB", level="ok",
                  ms=(time.perf_counter() - t0) * 1000)
        return data

    def transcribe(self, wav: bytes, job: Optional[str] = None) -> str:
        """웹에서 녹음한 음성(WAV)을 받아 적는다. cookbook quickstarts/Audio.ipynb 와 같은
        방식 - interactions 에 audio 를 base64 로 넣는다."""
        # 무음이면 모델을 부르지 않는다 - 조용한 녹음에서도 그럴듯한 문장을 지어내기 때문
        peak = self._voice_peak(wav)
        loud = peak >= VOICE_MIN_PEAK
        TRACE.add(job, "음량 검사", f"최대 {peak:.0f} / 기준 {VOICE_MIN_PEAK} - "
                  + ("통과" if loud else "너무 작아 모델을 부르지 않는다"), level="info" if loud else "warn")
        if not loud:
            self.log(f"      (음성이 너무 작다 - 최대 {peak:.0f} < {VOICE_MIN_PEAK})")
            return ""
        prompt = ("이 음성은 사람이 로봇에게 내리는 명령이다. 들리는 말을 그대로 받아 적어라. "
                  "한국어는 한국어로 적고, 받아 적은 문장 하나만 출력하라. "
                  "설명, 따옴표, 타임스탬프는 붙이지 마라. 사람 말소리가 분명히 들리지 않으면 "
                  "추측하지 말고 정확히 NONE 한 단어만 출력하라.")
        TRACE.add(job, "받아쓰기 요청", f"{self.stt_model} 에 WAV {len(wav) / 1024:.0f} KB 보냄",
                  detail=prompt)
        t0 = time.perf_counter()
        try:
            audio_b64 = base64.b64encode(wav).decode()
            res = self._call(lambda t: self.client.interactions.create(
                model=self.stt_model,
                input=[
                    {"type": "text", "text": prompt},
                    {"type": "audio", "data": audio_b64, "mime_type": "audio/wav"},
                ],
                timeout=t,
            ), STT_TIMEOUT, job, "받아쓰기")
        except Exception as e:
            TRACE.add(job, "받아쓰기 오류", f"{type(e).__name__}: {e}", level="error",
                      ms=(time.perf_counter() - t0) * 1000)
            raise
        dt = time.perf_counter() - t0
        self.log(f"      (받아쓰기 {dt:.1f}s)")
        raw = res.output_text or ""
        text = raw.strip().strip('"\'')
        text = "" if text.upper().rstrip(".") == "NONE" else text
        TRACE.add(job, "받아쓰기 응답", repr(text) if text else "말소리 없음 (NONE)",
                  level="ok" if text else "warn", ms=dt * 1000, detail=raw)
        return text

    @staticmethod
    def _voice_peak(wav: bytes) -> float:
        """16bit WAV 의 음량 - 상위 0.5% 샘플 크기 (튀는 잡음 한두 개는 무시)."""
        try:
            with wave.open(io.BytesIO(wav)) as w:
                if w.getsampwidth() != 2:
                    return float("inf")          # 모르는 형식이면 거르지 않는다
                return Vlm.pcm_peak(w.readframes(w.getnframes()))
        except (wave.Error, EOFError):
            return float("inf")

    @staticmethod
    def pcm_peak(pcm: bytes) -> float:
        a = np.frombuffer(pcm[:len(pcm) // 2 * 2], "<i2")
        return float(np.percentile(np.abs(a.astype(np.int32)), 99.5)) if a.size else 0.0

    @staticmethod
    def pcm_to_wav(pcm: bytes, rate: int = VOICE_RATE) -> bytes:
        b = io.BytesIO()
        with wave.open(b, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(rate)
            w.writeframes(pcm[:len(pcm) // 2 * 2])
        return b.getvalue()

    @staticmethod
    def _b64(bgr: np.ndarray) -> tuple[str, int, int]:
        h, w = bgr.shape[:2]
        small = cv2.resize(bgr, (SEND_WIDTH, int(SEND_WIDTH * h / w)),
                           interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".png", small)
        if not ok:
            raise RuntimeError("이미지 인코딩 실패")
        return base64.b64encode(buf.tobytes()).decode(), w, h

    def ask(self, bgr: np.ndarray, prompt: str, schema: Optional[dict] = None,
            job: Optional[str] = None, label: str = "") -> str:
        """label 은 처리 단계 로그의 단계 이름 앞에 붙는다 (예: '3단계 ')."""
        img_b64, w, h = self._b64(bgr)
        TRACE.add(job, f"{label}모델 요청",
                  f"{self.model} | 화면 {w}x{h} -> {SEND_WIDTH}px | 프롬프트 {len(prompt)}자"
                  + (" | JSON 스키마" if schema else ""), detail=prompt)
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
        try:
            res = self._call(lambda t: self.client.interactions.create(**body, timeout=t),
                             MODEL_TIMEOUT, job, f"{label}모델")
        except Exception as e:
            TRACE.add(job, f"{label}모델 오류", f"{type(e).__name__}: {e}", level="error",
                      ms=(time.perf_counter() - t0) * 1000)
            raise
        dt = time.perf_counter() - t0
        self.log(f"      (모델 {dt:.1f}s)")
        out = res.output_text or ""
        TRACE.add(job, f"{label}모델 응답", f"{len(out)}자 받음", ms=dt * 1000, detail=out)
        return out


class LiveStt:
    """음성 하나를 Gemini Live API 로 실시간 받아 적는다.

    브라우저가 0.2초마다 16 kHz PCM 조각을 보내면(feed) 그대로 Live 세션에 흘린다.
    서버가 0.5초마다 중간 결과(interim)를, 말이 끝나면(서버 VAD) 확정 결과를 준다 -
    text 는 '확정 + 지금 중간' 이라 말하는 동안 입력칸에 바로 보여줄 수 있다.
    연결은 녹음 시작과 함께 백그라운드에서 맺고, 그동안 온 조각은 큐에서 기다린다.
    Live 가 안 되면(finish 때) 쌓아 둔 PCM 을 일괄 모델로 받아 적는다.
    """

    def __init__(self, vlm: "Vlm", job: str):
        self.vlm, self.job = vlm, job
        self.q: "queue.Queue" = queue.Queue()     # bytes = 오디오, None = 끝, False = 취소
        self.lock = threading.Lock()
        self.pcm = bytearray()
        self.finals: List[str] = []
        self.interim = ""
        self.speaking = False                     # 서버 VAD: 말하는 중
        self.heard = False                        # 서버가 말소리를 한 번이라도 들었나
        self.error = ""
        self.connected = threading.Event()
        self.closed = threading.Event()
        self.t0 = time.perf_counter()
        self._first = True
        if vlm.live_model:
            TRACE.add(job, "서버 받아쓰기 시작", f"{vlm.live_model} 연결을 백그라운드로 맺는다 (그동안 조각은 큐에)")
            threading.Thread(target=self._run, daemon=True, name=f"live-{job}").start()
        else:
            self.error = "실시간 받아쓰기 꺼짐 (--live-model none)"
            TRACE.add(job, "서버 받아쓰기 시작", f"실시간 꺼짐 - 녹음이 끝나면 {vlm.stt_model} 로 한 번에")
            self.closed.set()

    # --- 웹 핸들러 스레드에서 부른다 -------------------------------------------
    @property
    def text(self) -> str:
        with self.lock:
            return " ".join(self.finals + ([self.interim] if self.interim else [])).strip()

    def feed(self, pcm: bytes) -> None:
        if len(self.pcm) > MAX_VOICE_BYTES:       # 상한을 넘으면 버린다 (브라우저도 30초에 멈춘다)
            return
        self.pcm += pcm
        if not self.closed.is_set():
            self.q.put(bytes(pcm))

    def cancel(self) -> None:
        self.q.put(False)

    def finish(self) -> str:
        """녹음이 끝났다. 확정 결과를 기다려 최종 문장을 돌려준다."""
        t0 = time.perf_counter()
        self.q.put(None)
        self.closed.wait(LIVE_END_WAIT + 3)
        sec = len(self.pcm) / 2 / VOICE_RATE
        text = self.text
        TRACE.add(self.job, "Live 끝", f"오디오 {sec:.1f}초, 확정 {len(self.finals)}개"
                  + (f", 오류: {self.error}" if self.error else ""),
                  ms=(time.perf_counter() - t0) * 1000, level="warn" if self.error else "info")
        peak = Vlm.pcm_peak(bytes(self.pcm))
        if peak < VOICE_MIN_PEAK:                 # 조용한 녹음에서 지어낸 문장은 버린다
            TRACE.add(self.job, "음량 검사", f"최대 {peak:.0f} / 기준 {VOICE_MIN_PEAK} - 너무 작아 버린다",
                      level="warn")
            return ""
        if not text and (self.error or not self.connected.is_set()) and self.pcm:
            TRACE.add(self.job, "일괄 받아쓰기로 대신", f"Live 를 못 써서 {self.vlm.stt_model} 로", level="warn")
            return self.vlm.transcribe(Vlm.pcm_to_wav(bytes(self.pcm)), self.job)
        return text

    # --- Live 세션 (자기 스레드의 asyncio) -------------------------------------
    def _run(self) -> None:
        try:
            asyncio.run(self._main())
        except Exception as e:                    # asyncio 자체 오류까지
            self.error = self.error or f"{type(e).__name__}: {e}"
        finally:
            self.closed.set()

    async def _main(self) -> None:
        cfg = {"response_modalities": ["TEXT"], "input_audio_transcription": {}}
        t0 = time.perf_counter()
        try:
            async with self.vlm.client.aio.live.connect(model=self.vlm.live_model, config=cfg) as s:
                self.connected.set()
                TRACE.add(self.job, "Live 연결", f"{self.vlm.live_model} (대기 중이던 조각 {self.q.qsize()}개)",
                          ms=(time.perf_counter() - t0) * 1000)
                recv = asyncio.create_task(self._recv(s))
                try:
                    if await self._send(s):       # 정상 끝 -> 확정 결과를 조금 기다린다
                        await self._settle(recv)
                finally:
                    recv.cancel()
        except Exception as e:
            self.error = f"{type(e).__name__}: {e}"
            TRACE.add(self.job, "Live 오류", self.error[:300], level="error",
                      ms=(time.perf_counter() - t0) * 1000)

    async def _send(self, s) -> bool:
        """오디오를 흘린다. 끝(None)이면 True, 취소/방치면 False."""
        loop = asyncio.get_running_loop()
        while True:
            try:
                item = await loop.run_in_executor(None, self.q.get, True, LIVE_IDLE_SEC)
            except queue.Empty:
                TRACE.add(self.job, "Live 닫음", f"{LIVE_IDLE_SEC:.0f}초 동안 브라우저 소식이 없다", level="warn",
                          end=True)
                return False
            if item is False:
                return False
            if item is None:
                await s.send_realtime_input(audio_stream_end=True)
                return True
            await s.send_realtime_input(
                audio=genai_types.Blob(data=item, mime_type=f"audio/pcm;rate={VOICE_RATE}"))

    async def _settle(self, recv: "asyncio.Task") -> None:
        """말이 끝나 확정됐으면 바로, 아니면 확정 결과(서버 VAD)를 LIVE_END_WAIT 까지 기다린다."""
        t_end = time.perf_counter() + LIVE_END_WAIT
        t_quiet = time.perf_counter() + 1.5       # 아무 소식도 없던 경우 잠깐만 더 본다
        while time.perf_counter() < t_end and not recv.done():
            with self.lock:
                pending = self.speaking or bool(self.interim)
                anything = self.heard or bool(self.finals)
            if not pending and (anything or time.perf_counter() > t_quiet):
                return
            await asyncio.sleep(0.05)

    async def _recv(self, s) -> None:
        while True:
            async for msg in s.receive():
                va = getattr(msg, "voice_activity", None)
                kind = str(getattr(va, "voice_activity_type", "") or "")
                if kind.endswith("START"):
                    with self.lock:
                        self.speaking = self.heard = True
                elif kind.endswith("END"):
                    with self.lock:
                        self.speaking = False
                sc = msg.server_content
                if not sc:
                    continue
                part = getattr(sc, "interim_input_transcription", None)
                if part and part.text:
                    with self.lock:
                        self.interim = part.text
                    if self._first:
                        self._first = False
                        TRACE.add(self.job, "첫 글자", part.text, ms=(time.perf_counter() - self.t0) * 1000)
                fin = sc.input_transcription
                if fin and fin.text:
                    with self.lock:
                        self.finals.append(fin.text.strip())
                        self.interim = ""
                    TRACE.add(self.job, "확정", fin.text.strip(), level="ok",
                              ms=(time.perf_counter() - self.t0) * 1000)


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

    def speak(self, text: str, job: Optional[str] = None) -> None:
        print(f"    (음성 답변) {text}")        # 터미널에는 소리 대신 글로

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
  button.mic { background: #24303c; border-color: #3d5a73; }
  button.mic.rec { background: #7a1f2b; border-color: #c0394d; animation: pulse 1s infinite; }
  button.mic.wait { background: #3a2d18; border-color: #7a5c22; cursor: progress; }
  @keyframes pulse { 50% { opacity: .65; } }
  .voice { display: flex; gap: 12px; align-items: center; flex-wrap: wrap;
           color: #8b939c; font-size: 12.5px; margin-top: 6px; }
  .voice input { flex: none; padding: 0; }
  #voiceMsg.err { color: #ff8a8a; }
  input.live { color: #9fc4e8; font-style: italic; }
  .tracebar { display: flex; gap: 14px; flex-wrap: wrap; align-items: center;
              color: #8b939c; font-size: 12.5px; margin: 8px 0; }
  .tracebar input { flex: none; padding: 0; }
  .tracebar a { color: #7ec8ff; }
  .tracebar button { padding: 2px 8px; font-size: 12px; }
  #traceActive div { color: #ffc46b; font-size: 12.5px; margin: 2px 0 6px; }
  #traceActive div::before { content: "⏳ "; }
  .tracewrap { max-height: 380px; overflow: auto; border-top: 1px solid #262a30; }
  #traceTbl td { word-break: break-word; }
  #traceTbl td.tt { white-space: nowrap; color: #8b939c; font-variant-numeric: tabular-nums; }
  #traceTbl td.st { white-space: nowrap; }
  #traceTbl td.dur { white-space: nowrap; color: #8b939c; text-align: right;
                     font-variant-numeric: tabular-nums; }
  #traceTbl th { position: sticky; top: 0; background: #0e1013; }
  .jt { display: inline-block; min-width: 34px; padding: 0 6px; border-radius: 4px;
        font-size: 11.5px; text-align: center; background: #2a2d33; color: #aab0b8; }
  tr.k-V .jt { background: #3a2550; color: #d6b4ff; }
  tr.k-O .jt { background: #1d3550; color: #7ec8ff; }
  tr.k-B .jt { background: #173c38; color: #7ee0cf; }
  tr.lv-ok td.st { color: #7ee08a; }
  tr.lv-warn td.st, tr.lv-warn td.msg { color: #ffc46b; }
  tr.lv-error td.st, tr.lv-error td.msg { color: #ff8a8a; }
  tr.end td { border-bottom: 1px solid #3a4250; }
  tr.has-det { cursor: pointer; }
  tr.has-det:hover td { background: #161a20; }
  .more { color: #7ec8ff; font-size: 11.5px; margin-left: 6px; white-space: nowrap; }
  tr.det pre { height: auto; max-height: 320px; margin: 2px 0 6px; }
  table.hideV tr.k-V, table.hideO tr.k-O, table.hideB tr.k-B, table.hideS tr.k-S { display: none; }
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
    <button type="button" id="mic" class="mic" onclick="toggleMic()"
            title="눌러서 말하고, 다시 눌러 끝낸다">🎤 음성 명령</button>
    <button type="button" onclick="quick('r')">화면 다시 보기</button>
    <button type="button" class="no" onclick="quick('q')">종료</button>
  </form>
  <div class="voice">
    <label><input type="checkbox" id="autoSend" onchange="saveAutoSend()">
      받아 적으면 바로 보내기</label>
    <label><input type="checkbox" id="speakOn" checked onchange="saveSpeakOn()">
      🔊 음성 답변 듣기</label>
    <span id="speakMsg"></span>
    <span id="voiceMsg"></span>
  </div>
  <div class="hint">지시 하나가 끝나면 화면을 새로 받고 다음 지시를 기다린다.
    pick/place 는 묻지 않고 바로 실행하고, 실패했을 때만 위에서 다시 할지 묻는다.</div>
  <details id="traceBox" open>
    <summary>처리 단계 로그 <span id="traceCount"></span></summary>
    <div class="tracebar">
      <span>보기:</span>
      <label><input type="checkbox" checked onchange="traceFilter('V', this.checked)"> 음성</label>
      <label><input type="checkbox" checked onchange="traceFilter('O', this.checked)"> 지시(AI)</label>
      <label><input type="checkbox" checked onchange="traceFilter('B', this.checked)"> 화면 설명</label>
      <label><input type="checkbox" checked onchange="traceFilter('S', this.checked)"> 시스템</label>
      <label><input type="checkbox" id="traceFollow" checked> 새 줄 따라가기</label>
      <a href="/trace.jsonl" download>로그 파일 받기 (llm_trace.jsonl)</a>
      <button type="button" onclick="clearTrace()">화면에서 지우기</button>
    </div>
    <div id="traceActive"></div>
    <div class="tracewrap" id="traceWrap">
      <table id="traceTbl">
        <thead><tr><th>시각</th><th>작업</th><th>단계</th><th>내용 (줄을 누르면 원문)</th>
          <th>걸린 시간 / 경과</th></tr></thead>
        <tbody id="traceBody"></tbody>
      </table>
    </div>
  </details>
  <details id="skills" open>
    <summary>등록된 스킬 <span id="skillCount"></span></summary>
    <div id="skillBody" class="hint">불러오는 중...</div>
  </details>
</div>
<script>
let since = 0, viewSeq = -1, asking = 0, tsince = 0, ssince = -1;
const $ = function (id) { return document.getElementById(id); };

async function poll() {
  try {
    const r = await fetch('/poll?since=' + since + '&tsince=' + tsince + '&ssince=' + ssince);
    const d = await r.json();
    since = d.seq;
    onTrace(d);
    onSpeech(d);
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

async function send(src) {
  const el = $('order');
  const v = el.value.trim();
  if (!v) return;
  el.value = '';
  await fetch('/order?text=' + encodeURIComponent(v) + (src ? '&src=' + src : ''));
}

// --- 처리 단계 로그 --------------------------------------------------------
// 서버가 /poll 로 새 줄을 준다. 작업(job)마다 마지막 줄을 기억해 '진행 중' 을 보여준다.
const TRACE_ROWS = 1500;
let traceJobs = {}, clockOff = 0, traceN = 0;

function fmtSec(ms) { return ms < 1000 ? ms + 'ms' : (ms / 1000).toFixed(1) + 's'; }

function onTrace(d) {
  if (d.now) clockOff = d.now - Date.now() / 1000;
  if (d.treset) { $('traceBody').innerHTML = ''; traceJobs = {}; traceN = 0; }
  tsince = d.tseq;
  const wrap = $('traceWrap');
  (d.trace || []).forEach(function (e) {
    traceJobs[e.job] = e;
    addTraceRow(e);
  });
  if (d.trace && d.trace.length && $('traceFollow').checked) wrap.scrollTop = wrap.scrollHeight;
  const body = $('traceBody');
  while (body.rows.length > TRACE_ROWS) body.deleteRow(0);
  renderActive();
}

function addTraceRow(e) {
  traceN++;
  const tr = document.createElement('tr');
  tr.className = 'k-' + e.job[0] + ' lv-' + e.level + (e.end ? ' end' : '') + (e.detail ? ' has-det' : '');
  const dur = (e.ms != null ? fmtSec(e.ms) : '') + ' <span class="hint">+' + e.el.toFixed(1) + 's</span>';
  tr.innerHTML = '<td class="tt">' + esc(e.t) + '</td><td><span class="jt" title="' + esc(e.kind) + '">'
    + esc(e.job) + '</span></td><td class="st">' + esc(e.stage) + '</td><td class="msg">' + esc(e.msg)
    + (e.detail ? '<span class="more">▸ 원문</span>' : '') + '</td><td class="dur">' + dur + '</td>';
  if (e.detail) {
    tr.onclick = function () {
      const nx = tr.nextSibling;
      if (nx && nx.classList.contains('det')) { nx.remove(); tr.querySelector('.more').textContent = '▸ 원문'; return; }
      const det = document.createElement('tr');
      det.className = 'det k-' + e.job[0];
      det.innerHTML = '<td colspan="5"><pre></pre></td>';
      det.querySelector('pre').textContent = e.detail;
      tr.after(det);
      tr.querySelector('.more').textContent = '▾ 접기';
    };
  }
  $('traceBody').appendChild(tr);
  $('traceCount').textContent = '(' + traceN + '줄)';
}

function renderActive() {
  // 끝나지 않은 작업 = 진행 중. 10분 넘게 소식이 없으면 버려진 것으로 본다.
  const now = Date.now() / 1000 + clockOff;
  let html = '';
  Object.keys(traceJobs).forEach(function (j) {
    const e = traceJobs[j];
    if (e.end || now - e.ts > 600) return;
    html += '<div><span class="jt">' + esc(j) + '</span> ' + esc(e.kind) + ' · ' + esc(e.stage)
          + ' 다음 단계 기다리는 중 ' + (now - e.ts).toFixed(1) + 's (작업 전체 '
          + (now - e.ts + e.el).toFixed(1) + 's)</div>';
  });
  $('traceActive').innerHTML = html;
}

function traceFilter(k, on) { $('traceTbl').classList.toggle('hide' + k, !on); }

function clearTrace() { $('traceBody').innerHTML = ''; traceN = 0; $('traceCount').textContent = ''; }

// 브라우저 쪽 음성 단계를 서버 로그에 남긴다 (실패해도 녹음은 계속)
function vtrace(job, stage, msg, level, end) {
  if (!job) return;
  fetch('/trace?job=' + job + '&stage=' + encodeURIComponent(stage) + '&msg=' + encodeURIComponent(msg || '')
        + '&level=' + (level || 'info') + (end ? '&end=1' : '')).catch(function () {});
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

// --- 음성 명령 (실시간 받아쓰기) -------------------------------------------
// 녹음을 시작하면 서버가 Gemini Live 세션을 연다(/voice_start). 마이크 소리를 16 kHz
// 16bit PCM 으로 바꿔 0.2초마다 /voice_chunk 로 흘리고, 응답에 오는 중간 결과를 입력칸에
// 바로 보여준다. 녹음이 끝나면 /voice_end 가 확정 문장을 돌려준다.
const VOICE_RATE = 16000, VOICE_MAX_SEC = 30;
// 말소리 감지 (조각마다 RMS): 말을 시작하지 않으면 VOICE_WAIT_SEC 뒤 녹음을 멈추고(버림),
// 말을 시작한 뒤 VOICE_TAIL_SEC 동안 조용하면 거기서 끝낸다.
const VOICE_WAIT_SEC = 5, VOICE_TAIL_SEC = 3;
const VOICE_LEVEL = 0.015;      // 이 RMS(약 -36 dBFS) 이상이고 배경 잡음의 3배를 넘으면 말소리
const VOICE_START_MS = 150;     // 이만큼 이어져야 말 시작으로 본다 (딸깍 소리 무시)
const VOICE_SEND_MS = 200;      // 서버로 흘리는 간격
const HTTPS_PORT = __HTTPS_PORT__;   // 서버가 채운다. 0 = https 없음
let rec = null;

function httpsUrl() {
  return 'https://' + location.hostname + ':' + HTTPS_PORT + location.pathname;
}

// 다른 PC 에서 http 로 들어오면 마이크가 막힌다 - https 주소를 미리 알려 준다
if (!window.isSecureContext) {
  if (HTTPS_PORT) {
    $('voiceMsg').innerHTML = '음성 명령은 <a href="' + httpsUrl() + '" style="color:#7ec8ff">'
      + httpsUrl() + '</a> 에서 (처음 한 번 인증서 경고에서 계속)';
  } else {
    voiceMsg('이 주소(http)에선 마이크를 쓸 수 없다 - 서버의 https 포트가 꺼져 있다', true);
  }
}

function voiceMsg(text, err) {
  $('voiceMsg').textContent = text || '';
  $('voiceMsg').className = err ? 'err' : '';
}

function micState(cls, label) {
  $('mic').className = 'mic' + (cls ? ' ' + cls : '');
  $('mic').textContent = label;
}

function showLive(text) {          // 말하는 동안의 중간 결과 (확정 전이라 흐리게)
  const el = $('order');
  el.value = text;
  el.classList.add('live');
}

// 마이크 샘플레이트(보통 48 kHz) -> 16 kHz Int16. 조각 경계에서 남는 샘플은 다음으로 넘긴다.
function resample(r, d) {
  const x = new Float32Array(r.carry.length + d.length);
  x.set(r.carry); x.set(d, r.carry.length);
  const ratio = r.ctx.sampleRate / VOICE_RATE, n = Math.floor(x.length / ratio);
  const out = new Int16Array(n);
  for (let i = 0; i < n; i++) {
    const a = Math.floor(i * ratio), b = Math.min(x.length, Math.floor((i + 1) * ratio));
    let s = 0;
    for (let j = a; j < b; j++) s += x[j];
    s = Math.max(-1, Math.min(1, b > a ? s / (b - a) : x[a]));
    out[i] = s < 0 ? s * 0x8000 : s * 0x7fff;
  }
  r.carry = x.slice(Math.floor(n * ratio));
  return out;
}

// 모인 PCM 을 한 번에 하나씩(순서 보장) 서버로 보낸다
function pumpVoice(r) {
  if (r.busy || !r.out.length) return;
  let n = 0;
  r.out.forEach(function (a) { n += a.length; });
  const buf = new Int16Array(n);
  let off = 0;
  r.out.forEach(function (a) { buf.set(a, off); off += a.length; });
  r.out = [];
  r.sent += n;
  r.busy = fetch('/voice_chunk?job=' + r.job, {method: 'POST',
      headers: {'Content-Type': 'application/octet-stream'}, body: buf.buffer})
    .then(function (res) { return res.json(); })
    .then(function (d) {
      if (d.text && !r.stopped) showLive(d.text);
      if (d.error && !r.warned) { r.warned = true; r.err = d.error; }
    })
    .catch(function () {})
    .finally(function () { r.busy = null; });
}

async function flushVoice(r) {
  while (r.busy) await r.busy;
  pumpVoice(r);
  while (r.busy) await r.busy;
}

async function toggleMic() {
  if ($('mic').classList.contains('wait')) return;
  if (rec) { stopMic('manual'); return; }
  if (!window.isSecureContext && HTTPS_PORT) {
    location.href = httpsUrl();          // 같은 화면의 https 쪽으로 옮긴다
    return;
  }
  if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
    voiceMsg('이 주소에선 마이크를 쓸 수 없다 - 브라우저는 https 또는 localhost(127.0.0.1) 에서만 마이크를 허용한다', true);
    return;
  }
  if (speaking) {                          // 답변을 읽는 중이면 끊고 듣는다
    if (speaking.pause) speaking.pause();
    if (window.speechSynthesis) speechSynthesis.cancel();
    speaking = null;
    speakMsg('');
  }
  micState('wait', '준비 중...');
  let stream, job = null;
  try {
    job = (await (await fetch('/trace_new')).json()).job;
    const st = await (await fetch('/voice_start?job=' + job)).json();
    if (!st.ok) throw new Error(st.error || '받아쓰기를 시작하지 못했다');
  } catch (e) {
    micState('', '🎤 음성 명령');
    voiceMsg('서버 받아쓰기를 시작하지 못했다: ' + e.message, true);
    return;
  }
  vtrace(job, '마이크 열기', '권한 요청');
  try {
    stream = await navigator.mediaDevices.getUserMedia(
      {audio: {channelCount: 1, echoCancellation: true, noiseSuppression: true}});
  } catch (e) {
    micState('', '🎤 음성 명령');
    voiceMsg('마이크를 열지 못했다: ' + e.message, true);
    vtrace(job, '마이크 오류', e.name + ': ' + e.message, 'error', true);
    fetch('/voice_cancel?job=' + job);
    return;
  }
  const ctx = new (window.AudioContext || window.webkitAudioContext)();
  const src = ctx.createMediaStreamSource(stream);
  const proc = ctx.createScriptProcessor(4096, 1, 1);
  const r = {stream: stream, ctx: ctx, src: src, proc: proc, t0: Date.now(), job: job,
             noise: VOICE_LEVEL / 3, run: 0, first: -1, lastAt: 0, n: 0,
             carry: new Float32Array(0), out: [], busy: null, sent: 0, stopped: false};
  proc.onaudioprocess = function (e) {
    const d = new Float32Array(e.inputBuffer.getChannelData(0));
    r.out.push(resample(r, d));
    r.n++;
    let s = 0;
    for (let i = 0; i < d.length; i++) s += d[i] * d[i];
    const rms = Math.sqrt(s / d.length), ms = d.length / ctx.sampleRate * 1000;
    if (rms > Math.max(VOICE_LEVEL, r.noise * 3)) {
      r.run += ms;
      if (r.first < 0 && r.run >= VOICE_START_MS) {
        r.first = r.n;
        vtrace(r.job, '말 감지', ((Date.now() - r.t0) / 1000).toFixed(1) + 's 에서 말 시작 (RMS '
               + rms.toFixed(3) + ', 배경 ' + r.noise.toFixed(3) + ')');
      }
      if (r.first >= 0) r.lastAt = Date.now();
    } else {
      r.run = 0;
      r.noise = r.noise * 0.95 + rms * 0.05;     // 조용한 조각으로만 배경 잡음을 따라간다
    }
  };
  src.connect(proc);
  proc.connect(ctx.destination);
  r.pump = setInterval(function () { pumpVoice(r); }, VOICE_SEND_MS);
  r.timer = setInterval(function () {
    const now = Date.now(), sec = (now - r.t0) / 1000;
    if (r.first < 0) {
      const left = Math.ceil(VOICE_WAIT_SEC - sec);
      micState('rec', '● 듣는 중 - 눌러서 취소');
      voiceMsg('말하세요... (' + Math.max(0, left) + '초 안에 말이 없으면 멈춘다)');
      if (sec >= VOICE_WAIT_SEC) stopMic('silent');
    } else {
      const quiet = (now - r.lastAt) / 1000;
      micState('rec', '● 받아 적는 중 ' + Math.floor(sec) + 's - 눌러서 끝내기');
      voiceMsg(quiet >= 1 ? '조용하면 ' + Math.max(0, Math.ceil(VOICE_TAIL_SEC - quiet)) + '초 뒤 끝낸다'
                          : (r.err ? '실시간 받아쓰기 안 됨 - 끝나면 한 번에 받아 적는다' : '듣고 있다...'));
      if (quiet >= VOICE_TAIL_SEC) stopMic('tail');
    }
    if (rec && sec >= VOICE_MAX_SEC) stopMic('max');
  }, 200);
  rec = r;
  micState('rec', '● 듣는 중 - 눌러서 취소');
  voiceMsg('말하세요...');
  vtrace(job, '녹음 시작', ctx.sampleRate + ' Hz -> 16 kHz 로 ' + VOICE_SEND_MS + 'ms 마다 서버로, '
         + VOICE_WAIT_SEC + '초 안에 말이 없으면 멈춘다');
}

async function stopMic(why) {
  const r = rec;
  rec = null;
  r.stopped = true;
  clearInterval(r.timer);
  clearInterval(r.pump);
  r.proc.disconnect(); r.src.disconnect();
  r.stream.getTracks().forEach(function (t) { t.stop(); });
  r.ctx.close();
  const total = ((Date.now() - r.t0) / 1000).toFixed(1);
  if (r.first < 0) {                     // 말소리가 한 번도 없었다 - 세션을 버린다
    const silent = why === 'silent';
    micState('', '🎤 음성 명령');
    $('order').classList.remove('live');
    voiceMsg(silent ? VOICE_WAIT_SEC + '초 동안 말이 없어 녹음을 멈췄다' : '말소리가 없어 취소했다', silent);
    vtrace(r.job, '녹음 끝', (silent ? VOICE_WAIT_SEC + '초 동안 말 없음' : '사용자가 취소')
           + ' - 버림 (' + total + 's)', 'warn', true);
    fetch('/voice_cancel?job=' + r.job);
    playNext();                            // 녹음 중 밀린 답변
    return;
  }
  const reason = {tail: VOICE_TAIL_SEC + '초 조용해서', max: VOICE_MAX_SEC + '초 상한', manual: '사용자가 끝냄'}[why] || why;
  micState('wait', '마무리 중...');
  await flushVoice(r);
  vtrace(r.job, '녹음 끝', reason + ' - 전체 ' + total + 's, 보낸 오디오 ' + (r.sent / VOICE_RATE).toFixed(1) + 's');
  finishVoice(r.job);
}

async function finishVoice(job) {
  voiceMsg('확정 결과를 받는 중...');
  try {
    const d = await (await fetch('/voice_end?job=' + job)).json();
    if (!d.ok) { voiceMsg(d.error || '받아쓰기 실패', true); return; }      // 서버가 이미 끝을 남겼다
    if (!d.text) {
      $('order').value = '';
      voiceMsg('말소리를 알아듣지 못했다 - 다시 해보세요', true);
      return;
    }
    $('order').value = d.text;
    if ($('autoSend').checked) {
      voiceMsg('보냄: ' + d.text);
      vtrace(job, '지시로 보냄', d.text, 'ok', true);
      send(job);
    } else {
      voiceMsg('받아 적었다 - 확인하고 [보내기] 를 누르세요');
      vtrace(job, '입력칸에 넣음', '사용자가 확인하고 [보내기] 를 누를 차례', 'ok', true);
      $('order').focus();
    }
  } catch (e) {
    voiceMsg('확정 결과를 받지 못했다: ' + e.message, true);
    vtrace(job, '결과 못 받음', e.message, 'error', true);
  } finally {
    $('order').classList.remove('live');
    micState('', '🎤 음성 명령');
    playNext();                            // 녹음 중 밀린 답변
  }
}

// --- 음성 답변 ------------------------------------------------------------
// 서버가 /poll 로 새 답변(번호, 글)을 알려 주면 차례로 /tts?id= 의 WAV 를 재생한다.
// 서버 음성이 없으면(실패/--tts-model none) 브라우저 내장 음성(ko-KR)으로 읽는다.
let speakQ = [], speaking = null;

function onSpeech(d) {
  if (ssince >= 0 && d.speech && $('speakOn').checked) {
    d.speech.forEach(function (it) { speakQ.push(it); });
    playNext();
  }
  if (d.sseq < ssince) speakQ = [];          // 서버가 다시 시작했다
  ssince = d.sseq;
}

function speakMsg(t) { $('speakMsg').textContent = t ? '🔊 ' + t : ''; }

function playNext() {
  if (speaking || !speakQ.length || rec) return;   // 녹음 중엔 기다린다 (마이크에 섞이지 않게)
  const it = speakQ.shift();
  const done = function () { speaking = null; speakMsg(''); playNext(); };
  speakMsg(it.text);
  const a = new Audio('/tts?id=' + it.id);
  speaking = a;
  a.onended = done;
  a.onerror = function () { speaking = null; sayBuiltin(it.text, done); };
  a.play().catch(function (e) {
    if (e.name === 'NotAllowedError') {
      speakMsg('브라우저가 자동 재생을 막았다 - 페이지를 한 번 누르면 들린다');
      speaking = null;
      document.addEventListener('click', playNext, {once: true});
      speakQ.unshift(it);
    }
  });
}

function sayBuiltin(text, done) {
  if (!window.speechSynthesis) { done(); return; }
  const u = new SpeechSynthesisUtterance(text);
  u.lang = 'ko-KR';
  u.onend = u.onerror = done;
  speaking = u;
  speechSynthesis.speak(u);
}

function stopSpeaking() {
  speakQ = [];
  if (speaking && speaking.pause) speaking.pause();
  if (window.speechSynthesis) speechSynthesis.cancel();
  speaking = null;
  speakMsg('');
}

function saveSpeakOn() {
  if (!$('speakOn').checked) stopSpeaking();
  try { localStorage.setItem('llm_speakOn', $('speakOn').checked ? '1' : '0'); } catch (e) {}
}

try { $('speakOn').checked = localStorage.getItem('llm_speakOn') !== '0'; } catch (e) {}

function saveAutoSend() {
  try { localStorage.setItem('llm_autoSend', $('autoSend').checked ? '1' : '0'); } catch (e) {}
}

try { $('autoSend').checked = localStorage.getItem('llm_autoSend') === '1'; } catch (e) {}

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
                page = PAGE.replace("__HTTPS_PORT__", str(ui.https_port))
                self._send(200, "text/html; charset=utf-8", page.encode("utf-8"))
            elif u.path == "/view.jpg":
                data = ui.view_bytes
                if data:
                    self._send(200, "image/jpeg", data)
                else:
                    self._send(404, "text/plain; charset=utf-8", b"no view yet")
            elif u.path == "/poll":
                try:
                    ssince = int(q.get("ssince", ["-1"])[0])
                except ValueError:
                    ssince = -1
                self._json(ui.snapshot(self._int(q, "since"), self._int(q, "tsince"), ssince))
            elif u.path == "/tts":                 # 음성 답변 WAV (만드는 중이면 기다렸다가)
                data = ui.speech_wav(self._int(q, "id"))
                if data:
                    self._send(200, "audio/wav", data)
                else:                              # 브라우저가 내장 음성으로 대신 읽는다
                    self._send(503, "text/plain; charset=utf-8", "음성 없음".encode("utf-8"))
            elif u.path == "/voice_start":
                self._json(ui.voice_start(q.get("job", [""])[0]))
            elif u.path == "/voice_end":
                self._json(ui.voice_end(q.get("job", [""])[0]))
            elif u.path == "/voice_cancel":
                self._json(ui.voice_cancel(q.get("job", [""])[0]))
            elif u.path == "/trace_new":           # 브라우저가 음성 녹음을 시작할 때 id 를 받는다
                self._json({"job": TRACE.new_job("V")})
            elif u.path == "/trace":               # 브라우저 쪽 음성 단계 (녹음 시작/말 감지/업로드 ...)
                job = q.get("job", [""])[0]
                if not valid_job(job, "V"):
                    self._json({"ok": False})
                    return
                level = q.get("level", ["info"])[0]
                TRACE.add(job, "[웹] " + q.get("stage", ["?"])[0][:40], q.get("msg", [""])[0][:300],
                          level=level if level in ("info", "ok", "warn", "error") else "info",
                          end=q.get("end", ["0"])[0] == "1")
                self._json({"ok": True})
            elif u.path == "/trace.jsonl":         # 처리 단계 로그 파일 내려받기
                try:
                    with open(TRACE.path, "rb") as f:
                        data = f.read()
                except OSError:
                    data = b""
                self.send_response(200)
                self.send_header("Content-Type", "application/x-ndjson; charset=utf-8")
                self.send_header("Content-Disposition", 'attachment; filename="llm_trace.jsonl"')
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
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
                    src = q.get("src", [""])[0]
                    ui.put_order(text, src if valid_job(src, "V") else "")
                self._json({"ok": bool(text)})
            elif u.path == "/answer":
                ok = ui.answer(self._int(q, "id"), q.get("v", ["n"])[0] == "y")
                self._json({"ok": ok})
            else:
                self._send(404, "text/plain; charset=utf-8",
                           "없는 주소".encode("utf-8"))

        def do_POST(self):
            u = urllib.parse.urlparse(self.path)
            q = urllib.parse.parse_qs(u.query)
            if u.path == "/voice_chunk":
                try:
                    n = int(self.headers.get("Content-Length", "0"))
                except ValueError:
                    n = -1
                if not 0 <= n <= MAX_VOICE_BYTES:
                    self.close_connection = True
                    self._json({"ok": False, "error": f"조각 크기가 이상하다 ({n})"})
                    return
                self._json(ui.voice_chunk(q.get("job", [""])[0], self.rfile.read(n)))
                return
            if u.path != "/voice":
                self._send(404, "text/plain; charset=utf-8", "없는 주소".encode("utf-8"))
                return
            try:
                n = int(self.headers.get("Content-Length", "0"))
            except ValueError:
                n = 0
            job = q.get("job", [""])[0]
            if n <= 44 or n > MAX_VOICE_BYTES:
                if 0 < n <= MAX_VOICE_BYTES * 4:
                    self.rfile.read(n)            # 연결을 깔끔히 끝내려고 버린다
                self.close_connection = True
                if valid_job(job, "V"):
                    TRACE.add(job, "서버 수신", f"크기가 이상해 거부 ({n} 바이트)", level="error", end=True)
                self._json({"ok": False, "error": f"음성 크기가 이상하다 ({n} 바이트)"})
                return
            self._json(ui.voice(self.rfile.read(n), job))

    return Handler


class _QuietHTTPServer(ThreadingHTTPServer):
    """자체 서명 인증서를 거부한 브라우저가 끊는 연결마다 스택트레이스를 찍지 않는다."""
    daemon_threads = True

    def handle_error(self, request, client_address):
        if isinstance(sys.exc_info()[1], (ssl.SSLError, ConnectionError, TimeoutError)):
            return
        super().handle_error(request, client_address)


def local_ips() -> List[str]:
    """이 기계의 바깥 IPv4 (접속 주소 안내와 인증서 SAN 에 쓴다)."""
    ips: List[str] = []
    try:                                       # 기본 경로의 IP (패킷은 실제로 안 나간다)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            ips.append(s.getsockname()[0])
    except OSError:
        pass
    try:                                       # 리눅스: 모든 인터페이스
        out = subprocess.run(["hostname", "-I"], capture_output=True, text=True, timeout=2).stdout
        ips += [ip for ip in out.split() if "." in ip]
    except (OSError, subprocess.SubprocessError):
        pass
    return [ip for i, ip in enumerate(ips) if ip not in ips[:i] and not ip.startswith("127.")]


def ensure_tls_cert(ips: List[str]) -> tuple[str, str]:
    """https 용 자체 서명 인증서. 없거나 지금 IP 가 빠져 있으면 새로 만든다 (10년)."""
    import datetime
    import ipaddress
    from cryptography import x509
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import ec
    from cryptography.x509.oid import NameOID

    want = {"127.0.0.1", *ips}
    try:
        with open(TLS_CERT, "rb") as f:
            cert = x509.load_pem_x509_certificate(f.read())
        have = {str(a) for a in cert.extensions.get_extension_for_class(
            x509.SubjectAlternativeName).value.get_values_for_type(x509.IPAddress)}
        if want <= have and os.path.exists(TLS_KEY):
            return TLS_CERT, TLS_KEY
    except (OSError, ValueError, x509.ExtensionNotFound):
        pass

    key = ec.generate_private_key(ec.SECP256R1())
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, f"llm_client {socket.gethostname()}")])
    now = datetime.datetime.now(datetime.timezone.utc)
    san = [x509.DNSName("localhost"), x509.DNSName(socket.gethostname())]
    san += [x509.IPAddress(ipaddress.ip_address(ip)) for ip in sorted(want)]
    cert = (x509.CertificateBuilder().subject_name(name).issuer_name(name)
            .public_key(key.public_key()).serial_number(x509.random_serial_number())
            .not_valid_before(now - datetime.timedelta(days=1))
            .not_valid_after(now + datetime.timedelta(days=3650))
            .add_extension(x509.SubjectAlternativeName(san), critical=False)
            .sign(key, hashes.SHA256()))
    fd = os.open(TLS_KEY, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "wb") as f:
        f.write(key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8,
                                  serialization.NoEncryption()))
    with open(TLS_CERT, "wb") as f:
        f.write(cert.public_bytes(serialization.Encoding.PEM))
    print(f"[web] https 인증서를 새로 만들었다: {TLS_CERT} ({', '.join(sorted(want))})")
    return TLS_CERT, TLS_KEY


class WebUi:
    """브라우저로 지시를 받고, 화면 / 로그 / 재시도 질문을 보여준다.

    에이전트는 메인 스레드에서 돌고(next_order 가 큐에서 막힌다), HTTP 는 데몬
    스레드에서 받는다. 둘 사이는 큐 하나와 Event 하나로만 만난다.
    """

    def __init__(self, port: int = WEB_PORT, host: str = "0.0.0.0",
                 state_fn=None, open_browser: bool = True, https_port: int = 0):
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
        self.transcribe_fn = None            # Vlm.transcribe - (WAV bytes, job) -> 글
        self.live_fn = None                  # Vlm.live - job -> LiveStt (실시간 받아쓰기)
        self.live: dict = {}                 # job -> 진행 중인 LiveStt
        self.tts_fn = None                   # Vlm.tts - (글, job) -> WAV. None 이면 브라우저 내장 음성
        self.speech: List[dict] = []         # 음성 답변 {id, text, job} (브라우저가 차례로 재생)
        self.speech_audio: dict = {}         # id -> WAV bytes / None(실패)
        self._speech_seq = 0
        self._speech_q: "queue.Queue" = queue.Queue()
        self._speech_ready = threading.Condition(self.lock)
        threading.Thread(target=self._speech_worker, daemon=True, name="tts").start()
        self.last_job: Optional[str] = None  # next_order 가 꺼낸 지시의 처리 단계 로그 id

        handler = _make_handler(self)
        self.servers = [_QuietHTTPServer((host, port), handler)]

        # 브라우저는 https 나 localhost 에서만 마이크를 열어 준다. 다른 PC 에서 음성 명령을
        # 쓰려면 https 가 필요해서, 자체 서명 인증서로 https 포트를 하나 더 연다.
        self.https_port = 0
        lan = local_ips() if host in ("0.0.0.0", "") else []
        if https_port:
            try:
                ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
                ctx.load_cert_chain(*ensure_tls_cert(lan or ([host] if host[:1].isdigit() else [])))
                tls = _QuietHTTPServer((host, https_port), handler)
                # 핸드셰이크는 accept 스레드가 아니라 요청 스레드에서 (느린 클라이언트가 막지 않게)
                tls.socket = ctx.wrap_socket(tls.socket, server_side=True,
                                             do_handshake_on_connect=False)
                self.servers.append(tls)
                self.https_port = https_port
            except (OSError, ssl.SSLError, ImportError, ValueError) as e:
                print(f"[web] https 포트 {https_port} 를 열지 못했다 ({type(e).__name__}: {e})"
                      " - 다른 PC 에서는 음성 명령을 쓸 수 없다")
        for srv in self.servers:
            threading.Thread(target=srv.serve_forever, daemon=True).start()

        local = "127.0.0.1" if host in ("0.0.0.0", "") else host
        self.url = f"http://{local}:{port}/"
        print(f"[web] 지시는 여기서 넣으세요: {self.url}")
        for ip in lan:
            print(f"[web]   다른 PC 에서: http://{ip}:{port}/"
                  + (f"   (음성 명령: https://{ip}:{self.https_port}/)" if self.https_port else ""))
        if self.https_port and lan:
            print("[web]   https 는 자체 서명 인증서라 처음 한 번 브라우저 경고에서 '계속' 을 눌러야 한다")
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

    def snapshot(self, since: int, tsince: int = 0, ssince: int = -1) -> dict:
        trace, reset = TRACE.since(tsince)
        with self.lock:
            since = max(0, min(since, len(self.lines)))
            out = {
                "seq": len(self.lines),
                "text": "".join(line + "\n" for line in self.lines[since:]),
                "view": self.view_seq,
                "status": self.status_text,
                "ask": dict(self.pending) if self.pending else None,
                "trace": trace,
                "treset": reset,
                "tseq": trace[-1]["seq"] if trace else (0 if reset else tsince),
                "now": time.time(),          # 브라우저 시계와 차이를 맞춘다 (진행 경과 시간)
                # 음성 답변: 처음(ssince<0)엔 번호만 알려 준다 - 새로 연 탭이 지난 답변을 읽지 않게
                "speech": [] if ssince < 0 else
                          [{"id": it["id"], "text": it["text"]} for it in self.speech if it["id"] > ssince],
                "sseq": self._speech_seq,
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
    def voice(self, wav: bytes, job: str = "") -> dict:
        """웹에서 녹음한 음성 명령을 받아 적는다. 보내기는 브라우저가 한다.
        job: 브라우저가 녹음을 시작할 때 받은 처리 단계 로그 id."""
        job = job if valid_job(job, "V") else TRACE.new_job("V")
        try:
            with wave.open(io.BytesIO(wav)) as w:
                info = f"{w.getnframes() / w.getframerate():.1f}초, {w.getframerate()} Hz"
        except (wave.Error, EOFError, ZeroDivisionError):
            info = "WAV 형식 아님"
        TRACE.add(job, "서버 수신", f"{len(wav) / 1024:.0f} KB ({info})")
        if self.transcribe_fn is None:
            TRACE.add(job, "실패", "아직 준비 중 (모델 연결 전)", level="error", end=True)
            return {"ok": False, "error": "아직 준비 중이다 - 잠시 뒤에 다시"}
        t0 = time.perf_counter()
        try:
            text = self.transcribe_fn(wav, job)
        except Exception as e:
            self.log(f"[음성] 받아쓰기 실패: {type(e).__name__}: {e}")
            TRACE.add(job, "실패", f"{type(e).__name__}: {e}", level="error", end=True,
                      ms=(time.perf_counter() - t0) * 1000)
            return {"ok": False, "error": f"받아쓰기 실패: {type(e).__name__}: {e}"}
        self.log(f"[음성] {text or '(알아듣지 못함)'}")
        if not text:                         # 브라우저는 더 할 일이 없다 - 여기서 끝
            TRACE.add(job, "끝", "알아듣지 못함", level="warn", end=True,
                      ms=(time.perf_counter() - t0) * 1000)
        return {"ok": True, "text": text, "job": job}

    # --- 음성 답변 ------------------------------------------------------------
    def speak(self, text: str, job: Optional[str] = None) -> None:
        """한 문장을 소리로 들려준다. 음성은 뒤에서 만들고, 브라우저가 /poll 로 알고 /tts 로 받는다."""
        text = (text or "").strip()
        if not text:
            return
        with self.lock:
            self._speech_seq += 1
            item = {"id": self._speech_seq, "text": text, "job": job}
            self.speech.append(item)
            del self.speech[:-TTS_KEEP]
        TRACE.add(job, "음성 답변", text)
        self.log(f"      🔊 {text}")
        self._speech_q.put(item)

    def _speech_worker(self) -> None:
        """음성을 차례로 만든다 (한 번에 하나 - 순서가 지켜지고 API 를 몰아치지 않는다)."""
        while True:
            item = self._speech_q.get()
            data = None
            if self.tts_fn is not None:
                try:
                    data = self.tts_fn(item["text"], item["job"])
                except Exception:
                    data = None                # 브라우저가 내장 음성으로 읽는다
            with self.lock:
                self.speech_audio[item["id"]] = data
                for old in [k for k in self.speech_audio if k <= item["id"] - TTS_KEEP]:
                    del self.speech_audio[old]
                self._speech_ready.notify_all()

    def speech_wav(self, sid: int, wait: float = 20.0) -> Optional[bytes]:
        """그 음성 답변의 WAV. 아직 만드는 중이면 wait 초까지 기다린다. 실패/없음이면 None."""
        with self.lock:
            if not any(it["id"] == sid for it in self.speech):
                return None
            self._speech_ready.wait_for(lambda: sid in self.speech_audio, timeout=wait)
            return self.speech_audio.get(sid)

    # --- 실시간 음성 (녹음 시작 -> 조각들 -> 끝/취소) ------------------------------
    def voice_start(self, job: str) -> dict:
        if not valid_job(job, "V"):
            return {"ok": False, "error": "잘못된 음성 id"}
        if self.live_fn is None:
            TRACE.add(job, "실패", "아직 준비 중 (모델 연결 전)", level="error", end=True)
            return {"ok": False, "error": "아직 준비 중이다 - 잠시 뒤에 다시"}
        with self.lock:
            for j in [j for j, ls in self.live.items() if ls.closed.is_set() or j == job]:
                self.live.pop(j).cancel()         # 끝난 세션 / 같은 id 재시작 정리
            while len(self.live) >= 3:            # 탭 여러 개가 동시에 녹음하는 경우의 상한
                self.live.pop(next(iter(self.live))).cancel()
            self.live[job] = self.live_fn(job)
        return {"ok": True}

    def voice_chunk(self, job: str, pcm: bytes) -> dict:
        with self.lock:
            ls = self.live.get(job)
        if ls is None:
            return {"ok": False, "error": "받아쓰기 세션이 없다 (서버 재시작?)"}
        ls.feed(pcm)
        return {"ok": True, "text": ls.text, "live": ls.connected.is_set(), "error": ls.error}

    def voice_end(self, job: str) -> dict:
        with self.lock:
            ls = self.live.pop(job, None)
        if ls is None:
            return {"ok": False, "error": "받아쓰기 세션이 없다 (서버 재시작?)"}
        t0 = time.perf_counter()
        try:
            text = ls.finish()
        except Exception as e:
            self.log(f"[음성] 받아쓰기 실패: {type(e).__name__}: {e}")
            TRACE.add(job, "실패", f"{type(e).__name__}: {e}", level="error", end=True,
                      ms=(time.perf_counter() - t0) * 1000)
            return {"ok": False, "error": f"받아쓰기 실패: {type(e).__name__}: {e}"}
        self.log(f"[음성] {text or '(알아듣지 못함)'}")
        if not text:
            TRACE.add(job, "끝", "알아듣지 못함", level="warn", end=True)
        return {"ok": True, "text": text, "job": job}

    def voice_cancel(self, job: str) -> dict:
        with self.lock:
            ls = self.live.pop(job, None)
        if ls:
            ls.cancel()
        return {"ok": True}

    def put_order(self, text: str, src: str = "") -> None:
        """src: 음성 입력에서 바로 보낸 지시면 그 처리 단계 로그 id (V3 ...)."""
        job = None
        if text.strip().lower() not in ("r", "q", "quit", "exit", "종료"):
            job = TRACE.new_job("O")
            TRACE.add(job, "지시 접수", text + (f"  (음성 {src} 에서)" if src else "")
                      + (f" - 앞에 {self.orders.qsize()}개 대기" if self.orders.qsize() else ""))
        self.orders.put((text, job))

    def next_order(self) -> Optional[str]:
        self.set_status("")
        try:
            text, self.last_job = self.orders.get()
            return text
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
            for srv in self.servers:
                srv.shutdown()
                srv.server_close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# 5. 에이전트 루프
# ---------------------------------------------------------------------------

class Agent:
    def __init__(self, api: Api, vlm: Vlm, ui, auto: bool = False, base: Optional[MapApi] = None):
        self.api, self.vlm, self.ui, self.auto, self.base = api, vlm, ui, auto, base
        self.frame: Optional[np.ndarray] = None
        self.job: Optional[str] = None       # 지금 처리 중인 일의 처리 단계 로그 id (O3, B1 ...)
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

    def speak(self, text: Optional[str], job: Optional[str] = None) -> None:
        """사용자에게 음성으로 한 문장 (웹이면 브라우저가 재생, 터미널이면 글로)."""
        if text and text.strip() and hasattr(self.ui, "speak"):
            self.ui.speak(text.strip(), job)

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
        t0 = time.perf_counter()
        frame = self.api.rgb()
        if frame is None:
            self.ui.log("[error] /get_rgb 실패 - test4.py 가 떠 있는지 확인하세요")
            TRACE.add(self.job, "화면 받기", "/get_rgb 실패", level="error",
                      ms=(time.perf_counter() - t0) * 1000)
            return False
        self.frame = frame
        cv2.imwrite(VIEW_PATH, frame)
        self.ui.set_view(frame)
        h, w = frame.shape[:2]
        self.ui.log(f"[view] 현재 화면 {w}x{h} (저장: {VIEW_PATH})")
        TRACE.add(self.job, "화면 받기", f"{w}x{h}", ms=(time.perf_counter() - t0) * 1000)
        return True

    def to_pixel(self, point: List[int]) -> tuple[int, int]:
        """모델의 [y, x](0~1000) -> 실제 픽셀 (x, y)."""
        h, w = self.frame.shape[:2]
        y = int(round(point[0] / 1000.0 * (h - 1)))
        x = int(round(point[1] / 1000.0 * (w - 1)))
        return max(0, min(w - 1, x)), max(0, min(h - 1, y))

    # --- 첫 인사: 무엇이 보이고 무엇을 할 수 있는지 -------------------------
    def brief(self) -> str:
        job = self.job = TRACE.new_job("B")
        TRACE.add(job, "시작", "로봇 상태를 읽고 화면 설명을 요청한다")
        t0 = time.perf_counter()
        try:
            out = self._brief(job)
        except Exception as e:
            TRACE.add(job, "실패", f"{type(e).__name__}: {e}", level="error", end=True,
                      ms=(time.perf_counter() - t0) * 1000)
            self.job = None
            raise
        TRACE.add(job, "완료", (out.strip().splitlines() or [""])[0][:120], level="ok",
                  ms=(time.perf_counter() - t0) * 1000, detail=out, end=True)
        self.job = None
        return out

    def _brief(self, job: str) -> str:
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
        return self.vlm.ask(self.frame, prompt, job=job)

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
            TRACE.add(self.job, f"{action} 실패", f"({x},{y}) {attempt}번째 시도: {why}", level="warn")
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
        TRACE.add(self.job, "재시도 질문", "사용자의 예/아니오를 기다린다")
        t0 = time.perf_counter()
        yes = self.ui.ask(f"{action}({x},{y}) 실패: {why} / 다시 할까요?")
        TRACE.add(self.job, "재시도 답", "예 - 다시 한다" if yes else "아니오",
                  ms=(time.perf_counter() - t0) * 1000)
        return yes

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
    def follow(self, order: str, job: Optional[str] = None) -> None:
        """job: 웹에서 지시를 받을 때 이미 붙인 처리 단계 로그 id (없으면 새로)."""
        job = self.job = job or TRACE.new_job("O")
        TRACE.add(job, "지시 시작", order)
        t0 = time.perf_counter()
        try:
            self._follow(order, job, t0)
        except BaseException as e:           # Ctrl-C 도 '끝났다' 는 줄을 남긴다
            TRACE.add(job, "중단", f"{type(e).__name__}: {e}", level="error", end=True,
                      ms=(time.perf_counter() - t0) * 1000)
            raise
        finally:
            self.job = None                  # 끝난 일에 화면 갱신 같은 줄이 붙지 않게

    def _follow(self, order: str, job: str, t0: float) -> None:
        history: list[str] = []
        for step in range(1, MAX_STEPS + 1):
            state = self.api.json("/get_state")
            self.ui.set_status(f"생각 중 ({step}/{MAX_STEPS})")
            prompt = (
                f"{self.skill_doc()}\n\n"
                f"사용자 지시: {order}\n"
                f"로봇 상태: {state.get('state')} ({state.get('label')})\n"
                + ("지금까지 한 일:\n" + "\n".join(history) + "\n" if history else "")
                + ("이번이 이 지시의 첫 스킬이다 - speech 에 무엇을 할지 짧은 한 문장을 꼭 써라.\n"
                   if step == 1 else "")
                + "done 을 부른다면 speech 에 결과를 짧은 한 문장으로 꼭 써라.\n"
                + "다음에 부를 스킬 하나를 JSON 으로 내라."
            )
            TRACE.add(job, f"{step}단계 판단", f"로봇 상태 {state.get('state')} ({state.get('label')})"
                      f", 지금까지 스킬 {len(history)}개")
            raw = ""
            try:
                raw = self.vlm.ask(self.frame, prompt, SkillCall.model_json_schema(),
                                   job=job, label=f"{step}단계 ")
                call = SkillCall(**json.loads(raw))
            except Exception as e:
                self.ui.log(f"[error] 모델 응답을 읽지 못했다: {type(e).__name__}: {e}")
                quota = quota_message(e)
                if quota:
                    self.ui.log(f"[error] {quota}")
                self.speak("오늘 쓸 수 있는 모델 사용량을 다 써서 지금은 할 수 없어요." if quota
                           else "모델 응답이 너무 늦어 멈췄어요. 다시 말씀해 주세요."
                           if isinstance(e, ModelTimeout) else "죄송해요, 지금은 처리하지 못했어요.", job)
                TRACE.add(job, f"{step}단계 응답 해석 실패", f"{type(e).__name__}: {e}", level="error",
                          detail=raw, end=True, ms=(time.perf_counter() - t0) * 1000)
                return

            arg = "".join(f" {v}" for v in (call.point, call.label, call.meters, call.degrees, call.gap)
                          if v is not None)
            self.ui.log(f"  [{step}] {call.skill}{arg}  - {call.reason}")
            if call.message:
                self.ui.log(f"      말: {call.message}")
            TRACE.add(job, f"{step}단계 스킬 선택", f"{call.skill}{arg} - {call.reason}"
                      + (f" / 말: {call.message}" if call.message else ""))
            # 음성 답변: 첫 스킬에서 할 일, say 의 말, done 에서 결과 - 한 문장씩만
            # (모델이 speech 를 비우기도 해서 비었을 때 할 말을 정해 둔다)
            if call.skill == "done":
                self.speak(call.speech or call.message or call.reason, job)
            elif call.skill == "say":
                self.speak(call.message or call.speech, job)
            elif step == 1:
                self.speak(call.speech or "네, 시작할게요.", job)

            ts = time.perf_counter()
            result = self.run_skill(call)
            res_txt = json.dumps(result, ensure_ascii=False)
            self.ui.log(f"      결과: {res_txt[:200]}")
            TRACE.add(job, f"{step}단계 스킬 결과",
                      f"{call.skill} " + ("성공" if result.get("ok") else "실패")
                      + (f" - {result.get('message')}" if result.get("message") else ""),
                      level="ok" if result.get("ok") else "warn",
                      ms=(time.perf_counter() - ts) * 1000, detail=res_txt)
            history.append(f"- {call.skill}{arg} -> {res_txt[:160]}")

            if call.skill == "done":
                TRACE.add(job, "지시 완료", call.message or "", level="ok", end=True,
                          ms=(time.perf_counter() - t0) * 1000)
                return
            if call.skill in ("pick", "place") and result.get("ok"):
                self.look()          # 움직였으면 화면이 바뀐다
            if call.skill in BASE_MOVING and (result.get("moved_m") or result.get("turned_deg")
                                              or result.get("state") in ("succeeded", "done")):
                self.look()          # 몸체가 움직였으면 카메라가 보는 곳이 바뀐다
        self.ui.log("[warn] 스킬 호출 상한에 도달했다 - 지시를 더 구체적으로 주세요")
        self.speak("단계가 너무 많아 멈췄어요. 조금 더 구체적으로 말씀해 주세요.", job)
        TRACE.add(job, "상한 도달", f"스킬 {MAX_STEPS}번을 불렀는데 끝나지 않았다", level="warn",
                  end=True, ms=(time.perf_counter() - t0) * 1000)


# ---------------------------------------------------------------------------
# 6. 실행
# ---------------------------------------------------------------------------

def safe_brief(agent: Agent) -> str:
    """화면 설명이 실패해도(시간 초과 등) 프로그램은 계속 지시를 받는다."""
    try:
        return agent.brief()
    except Exception as e:
        quota = quota_message(e)
        if quota:
            return f"[error] 화면 설명을 받지 못했다 - {quota}"
        return f"[error] 화면 설명을 받지 못했다 ({type(e).__name__}: {e}) - 지시는 그대로 할 수 있다"


def run(agent: Agent, api: Api, ui, first_order: str = "") -> None:
    """지시 하나를 끝내면 화면을 새로 받고 다음 지시를 기다린다."""
    if not agent.look():
        return

    ui.set_status("화면 살펴보는 중")
    ui.log("[화면 설명]")
    ui.log(safe_brief(agent))
    ui.log("")
    ui.log("무엇을 시킬까요? ('r' 화면 다시 보기, 'q' 종료)")

    pending = (first_order or "").strip()
    while True:
        if pending:
            order, pending = pending, ""
            ui.log("")
            ui.log(f"[지시] {order}")
            try:
                agent.follow(order, getattr(ui, "last_job", None))
            except Exception as e:             # 한 지시가 실패해도 다음 지시는 받는다
                ui.log(f"[error] 지시를 처리하다 멈췄다: {type(e).__name__}: {e}")
                agent.speak("문제가 생겨 지시를 멈췄어요.")
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
            ui.log(safe_brief(agent))
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
    ap.add_argument("--host", default="0.0.0.0",
                    help="웹 입력 UI 바인드 주소 (기본 0.0.0.0 = 다른 PC 에서도 접속. "
                         "이 기계에서만 쓰려면 127.0.0.1)")
    ap.add_argument("--https-port", type=int, default=None,
                    help="음성 명령용 https 포트 (자체 서명 인증서. 기본 --port + 1, 0 = 끔)")
    ap.add_argument("--no-browser", action="store_true",
                    help="브라우저를 자동으로 열지 않는다")
    ap.add_argument("--map-api", default="",
                    help=f"이동 로봇 map_web REST 주소 (기본: --api 와 같은 호스트의 {MAP_WEB_PORT} 포트)")
    ap.add_argument("--no-base", action="store_true",
                    help="이동 로봇 스킬을 쓰지 않는다 (팔만)")
    ap.add_argument("--live-model", default=LIVE_STT_MODEL_ID,
                    help=f"웹 음성 명령 실시간 받아쓰기 모델 (Live API, 기본 {LIVE_STT_MODEL_ID}, "
                         "환경변수 GEMINI_LIVE_STT_MODEL, none = 끄고 일괄만)")
    ap.add_argument("--tts-model", default=TTS_MODEL_ID,
                    help=f"음성 답변 모델 (기본 {TTS_MODEL_ID}, 환경변수 GEMINI_TTS_MODEL, "
                         "none = 브라우저 내장 음성으로)")
    ap.add_argument("--stt-model", default=STT_MODEL_ID,
                    help=f"Live 를 못 쓸 때의 일괄 받아쓰기 모델 (기본 {STT_MODEL_ID}, 환경변수 GEMINI_STT_MODEL)")
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
                       open_browser=not args.no_browser,
                       https_port=args.port + 1 if args.https_port is None else args.https_port)
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

    TRACE.add(TRACE.new_job("S"), "시작",
              f"모델 {MODEL_ID}, 실시간 받아쓰기 {args.live_model}, 일괄 받아쓰기 {args.stt_model}, "
              f"음성 답변 {args.tts_model}, 팔 {args.api}, "
              f"이동 로봇 {args.map_api if base else '없음'}, 로그 {TRACE.path}", end=True)
    vlm = Vlm(key, ui.log, stt_model=args.stt_model,
              live_model=None if args.live_model.lower() in ("", "none", "off") else args.live_model,
              tts_model=None if args.tts_model.lower() in ("", "none", "off") else args.tts_model)
    agent = Agent(api, vlm, ui, auto=args.auto, base=base)
    if isinstance(ui, WebUi):
        ui.toggle_fn = agent.set_skills
        ui.skills_fn = agent.skill_groups
        ui.transcribe_fn = vlm.transcribe
        ui.live_fn = vlm.live
        ui.tts_fn = vlm.tts if vlm.tts_model else None
    try:
        run(agent, api, ui, " ".join(args.order))
    except KeyboardInterrupt:
        print()
    finally:
        ui.close()


if __name__ == "__main__":
    main()
