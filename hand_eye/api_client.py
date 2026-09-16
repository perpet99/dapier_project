"""
api_client.py - test4.py 의 REST API 를 눈으로 확인하는 테스트 클라이언트.

무엇을 하나:
  [상태]        GET /get_state   - 지금 무엇을 하는 중인지
  [RGB]         GET /get_rgb     - 카메라 영상을 받아 화면에 띄운다
  [Depth]       GET /get_depth   - 16bit PNG(mm) 를 받아 컬러맵으로 띄운다
  이미지 클릭   GET /get_depth?x=&y= - 그 지점의 depth/로봇좌표/집기 가능 여부
  [PICK]        GET /pick?x=&y=  - 클릭한 지점을 집는다
  [PLACE]       GET /place?x=&y= - 클릭한 지점에 놓는다

좌표는 받은 영상의 픽셀 그대로다. 서버의 /get_rgb, /get_depth, /pick 이 모두
같은 좌표계를 쓰므로 화면에서 클릭한 자리가 곧 로봇이 갈 자리다.

의존성은 tkinter(표준) + cv2/numpy(이미 쓰는 것)뿐이다. tkinter 를 쓰는 이유는
한글이 제대로 보여서다 - cv2 창은 한글을 네모로 그린다.

사용:
  python hand_eye/api_client.py [http://127.0.0.1:8765]
"""

from __future__ import annotations

import json
import queue
import sys
import threading
import tkinter as tk
import urllib.error
import urllib.parse
import urllib.request
from tkinter import ttk

import cv2
import numpy as np

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass

DEFAULT_API = "http://127.0.0.1:8765"
TIMEOUT = 5.0


# ---------------------------------------------------------------------------
# REST 호출
# ---------------------------------------------------------------------------

class Api:
    def __init__(self, base: str):
        self.base = base.rstrip("/")

    def _url(self, path: str, params: dict | None) -> str:
        q = f"?{urllib.parse.urlencode(params)}" if params else ""
        return f"{self.base}{path}{q}"

    def raw(self, path: str, params: dict | None = None) -> tuple[int, dict, bytes]:
        """(상태코드, 헤더, 본문). HTTP 오류도 예외 대신 그대로 돌려준다 -
        409/400 의 본문에 거부 사유가 들어 있어서 그게 정보다."""
        try:
            with urllib.request.urlopen(self._url(path, params), timeout=TIMEOUT) as r:
                return r.status, dict(r.headers), r.read()
        except urllib.error.HTTPError as e:
            return e.code, dict(e.headers), e.read()

    def json(self, path: str, params: dict | None = None) -> tuple[int, dict]:
        code, _, body = self.raw(path, params)
        try:
            return code, json.loads(body.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            return code, {"ok": False, "error": body[:200].decode("utf-8", "replace")}


# ---------------------------------------------------------------------------
# UI
# ---------------------------------------------------------------------------

class ClientUI(tk.Tk):
    def __init__(self, base: str):
        super().__init__()
        self.api = Api(base)
        self.title(f"SO-101 REST 테스트 클라이언트 - {base}")

        self.frame_bgr: np.ndarray | None = None    # 지금 띄운 영상
        self.depth_mm: np.ndarray | None = None     # depth 를 띄웠을 때의 원값 [mm]
        self.sel: tuple[int, int] | None = None     # 클릭한 좌표
        self.tk_img: tk.PhotoImage | None = None    # GC 방지용 참조
        self._results: queue.Queue = queue.Queue()  # 작업 스레드 -> 메인 스레드

        self._build()
        self._drain()
        self.auto_poll()

    # --- 위젯 -------------------------------------------------------------
    def _build(self) -> None:
        pad = {"padx": 4, "pady": 4}

        top = ttk.Frame(self)
        top.grid(row=0, column=0, sticky="ew", **pad)
        ttk.Label(top, text="서버").pack(side="left")
        self.url_var = tk.StringVar(value=self.api.base)
        ttk.Entry(top, textvariable=self.url_var, width=30).pack(side="left", padx=4)
        ttk.Button(top, text="상태 가져오기", command=self.on_state).pack(side="left", padx=2)
        ttk.Button(top, text="RGB 가져오기", command=self.on_rgb).pack(side="left", padx=2)
        ttk.Button(top, text="Depth 가져오기", command=self.on_depth).pack(side="left", padx=2)
        self.auto_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(top, text="상태 자동갱신(1초)",
                        variable=self.auto_var).pack(side="left", padx=8)

        self.state_var = tk.StringVar(value="- 상태를 가져오세요 -")
        ttk.Label(self, textvariable=self.state_var, font=("Malgun Gothic", 14, "bold"),
                  anchor="w").grid(row=1, column=0, sticky="ew", padx=8)

        self.canvas = tk.Canvas(self, width=640, height=480, bg="#202428",
                                highlightthickness=0)
        self.canvas.grid(row=2, column=0, **pad)
        self.canvas.bind("<Button-1>", self.on_click)
        self.canvas.create_text(320, 240, text="RGB / Depth 를 가져오면 여기에 보입니다",
                                fill="#8b969e", font=("Malgun Gothic", 11))

        act = ttk.Frame(self)
        act.grid(row=3, column=0, sticky="ew", **pad)
        self.pick_var = tk.StringVar(value="선택된 좌표 없음 - 이미지를 클릭하세요")
        ttk.Label(act, textvariable=self.pick_var, width=64,
                  font=("Malgun Gothic", 10)).pack(side="left")
        ttk.Button(act, text="PICK", command=lambda: self.on_action("pick")).pack(side="left", padx=3)
        ttk.Button(act, text="PLACE", command=lambda: self.on_action("place")).pack(side="left", padx=3)
        ttk.Button(act, text="지우기", command=self.clear_log).pack(side="left", padx=12)

        box = ttk.Frame(self)
        box.grid(row=4, column=0, sticky="nsew", **pad)
        self.log = tk.Text(box, height=12, width=92, font=("Consolas", 9),
                           bg="#16191c", fg="#dfe6ea", insertbackground="#dfe6ea")
        sb = ttk.Scrollbar(box, command=self.log.yview)
        self.log.configure(yscrollcommand=sb.set)
        self.log.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

        self.rowconfigure(4, weight=1)
        self.columnconfigure(0, weight=1)

    # --- 유틸 -------------------------------------------------------------
    def clear_log(self) -> None:
        self.log.delete("1.0", "end")

    def say(self, *lines: str) -> None:
        for line in lines:
            self.log.insert("end", line + "\n")
        self.log.see("end")

    def say_json(self, title: str, code: int, obj: dict) -> None:
        mark = "OK " if 200 <= code < 300 else "ERR"
        self.say(f"[{mark} {code}] {title}",
                 json.dumps(obj, ensure_ascii=False, indent=2), "")

    def _async(self, work, done) -> None:
        """HTTP 는 별도 스레드에서. 팔이 움직이는 동안 UI 가 멈추면 안 된다.

        결과는 큐에 넣고 메인 스레드가 꺼내 쓴다. tkinter 는 스레드 안전하지
        않아서 작업 스레드에서 위젯을 건드리면(after 조차) 터진다.
        """
        def run():
            try:
                res = work()
            except Exception as e:                     # 연결 거부/타임아웃 등
                res = ("error", f"{type(e).__name__}: {e}")
            self._results.put((done, res))
        threading.Thread(target=run, daemon=True).start()

    def _drain(self) -> None:
        """메인 스레드에서 결과를 꺼내 콜백을 돌린다."""
        while True:
            try:
                done, res = self._results.get_nowait()
            except queue.Empty:
                break
            try:
                done(res)
            except Exception as e:
                self.say(f"콜백 오류: {type(e).__name__}: {e}", "")
        self.after(50, self._drain)

    def _api(self) -> Api:
        """주소를 읽어 갱신한 Api 를 돌려준다.

        반드시 **메인 스레드에서** 부를 것. tkinter 변수를 작업 스레드에서 읽으면
        Tcl 인터프리터를 기다리다 그대로 멈춘다(예외도 안 난다).
        """
        self.api.base = self.url_var.get().rstrip("/")   # 주소를 바꿔도 바로 반영
        return self.api

    # --- 이미지 -----------------------------------------------------------
    def show(self, bgr: np.ndarray) -> None:
        self.frame_bgr = bgr
        h, w = bgr.shape[:2]
        self.canvas.config(width=w, height=h)
        view = bgr.copy()
        if self.sel is not None:
            x, y = self.sel
            if 0 <= x < w and 0 <= y < h:
                cv2.drawMarker(view, (x, y), (0, 255, 255), cv2.MARKER_CROSS, 22, 2)
                cv2.circle(view, (x, y), 12, (0, 255, 255), 2)
        ok, buf = cv2.imencode(".png", view)             # PhotoImage 는 PNG 를 받는다
        if not ok:
            self.say("이미지 인코딩 실패")
            return
        self.tk_img = tk.PhotoImage(data=buf.tobytes())
        self.canvas.delete("all")
        self.canvas.create_image(0, 0, anchor="nw", image=self.tk_img)

    # --- 버튼 -------------------------------------------------------------
    def on_state(self) -> None:
        api = self._api()            # 주소 읽기는 메인 스레드에서
        self._async(lambda: api.json("/get_state"), self._got_state)

    def _got_state(self, res) -> None:
        if res[0] == "error":
            self.state_var.set(f"연결 실패 - {res[1]}")
            self.say(f"[ERR] /get_state {res[1]}", "")
            return
        code, d = res
        self.state_var.set(f"{d.get('label', '?')}  ({d.get('state', '?')})   "
                           f"· {d.get('status', '')}")
        self.say_json("/get_state", code, d)

    def on_rgb(self) -> None:
        api = self._api()
        self._async(lambda: api.raw("/get_rgb", {"format": "png"}),
                    lambda r: self._got_image(r, "rgb"))

    def on_depth(self) -> None:
        api = self._api()
        self._async(lambda: api.raw("/get_depth"),               # 16bit PNG (mm)
                    lambda r: self._got_image(r, "depth"))

    def _got_image(self, res, kind: str) -> None:
        if res[0] == "error":
            self.say(f"[ERR] /get_{kind} {res[1]}", "")
            return
        code, hdr, body = res
        if code != 200:
            try:
                self.say_json(f"/get_{kind}", code, json.loads(body.decode("utf-8")))
            except ValueError:
                self.say(f"[ERR {code}] /get_{kind}", "")
            return

        raw = cv2.imdecode(np.frombuffer(body, np.uint8), cv2.IMREAD_UNCHANGED)
        if raw is None:
            self.say(f"[ERR] /get_{kind} 디코드 실패", "")
            return

        if kind == "depth":
            # 16bit mm 를 그대로 들고 있다가 클릭할 때 값을 보여준다.
            self.depth_mm = raw.astype(np.float32)
            valid = self.depth_mm[self.depth_mm > 0]
            lo, hi = (np.percentile(valid, [2, 98]) if valid.size else (0, 1))
            norm = np.clip((self.depth_mm - lo) / max(hi - lo, 1), 0, 1)
            view = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
            view[self.depth_mm <= 0] = (0, 0, 0)
            self.say(f"[OK {code}] /get_depth  {raw.shape[1]}x{raw.shape[0]} "
                     f"{raw.dtype} {len(body)}B  단위={hdr.get('X-Depth-Unit')} "
                     f"메운화소={hdr.get('X-Depth-Filled-Px')}  "
                     f"표시범위 {lo:.0f}~{hi:.0f}mm", "")
        else:
            self.depth_mm = None
            view = raw if raw.ndim == 3 else cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
            self.say(f"[OK {code}] /get_rgb  {view.shape[1]}x{view.shape[0]} "
                     f"{len(body)}B  size={hdr.get('X-Image-Size')}", "")
        self.show(view)

    def on_click(self, ev) -> None:
        if self.frame_bgr is None:
            self.say("먼저 RGB 나 Depth 를 가져오세요", "")
            return
        h, w = self.frame_bgr.shape[:2]
        x, y = int(ev.x), int(ev.y)
        if not (0 <= x < w and 0 <= y < h):
            return
        self.sel = (x, y)
        local = ""
        if self.depth_mm is not None:
            local = f"  (받은 depth 이미지 값 {self.depth_mm[y, x]:.0f}mm)"
        self.pick_var.set(f"선택 ({x}, {y}){local}  - 확인 중...")
        self.show(self.frame_bgr)
        # 클릭한 지점이 실제로 어디인지 서버에 물어본다. 팔은 움직이지 않는다.
        api = self._api()
        self._async(lambda: api.json("/get_depth", {"x": x, "y": y}),
                    lambda r: self._got_point(r, x, y))

    def _got_point(self, res, x: int, y: int) -> None:
        if res[0] == "error":
            self.pick_var.set(f"선택 ({x}, {y}) - 조회 실패")
            self.say(f"[ERR] /get_depth?x={x}&y={y} {res[1]}", "")
            return
        code, d = res
        if d.get("ok"):
            b = d.get("target_base_mm")
            pickable = d.get("pickable")
            tag = "집기 가능" if pickable else f"집기 불가 ({d.get('reason', '')})"
            est = " [추정 depth]" if d.get("estimated_depth") else ""
            self.pick_var.set(f"선택 ({x}, {y})  depth {d.get('depth_m')}m{est}  "
                              f"로봇 {b}mm  {tag}")
        else:
            self.pick_var.set(f"선택 ({x}, {y}) - {d.get('message', '조회 실패')}")
        self.say_json(f"/get_depth?x={x}&y={y}", code, d)

    def on_action(self, action: str) -> None:
        if self.sel is None:
            self.say("먼저 이미지에서 좌표를 클릭하세요", "")
            return
        x, y = self.sel
        self.say(f"-> {action}({x}, {y}) 요청")
        api = self._api()
        self._async(lambda: api.json(f"/{action}", {"x": x, "y": y}),
                    lambda r: self._got_action(r, action))

    def _got_action(self, res, action: str) -> None:
        if res[0] == "error":
            self.say(f"[ERR] /{action} {res[1]}", "")
            return
        code, d = res
        self.say_json(f"/{action}", code, d)
        self.on_state()                     # 결과를 바로 상태에 반영

    # --- 자동 갱신 --------------------------------------------------------
    def auto_poll(self) -> None:
        if self.auto_var.get():
            api = self._api()
            self._async(lambda: api.json("/get_state"), self._poll_state)
        self.after(1000, self.auto_poll)

    def _poll_state(self, res) -> None:
        """자동 갱신은 로그를 남기지 않는다. 1초마다 쌓이면 로그가 못 쓰게 된다."""
        if res[0] == "error":
            self.state_var.set(f"연결 실패 - {res[1]}")
            return
        _, d = res
        self.state_var.set(f"{d.get('label', '?')}  ({d.get('state', '?')})   "
                           f"· {d.get('status', '')}")


def main() -> None:
    base = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_API
    ClientUI(base).mainloop()


if __name__ == "__main__":
    main()
