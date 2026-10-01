#!/usr/bin/env python3
"""Live web viewer for the robot's Astra Pro RGB + depth streams.

Runs on the Raspberry Pi next to robot_rgbd.launch.py: subscribes to the
camera topics locally (the camera devices themselves are already opened by
openni2/usb_cam, so this never touches /dev/video* or OpenNI) and serves

  http://<pi-ip>:8080/                 page with both streams + live stats
  http://<pi-ip>:8080/stream/rgb.mjpg  MJPEG of the RGB image
  http://<pi-ip>:8080/stream/depth.mjpg?max=4000
                                       MJPEG of the depth image, colorized 0..max mm
  http://<pi-ip>:8080/api/stats        JSON: fps, depth min/median/max, center distance
  http://<pi-ip>:8080/api/depth_at?x=0.5&y=0.5
                                       JSON: depth (mm) at a normalized image point

JPEG encoding only happens while a browser is watching, once per new frame
regardless of how many browsers are open.

Usage (on the Pi, robot_rgbd.launch.py already running, same ROS_DOMAIN_ID):
  source /opt/ros/jazzy/setup.bash
  /usr/bin/python3 camera_test.py
  /usr/bin/python3 camera_test.py --ros-args -p port:=8081 -p max_fps:=10.0
"""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def image_to_array(msg: Image) -> np.ndarray:
    """sensor_msgs/Image -> numpy without cv_bridge (BGR for color, uint16 mm for depth)."""
    if msg.encoding in ('rgb8', 'bgr8'):
        arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        arr = arr.reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else arr.copy()
    if msg.encoding in ('16UC1', 'mono16'):
        dtype = np.dtype('>u2' if msg.is_bigendian else '<u2')
        arr = np.frombuffer(msg.data, dtype).reshape(msg.height, msg.step // 2)[:, :msg.width]
        return arr.astype(np.uint16)
    if msg.encoding == '32FC1':  # meters -> mm
        arr = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width]
        return np.nan_to_num(arr * 1000.0).astype(np.uint16)
    raise ValueError(f'unsupported encoding {msg.encoding}')


class Stream:
    """Latest frame of one topic + its rate, and a JPEG cache shared by all viewers."""

    def __init__(self):
        self.lock = threading.Condition()
        self.frame = None
        self.seq = 0
        self.stamps = []          # receive times, for fps
        self.jpeg_cache = {}      # (seq, key) -> bytes

    def put(self, frame):
        with self.lock:
            self.frame = frame
            self.seq += 1
            now = time.time()
            self.stamps = [t for t in self.stamps if now - t < 2.0] + [now]
            self.jpeg_cache.clear()
            self.lock.notify_all()

    def fps(self):
        with self.lock:
            now = time.time()
            recent = [t for t in self.stamps if now - t < 2.0]
            return len(recent) / 2.0

    def wait_newer(self, seq, timeout=2.0):
        with self.lock:
            self.lock.wait_for(lambda: self.seq != seq, timeout=timeout)
            return self.seq, self.frame

    def jpeg(self, seq, key, render):
        with self.lock:
            cached = self.jpeg_cache.get((seq, key))
        if cached is None:
            cached = render()
            with self.lock:
                if self.seq == seq:
                    self.jpeg_cache[(seq, key)] = cached
        return cached


def colorize_depth(depth: np.ndarray, max_mm: int) -> np.ndarray:
    vis = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=255.0 / max(max_mm, 1)), cv2.COLORMAP_JET)
    vis[depth == 0] = 0  # no return -> black
    # Upscale small depth (320x240) so it lines up with the 640x480 RGB view.
    if vis.shape[1] < 640:
        vis = cv2.resize(vis, (640, 640 * vis.shape[0] // vis.shape[1]), interpolation=cv2.INTER_NEAREST)
    return vis


class CameraViewer(Node):

    def __init__(self):
        super().__init__('camera_web_viewer')
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth_raw/image')
        self.declare_parameter('port', 8080)
        self.declare_parameter('max_fps', 15.0)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('depth_max_mm', 4000)

        self.port = self.get_parameter('port').value
        self.min_period = 1.0 / max(self.get_parameter('max_fps').value, 0.1)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.depth_max_mm = int(self.get_parameter('depth_max_mm').value)

        self.rgb = Stream()
        self.depth = Stream()
        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        self.create_subscription(Image, rgb_topic, lambda m: self._on(self.rgb, m), qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, lambda m: self._on(self.depth, m), qos_profile_sensor_data)
        self.get_logger().info(f'subscribed: {rgb_topic}, {depth_topic}')

    def _on(self, stream: Stream, msg: Image):
        try:
            stream.put(image_to_array(msg))
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)

    def encode(self, img: np.ndarray) -> bytes:
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        return buf.tobytes() if ok else b''

    def stats(self) -> dict:
        out = {'rgb_fps': self.rgb.fps(), 'depth_fps': self.depth.fps()}
        with self.rgb.lock:
            rgb = self.rgb.frame
        with self.depth.lock:
            depth = self.depth.frame
        if rgb is not None:
            out['rgb_size'] = [int(rgb.shape[1]), int(rgb.shape[0])]
        if depth is not None:
            valid = depth[depth > 0]
            h, w = depth.shape
            center = depth[h // 2 - 2:h // 2 + 3, w // 2 - 2:w // 2 + 3]
            center = center[center > 0]
            out.update({
                'depth_size': [int(w), int(h)],
                'valid_pct': round(100.0 * valid.size / depth.size, 1),
                'min_mm': int(valid.min()) if valid.size else None,
                'median_mm': int(np.median(valid)) if valid.size else None,
                'max_mm': int(valid.max()) if valid.size else None,
                'center_mm': int(np.median(center)) if center.size else None,
            })
        return out

    def depth_at(self, x: float, y: float):
        with self.depth.lock:
            depth = self.depth.frame
        if depth is None:
            return None
        h, w = depth.shape
        px = min(max(int(x * w), 0), w - 1)
        py = min(max(int(y * h), 0), h - 1)
        # Median of a 5x5 patch: single pixels are often 0 (no return) at edges.
        patch = depth[max(py - 2, 0):py + 3, max(px - 2, 0):px + 3]
        patch = patch[patch > 0]
        return {'x': px, 'y': py, 'mm': int(np.median(patch)) if patch.size else None}


PAGE = """<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>car2 camera</title>
<style>
  :root { color-scheme: dark; --bg:#16181d; --panel:#21252c; --text:#e6e8eb; --muted:#9aa3ad; --accent:#4cc2ff; }
  body { margin:0; padding:16px; background:var(--bg); color:var(--text); font:14px system-ui, sans-serif; }
  h1 { font-size:18px; margin:0 0 12px; }
  .views { display:grid; grid-template-columns:repeat(auto-fit, minmax(300px, 1fr)); gap:12px; }
  .view { background:var(--panel); border-radius:8px; padding:8px; }
  .view h2 { font-size:13px; margin:0 0 6px; color:var(--muted); font-weight:500; display:flex; justify-content:space-between; }
  .frame { position:relative; }
  .frame img { width:100%; display:block; border-radius:4px; background:#000; aspect-ratio:4/3; }
  #depth { cursor:crosshair; }
  .marker { position:absolute; width:14px; height:14px; margin:-7px 0 0 -7px; border:2px solid #fff;
            border-radius:50%; pointer-events:none; box-shadow:0 0 0 1px #000; }
  .marker span { position:absolute; left:16px; top:-4px; white-space:nowrap; background:#000c; padding:1px 5px;
                 border-radius:3px; font-size:12px; }
  .stats { margin-top:12px; background:var(--panel); border-radius:8px; padding:10px 12px;
           display:grid; grid-template-columns:repeat(auto-fit, minmax(140px, 1fr)); gap:6px 16px; }
  .stats div b { display:block; color:var(--muted); font-weight:400; font-size:12px; }
  .stats div span { font-variant-numeric:tabular-nums; font-size:16px; }
  label { color:var(--muted); font-size:12px; display:flex; align-items:center; gap:6px; }
  input[type=range] { width:120px; }
  .hint { color:var(--muted); font-size:12px; margin-top:8px; }
</style></head><body>
<h1>car2 카메라 실시간 보기</h1>
<div class="views">
  <div class="view"><h2>RGB</h2><div class="frame"><img id="rgb" src="/stream/rgb.mjpg" alt="RGB"></div></div>
  <div class="view">
    <h2>Depth
      <label>최대 <input id="max" type="range" min="1000" max="10000" step="500" value="__MAX__"> <span id="maxv">__MAX__</span> mm</label>
    </h2>
    <div class="frame"><img id="depth" src="/stream/depth.mjpg?max=__MAX__" alt="Depth">
      <div id="marker" class="marker" hidden><span id="markerv"></span></div></div>
  </div>
</div>
<div class="stats">
  <div><b>RGB fps</b><span id="rgb_fps">-</span></div>
  <div><b>Depth fps</b><span id="depth_fps">-</span></div>
  <div><b>중앙 거리</b><span id="center_mm">-</span></div>
  <div><b>최소 / 중간 / 최대</b><span id="range">-</span></div>
  <div><b>유효 픽셀</b><span id="valid_pct">-</span></div>
  <div><b>해상도 (RGB / Depth)</b><span id="sizes">-</span></div>
</div>
<p class="hint">Depth 화면을 클릭하면 그 지점의 거리를 표시합니다. 검은 부분은 측정값이 없는 곳입니다 (Astra Pro는 약 0.6m 이내를 잘 못 잽니다).</p>
<script>
const $ = id => document.getElementById(id);
const mm = v => v == null ? '-' : (v / 1000).toFixed(2) + ' m';
let pick = null;
async function refresh() {
  try {
    const s = await (await fetch('/api/stats')).json();
    $('rgb_fps').textContent = s.rgb_fps.toFixed(1);
    $('depth_fps').textContent = s.depth_fps.toFixed(1);
    $('center_mm').textContent = mm(s.center_mm);
    $('range').textContent = s.min_mm == null ? '-' : `${mm(s.min_mm)} / ${mm(s.median_mm)} / ${mm(s.max_mm)}`;
    $('valid_pct').textContent = s.valid_pct == null ? '-' : s.valid_pct + ' %';
    $('sizes').textContent = `${s.rgb_size ? s.rgb_size.join('x') : '-'} / ${s.depth_size ? s.depth_size.join('x') : '-'}`;
    if (pick) {
      const d = await (await fetch(`/api/depth_at?x=${pick.x}&y=${pick.y}`)).json();
      $('markerv').textContent = mm(d.mm);
    }
  } catch (e) { $('rgb_fps').textContent = '연결 끊김'; }
}
setInterval(refresh, 500); refresh();
$('depth').addEventListener('click', e => {
  const r = e.target.getBoundingClientRect();
  pick = { x: (e.clientX - r.left) / r.width, y: (e.clientY - r.top) / r.height };
  const m = $('marker');
  m.style.left = (pick.x * 100) + '%'; m.style.top = (pick.y * 100) + '%'; m.hidden = false;
  refresh();
});
$('max').addEventListener('input', e => { $('maxv').textContent = e.target.value; });
$('max').addEventListener('change', e => { $('depth').src = '/stream/depth.mjpg?max=' + e.target.value; });
// Reconnect a stream if it stalls (robot launch restarted, WiFi hiccup).
for (const id of ['rgb', 'depth']) $(id).addEventListener('error', e => {
  setTimeout(() => { e.target.src = e.target.src.split('#')[0] + '#' + Date.now(); }, 1000);
});
</script></body></html>"""


def make_handler(node: CameraViewer):

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.0'

        def log_message(self, fmt, *args):  # keep the terminal quiet
            pass

        def _send(self, code, body: bytes, ctype: str):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(code, json.dumps(obj).encode(), 'application/json')

        def _mjpeg(self, stream: Stream, render, key):
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            seq, last_sent = -1, 0.0
            try:
                while rclpy.ok():
                    seq, frame = stream.wait_newer(seq)
                    if frame is None:
                        continue
                    wait = node.min_period - (time.time() - last_sent)
                    if wait > 0:
                        time.sleep(wait)
                        seq, frame = stream.wait_newer(-1, timeout=0)  # take the newest after sleeping
                    jpg = stream.jpeg(seq, key, lambda: node.encode(render(frame)))
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n'
                                     + f'Content-Length: {len(jpg)}\r\n\r\n'.encode() + jpg + b'\r\n')
                    last_sent = time.time()
            except (BrokenPipeError, ConnectionResetError):
                pass  # browser closed the tab

        def do_GET(self):
            url = urlparse(self.path)
            q = parse_qs(url.query)
            if url.path == '/':
                page = PAGE.replace('__MAX__', str(node.depth_max_mm))
                self._send(200, page.encode(), 'text/html; charset=utf-8')
            elif url.path == '/stream/rgb.mjpg':
                self._mjpeg(node.rgb, lambda f: f, 'rgb')
            elif url.path == '/stream/depth.mjpg':
                max_mm = int(q.get('max', [node.depth_max_mm])[0])
                self._mjpeg(node.depth, lambda f: colorize_depth(f, max_mm), f'depth{max_mm}')
            elif url.path == '/api/stats':
                self._json(node.stats())
            elif url.path == '/api/depth_at':
                try:
                    x, y = float(q['x'][0]), float(q['y'][0])
                except (KeyError, ValueError):
                    self._json({'error': 'x, y (0..1) required'}, 400)
                    return
                self._json(node.depth_at(x, y) or {'mm': None})
            else:
                self._json({'error': 'not found'}, 404)

    return Handler


def main():
    rclpy.init()
    node = CameraViewer()
    server = ThreadingHTTPServer(('0.0.0.0', node.port), make_handler(node))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f'web UI: http://<this-host>:{node.port}/')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
