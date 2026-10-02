#!/usr/bin/env python3
"""Web UI: the map with the robot's live position / heading, plus manual driving.

Runs on the Raspberry Pi next to robot_lidar.launch.py (or robot_rgbd.launch.py).
The map and the map->odom TF come from the laptop (lidar_mapping /
lidar_navigation / rtabmap_* launches, same ROS_DOMAIN_ID); the lidar scan,
/odom and odom->base_link are local.

  http://<pi-ip>:8081/          page: map (pan/zoom), robot pose + heading, lidar
                                scan, driven trail, Nav2 path, drive pad
  GET  /api/state               JSON: pose in the map (or odom when there is no map
                                yet), scan points, trail, Nav2 plan, AMCL spread
  GET  /api/map/meta            JSON: map size, resolution, origin, version
  GET  /map.png                 the occupancy grid (row 0 = top = max y)
  POST /api/drive {"linear": 0.1, "angular": 0.0}   (see drive_control.py)
  POST /api/drive/stop
  GET  /api/drive
  POST /api/trail/clear         forget the driven trail
  GET  /api/labels              JSON: saved location labels (map frame)
  POST /api/labels {"name": "주방"}   label the robot's current map pose
                                (or {"name", "x", "y", "yaw"} for an explicit pose)
  POST /api/labels/delete {"id": "..."}
  POST /api/nav/goto {"id": "..."}    Nav2 NavigateToPose to a label
  POST /api/nav/cancel          cancel the Nav2 goal (manual driving cancels it too)
  GET  /api/nav/tolerance       JSON: arrival tolerance saved here / active in Nav2, limits
  POST /api/nav/tolerance {"xy": 0.05, "yaw_deg": 14}
                                set Nav2's goal checker tolerance (within the min..max
                                limits), saved and re-applied whenever Nav2 (re)starts
  POST /api/relocalize {"global": false, "dry_run": false}
                                re-find the robot in the map by matching the current
                                lidar scan to it, then send that pose to AMCL
                                (/initialpose, like RViz "2D Pose Estimate")

Relocalize: the scan is scored against a likelihood field of the map's
occupied cells (Gaussian of the distance to the nearest wall, sigma 0.1 m).
Local search = +-1 m / +-40 deg around the current estimate; global = every
free cell of the map, all headings (a few seconds on the Pi), then a fine
search around the best candidates. The pose is only sent when enough of the
scan fits (match >= min); AMCL is then asked for a few no-motion updates.
The robot must be standing still.

Labels are poses in the map frame, saved in labels_file
(~/.ros/car2_map_labels.json): they only mean something on the map they were
made on (keep using the same saved map for navigation). Go-to needs Nav2
running on the laptop (./scripts/run_lidar_navigation.sh or
run_rtabmap_navigation.sh); this node is the action client of
/navigate_to_pose, like RViz's "Nav2 Goal".

Pose: TF map->base_link when the map frame exists (SLAM / AMCL / RTAB-Map
running), otherwise odom->base_link (pure wheel odometry, drifts).
The page polls /api/state at ~5 Hz; the state is computed in the ROS thread
(timer), so HTTP threads never touch TF.

Usage (on the Pi, same ROS_DOMAIN_ID as the laptop):
  source /opt/ros/jazzy/setup.bash
  /usr/bin/python3 map_web.py
  /usr/bin/python3 map_web.py --ros-args -p port:=8082 -p max_linear:=0.1
(from the laptop: ./scripts/remote_map_web.sh start)
"""
import json
import math
import os
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import cv2
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from std_srvs.srv import Empty
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
import tf2_ros

from drive_control import DRIVE_CSS, DRIVE_HTML, DRIVE_JS, DriveControl


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def tf2d(t):
    """(x, y, yaw) of a geometry_msgs/TransformStamped."""
    tr = t.transform
    return tr.translation.x, tr.translation.y, yaw_of(tr.rotation)


class Labels:
    """Named map poses, persisted as JSON (atomic rewrite on every change)."""

    def __init__(self, path: str):
        self.path = path
        self.lock = threading.Lock()
        self.items = []
        try:
            with open(path) as f:
                self.items = json.load(f).get('labels', [])
        except FileNotFoundError:
            pass
        except (OSError, ValueError, AttributeError) as exc:
            print(f'labels: cannot read {path}: {exc}')

    def _save(self):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump({'labels': self.items}, f, ensure_ascii=False, indent=1)
        os.replace(tmp, self.path)

    def list(self):
        with self.lock:
            return list(self.items)

    def get(self, label_id):
        with self.lock:
            return next((lb for lb in self.items if lb['id'] == label_id), None)

    def add(self, name: str, x: float, y: float, yaw: float, frame: str) -> dict:
        name = name.strip()
        if not name:
            raise ValueError('라벨 이름을 입력하세요')
        if len(name) > 40:
            raise ValueError('라벨 이름은 40자까지')
        label = {'id': uuid.uuid4().hex[:8], 'name': name, 'x': round(x, 3), 'y': round(y, 3),
                 'yaw': round(math.atan2(math.sin(yaw), math.cos(yaw)), 4), 'frame': frame,
                 'created': time.strftime('%Y-%m-%d %H:%M:%S')}
        with self.lock:
            if any(lb['name'] == name for lb in self.items):
                raise ValueError(f'이미 있는 이름입니다: {name}')
            self.items.append(label)
            self._save()
        return label

    def delete(self, label_id):
        with self.lock:
            n = len(self.items)
            self.items = [lb for lb in self.items if lb['id'] != label_id]
            if len(self.items) == n:
                raise KeyError(label_id)
            self._save()


class ScanMatcher:
    """Brute-force correlative scan matching of a 2D scan against an occupancy grid."""

    SIGMA = 0.10        # m, likelihood field width
    HIT = 0.10          # m, a point "matches" when this close to a wall

    def __init__(self, grid: np.ndarray, res: float, origin):
        self.res, self.ox, self.oy = res, origin[0], origin[1]
        occ = grid >= 65
        # distance (m) from every cell to the nearest occupied cell
        self.dist = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5).astype(np.float32) * res
        self.lik = np.exp(-(self.dist / self.SIGMA) ** 2).astype(np.float32)
        self.h, self.w = grid.shape
        # where the robot can stand: known free, not hugging a wall
        self.free = (grid >= 0) & (grid <= 25) & (self.dist > 0.12)

    def score(self, cx, cy, yaw, pts):
        """Mean likelihood of pts (N x 2, robot frame) for robot poses (cx[i], cy[i], yaw) -> (M,)."""
        c, s = math.cos(yaw), math.sin(yaw)
        rx = pts[:, 0] * c - pts[:, 1] * s
        ry = pts[:, 0] * s + pts[:, 1] * c
        i = ((cx[:, None] + rx[None, :] - self.ox) / self.res).astype(np.int32)
        j = ((cy[:, None] + ry[None, :] - self.oy) / self.res).astype(np.int32)
        ok = (i >= 0) & (i < self.w) & (j >= 0) & (j < self.h)
        v = np.where(ok, self.lik[np.clip(j, 0, self.h - 1), np.clip(i, 0, self.w - 1)], 0.0)
        return v.mean(axis=1)

    def hit_ratio(self, x, y, yaw, pts):
        c, s = math.cos(yaw), math.sin(yaw)
        i = ((x + pts[:, 0] * c - pts[:, 1] * s - self.ox) / self.res).astype(np.int32)
        j = ((y + pts[:, 0] * s + pts[:, 1] * c - self.oy) / self.res).astype(np.int32)
        ok = (i >= 0) & (i < self.w) & (j >= 0) & (j < self.h)
        d = np.where(ok, self.dist[np.clip(j, 0, self.h - 1), np.clip(i, 0, self.w - 1)], 99.0)
        return float((d <= self.HIT).mean())

    def search(self, x0, y0, yaw0, pts, xy_range, xy_step, yaw_range, yaw_step, free_only=True):
        """Best (score, x, y, yaw) on a grid around (x0, y0, yaw0)."""
        n = int(round(xy_range / xy_step))
        g = np.arange(-n, n + 1) * xy_step
        cx, cy = np.meshgrid(x0 + g, y0 + g)
        cx, cy = cx.ravel(), cy.ravel()
        if free_only:
            keep = self._is_free(cx, cy)
            if keep.any():
                cx, cy = cx[keep], cy[keep]
        best = (-1.0, x0, y0, yaw0)
        m = int(round(yaw_range / yaw_step))
        for k in range(-m, m + 1):
            yaw = yaw0 + k * yaw_step
            sc = self.score(cx, cy, yaw, pts)
            b = int(np.argmax(sc))
            if sc[b] > best[0]:
                best = (float(sc[b]), float(cx[b]), float(cy[b]), yaw)
        return best

    def _is_free(self, cx, cy):
        i = ((cx - self.ox) / self.res).astype(np.int32)
        j = ((cy - self.oy) / self.res).astype(np.int32)
        ok = (i >= 0) & (i < self.w) & (j >= 0) & (j < self.h)
        return ok & self.free[np.clip(j, 0, self.h - 1), np.clip(i, 0, self.w - 1)]

    def global_search(self, pts, step=0.1, yaw_step=math.radians(5), keep=12):
        """Every free cell (at `step`) x all headings -> best few distinct candidates, refined."""
        jj, ii = np.nonzero(self.free)
        cx = self.ox + (ii + 0.5) * self.res
        cy = self.oy + (jj + 0.5) * self.res
        sel = ((np.round(cx / step) * step - cx) ** 2 + (np.round(cy / step) * step - cy) ** 2) < (self.res * 0.75) ** 2
        cx, cy = cx[sel], cy[sel]
        if cx.size == 0:
            return []
        sub = pts[::max(1, len(pts) // 90)]          # coarse pass on fewer points
        yaws = np.arange(0, 2 * math.pi, yaw_step)
        scores = np.stack([self.score(cx, cy, y, sub) for y in yaws], axis=1)   # M x Y
        best_yaw = scores.argmax(axis=1)
        best = scores[np.arange(len(cx)), best_yaw]
        cands = []
        for idx in np.argsort(-best):                  # distinct candidates (> 0.5 m apart)
            x, y = cx[idx], cy[idx]
            if all(math.hypot(x - c[1], y - c[2]) > 0.5 for c in cands):
                cands.append((float(best[idx]), float(x), float(y), float(yaws[best_yaw[idx]])))
                if len(cands) >= keep:
                    break
        out = []
        for _, x, y, yaw in cands:
            r = self.search(x, y, yaw, pts, 0.15, 0.025, math.radians(6), math.radians(1))
            r = self.search(r[1], r[2], r[3], pts, 0.04, 0.01, math.radians(1.5), math.radians(0.5))
            out.append(r)
        return sorted(out, key=lambda r: -r[0])


class GoalTolerance:
    """Nav2 goal checker tolerance, set from the page and kept applied.

    The value is saved in tolerance_file; every 2 s the controller_server's
    parameters are read and, if they differ from the saved value (Nav2 was
    restarted with the yaml's value), set again. SimpleGoalChecker takes the
    new tolerance immediately (dynamic parameters).
    """

    def __init__(self, node: Node):
        self.node = node
        p = lambda name, default: node.declare_parameter(name, default).value
        self.xy_min, self.xy_max = float(p('goal_xy_min', 0.03)), float(p('goal_xy_max', 0.5))
        self.yaw_min, self.yaw_max = float(p('goal_yaw_min_deg', 3.0)), float(p('goal_yaw_max_deg', 90.0))
        self.path = os.path.expanduser(p('tolerance_file', '~/.ros/car2_nav_tolerance.json'))
        ctrl = p('controller_node', '/controller_server')
        checker = p('goal_checker', 'general_goal_checker')
        self.names = (f'{checker}.xy_goal_tolerance', f'{checker}.yaw_goal_tolerance')
        self.get_cli = node.create_client(GetParameters, f'{ctrl}/get_parameters')
        self.set_cli = node.create_client(SetParameters, f'{ctrl}/set_parameters')
        self.saved = None         # {'xy': m, 'yaw': rad}
        self.active = None        # what Nav2 has, None = unknown / Nav2 not running
        self.message = ''
        self.busy = False
        self.kick = False
        try:
            with open(self.path) as f:
                d = json.load(f)
            self.saved = {'xy': float(d['xy']), 'yaw': float(d['yaw'])}
        except FileNotFoundError:
            pass
        except (OSError, ValueError, KeyError, TypeError) as exc:
            node.get_logger().warn(f'tolerance: cannot read {self.path}: {exc}')
        node.create_timer(2.0, self.tick)
        node.create_timer(0.2, lambda: self.kick and self.tick())

    def set(self, xy, yaw_deg) -> dict:
        xy, yaw_deg = float(xy), float(yaw_deg)
        if not (self.xy_min <= xy <= self.xy_max):
            raise ValueError(f'위치 오차는 {self.xy_min:g} ~ {self.xy_max:g} m 범위로 입력하세요')
        if not (self.yaw_min <= yaw_deg <= self.yaw_max):
            raise ValueError(f'방향 오차는 {self.yaw_min:g} ~ {self.yaw_max:g}° 범위로 입력하세요')
        self.saved = {'xy': round(xy, 3), 'yaw': round(math.radians(yaw_deg), 4)}
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        with open(self.path + '.tmp', 'w') as f:
            json.dump(self.saved, f)
        os.replace(self.path + '.tmp', self.path)
        self.kick = True
        return self.status()

    def tick(self):
        self.kick = False
        if self.busy:
            return
        if not self.get_cli.service_is_ready():
            self.active = None
            self.message = 'Nav2 꺼짐 — 저장값은 내비게이션을 시작하면 자동 적용됩니다'
            return
        self.busy = True
        fut = self.get_cli.call_async(GetParameters.Request(names=list(self.names)))
        fut.add_done_callback(self._got)

    def _got(self, fut):
        try:
            vals = fut.result().values
            if len(vals) != 2 or any(v.type != ParameterType.PARAMETER_DOUBLE for v in vals):
                self.active, self.message = None, f'controller_server에 {self.names[0]} 파라미터가 없습니다'
                self.busy = False
                return
            self.active = {'xy': vals[0].double_value, 'yaw': vals[1].double_value}
        except Exception as exc:
            self.active, self.message, self.busy = None, f'Nav2 파라미터 읽기 실패: {exc}', False
            return
        want = self.saved
        if want and (abs(want['xy'] - self.active['xy']) > 1e-4 or abs(want['yaw'] - self.active['yaw']) > 1e-4):
            req = SetParameters.Request(parameters=[
                Parameter(name=n, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=v))
                for n, v in zip(self.names, (want['xy'], want['yaw']))])
            self.set_cli.call_async(req).add_done_callback(self._set_done)
            return
        self.message = 'Nav2에 적용됨'
        self.busy = False

    def _set_done(self, fut):
        try:
            bad = [r.reason for r in fut.result().results if not r.successful]
            if bad:
                self.message = 'Nav2가 거부: ' + '; '.join(bad)
            else:
                self.active = dict(self.saved)
                self.message = 'Nav2에 적용됨'
                self.node.get_logger().info(f"goal tolerance set: xy {self.saved['xy']} m, "
                                            f"yaw {math.degrees(self.saved['yaw']):.1f} deg")
        except Exception as exc:
            self.message = f'Nav2 파라미터 설정 실패: {exc}'
        self.busy = False

    def status(self) -> dict:
        deg = lambda d: None if d is None else {'xy': d['xy'], 'yaw_deg': round(math.degrees(d['yaw']), 1)}
        return {'saved': deg(self.saved), 'active': deg(self.active), 'message': self.message,
                'limits': {'xy': [self.xy_min, self.xy_max], 'yaw_deg': [self.yaw_min, self.yaw_max]}}

    def arrive_check(self) -> float:
        """Distance beyond which a Nav2 'succeeded' is reported as not really arrived."""
        xy = (self.active or self.saved or {'xy': 0.1})['xy']
        return max(0.35, xy + 0.15)


class MapWeb(Node):

    def __init__(self):
        super().__init__('map_web')
        self.port = int(self.declare_parameter('port', 8081).value)
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.robot_radius = float(self.declare_parameter('robot_radius', 0.26).value)
        self.trail_max = int(self.declare_parameter('trail_max', 3000).value)
        self.drive = DriveControl(self)
        self.labels = Labels(os.path.expanduser(
            self.declare_parameter('labels_file', '~/.ros/car2_map_labels.json').value))

        # Nav2 go-to: requests come from HTTP threads, handled in the ROS thread
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav = {'state': 'idle'}
        self.nav_requests = []
        self.nav_seq = 0
        self.goal_handle = None

        # relocalization
        self.reloc_min_match = float(self.declare_parameter('reloc_min_match', 0.5).value)
        self.map_grid = None          # (grid int16 rows from origin y, resolution, origin, version)
        self.matcher = None           # ScanMatcher for matcher_version
        self.matcher_version = -1
        self.reloc = {'state': 'idle'}
        self.reloc_request = None
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.nomotion_client = self.create_client(Empty, 'request_nomotion_update')

        self.lock = threading.Lock()
        self.map_png = None
        self.map_meta = None          # dict, see _on_map
        self.map_version = 0
        self.scan = None              # (LaserScan, receive time)
        self.scan_times = []
        self.plan = None              # (frame, [[x, y], ...], receive time)
        self.amcl = None
        self.trail = []               # [[x, y], ...] in trail_frame
        self.trail_frame = None
        self.cur_pose = None          # (frame, x, y, yaw, time) for labelling
        self.state = {'frame': None, 'pose': None}
        self.state_json = json.dumps(self.state).encode()

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # map_server / slam_toolbox / rtabmap all latch /map (transient local)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, 'map', self._on_map, latched)
        # Nav2's global costmap: is a label somewhere the robot can actually be?
        self.costmap = None
        self.create_subscription(OccupancyGrid, 'global_costmap/costmap', self._on_costmap, latched)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Path, 'plan', self._on_plan, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, 10)
        self.create_timer(0.2, self._update_state)
        self.create_timer(0.1, self._nav_tick)
        self.tolerance = GoalTolerance(self)

    # ---- inputs -------------------------------------------------------------
    def _on_map(self, msg: OccupancyGrid):
        w, h = msg.info.width, msg.info.height
        if not w or not h:
            return
        grid = np.asarray(msg.data, dtype=np.int16).reshape(h, w)
        img = np.full((h, w), 128, np.uint8)                  # unknown
        known = grid >= 0
        # 0 (free) -> 235, 100 (occupied) -> 0
        img[known] = (235 - np.clip(grid[known], 0, 100) * 2.35).astype(np.uint8)
        ok, png = cv2.imencode('.png', np.flipud(img))       # row 0 = top = max y
        if not ok:
            return
        o = msg.info.origin
        with self.lock:
            self.map_version += 1
            self.map_png = png.tobytes()
            self.map_grid = (grid, msg.info.resolution, (o.position.x, o.position.y, yaw_of(o.orientation)),
                             self.map_version)
            self.map_meta = {
                'version': self.map_version, 'frame': msg.header.frame_id or self.map_frame,
                'width': w, 'height': h, 'resolution': msg.info.resolution,
                'origin': [o.position.x, o.position.y, yaw_of(o.orientation)],
                'received': time.time(),
                'known_m2': round(float(known.sum()) * msg.info.resolution ** 2, 1),
            }

    def _on_costmap(self, msg: OccupancyGrid):
        self.costmap = (np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width),
                        msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y,
                        msg.header.frame_id, time.time())

    def cost_at(self, x, y):
        """Global costmap value (0..100, -1 unknown) at a map point, None without a fresh costmap."""
        cm = self.costmap
        if cm is None or time.time() - cm[5] > 30.0:
            return None
        grid, res, ox, oy = cm[:4]
        i, j = int((x - ox) / res), int((y - oy) / res)
        if not (0 <= i < grid.shape[1] and 0 <= j < grid.shape[0]):
            return -1
        return int(grid[j, i])

    def _on_scan(self, msg: LaserScan):
        now = time.time()
        self.scan = (msg, now)
        self.scan_times = [t for t in self.scan_times if now - t < 2.0] + [now]

    def _on_plan(self, msg: Path):
        pts = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in msg.poses]
        if len(pts) > 400:                       # plenty for drawing
            step = math.ceil(len(pts) / 400)
            pts = pts[::step] + [pts[-1]]
        self.plan = (msg.header.frame_id, pts, time.time())

    def _on_amcl(self, msg: PoseWithCovarianceStamped):
        c = msg.pose.covariance
        self.amcl = {'sx': math.sqrt(max(c[0], 0.0)), 'sy': math.sqrt(max(c[7], 0.0)),
                     'syaw': math.sqrt(max(c[35], 0.0)), 'received': time.time()}

    # ---- relocalization -------------------------------------------------------------
    def request_reloc(self, global_search: bool, dry_run: bool = False):
        if self.reloc.get('state') == 'running':
            raise ValueError('이미 위치를 찾는 중입니다')
        with self.lock:
            self.reloc_request = {'global': bool(global_search), 'dry_run': bool(dry_run)}

    def _start_reloc(self, req):
        """ROS thread: gather inputs (scan in base_link, pose, map), then match in a worker thread."""
        def fail(text):
            self.reloc = {'state': 'failed', 'message': text, 'finished': time.time()}
        if self.map_grid is None:
            return fail('지도(/map)가 없습니다')
        if abs(self.map_grid[2][2]) > 1e-3:
            return fail('회전된 지도 원점은 지원하지 않습니다')
        if self.initialpose_pub.get_subscription_count() == 0 and not req['dry_run']:
            return fail('/initialpose를 받는 위치 추정 노드(AMCL)가 없습니다 — 노트북에서 run_lidar_navigation.sh 실행')
        odom = self.drive.odom
        if odom and time.time() - odom[2] < 2.0 and (abs(odom[0]) > 0.01 or abs(odom[1]) > 0.02):
            return fail('로봇이 움직이는 중입니다 — 멈춘 뒤 다시 시도하세요')
        if self.scan is None or time.time() - self.scan[1] > 1.0:
            return fail('라이다 스캔(/scan)이 없습니다')
        msg = self.scan[0]
        t = self._lookup(self.base_frame, msg.header.frame_id)
        if t is None:
            return fail(f'TF {self.base_frame}->{msg.header.frame_id} 없음')
        tx, ty, tyaw = tf2d(t)
        r = np.asarray(msg.ranges, dtype=np.float32)
        a = msg.angle_min + np.arange(r.size, dtype=np.float32) * msg.angle_increment + tyaw
        ok = np.isfinite(r) & (r >= max(msg.range_min, 0.15)) & (r <= min(msg.range_max, 8.0))
        pts = np.stack([tx + r[ok] * np.cos(a[ok]), ty + r[ok] * np.sin(a[ok])], axis=1).astype(np.float32)
        if len(pts) < 40:
            return fail(f'스캔 점이 너무 적습니다 ({len(pts)}개)')
        pts = pts[::max(1, len(pts) // 240)]
        cur = self.cur_pose if self.cur_pose and self.cur_pose[0] == self.map_frame else None
        if cur is None and not req['global']:
            req['global'] = True             # no estimate to search around
        self.reloc = {'state': 'running', 'global': req['global'], 'started': time.time(),
                      'message': '전체 지도에서 찾는 중…' if req['global'] else '현재 위치 주변에서 찾는 중…'}
        threading.Thread(target=self._reloc_worker, args=(pts, cur, req['global'], self.map_grid, req['dry_run']),
                         daemon=True).start()

    def _reloc_worker(self, pts, cur, global_search, map_grid, dry_run=False):
        t0 = time.time()
        try:
            grid, res, origin, version = map_grid
            if self.matcher_version != version:
                self.matcher, self.matcher_version = ScanMatcher(grid, res, origin), version
            m = self.matcher
            before = None
            if cur is not None:
                before = m.hit_ratio(cur[1], cur[2], cur[3], pts)
            ambiguous = False
            if global_search:
                cands = m.global_search(pts)
                if not cands:
                    raise ValueError('지도에 빈 공간이 없습니다')
                best = cands[0]
                # another distinct place that fits almost as well -> not sure
                ambiguous = any(c[0] > best[0] * 0.93 and math.hypot(c[1] - best[1], c[2] - best[2]) > 0.5
                                for c in cands[1:])
            else:
                best = m.search(cur[1], cur[2], cur[3], pts, 1.0, 0.05, math.radians(40), math.radians(2))
                best = m.search(best[1], best[2], best[3], pts, 0.1, 0.02, math.radians(3), math.radians(0.5))
                best = m.search(best[1], best[2], best[3], pts, 0.03, 0.01, math.radians(1), math.radians(0.25))
            _, x, y, yaw = best
            yaw = math.atan2(math.sin(yaw), math.cos(yaw))
            match = m.hit_ratio(x, y, yaw, pts)
            result = {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 4), 'match': round(match, 3),
                      'before': None if before is None else round(before, 3), 'ambiguous': ambiguous,
                      'global': global_search, 'seconds': round(time.time() - t0, 1)}
            if cur is not None:
                result['shift'] = round(math.hypot(x - cur[1], y - cur[2]), 3)
                result['turn'] = round(math.atan2(math.sin(yaw - cur[3]), math.cos(yaw - cur[3])), 4)
            if match < self.reloc_min_match:
                self.reloc = {'state': 'failed', 'result': result, 'finished': time.time(),
                              'message': f'맞는 위치를 찾지 못했습니다 (일치 {match * 100:.0f}% < '
                                         f'{self.reloc_min_match * 100:.0f}%)'
                                         + ('' if global_search else ' — "전체 지도에서 찾기"로 다시 시도')}
                return
            if not dry_run:
                self._send_initialpose(x, y, yaw, ambiguous)
            self.reloc = {'state': 'done', 'result': result, 'finished': time.time(), 'dry_run': dry_run,
                          'message': ('찾음 (dry run, AMCL에 보내지 않음)' if dry_run else '위치를 다시 잡았습니다')
                                     + (' (비슷한 후보가 있어 확인 필요)' if ambiguous else '')}
            self.get_logger().info(f'relocalized: x {x:.2f} y {y:.2f} yaw {math.degrees(yaw):.1f} '
                                   f'match {match:.2f} (before {before}) {time.time() - t0:.1f}s')
        except Exception as exc:     # report anything to the page instead of dying silently
            self.reloc = {'state': 'failed', 'message': f'오류: {exc}', 'finished': time.time()}
            self.get_logger().error(f'relocalize failed: {exc!r}')

    def _send_initialpose(self, x, y, yaw, ambiguous):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        sxy, syaw = (0.3, math.radians(15)) if ambiguous else (0.08, math.radians(4))
        cov = [0.0] * 36
        cov[0] = cov[7] = sxy ** 2
        cov[35] = syaw ** 2
        msg.pose.covariance = cov
        self.initialpose_pub.publish(msg)
        # let AMCL refine on the spot (no motion needed)
        if self.nomotion_client.service_is_ready():
            for _ in range(3):
                time.sleep(0.4)
                self.nomotion_client.call_async(Empty.Request())

    # ---- Nav2 go-to ----------------------------------------------------------------
    def request_nav(self, req):
        with self.lock:
            self.nav_requests.append(req)

    def nav_active(self):
        return self.nav.get('state') in ('sending', 'active', 'canceling')

    def _set_nav(self, seq, **kw):
        if seq == self.nav_seq:          # ignore callbacks of a replaced goal
            self.nav = {**self.nav, **kw}

    def _nav_tick(self):
        with self.lock:
            reqs, self.nav_requests = self.nav_requests, []
            reloc, self.reloc_request = self.reloc_request, None
        if reloc is not None:
            self._start_reloc(reloc)
        for req in reqs:
            if req[0] == 'cancel':
                if self.goal_handle is not None and self.nav_active():
                    self.goal_handle.cancel_goal_async()
                    self.nav = {**self.nav, 'state': 'canceling', 'message': '취소 요청'}
                continue
            label = req[1]
            self.nav_seq += 1
            if not self.nav_client.server_is_ready():
                self.nav = {'state': 'unavailable', 'target': label, 'message':
                            'Nav2(navigate_to_pose)가 없습니다 — 노트북에서 run_lidar_navigation.sh 실행'}
                continue
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = label.get('frame', self.map_frame)
            goal.pose.pose.position.x = float(label['x'])
            goal.pose.pose.position.y = float(label['y'])
            goal.pose.pose.orientation.z = math.sin(label['yaw'] / 2)
            goal.pose.pose.orientation.w = math.cos(label['yaw'] / 2)
            seq = self.nav_seq
            self.nav = {'state': 'sending', 'target': label, 'started': time.time(), 'message': '목표 전송 중'}
            fut = self.nav_client.send_goal_async(goal, feedback_callback=lambda fb, s=seq: self._nav_feedback(s, fb))
            fut.add_done_callback(lambda f, s=seq: self._nav_accepted(s, f))
            self.get_logger().info(f"nav goal '{label['name']}' x {label['x']:.2f} y {label['y']:.2f}")

    def _nav_feedback(self, seq, msg):
        fb = msg.feedback
        eta = fb.estimated_time_remaining
        self._set_nav(seq, distance=round(fb.distance_remaining, 2),
                      eta=round(eta.sec + eta.nanosec * 1e-9, 1), recoveries=fb.number_of_recoveries)

    def _nav_accepted(self, seq, fut):
        gh = fut.result()
        if seq != self.nav_seq:
            return
        if not gh.accepted:
            self._set_nav(seq, state='rejected', message='Nav2가 목표를 거부했습니다')
            return
        self.goal_handle = gh
        self._set_nav(seq, state='active', message='이동 중')
        gh.get_result_async().add_done_callback(lambda f, s=seq: self._nav_result(s, f))

    def _nav_result(self, seq, fut):
        res = fut.result()
        state = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_CANCELED: 'canceled',
                 GoalStatus.STATUS_ABORTED: 'aborted'}.get(res.status, f'status {res.status}')
        msg = {'succeeded': '도착', 'canceled': '취소됨', 'aborted': '실패 (경로 없음 / 막힘)'}.get(state, state)
        err = getattr(res.result, 'error_msg', '')
        target, cur = self.nav.get('target'), self.cur_pose
        if state == 'succeeded' and target and cur and cur[0] == target.get('frame', self.map_frame):
            # Nav2's controller can report success when a map->odom lookup fails
            # (its goal check then compares against an untransformed goal) -- check it.
            left = math.hypot(cur[1] - target['x'], cur[2] - target['y'])
            if left > self.tolerance.arrive_check():
                state, msg = 'failed', (f'Nav2가 도착이라고 했지만 목표까지 {left:.2f} m 남음 — 목표가 장애물 영역이라 '
                                        '가까운 지점에서 멈췄거나 TF 지연 오판')
        self._set_nav(seq, state=state, message=msg + (f': {err}' if err else ''), finished=time.time())
        self.get_logger().info(f'nav result: {state} {err}')

    # ---- state (ROS thread, 5 Hz) ---------------------------------------------
    def _lookup(self, target, source):
        try:
            return self.tf_buffer.lookup_transform(target, source, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

    def _scan_points(self, frame):
        if self.scan is None or time.time() - self.scan[1] > 1.0:
            return []
        msg = self.scan[0]
        t = self._lookup(frame, msg.header.frame_id)
        if t is None:
            return []
        tx, ty, tyaw = tf2d(t)
        r = np.asarray(msg.ranges, dtype=np.float32)
        a = msg.angle_min + np.arange(r.size, dtype=np.float32) * msg.angle_increment + tyaw
        ok = np.isfinite(r) & (r >= max(msg.range_min, 0.01)) & (r <= msg.range_max)
        xs = tx + r[ok] * np.cos(a[ok])
        ys = ty + r[ok] * np.sin(a[ok])
        return np.round(np.stack([xs, ys], axis=1), 3).tolist()

    def _update_state(self):
        now = time.time()
        frame, t = self.map_frame, self._lookup(self.map_frame, self.base_frame)
        if t is None:
            frame, t = self.odom_frame, self._lookup(self.odom_frame, self.base_frame)
        pose = None
        if t is not None:
            x, y, yaw = tf2d(t)
            stamp = Time.from_msg(t.header.stamp)
            age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
            pose = {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 4), 'age': round(age, 2)}
            self.cur_pose = (frame, x, y, yaw, now)
            with self.lock:
                if frame != self.trail_frame:
                    self.trail, self.trail_frame = [], frame
                if not self.trail or math.hypot(x - self.trail[-1][0], y - self.trail[-1][1]) > 0.03:
                    self.trail.append([round(x, 3), round(y, 3)])
                    del self.trail[:-self.trail_max]

        plan = []
        if self.plan and self.plan[0] == frame and now - self.plan[2] < 30.0:
            plan = self.plan[1]
        amcl = self.amcl if self.amcl and now - self.amcl['received'] < 30.0 else None
        scan_hz = (len(self.scan_times) - 1) / (self.scan_times[-1] - self.scan_times[0]) \
            if len(self.scan_times) > 2 else 0.0

        with self.lock:
            meta = self.map_meta
            state = {
                'frame': frame if t is not None else None,
                'pose': pose,
                'robot_radius': self.robot_radius,
                'scan': self._scan_points(frame) if t is not None else [],
                'scan_hz': round(scan_hz, 1),
                'trail': list(self.trail),
                'plan': plan,
                'amcl': amcl,
                'map': None if meta is None else {**meta, 'age': round(now - meta['received'], 1)},
                'labels': [{**lb, 'cost': self.cost_at(lb['x'], lb['y'])} for lb in self.labels.list()],
                'robot_cost': None if pose is None or frame != self.map_frame else self.cost_at(pose['x'], pose['y']),
                'nav': {**self.nav, 'server': self.nav_client.server_is_ready()},
                'tolerance': self.tolerance.status(),
                'reloc': {**self.reloc, 'amcl': self.initialpose_pub.get_subscription_count() > 0},
            }
            self.state_json = json.dumps(state).encode()

    def add_label(self, req: dict) -> dict:
        name = str(req.get('name', ''))
        if all(k in req for k in ('x', 'y')):
            return self.labels.add(name, float(req['x']), float(req['y']), float(req.get('yaw', 0.0)),
                                   self.map_frame)
        cur = self.cur_pose
        if cur is None or time.time() - cur[4] > 2.0:
            raise ValueError('현재 위치를 모릅니다 (TF 없음)')
        if cur[0] != self.map_frame:
            raise ValueError('map 좌표계가 없습니다 — 노트북에서 SLAM/내비게이션을 실행한 뒤 저장하세요 '
                             '(odom 좌표는 재시작하면 바뀝니다)')
        return self.labels.add(name, cur[1], cur[2], cur[3], self.map_frame)

    def clear_trail(self):
        with self.lock:
            self.trail = []


PAGE = """<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>car2 map</title>
<style>
  :root { color-scheme: dark; --bg:#16181d; --panel:#21252c; --text:#e6e8eb; --muted:#9aa3ad; --accent:#4cc2ff; }
  body { margin:0; padding:16px; background:var(--bg); color:var(--text); font:14px system-ui, sans-serif; }
  h1 { font-size:18px; margin:0 0 12px; }
  .mapbox { position:relative; background:var(--panel); border-radius:8px; overflow:hidden; height:min(62vh, 640px);
            min-height:300px; touch-action:none; }
  #cv { width:100%; height:100%; display:block; cursor:grab; }
  #cv.drag { cursor:grabbing; }
  .tools { position:absolute; top:8px; right:8px; display:flex; gap:6px; flex-wrap:wrap; justify-content:flex-end; }
  .tools button, .tools label { background:#000a; color:var(--text); border:1px solid #ffffff22; border-radius:6px;
                                padding:5px 10px; font:13px system-ui, sans-serif; cursor:pointer; }
  .tools label { display:flex; align-items:center; gap:5px; }
  .tools button.on { background:var(--accent); color:#06222f; }
  #cursor { position:absolute; left:8px; bottom:8px; background:#000a; padding:2px 8px; border-radius:4px;
            font-size:12px; font-variant-numeric:tabular-nums; color:var(--muted); }
  #banner { position:absolute; left:8px; top:8px; background:#000b; padding:4px 10px; border-radius:4px;
            font-size:13px; max-width:60%; }
  #banner.warn { color:#ffb74c; } #banner:empty { display:none; }
  #cursor:empty { display:none; }
  .stats { margin-top:12px; background:var(--panel); border-radius:8px; padding:10px 12px;
           display:grid; grid-template-columns:repeat(auto-fit, minmax(140px, 1fr)); gap:6px 16px; }
  .stats div b { display:block; color:var(--muted); font-weight:400; font-size:12px; }
  .stats div span { font-variant-numeric:tabular-nums; font-size:16px; }
  .hint { color:var(--muted); font-size:12px; margin-top:8px; }
  .legend { display:flex; flex-wrap:wrap; gap:4px 14px; color:var(--muted); font-size:12px; margin-top:6px; }
  .legend i { display:inline-block; width:10px; height:10px; border-radius:2px; margin-right:4px; vertical-align:-1px; }
  .labels { margin-top:12px; background:var(--panel); border-radius:8px; padding:10px 12px; display:flex;
            flex-direction:column; gap:8px; }
  .labels h2 { font-size:13px; margin:0; color:var(--muted); font-weight:500; }
  .labelbar { display:flex; flex-wrap:wrap; align-items:center; gap:8px 12px; }
  .labelbar input:not([type=checkbox]):not([type=range]):not([type=number]) { flex:1; min-width:160px; max-width:280px; background:#14171c; color:var(--text);
                    border:1px solid #ffffff22; border-radius:6px; padding:7px 9px; font:14px system-ui, sans-serif; }
  .labelbar button { background:var(--accent); color:#06222f; border:0; border-radius:6px; padding:7px 14px;
                     font:600 14px system-ui, sans-serif; cursor:pointer; }
  .labelbar button.go { background:#69f0ae; color:#05301a; }
  .labelbar button.ghost { background:transparent; color:var(--text); border:1px solid #ffffff33; }
  .labelbar button.danger { color:#ff8a80; border-color:#ff8a8066; }
  .labelbar button:disabled { opacity:.4; cursor:default; }
  #lmsg, #navstate { font-size:13px; color:var(--muted); }
  #lmsg.ok, #navstate.ok { color:#7ddc8a; } #lmsg.err, #navstate.err { color:#ff8a80; }
  #navstate.busy { color:#ffb74c; }
  .ltable { max-height:240px; overflow:auto; border-radius:6px; }
  .labels table { width:100%; border-collapse:collapse; font-variant-numeric:tabular-nums; }
  .labels th { text-align:left; color:var(--muted); font-weight:400; font-size:12px; padding:4px 8px;
               position:sticky; top:0; background:var(--panel); }
  .labels td { padding:6px 8px; border-top:1px solid #ffffff10; cursor:pointer; }
  .labels tr.sel td { background:#4cc2ff26; }
  .labels tr.target td:first-child::after { content:' ▶ 이동 목표'; color:#69f0ae; font-size:12px; }
  .labels td:first-child::before { content:''; display:inline-block; width:9px; height:9px; border-radius:50%;
                                   background:#ff1744; margin-right:7px; }
  .reloc { margin-top:12px; background:var(--panel); border-radius:8px; padding:10px 12px; }
  .reloc button { background:#ffb300; color:#2b1d00; border:0; border-radius:6px; padding:7px 14px;
                  font:600 14px system-ui, sans-serif; cursor:pointer; }
  .reloc button:disabled { opacity:.4; cursor:default; }
  .reloc label { font-size:13px; color:var(--muted); display:flex; align-items:center; gap:5px; }
  #relocstate { font-size:13px; color:var(--muted); }
  #relocstate.ok { color:#7ddc8a; } #relocstate.err { color:#ff8a80; } #relocstate.busy { color:#ffb74c; }
  .labels .blocked { color:#ffb74c; font-size:12px; }
  .tol { display:flex; flex-wrap:wrap; align-items:center; gap:8px 16px; padding-top:8px; border-top:1px solid #ffffff14; }
  .tol .f { display:flex; align-items:center; gap:6px; font-size:13px; color:var(--muted); }
  .tol input[type=range] { width:140px; }
  .tol input[type=number] { width:4.8em; background:#14171c; color:var(--text); border:1px solid #ffffff22;
                            border-radius:4px; padding:4px 5px; }
  .tol small { color:var(--muted); }
  #tolstate { font-size:13px; color:var(--muted); }
  #tolstate.ok { color:#7ddc8a; } #tolstate.err { color:#ff8a80; } #tolstate.busy { color:#ffb74c; }
  .labels .empty { color:var(--muted); font-size:13px; padding:6px 8px; }
__DRIVE_CSS__
</style></head><body>
<h1>car2 지도 · 위치 · 주행</h1>
<div class="mapbox">
  <canvas id="cv"></canvas>
  <div id="banner"></div>
  <div class="tools">
    <label title="로봇이 항상 화면 가운데 오도록"><input id="follow" type="checkbox" checked> 따라가기</label>
    <label title="로봇 앞쪽이 화면 위를 향하도록 지도를 회전"><input id="headup" type="checkbox"> 전방 위</label>
    <button id="zin" title="확대">＋</button><button id="zout" title="축소">－</button>
    <button id="fit" title="지도 전체 보기">전체</button>
    <button id="clr" title="주행 궤적 지우기">궤적 지우기</button>
  </div>
  <div id="cursor"></div>
</div>
<div class="legend">
  <span><i style="background:#ffb300"></i>로봇 (원 = 차체 반경)</span><span><i style="background:#e040fb"></i>라이다 스캔</span>
  <span><i style="background:#ff1744;border-radius:50%"></i>위치 라벨</span>
  <span><i style="background:#4cc2ff"></i>주행 궤적</span><span><i style="background:#69f0ae"></i>Nav2 경로</span>
  <span><i style="background:#eceff1"></i>빈 공간</span><span><i style="background:#000;border:1px solid #555"></i>장애물</span>
  <span><i style="background:#808080"></i>미탐색</span>
  <span>✛ X→ Y↑ = 지도 원점 (0, 0)</span>
</div>
<div class="stats">
  <div><b>X (m)</b><span id="px">-</span></div>
  <div><b>Y (m)</b><span id="py">-</span></div>
  <div><b>방향</b><span id="pyaw">-</span></div>
  <div><b>기준 좌표계</b><span id="pframe">-</span></div>
  <div><b>위치 신뢰도 (AMCL ±)</b><span id="pcov">-</span></div>
  <div><b>지도</b><span id="pmap">-</span></div>
  <div><b>라이다</b><span id="pscan">-</span></div>
  <div><b>원점까지</b><span id="pdist">-</span></div>
  <div><b>지도 범위 X (m)</b><span id="prx">-</span></div>
  <div><b>지도 범위 Y (m)</b><span id="pry">-</span></div>
</div>
<div class="reloc labelbar">
  <button id="relocbtn" title="현재 라이다 스캔을 지도와 맞춰 로봇 위치를 찾고 AMCL에 다시 설정 (RViz 2D Pose Estimate와 같음)">
    현재 위치 자동 다시 잡기</button>
  <label title="위치가 크게 틀렸거나 전혀 모를 때: 지도 전체·모든 방향을 검색 (몇 초 걸림)">
    <input id="relocglobal" type="checkbox"> 전체 지도에서 찾기</label>
  <span id="relocstate"></span>
</div>
<div class="labels">
  <h2>위치 라벨 (map 좌표) · 지도에서 빨간 점을 누르거나 목록에서 선택</h2>
  <div class="labelbar">
    <input id="lname" maxlength="40" placeholder="라벨 이름 (예: 주방, 충전기 앞)">
    <button id="ladd" title="로봇의 현재 위치·방향을 이 이름으로 저장">현재 위치 라벨 추가</button>
    <span id="lmsg"></span>
  </div>
  <div class="ltable"><table>
    <thead><tr><th>이름</th><th>X (m)</th><th>Y (m)</th><th>방향</th><th>로봇에서 거리</th></tr></thead>
    <tbody id="lbody"></tbody>
  </table></div>
  <div class="labelbar">
    <button id="lgo" class="go" disabled title="Nav2로 선택한 라벨 위치까지 자율 주행">Go to (Navi)</button>
    <button id="lcancel" class="ghost" title="Nav2 목표 취소 (주행 패드를 눌러도 취소됨)">Navi 취소</button>
    <button id="ldel" class="ghost danger" disabled>선택 라벨 삭제</button>
    <span id="navstate"></span>
  </div>
  <div class="tol labelbar">
    <b style="font-size:13px">도착 오차</b>
    <span class="f" title="목표 위치에서 이만큼 안에 들어오면 도착">위치
      <input id="tolxy" type="range" step="0.01"> <input id="tolxyn" type="number" step="0.01"> m
      <small id="tolxyr"></small></span>
    <span class="f" title="목표 방향과 이만큼 안이면 도착">방향
      <input id="tolyaw" type="range" step="1"> <input id="tolyawn" type="number" step="1"> °
      <small id="tolyawr"></small></span>
    <button id="tolapply">적용</button>
    <span id="tolstate"></span>
  </div>
</div>
__DRIVE_HTML__
<div class="hint">지도: 드래그 = 이동, 휠/핀치 = 확대·축소 (이동하면 '따라가기'가 꺼집니다). 방향 0° = 지도 +X(오른쪽), 반시계 +.
  지도(/map)와 map 좌표계는 노트북의 SLAM/내비게이션이 실행 중일 때만 있습니다 — 없으면 odom 기준(바퀴 주행거리)으로 표시합니다.</div>
<script>
const $ = id => document.getElementById(id);
const cv = $('cv'), ctx = cv.getContext('2d');
let S = null;                    // last /api/state
let mapImg = null, mapMeta = null, mapLoading = 0;
let selId = null;                // selected label id
const view = { x: 0, y: 0, s: 60, rot: 0 };   // world point at the canvas center, px per m, rotation (rad)

function resize() {
  const r = cv.getBoundingClientRect(), dpr = devicePixelRatio || 1;
  cv.width = Math.round(r.width * dpr); cv.height = Math.round(r.height * dpr);
  draw();
}
addEventListener('resize', resize);

// world (m) <-> canvas (css px). Screen y points down; view.rot rotates the world.
function w2s(x, y) {
  const W = cv.clientWidth, H = cv.clientHeight, c = Math.cos(view.rot), s = Math.sin(view.rot);
  const dx = x - view.x, dy = y - view.y;
  return [W / 2 + (dx * c - dy * s) * view.s, H / 2 - (dx * s + dy * c) * view.s];
}
function s2w(px, py) {
  const W = cv.clientWidth, H = cv.clientHeight, c = Math.cos(view.rot), s = Math.sin(view.rot);
  const u = (px - W / 2) / view.s, v = -(py - H / 2) / view.s;
  return [view.x + u * c + v * s, view.y - u * s + v * c];
}

function draw() {
  const dpr = devicePixelRatio || 1, W = cv.clientWidth, H = cv.clientHeight;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = '#2a2e35'; ctx.fillRect(0, 0, W, H);
  if (S && S.pose && $('follow').checked) { view.x = S.pose.x; view.y = S.pose.y; }
  view.rot = $('headup').checked && S && S.pose ? Math.PI / 2 - S.pose.yaw : 0;

  // map image
  if (mapImg && mapMeta && S && S.frame === mapMeta.frame) {
    const [ox, oy, oyaw] = mapMeta.origin, res = mapMeta.resolution;
    const [sx, sy] = w2s(ox, oy);
    ctx.save();
    ctx.translate(sx, sy); ctx.rotate(-(oyaw + view.rot));
    ctx.imageSmoothingEnabled = view.s * res < 2;
    ctx.drawImage(mapImg, 0, -mapMeta.height * res * view.s, mapMeta.width * res * view.s, mapMeta.height * res * view.s);
    ctx.restore();
  }
  drawGrid(W, H);
  if (!S) return;

  const line = (pts, color, width) => {
    if (pts.length < 2) return;
    ctx.beginPath();
    pts.forEach((p, i) => { const [x, y] = w2s(p[0], p[1]); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.lineJoin = 'round'; ctx.stroke();
  };
  line(S.trail, '#4cc2ffaa', 2);
  line(S.plan, '#69f0ae', 3);

  ctx.fillStyle = '#e040fb';
  const d = Math.max(2, Math.min(4, view.s * 0.03));
  for (const p of S.scan) { const [x, y] = w2s(p[0], p[1]); ctx.fillRect(x - d / 2, y - d / 2, d, d); }

  drawAxes();
  if (S.frame === 'map') drawLabels();
  if (S.pose) drawRobot(S.pose, S.robot_radius);
}

function arrow(x0, y0, x1, y1, color, text) {
  const a = Math.atan2(y1 - y0, x1 - x0);
  ctx.strokeStyle = ctx.fillStyle = color; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(x1, y1);
  ctx.lineTo(x1 - 8 * Math.cos(a - 0.4), y1 - 8 * Math.sin(a - 0.4));
  ctx.lineTo(x1 - 8 * Math.cos(a + 0.4), y1 - 8 * Math.sin(a + 0.4)); ctx.closePath(); ctx.fill();
  ctx.font = 'bold 12px system-ui'; ctx.fillText(text, x1 + 4 * Math.cos(a) - 4, y1 + 12 * Math.sin(a) + 4 + (Math.abs(Math.sin(a)) < .5 ? -8 : 0));
}

function drawAxes() {
  // map origin (0, 0) with +X / +Y arrows, 0.5 m long (at least 30 px)
  const L = Math.max(0.5, 30 / view.s), [ox, oy] = w2s(0, 0);
  arrow(ox, oy, ...w2s(L, 0), '#ffffffcc', 'X');
  arrow(ox, oy, ...w2s(0, L), '#ffffffcc', 'Y');
  ctx.fillStyle = '#ffffffcc'; ctx.font = '11px system-ui'; ctx.fillText('(0, 0)', ox + 4, oy + 14);
}

function drawLabels() {
  const target = S.nav && ['sending', 'active', 'canceling'].includes(S.nav.state) && S.nav.target ? S.nav.target.id : null;
  ctx.font = '12px system-ui'; ctx.textBaseline = 'middle';
  for (const lb of S.labels) {
    const [x, y] = w2s(lb.x, lb.y), sel = lb.id === selId;
    if (lb.id === target) {     // Nav2 goal: green ring
      ctx.beginPath(); ctx.arc(x, y, 14, 0, 2 * Math.PI); ctx.strokeStyle = '#69f0ae'; ctx.lineWidth = 3; ctx.stroke();
    }
    if (sel) {                  // selected: ring + saved heading
      const a = -(lb.yaw + view.rot);
      ctx.beginPath(); ctx.arc(x, y, 10, 0, 2 * Math.PI); ctx.strokeStyle = '#fff'; ctx.lineWidth = 2; ctx.stroke();
      ctx.beginPath(); ctx.moveTo(x + 10 * Math.cos(a), y + 10 * Math.sin(a)); ctx.lineTo(x + 24 * Math.cos(a), y + 24 * Math.sin(a));
      ctx.stroke();
    }
    ctx.beginPath(); ctx.arc(x, y, sel ? 7 : 5.5, 0, 2 * Math.PI);
    ctx.fillStyle = '#ff1744'; ctx.fill(); ctx.strokeStyle = '#000'; ctx.lineWidth = 1.5; ctx.stroke();
    ctx.lineWidth = 3; ctx.strokeStyle = '#000c'; ctx.strokeText(lb.name, x + 10, y - 10);
    ctx.fillStyle = sel ? '#fff' : '#ffcdd2'; ctx.fillText(lb.name, x + 10, y - 10);
  }
  ctx.textBaseline = 'alphabetic';
}

function drawGrid(W, H) {
  // 1 m grid (or 5 m when zoomed out) + scale bar
  const step = view.s < 15 ? 5 : 1;
  const corners = [[0, 0], [W, 0], [0, H], [W, H]].map(p => s2w(p[0], p[1]));
  const xs = corners.map(c => c[0]), ys = corners.map(c => c[1]);
  ctx.strokeStyle = '#ffffff12'; ctx.lineWidth = 1; ctx.beginPath();
  for (let x = Math.floor(Math.min(...xs) / step) * step; x <= Math.max(...xs); x += step) {
    const a = w2s(x, Math.min(...ys)), b = w2s(x, Math.max(...ys)); ctx.moveTo(...a); ctx.lineTo(...b);
  }
  for (let y = Math.floor(Math.min(...ys) / step) * step; y <= Math.max(...ys); y += step) {
    const a = w2s(Math.min(...xs), y), b = w2s(Math.max(...xs), y); ctx.moveTo(...a); ctx.lineTo(...b);
  }
  ctx.stroke();
  // map coordinates of the grid lines along the top / left edges (north-up view only)
  if (!view.rot) {
    ctx.font = '11px system-ui'; ctx.fillStyle = '#ffffff99';
    const lbl = step >= 5 || view.s >= 30 ? step : 2;
    for (let x = Math.ceil(Math.min(...xs) / lbl) * lbl; x <= Math.max(...xs); x += lbl) {
      const [sx] = w2s(x, 0); if (sx > 30) ctx.fillText(x + '', sx + 2, 12);
    }
    for (let y = Math.ceil(Math.min(...ys) / lbl) * lbl; y <= Math.max(...ys); y += lbl) {
      const [, sy] = w2s(0, y); if (sy > 20) ctx.fillText(y + '', 4, sy - 2);
    }
  }
  const bar = [0.5, 1, 2, 5, 10, 20].find(m => m * view.s >= 60) || 20;
  ctx.fillStyle = '#000a'; ctx.fillRect(W - bar * view.s - 22, H - 30, bar * view.s + 14, 22);
  ctx.fillStyle = '#e6e8eb'; ctx.fillRect(W - bar * view.s - 15, H - 14, bar * view.s, 3);
  ctx.font = '11px system-ui'; ctx.fillText(bar + ' m', W - bar * view.s - 15, H - 18);
}

function drawRobot(p, radius) {
  const [x, y] = w2s(p.x, p.y), r = Math.max(radius * view.s, 6), a = -(p.yaw + view.rot);
  ctx.save(); ctx.translate(x, y);
  ctx.beginPath(); ctx.arc(0, 0, r, 0, 2 * Math.PI);
  ctx.fillStyle = '#ffb30033'; ctx.fill(); ctx.strokeStyle = '#ffb300'; ctx.lineWidth = 2; ctx.stroke();
  if (S.amcl) {   // AMCL 1-sigma position spread
    ctx.beginPath(); ctx.ellipse(0, 0, Math.max(S.amcl.sx * view.s, 1), Math.max(S.amcl.sy * view.s, 1), -view.rot, 0, 2 * Math.PI);
    ctx.strokeStyle = '#ffb30088'; ctx.setLineDash([4, 3]); ctx.lineWidth = 1; ctx.stroke(); ctx.setLineDash([]);
  }
  ctx.rotate(a);
  const L = Math.max(r * 1.6, 18);
  ctx.beginPath(); ctx.moveTo(L, 0); ctx.lineTo(L * 0.45, -L * 0.28); ctx.lineTo(L * 0.55, 0); ctx.lineTo(L * 0.45, L * 0.28);
  ctx.closePath(); ctx.fillStyle = '#ffb300'; ctx.fill();
  ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(L * 0.55, 0); ctx.strokeStyle = '#ffb300'; ctx.lineWidth = 3; ctx.stroke();
  ctx.restore();
}

// ---- pan / zoom ---------------------------------------------------------------
const ptrs = new Map();
let pinch0 = null;
function zoomAt(f, px, py) {
  const [wx, wy] = s2w(px, py);
  view.s = Math.min(800, Math.max(3, view.s * f));
  if (!$('follow').checked) { const [nx, ny] = s2w(px, py); view.x += wx - nx; view.y += wy - ny; }
  draw();
}
cv.addEventListener('wheel', e => { e.preventDefault(); const r = cv.getBoundingClientRect();
  zoomAt(Math.exp(-e.deltaY * 0.0015), e.clientX - r.left, e.clientY - r.top); }, { passive: false });
let downAt = null;
cv.addEventListener('pointerdown', e => { downAt = [e.clientX, e.clientY]; cv.setPointerCapture(e.pointerId); ptrs.set(e.pointerId, [e.clientX, e.clientY]); cv.classList.add('drag');
  if (ptrs.size === 2) { const [a, b] = [...ptrs.values()]; pinch0 = { d: Math.hypot(a[0] - b[0], a[1] - b[1]), s: view.s }; } });
cv.addEventListener('pointermove', e => {
  const r = cv.getBoundingClientRect();
  const [wx, wy] = s2w(e.clientX - r.left, e.clientY - r.top);
  $('cursor').textContent = `커서 x ${wx.toFixed(2)}  y ${wy.toFixed(2)} m`;
  const prev = ptrs.get(e.pointerId);
  if (!prev) return;
  ptrs.set(e.pointerId, [e.clientX, e.clientY]);
  if (ptrs.size === 2 && pinch0) {
    const [a, b] = [...ptrs.values()];
    view.s = Math.min(800, Math.max(3, pinch0.s * Math.hypot(a[0] - b[0], a[1] - b[1]) / pinch0.d)); draw(); return;
  }
  const dx = e.clientX - prev[0], dy = e.clientY - prev[1];
  if (Math.abs(dx) + Math.abs(dy) < 1) return;
  $('follow').checked = false;
  const c = Math.cos(view.rot), s = Math.sin(view.rot), u = -dx / view.s, v = dy / view.s;
  view.x += u * c + v * s; view.y += -u * s + v * c;
  draw();
});
for (const ev of ['pointerup', 'pointercancel']) cv.addEventListener(ev, e => { ptrs.delete(e.pointerId); pinch0 = null;
  if (!ptrs.size) cv.classList.remove('drag');
  if (ev === 'pointerup' && downAt && Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]) < 5 && S && S.frame === 'map') {
    const r = cv.getBoundingClientRect(), px = e.clientX - r.left, py = e.clientY - r.top;
    let best = null, bd = 14;
    for (const lb of S.labels) { const [x, y] = w2s(lb.x, lb.y), d = Math.hypot(x - px, y - py); if (d < bd) { bd = d; best = lb; } }
    if (best) { selId = best.id; renderLabels(); draw(); }
  }
  downAt = null; });
$('zin').onclick = () => zoomAt(1.4, cv.clientWidth / 2, cv.clientHeight / 2);
$('zout').onclick = () => zoomAt(1 / 1.4, cv.clientWidth / 2, cv.clientHeight / 2);
$('follow').onchange = draw; $('headup').onchange = draw;
$('fit').onclick = () => {
  if (!mapMeta) return;
  const res = mapMeta.resolution, w = mapMeta.width * res, h = mapMeta.height * res;
  $('follow').checked = false; $('headup').checked = false;
  view.x = mapMeta.origin[0] + w / 2; view.y = mapMeta.origin[1] + h / 2;
  view.s = Math.min(cv.clientWidth / w, cv.clientHeight / h) * 0.95; draw();
};
$('clr').onclick = () => fetch('/api/trail/clear', { method: 'POST' });

// ---- arrival tolerance --------------------------------------------------------------
let tolInit = false, tolEdited = false;
function bindPair(r, n) {
  $(r).addEventListener('input', () => { $(n).value = $(r).value; tolEdited = true; });
  $(n).addEventListener('input', () => { $(r).value = $(n).value; tolEdited = true; });
}
bindPair('tolxy', 'tolxyn'); bindPair('tolyaw', 'tolyawn');
function renderTol() {
  const t = S && S.tolerance; if (!t) return;
  const L = t.limits;
  if (!tolInit) {
    for (const [r, n, lim, rl, unit] of [['tolxy', 'tolxyn', L.xy, 'tolxyr', 'm'], ['tolyaw', 'tolyawn', L.yaw_deg, 'tolyawr', '°']]) {
      $(r).min = $(n).min = lim[0]; $(r).max = $(n).max = lim[1];
      $(rl).textContent = `(${lim[0]} ~ ${lim[1]} ${unit})`;
    }
    tolInit = true;
  }
  const cur = t.saved || t.active;
  if (cur && !tolEdited) {
    $('tolxy').value = $('tolxyn').value = cur.xy;
    $('tolyaw').value = $('tolyawn').value = Math.round(cur.yaw_deg);
  }
  const a = t.active, sv = t.saved;
  let txt = a ? `Nav2 적용값 ${a.xy.toFixed(2)} m / ${a.yaw_deg.toFixed(0)}°` : '';
  if (sv && a && (Math.abs(sv.xy - a.xy) > 1e-4 || Math.abs(sv.yaw_deg - a.yaw_deg) > 0.1)) txt += ' · 적용 중…';
  msg('tolstate', [txt, t.message].filter(Boolean).join(' · '), a ? (t.message === 'Nav2에 적용됨' ? 'ok' : 'busy') : 'err');
}
$('tolapply').onclick = async () => {
  const xy = parseFloat($('tolxyn').value), yaw = parseFloat($('tolyawn').value);
  try { await post('/api/nav/tolerance', { xy, yaw_deg: yaw }); tolEdited = false; msg('tolstate', '저장함 — Nav2에 적용 중…', 'busy'); }
  catch (e) { msg('tolstate', e.message, 'err'); }
};

// ---- relocalize -------------------------------------------------------------------
function renderReloc() {
  const r = S && S.reloc; if (!r) return;
  const running = r.state === 'running';
  $('relocbtn').disabled = running || !r.amcl || !S.map;
  $('relocbtn').textContent = running ? '찾는 중…' : '현재 위치 자동 다시 잡기';
  if (!r.amcl && !running) { msg('relocstate', 'AMCL이 없습니다 — 노트북에서 run_lidar_navigation.sh 실행 후 사용', 'err'); return; }
  if (r.state === 'idle') { msg('relocstate', '로봇을 멈춘 상태에서 누르세요', ''); return; }
  if (running) { msg('relocstate', `${r.message} ${((Date.now() / 1000) - r.started).toFixed(0)}초`, 'busy'); return; }
  const x = r.result;
  let t = r.message;
  if (x) {
    t += ` · x ${x.x.toFixed(2)}, y ${x.y.toFixed(2)}, ${deg(x.yaw).toFixed(1)}°`
      + ` · 스캔 일치 ${(x.match * 100).toFixed(0)}%` + (x.before != null ? ` (이전 ${(x.before * 100).toFixed(0)}%)` : '')
      + (x.shift != null ? ` · 보정 ${x.shift.toFixed(2)} m, ${deg(x.turn).toFixed(1)}°` : '') + ` · ${x.seconds}초`;
  }
  msg('relocstate', t, r.state === 'done' && !(x && x.ambiguous) ? 'ok' : 'err');
}
$('relocbtn').onclick = async () => {
  try { await post('/api/relocalize', { global: $('relocglobal').checked }); msg('relocstate', '요청함…', 'busy'); }
  catch (e) { msg('relocstate', e.message, 'err'); }
};

// ---- labels / Nav2 go-to ----------------------------------------------------------
const esc = t => t.replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]);
function msg(id, text, cls) { $(id).textContent = text; $(id).className = cls || ''; }
async function post(url, body) {
  const r = await fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body || {}) });
  const d = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(d.error || r.statusText);
  return d;
}
let lastRows = '';
function renderLabels() {
  const labels = S ? S.labels : [], p = S && S.frame === 'map' ? S.pose : null;
  if (!labels.some(lb => lb.id === selId)) selId = null;
  const target = S && S.nav && ['sending', 'active', 'canceling'].includes(S.nav.state) && S.nav.target ? S.nav.target.id : null;
  const rows = labels.length ? labels.map(lb => `<tr data-id="${lb.id}" class="${lb.id === selId ? 'sel' : ''} ${lb.id === target ? 'target' : ''}">
      <td>${esc(lb.name)}${lb.cost >= 99 ? ' <span class="blocked" title="Nav2 비용지도에서 장애물/로봇 반경 안 — 갈 수 없음">⚠ 도달 불가</span>' : ''}</td><td>${lb.x.toFixed(2)}</td><td>${lb.y.toFixed(2)}</td><td>${deg(lb.yaw).toFixed(0)}°</td>
      <td>${p ? Math.hypot(lb.x - p.x, lb.y - p.y).toFixed(2) + ' m' : '-'}</td></tr>`).join('')
    : '<tr><td colspan="5" class="empty">등록된 라벨이 없습니다 — 로봇을 원하는 곳에 두고 이름을 입력해 추가하세요</td></tr>';
  if (rows !== lastRows) { $('lbody').innerHTML = rows; lastRows = rows; }
  $('lgo').disabled = !selId || !(S && S.nav && S.nav.server);
  $('lgo').title = S && S.nav && S.nav.server ? 'Nav2로 선택한 라벨 위치까지 자율 주행'
                                              : 'Nav2가 실행 중이 아닙니다 (노트북: run_lidar_navigation.sh)';
  $('ldel').disabled = !selId;
  $('ladd').disabled = !(S && S.frame === 'map' && S.pose);
  $('ladd').title = $('ladd').disabled ? 'map 좌표계가 있어야 저장할 수 있습니다 (SLAM/내비게이션 실행)' : '로봇의 현재 위치·방향을 이 이름으로 저장';
  const n = S && S.nav, st = $('navstate');
  if (!n) return;
  const name = n.target ? `'${n.target.name}'` : '';
  if (n.state === 'idle') msg('navstate', n.server ? 'Nav2 대기 중' : 'Nav2 꺼짐 — Go to를 쓰려면 노트북에서 내비게이션 실행', n.server ? '' : 'err');
  else if (['sending', 'active', 'canceling'].includes(n.state))
    msg('navstate', `${name} ${n.message || ''}` + (n.distance != null ? ` · 남은 거리 ${n.distance.toFixed(2)} m` : '')
        + (n.eta ? ` · 약 ${n.eta.toFixed(0)}초` : '') + (n.recoveries ? ` · 복구 ${n.recoveries}회` : ''), 'busy');
  else msg('navstate', `${name} ${n.message || n.state}`, n.state === 'succeeded' ? 'ok' : 'err');
}
$('lbody').addEventListener('click', e => {
  const tr = e.target.closest('tr[data-id]'); if (!tr) return;
  selId = tr.dataset.id; lastRows = ''; renderLabels(); draw();
  const lb = S.labels.find(l => l.id === selId);
  if (lb && !$('follow').checked) { view.x = lb.x; view.y = lb.y; draw(); }
});
async function addLabel() {
  const name = $('lname').value.trim();
  if (!name) { msg('lmsg', '이름을 입력하세요', 'err'); $('lname').focus(); return; }
  try {
    const lb = await post('/api/labels', { name });
    selId = lb.id; $('lname').value = '';
    msg('lmsg', `'${lb.name}' 저장: x ${lb.x.toFixed(2)}, y ${lb.y.toFixed(2)}, ${deg(lb.yaw).toFixed(0)}°`, 'ok');
  } catch (e) { msg('lmsg', e.message, 'err'); }
}
$('ladd').onclick = addLabel;
$('lname').addEventListener('keydown', e => { if (e.key === 'Enter') addLabel(); });
$('ldel').onclick = async () => {
  const lb = S.labels.find(l => l.id === selId); if (!lb) return;
  if (!confirm(`라벨 '${lb.name}'을(를) 삭제할까요?`)) return;
  try { await post('/api/labels/delete', { id: lb.id }); selId = null; msg('lmsg', `'${lb.name}' 삭제됨`, 'ok'); }
  catch (e) { msg('lmsg', e.message, 'err'); }
};
$('lgo').onclick = async () => {
  const lb = S.labels.find(l => l.id === selId); if (!lb) return;
  if (!confirm(`로봇이 '${lb.name}' (x ${lb.x.toFixed(2)}, y ${lb.y.toFixed(2)})까지 자율 주행합니다.\n주변이 안전한가요? (중지: 'Navi 취소' 또는 주행 패드)`)) return;
  try { await post('/api/nav/goto', { id: lb.id }); msg('navstate', `'${lb.name}'(으)로 목표 전송`, 'busy'); }
  catch (e) { msg('navstate', e.message, 'err'); }
};
$('lcancel').onclick = () => post('/api/nav/cancel').catch(e => msg('navstate', e.message, 'err'));

// ---- polling ------------------------------------------------------------------
const deg = r => (r * 180 / Math.PI);
async function poll() {
  try {
    S = await (await fetch('/api/state')).json();
  } catch (e) { $('banner').className = 'warn'; $('banner').textContent = '서버 응답 없음'; return; }
  const m = S.map;
  if (m && (!mapMeta || m.version !== mapMeta.version) && !mapLoading) {
    mapLoading = 1;
    const img = new Image();
    img.onload = () => { const first = !mapImg; mapImg = img; mapMeta = m; mapLoading = 0; if (first && !S.pose) $('fit').click(); draw(); };
    img.onerror = () => { mapLoading = 0; };
    img.src = '/map.png?v=' + m.version;
  }
  const p = S.pose;
  $('px').textContent = p ? p.x.toFixed(2) : '-';
  $('py').textContent = p ? p.y.toFixed(2) : '-';
  $('pyaw').textContent = p ? `${deg(p.yaw).toFixed(1)}°` : '-';
  $('pdist').textContent = p ? `${Math.hypot(p.x, p.y).toFixed(2)} m` : '-';
  $('pframe').textContent = S.frame ? (S.frame === 'map' ? 'map (SLAM/AMCL)' : S.frame + ' (지도 없음)') : '-';
  $('pcov').textContent = S.amcl ? `${Math.hypot(S.amcl.sx, S.amcl.sy).toFixed(2)} m, ${deg(S.amcl.syaw).toFixed(1)}°` : '-';
  $('pmap').textContent = m ? `${(m.width * m.resolution).toFixed(1)}×${(m.height * m.resolution).toFixed(1)} m, ${m.resolution.toFixed(3)} m/px, ${m.age.toFixed(0)}초 전` : '없음';
  $('prx').textContent = m ? `${m.origin[0].toFixed(2)} ~ ${(m.origin[0] + m.width * m.resolution).toFixed(2)}` : '-';
  $('pry').textContent = m ? `${m.origin[1].toFixed(2)} ~ ${(m.origin[1] + m.height * m.resolution).toFixed(2)}` : '-';
  renderLabels();
  renderReloc();
  renderTol();
  $('pscan').textContent = S.scan_hz ? `${S.scan_hz.toFixed(1)} Hz, ${S.scan.length}점` : '없음';
  const b = $('banner');
  if (!S.frame) { b.className = 'warn'; b.textContent = 'TF 없음: 로봇 드라이버(odom→base_link)가 실행 중이 아닙니다'; }
  else if (S.frame !== 'map') { b.className = 'warn'; b.textContent = 'map 좌표계 없음 — odom(바퀴 주행거리) 기준 표시. 노트북에서 SLAM/내비게이션을 실행하세요'; }
  else if (S.robot_cost >= 99) { b.className = 'warn'; b.textContent = '로봇 위치가 Nav2 비용지도에서 장애물 영역 안입니다 — 위치를 다시 잡거나, 지도에 없는 장애물이 찍혀 있으면 지도를 새로 만드세요'; }
  else if (p && p.age > 2) { b.className = 'warn'; b.textContent = `위치가 ${p.age.toFixed(1)}초 지연됨`; }
  else { b.textContent = ''; }
  draw();
}
setInterval(poll, 200); poll(); resize();
__DRIVE_JS__
</script></body></html>
"""


def make_handler(node: MapWeb):

    page = (PAGE.replace('__DRIVE_CSS__', DRIVE_CSS).replace('__DRIVE_HTML__', DRIVE_HTML)
            .replace('__DRIVE_JS__', DRIVE_JS)).encode()

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.0'

        def log_message(self, fmt, *args):  # keep the terminal quiet
            pass

        def _send(self, code, body: bytes, ctype: str, cache='no-store'):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', cache)
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(code, json.dumps(obj).encode(), 'application/json')

        def do_GET(self):
            path = urlparse(self.path).path
            if path == '/':
                self._send(200, page, 'text/html; charset=utf-8')
            elif path == '/api/state':
                with node.lock:
                    body = node.state_json
                self._send(200, body, 'application/json')
            elif path == '/api/map/meta':
                with node.lock:
                    self._json(node.map_meta)
            elif path == '/map.png':
                with node.lock:
                    png = node.map_png
                if png is None:
                    self._json({'error': '아직 /map을 받지 못했습니다'}, 404)
                else:   # ?v=<version> makes every version a distinct URL
                    self._send(200, png, 'image/png', cache='max-age=3600')
            elif path == '/api/drive':
                self._json(node.drive.status())
            elif path == '/api/nav/tolerance':
                self._json(node.tolerance.status())
            elif path == '/api/labels':
                self._json({'labels': node.labels.list(), 'file': node.labels.path})
            else:
                self._json({'error': 'not found'}, 404)

        def do_POST(self):
            path = urlparse(self.path).path
            length = int(self.headers.get('Content-Length') or 0)
            body = self.rfile.read(length) if length else b''
            if path == '/api/drive' and node.nav_active():
                node.request_nav(('cancel',))      # manual driving overrides Nav2
            res = node.drive.handle_post(path, body)
            if res is not None:
                self._json(res[1], res[0])
            elif path == '/api/trail/clear':
                node.clear_trail()
                self._json({'ok': True})
            elif path in ('/api/labels', '/api/labels/delete', '/api/nav/goto'):
                try:
                    req = json.loads(body or b'{}')
                    if path == '/api/labels':
                        self._json(node.add_label(req))
                    elif path == '/api/labels/delete':
                        node.labels.delete(req.get('id'))
                        self._json({'ok': True})
                    else:
                        label = node.labels.get(req.get('id'))
                        if label is None:
                            raise KeyError(req.get('id'))
                        cost = node.cost_at(label['x'], label['y'])
                        if cost is not None and cost >= 99:
                            raise ValueError(f"'{label['name']}'은(는) Nav2 비용지도에서 장애물/로봇 반경 안(cost {cost})이라 "
                                             '갈 수 없습니다 — 지도에 없는 장애물이 찍혀 있으면 지도를 새로 만드세요')
                        node.request_nav(('goto', label))
                        self._json({'ok': True, 'target': label})
                except KeyError:
                    self._json({'error': '없는 라벨입니다'}, 404)
                except (ValueError, TypeError, AttributeError) as exc:
                    self._json({'error': str(exc)}, 400)
                except OSError as exc:
                    self._json({'error': f'라벨 파일 저장 실패: {exc}'}, 500)
            elif path == '/api/relocalize':
                try:
                    req = json.loads(body or b'{}')
                    if node.nav_active():
                        raise ValueError('Navi 주행 중에는 할 수 없습니다 — 먼저 취소하세요')
                    node.request_reloc(bool(req.get('global', False)), bool(req.get('dry_run', False)))
                    self._json({'ok': True})
                except (ValueError, TypeError, AttributeError) as exc:
                    self._json({'error': str(exc)}, 409)
            elif path == '/api/nav/tolerance':
                try:
                    req = json.loads(body or b'{}')
                    self._json(node.tolerance.set(req['xy'], req['yaw_deg']))
                except (KeyError, ValueError, TypeError, AttributeError) as exc:
                    self._json({'error': str(exc) if not isinstance(exc, KeyError) else 'xy, yaw_deg가 필요합니다'}, 400)
                except OSError as exc:
                    self._json({'error': f'저장 실패: {exc}'}, 500)
            elif path == '/api/nav/cancel':
                node.request_nav(('cancel',))
                self._json({'ok': True})
            else:
                self._json({'error': 'not found'}, 404)

    return Handler


def main():
    rclpy.init()
    node = MapWeb()
    server = ThreadingHTTPServer(('0.0.0.0', node.port), make_handler(node))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f'web UI ready on http://<this-host>:{node.port}/')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.drive.stop()
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
