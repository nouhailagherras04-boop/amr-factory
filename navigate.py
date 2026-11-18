#!/usr/bin/env python3
"""
AMR navigator (Gazebo only, no ROS):  map in memory + A* path planning + LiDAR obstacle avoidance.

  python3 navigate.py                     menu: pick a station, patrol, quit
  python3 navigate.py cable_cutter home   go to these stations in order
  python3 navigate.py --patrol            visit every station, forever

Install once:  sudo apt install -y python3-gz-transport13 python3-gz-msgs10
Needs factory_map.pgm next to this file.   World: factory_standalone.sdf
"""
import heapq
import math
import os
import sys
import threading
import time

RES = 0.1                       # planning grid resolution [m]
ORIGIN = (-15.0, -10.0)         # map origin (lower-left)
SPAWN = (-12.0, 0.0, 0.0)       # AMR start pose in the world file (x, y, yaw)

R_BLOCK = 0.40                  # keep the robot centre at least this far from known obstacles
R_DYN = 0.42                    # inflation of obstacles seen by the lidar
V_MAX = 0.55
W_MAX = 1.0
LOOKAHEAD = 0.9
STOP_FRONT = 0.50               # emergency stop distance (lidar, front sector)
DYN_TTL = 3.0                   # seconds a lidar-detected obstacle stays in memory
DWELL = 3.0

STATIONS = {                    # name: (x, y, yaw)
    'home': (-12.0, 0.0, 0.0),
    'pack_station': (-10.0, 3.4, math.pi / 2), 'assembly_ws1': (-6.0, 3.4, math.pi / 2),
    'assembly_ws2': (-2.0, 3.4, math.pi / 2), 'assembly_ws3': (2.0, 3.4, math.pi / 2),
    'assembly_ws4': (6.0, 3.4, math.pi / 2), 'reel_storage': (10.45, 3.4, math.pi / 2),
    'rack_r1': (6.0, -3.3, -math.pi / 2), 'ultrasonic_welder': (0.6, -3.3, -math.pi / 2),
    'crimping_press': (-3.2, -3.3, -math.pi / 2), 'cable_cutter': (-7.0, -3.3, -math.pi / 2),
    'qc_bench': (-11.8, -3.3, -math.pi / 2),
}
QR_TEXT = {                     # text encoded in the QR code on the floor in front of each station
    'home': 'ST:HOME', 'pack_station': 'ST:PACK', 'assembly_ws1': 'ST:WS1', 'assembly_ws2': 'ST:WS2',
    'assembly_ws3': 'ST:WS3', 'assembly_ws4': 'ST:WS4', 'reel_storage': 'ST:REEL', 'rack_r1': 'ST:RACK1',
    'ultrasonic_welder': 'ST:WELD', 'crimping_press': 'ST:CRIMP', 'cable_cutter': 'ST:CUT', 'qc_bench': 'ST:QC',
}
QR_TO_STATION = {v: k for k, v in QR_TEXT.items()}
PATROL = ['assembly_ws1', 'assembly_ws2', 'assembly_ws3', 'assembly_ws4', 'reel_storage', 'rack_r1',
          'ultrasonic_welder', 'crimping_press', 'cable_cutter', 'qc_bench', 'pack_station', 'home']


MAP_B64 = """
eNrt28sNAiEUQNFZU4UlECNTh/0XY9ybmAgDyOfcAl4IHDbzeaZwxnh7xBjuKYVDkiRJkiRJkiRJkiRJkiRJkqRBe0kZcSWuxJW4
4kpciStxxZW4ElfiiitxJa7EFVfiSlyJK67ElbgSV1ypmSvQxJW4EldciStxJa64ElfiSlxJXIkrcSVxJa7ElcSVuBJXElfiSlwV
zf91rHlz/DfFFVdcmccVBxu4+roErrjiyjyuOOCKK664Mo8rDrjiiiuuzLvi6jiqTObKPK7M48q5ccUVV1yZx5Vz46qrK40ZV+JK
XHHFFVfiSlxxxRVX4kpcccVV3S8GueKKK3ElrrjiiitxJa644oorreCWK3ElrsQVV+JKXIkrrsSVuBJXElfiSlxJXIkrcSVxJa7E
lcSVuBJXElfiSlyJK67ElbgSV1yJK3ElrrgSV+JKXHElrsSVuOJKXIkrccWVuBJX4spOiStxJa4krsSVuJImdgX1xPvHFVdciSuu
uOKKK67EFVdcccUVV+KKK668xxFX4kpccSWuxJW44kpciStxxZW4ElfiSuJKXIkriStxJa4krsSVuJK4ElfiSuJKXIkriStxJa4k
rsSVVnXFl7gSV+LKjokrcSWuJK7ElbiSuBJX4kriSlyJK4krcSWuJK40giuJK3ElriSuxJW4krgSV+JK4kpciSuJK3ElriSuxJW4
krgSV+JK4kpciStxxZW4ElfiiitxJa7EFVfiSlwVL80xTbd/XHHFlbjiiiuuuOJKXHHFFVdccSWuuOLKexxxJa7EFVfiSlyJK67U
2BVo4kpciSuuxJW4EldciStxJa4krsSVuJK4ElfiSuJKXIkriStxJa4krsSVuFLT88j/n7jdZK644oorrsQVV1xxxRVX4oorrrji
iitxxRVXXHHFlbjiiiuuuOJKXHGl/veDK3ElrsQVV+JKXImrT1fXnnSIK67ElbgSV/1c1Xtf1WitLhNXXPXhwxVXXHHFFVdcccUV
V1xxxRVXXHHFFVdcccUVV1xxxRVXXHE10HfIXHHFFVdcccUVV1xpq7gSV+JKXHElrsSVuOJKfwHHlbgSV+KKK3ElrsQVV+JKXIkr
rsSVuFLZeeR/9d9uMldcccUVV+KKK6644oorccUVV1xxxZW44oorrrha0JW44oorrsSVuOJKw98POyWuxJW4krgSV+JK4kpciSuJ
K3ElriSuxJV2cCVJkiRJkiRJkiRJkiRJkiRJUu/e/MDLyg==
"""   # factory map (zlib+base64): no separate file needed

def decode_qr(gray):
    """Try several image variants (older OpenCV versions are picky). Returns the text or ''."""
    import cv2
    det = cv2.QRCodeDetector()
    variants = [gray]
    for k in (2, 3):
        big = cv2.resize(gray, None, fx=k, fy=k, interpolation=cv2.INTER_CUBIC)
        variants.append(big)
        variants.append(cv2.threshold(big, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1])
    variants.append(cv2.copyMakeBorder(gray, 40, 40, 40, 40, cv2.BORDER_REPLICATE))
    for img in variants:
        try:
            text, _, _ = det.detectAndDecode(img)
        except cv2.error:
            continue
        if text:
            return text
    return ''


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Navigator:
    def __init__(self, pgm_path):
        self.load_map(pgm_path)
        self.pose = SPAWN
        self.dyn = {}                        # (ix, iy) -> last seen time
        self.dyn_block = set()
        self.path = []
        self.pi = 0
        self.goal = None
        self.state = 'idle'                  # idle | driving | rotating | blocked | arrived | failed
        self.why = ''
        self.fail = ''
        self.front_min = 99.0
        self.pts = []                        # lidar points in the robot frame (x fwd, y left)
        self.last_plan = -99.0
        self.blocked_since = None
        self.dyn_offsets = [(dx, dy) for dx in range(-5, 6) for dy in range(-5, 6)
                            if math.hypot(dx, dy) * RES <= R_DYN]

    # ---------------------------------------------------------------- map
    def load_map(self, path):
        if path and os.path.exists(path):
            with open(path, 'rb') as f:
                raw = f.read()
        else:
            import base64
            import zlib
            raw = zlib.decompress(base64.b64decode(MAP_B64))
        parts = raw.split(None, 4)
        assert parts[0] == b'P5', 'map must be a binary PGM'
        w, h = int(parts[1]), int(parts[2])
        data = raw[len(raw) - w * h:]
        res0 = 0.05
        self.W, self.H = int(w * res0 / RES), int(h * res0 / RES)
        k = int(round(RES / res0))
        self.occ = [bytearray(self.W) for _ in range(self.H)]
        for iy in range(self.H):
            for ix in range(self.W):
                hit = 0
                for dy in range(k):
                    row = h - 1 - (iy * k + dy)
                    for dx in range(k):
                        if data[row * w + ix * k + dx] < 200:
                            hit = 1
                self.occ[iy][ix] = hit
        self.build_distance()

    def build_distance(self, limit=10):
        INF = 99
        self.dist = [[INF] * self.W for _ in range(self.H)]
        heap = []
        for iy in range(self.H):
            for ix in range(self.W):
                if self.occ[iy][ix]:
                    self.dist[iy][ix] = 0.0
                    heap.append((0.0, ix, iy))
        heapq.heapify(heap)
        while heap:
            d, ix, iy = heapq.heappop(heap)
            if d > self.dist[iy][ix] or d >= limit:
                continue
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = ix + dx, iy + dy
                    if 0 <= nx < self.W and 0 <= ny < self.H:
                        nd = d + (1.414 if dx and dy else 1.0)
                        if nd < self.dist[ny][nx]:
                            self.dist[ny][nx] = nd
                            heapq.heappush(heap, (nd, nx, ny))
        self.dist_m = [[v * RES for v in row] for row in self.dist]

    def cell(self, x, y):
        return int((x - ORIGIN[0]) / RES), int((y - ORIGIN[1]) / RES)

    def world(self, ix, iy):
        return ORIGIN[0] + (ix + 0.5) * RES, ORIGIN[1] + (iy + 0.5) * RES

    def inside(self, ix, iy):
        return 0 <= ix < self.W and 0 <= iy < self.H

    def blocked(self, ix, iy):
        return (not self.inside(ix, iy)) or self.dist_m[iy][ix] < R_BLOCK or (ix, iy) in self.dyn_block

    # ---------------------------------------------------------------- sensing
    def update_scan(self, ranges, angle_min, angle_step, now):
        x, y, th = self.pose
        lx, ly = x + 0.25 * math.cos(th), y + 0.25 * math.sin(th)       # lidar is 0.25 m ahead of base
        front = 99.0
        pts = []
        for i, r in enumerate(ranges):
            if not (0.14 < r < 8.0):
                continue
            a = angle_min + i * angle_step
            bx, by = 0.25 + r * math.cos(a), r * math.sin(a)
            if abs(bx) < 0.42 and abs(by) < 0.35:        # robot's own body
                continue
            if r < 3.0:
                ex, ey = lx + r * math.cos(th + a), ly + r * math.sin(th + a)
                cx, cy = self.cell(ex, ey)
                unknown = not self.inside(cx, cy) or self.dist_m[cy][cx] > 0.30
                pts.append((bx, by, unknown))
            if abs(wrap(a)) < math.radians(25):
                front = min(front, r)
            if r > 5.0:
                continue
            ex, ey = lx + r * math.cos(th + a), ly + r * math.sin(th + a)
            ix, iy = self.cell(ex, ey)
            if self.inside(ix, iy) and self.dist_m[iy][ix] > 0.30:       # not part of the known map
                self.dyn[(ix, iy)] = now
        self.front_min = front
        self.pts = pts

    def corridor_blocked(self, ahead=0.60, half=0.40):
        """Anything inside the box the robot is about to sweep through?"""
        return any(unk and 0.0 < px < 0.36 + ahead and abs(py) < half for px, py, unk in self.pts)

    def too_close(self, radius=0.52):
        return any(math.hypot(px, py) < radius for px, py, _ in self.pts)

    def refresh_dyn(self, now):
        for c in [c for c, t in self.dyn.items() if now - t > DYN_TTL]:
            del self.dyn[c]
        blk = set()
        for (ix, iy) in self.dyn:
            for dx, dy in self.dyn_offsets:
                blk.add((ix + dx, iy + dy))
        self.dyn_block = blk

    # ---------------------------------------------------------------- planning
    def line_free(self, a, b):
        (x0, y0), (x1, y1) = a, b
        n = max(abs(x1 - x0), abs(y1 - y0))
        for i in range(n + 1):
            t = i / n if n else 0
            if self.blocked(int(round(x0 + (x1 - x0) * t)), int(round(y0 + (y1 - y0) * t))):
                return False
        return True

    def plan(self, goal_xy, now):
        self.refresh_dyn(now)
        self.fail = ''
        s = self.cell(*self.pose[:2])
        g = self.cell(*goal_xy)
        if not self.inside(*s):
            self.fail = 'robot position is outside the map (bad localisation)'
            return None
        if not self.inside(*g):
            self.fail = 'goal outside the map'
            return None
        near_start = lambda c: abs(c[0] - s[0]) <= 4 and abs(c[1] - s[1]) <= 4
        free = lambda c: near_start(c) or not self.blocked(*c)
        if not free(g):
            self.fail = 'goal is blocked by an obstacle'
            return None
        openh = [(0.0, 0.0, s)]
        gs = {s: 0.0}
        came = {}
        while openh:
            f, gc, cur = heapq.heappop(openh)
            if cur == g:
                break
            if gc > gs.get(cur, 1e9):
                continue
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nb = (cur[0] + dx, cur[1] + dy)
                    if not self.inside(*nb) or not free(nb):
                        continue
                    if dx and dy and not (free((cur[0] + dx, cur[1])) and free((cur[0], cur[1] + dy))):
                        continue
                    d = self.dist_m[nb[1]][nb[0]]
                    pen = ((0.9 - d) / 0.5) ** 2 * 3.0 if d < 0.9 else 0.0
                    ng = gc + (1.414 if dx and dy else 1.0) + pen
                    if ng < gs.get(nb, 1e9):
                        gs[nb] = ng
                        came[nb] = cur
                        h = math.hypot(g[0] - nb[0], g[1] - nb[1])
                        heapq.heappush(openh, (ng + h, ng, nb))
        else:
            self.fail = 'no route (aisle blocked by obstacles)'
            return None
        if g not in came and g != s:
            self.fail = 'no route (aisle blocked by obstacles)'
            return None
        cells = [g]
        while cells[-1] != s:
            cells.append(came[cells[-1]])
        cells.reverse()
        # shortcut smoothing
        out = [cells[0]]
        i = 0
        while i < len(cells) - 1:
            j = len(cells) - 1
            while j > i + 1 and not self.line_free(cells[i], cells[j]):
                j -= 1
            out.append(cells[j])
            i = j
        pts = [self.world(*c) for c in out]
        pts[-1] = goal_xy
        dense = []
        for a, b in zip(pts, pts[1:]):
            n = max(1, int(math.dist(a, b) / 0.1))
            dense += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(n)]
        dense.append(pts[-1])
        return dense

    # ---------------------------------------------------------------- goal handling
    def set_goal(self, name, now):
        self.goal = STATIONS[name]
        self.goal_name = name
        self.blocked_since = None
        self.replan(now, force=True)

    def replan(self, now, force=False):
        if not force and now - self.last_plan < 0.8:
            return
        self.last_plan = now
        p = self.plan(self.goal[:2], now)
        if p:
            self.path, self.pi = p, 0
            self.state = 'driving'
            self.blocked_since = None
        else:
            self.path = []
            self.state = 'blocked'
            if self.blocked_since is None:
                self.blocked_since = now

    def path_blocked_ahead(self):
        n = 0
        for k in range(self.pi, len(self.path)):
            if self.blocked(*self.cell(*self.path[k])):
                return True
            n += 1
            if n > 50:
                break
        return False

    # ---------------------------------------------------------------- control
    def step(self, now):
        """Returns (linear, angular)."""
        if self.goal is None or self.state in ('idle', 'arrived', 'failed'):
            return 0.0, 0.0
        x, y, th = self.pose
        self.refresh_dyn(now)
        gx, gy, gyaw = self.goal
        dgoal = math.hypot(gx - x, gy - y)
        if self.state == 'rotating':
            e = wrap(gyaw - th)
            if abs(e) < 0.08:
                self.state = 'arrived'
                return 0.0, 0.0
            if self.too_close():
                return 0.0, 0.0
            return 0.0, max(-0.8, min(0.8, 1.5 * e))
        self.why = ''
        if self.state == 'blocked':
            self.why = f'no free path: {self.fail}; dyn cells={len(self.dyn)}'
            self.replan(now)
            if self.blocked_since and now - self.blocked_since > 25.0:
                self.state = 'failed'
            return 0.0, 0.0
        if self.path_blocked_ahead():
            self.replan(now)
            if self.state == 'blocked':
                return 0.0, 0.0
        if dgoal < 0.15:
            self.state = 'rotating'
            return 0.0, 0.0
        # closest point, then lookahead point
        best, bd = self.pi, 1e9
        for k in range(self.pi, min(len(self.path), self.pi + 40)):
            d = math.dist(self.path[k], (x, y))
            if d < bd:
                best, bd = k, d
        self.pi = best
        tgt = self.path[-1]
        for k in range(best, len(self.path)):
            if math.dist(self.path[k], (x, y)) >= LOOKAHEAD:
                tgt = self.path[k]
                break
        alpha = wrap(math.atan2(tgt[1] - y, tgt[0] - x) - th)
        if self.front_min < STOP_FRONT or self.corridor_blocked():
            self.why = f'obstacle ahead (lidar front {self.front_min:.2f} m)'
            self.replan(now)
            return 0.0, 0.0
        if abs(alpha) > 0.9:
            if self.too_close():
                return 0.0, 0.0
            return 0.0, max(-W_MAX, min(W_MAX, 1.6 * alpha))
        v = V_MAX * max(0.25, 1.0 - abs(alpha) / 1.1)
        v = min(v, 0.15 + 0.8 * dgoal)
        v = min(v, max(0.12, (self.front_min - 0.35) * 0.9))
        curv = 2.0 * math.sin(alpha) / max(0.4, math.hypot(tgt[0] - x, tgt[1] - y))
        w = max(-W_MAX, min(W_MAX, v * curv + 0.4 * alpha))
        return v, w


# ============================================================================ Gazebo glue
def main():
    args = [a for a in sys.argv[1:]]
    if '--list' in args:
        print('\n'.join(STATIONS))
        return
    try:
        from gz.msgs10.image_pb2 import Image
        from gz.msgs10.laserscan_pb2 import LaserScan
        from gz.msgs10.odometry_pb2 import Odometry
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.twist_pb2 import Twist
        from gz.transport13 import Node
    except ImportError:
        print('Missing python bindings. Run:\n  sudo apt install -y python3-gz-transport13 python3-gz-msgs10')
        sys.exit(1)
    try:
        import cv2
        import numpy as np
        qr_ok = True
    except ImportError:
        qr_ok = False
        print('QR reading is disabled. To enable it run:  sudo apt install -y python3-opencv')
    here = os.path.dirname(os.path.abspath(__file__))
    pgm = next((p for p in (os.path.join(here, 'factory_map.pgm'), os.path.expanduser('~/factory_map.pgm'))
                if os.path.exists(p)), None)
    print('Loading map ...' if pgm else 'Loading built-in map ...')
    nav = Navigator(pgm)
    lock = threading.Lock()
    node = Node()
    pub = node.advertise('/cmd_vel', Twist)
    got = dict(odom=False, scan=False, pose=False, use_odom=False)

    def on_pose(m):
        for p in m.pose:
            if p.name == 'amr':
                q = p.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                with lock:
                    nav.pose = (p.position.x, p.position.y, yaw)
                got['pose'] = True

    def on_odom(m):
        if not got['use_odom']:
            got['odom'] = True
            return
        c, s = math.cos(SPAWN[2]), math.sin(SPAWN[2])
        x = SPAWN[0] + c * m.pose.position.x - s * m.pose.position.y
        y = SPAWN[1] + s * m.pose.position.x + c * m.pose.position.y
        q = m.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        with lock:
            nav.pose = (x, y, wrap(yaw + SPAWN[2]))
        got['odom'] = True

    def on_scan(m):
        if not m.ranges:
            return
        step = m.angle_step or (2 * math.pi / len(m.ranges))
        with lock:
            nav.update_scan(list(m.ranges), m.angle_min, step, time.time())
        got['scan'] = True

    frame = dict(t=0.0, gray=None, n=0, err='')

    def on_cam(m):
        if not qr_ok or not m.data:
            return
        try:
            arr = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
            frame['gray'] = cv2.cvtColor(arr, cv2.COLOR_RGB2GRAY)
            frame['t'] = time.time()
            frame['n'] += 1
        except Exception as e:                      # keep the reason, show it later
            frame['err'] = f'{type(e).__name__}: {e} (data={len(m.data)} bytes, {m.width}x{m.height})'

    node.subscribe(Image, '/qr_camera', on_cam)
    node.subscribe(Odometry, '/odom', on_odom)
    node.subscribe(LaserScan, '/scan', on_scan)
    node.subscribe(Pose_V, '/world/factory/dynamic_pose/info', on_pose)
    node.subscribe(Pose_V, '/world/factory/pose/info', on_pose)
    print('Waiting for the simulation (lidar + position) ...')
    while not got['scan']:
        time.sleep(0.3)
    t_end = time.time() + 5.0
    while not got['pose'] and time.time() < t_end:
        time.sleep(0.2)
    if got['pose']:
        print('Position source: simulator (exact).')
    else:
        got['use_odom'] = True
        print('Position source: wheel odometry (assumes the AMR starts at the dock).')
    time.sleep(0.5)
    print('Connected. Robot at x=%.2f y=%.2f yaw=%.2f' % nav.pose)

    queue, patrol = [], '--patrol' in args
    queue += [a for a in args if a in STATIONS]
    unknown = [a for a in args if not a.startswith('--') and a not in STATIONS]
    if unknown:
        print('Unknown station(s):', unknown, '- run with --list')
        return
    interactive = not queue and not patrol
    names = list(STATIONS)
    stop = threading.Event()

    def send(v, w):
        m = Twist()
        m.linear.x, m.angular.z = v, w
        pub.publish(m)

    qr_log = []

    def read_qr(station):
        """Scan the floor QR code in front of the station and check it is the right one."""
        if not qr_ok:
            return
        expected = QR_TEXT[station]
        n0, t0, last = frame['n'], time.time(), 0.0
        text = ''
        while time.time() - t0 < 10.0 and not text:
            if frame['gray'] is not None and frame['t'] > t0 + 0.3 and frame['t'] != last:
                last = frame['t']
                text = decode_qr(frame['gray'])
            send(0.0, 0.0)
            time.sleep(0.15)
        if not text:
            got_frames = frame['n'] - n0
            print('   [QR] no QR code could be read.')
            if frame['err']:
                print('        camera image error:', frame['err'])
            elif frame['n'] == 0:
                print('        NO camera image received from the simulator: you are probably running the OLD')
                print('        factory_standalone.sdf (it must be ~772 KB, not 378 KB). Copy the new one and restart gz.')
            else:
                path = os.path.expanduser('~/qr_debug.png')
                cv2.imwrite(path, frame['gray'])
                print(f'        {got_frames} camera image(s) received but no QR decoded. Image saved to {path}')
            qr_log.append((station, None))
        elif text == expected:
            print(f"   [QR] scanned '{text}' -> station {station}: CORRECT")
            qr_log.append((station, text))
        else:
            where = QR_TO_STATION.get(text, 'unknown station')
            print(f"   [QR] scanned '{text}' ({where}) but expected '{expected}': WRONG STATION")
            qr_log.append((station, text))

    def driver():
        """Runs the mission queue and the control loop at 10 Hz."""
        pi_ = 0
        dbg = [0.0]
        current = None
        dwell_until = 0.0
        last_state = None
        while not stop.is_set():
            now = time.time()
            with lock:
                if current is None and now >= dwell_until:
                    if queue:
                        current = queue.pop(0)
                    elif patrol:
                        current = PATROL[pi_ % len(PATROL)]
                        pi_ += 1
                    if current:
                        nav.set_goal(current, now)
                        print(f'-> Going to {current}')
                v, w = nav.step(now) if current else (0.0, 0.0)
                st = nav.state
            send(v, w)
            if current and now - dbg[0] > 2.0:
                dbg[0] = now
                print(f'   [{st}] pos=({nav.pose[0]:.1f},{nav.pose[1]:.1f}) cmd v={v:.2f} w={w:.2f} lidar_front={nav.front_min:.2f} {nav.why}')
            if current and st != last_state:
                if st == 'blocked':
                    print('   path blocked - waiting / re-planning ...')
                last_state = st
            if current and st == 'arrived':
                print(f'   arrived at {current}')
                read_qr(current)
                current, dwell_until, last_state = None, time.time() + DWELL, None
                with lock:
                    nav.state = 'idle'
            elif current and st == 'failed':
                print(f'   could not reach {current} - skipping')
                current, last_state = None, None
                with lock:
                    nav.state = 'idle'
            time.sleep(0.1)
        send(0.0, 0.0)

    t = threading.Thread(target=driver, daemon=True)
    t.start()
    try:
        if interactive:
            while True:
                print('\nStations:')
                for i, n in enumerate(names):
                    print(f'  {i + 1:2d}. {n}')
                s = input('Number (or names separated by spaces, p = patrol, q = quit): ').strip()
                if s == 'q':
                    break
                if s == 'p':
                    patrol = True
                    input('Patrolling... press Enter to stop ')
                    patrol = False
                    continue
                for tok in s.split():
                    if tok.isdigit() and 1 <= int(tok) <= len(names):
                        queue.append(names[int(tok) - 1])
                    elif tok in STATIONS:
                        queue.append(tok)
        else:
            while t.is_alive():
                time.sleep(0.5)
                if not patrol and not queue and nav.state in ('idle',) and nav.goal and True:
                    time.sleep(DWELL + 0.5)
                    break
    except (KeyboardInterrupt, EOFError):
        pass
    stop.set()
    time.sleep(0.3)
    if qr_log:
        good = sum(1 for n_, t_ in qr_log if t_ == QR_TEXT[n_])
        print(f'QR summary: {good}/{len(qr_log)} stations verified by QR code')
    print('Stopped.')


if __name__ == '__main__':
    main()
