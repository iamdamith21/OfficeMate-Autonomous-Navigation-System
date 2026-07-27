#!/usr/bin/env python3
"""
globalloc.py — find where the robot actually is, by matching /scan to the map.

Why: AMCL was seeded at (0,0,0) and only 15% of scan endpoints landed on mapped
walls, so that pose is wrong. AMCL cannot fix this itself — it only refines
locally, and a stationary robot never even runs a filter update.

This does a brute-force global search: score every candidate (x, y, yaw) by how
well the projected scan endpoints line up with mapped obstacles, using a
likelihood field (distance transform of the occupancy grid) — the same scoring
AMCL's likelihood_field model uses, just searched exhaustively instead of with
particles.

Coarse pass over free space, then a fine refinement around the best candidate.
Prints the winning pose and its match quality; --publish seeds AMCL with it.

  python3 globalloc.py ~/maps/office_map.yaml
  python3 globalloc.py ~/maps/office_map.yaml --publish
"""
import argparse
import math
import os
import sys
import time
from collections import deque

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

SIGMA = 0.20        # m — likelihood field width, matches amcl sigma_hit-ish
N_BEAMS = 90        # subsample; 769 beams is far more than the search needs
MAX_R = 8.0


def load_map(path):
    meta = yaml.safe_load(open(path))
    img = os.path.join(os.path.dirname(path), meta['image'])
    f = open(img, 'rb')
    assert f.readline().strip() == b'P5'
    line = f.readline()
    while line.startswith(b'#'):
        line = f.readline()
    w, h = map(int, line.split())
    f.readline()
    data = np.frombuffer(f.read(), dtype=np.uint8)[:w * h].reshape(h, w)
    return meta, w, h, data


def distance_transform(occ, res):
    """Euclidean-ish distance (metres) from each cell to the nearest occupied."""
    h, w = occ.shape
    INF = 10 ** 9
    dist = np.full((h, w), INF, dtype=np.int32)
    dq = deque()
    ys, xs = np.nonzero(occ)
    for y, x in zip(ys, xs):
        dist[y, x] = 0
        dq.append((y, x))
    while dq:
        y, x = dq.popleft()
        d = dist[y, x] + 1
        for ny, nx in ((y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)):
            if 0 <= ny < h and 0 <= nx < w and dist[ny, nx] > d:
                dist[ny, nx] = d
                dq.append((ny, nx))
    return dist.astype(np.float32) * res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map_yaml')
    ap.add_argument('--publish', action='store_true',
                    help='seed AMCL with the winning pose')
    args = ap.parse_args()

    meta, w, h, data = load_map(args.map_yaml)
    res = meta['resolution']
    ox, oy = meta['origin'][0], meta['origin'][1]
    occ = data < 64
    free = data > 200
    print(f'map {w}x{h} @ {res} m, origin ({ox}, {oy})')
    print('building likelihood field...')
    dist = distance_transform(occ, res)
    lik = np.exp(-(dist ** 2) / (2 * SIGMA ** 2)).astype(np.float32)

    rclpy.init()
    n = Node('globalloc')
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)
    scans = []
    n.create_subscription(LaserScan, '/scan', lambda m: scans.append(m), qos)
    t = time.time()
    while time.time() - t < 10 and len(scans) < 3:
        rclpy.spin_once(n, timeout_sec=0.1)
    if not scans:
        print('no /scan')
        return
    m = scans[-1]

    rng = np.array(m.ranges, dtype=np.float32)
    ang = m.angle_min + np.arange(len(rng), dtype=np.float32) * m.angle_increment
    ok = np.isfinite(rng) & (rng > 0.15) & (rng < MAX_R)
    rng, ang = rng[ok], ang[ok]
    if len(rng) > N_BEAMS:
        idx = np.linspace(0, len(rng) - 1, N_BEAMS).astype(int)
        rng, ang = rng[idx], ang[idx]
    print(f'using {len(rng)} beams')

    def score_batch(xs, ys, yaws):
        """Mean likelihood over beams for each candidate pose."""
        best = np.full(len(xs), -1.0, dtype=np.float32)
        for k in range(len(xs)):
            a = ang + yaws[k]
            px = xs[k] + rng * np.cos(a)
            py = ys[k] + rng * np.sin(a)
            c = ((px - ox) / res).astype(np.int32)
            r = (h - 1 - ((py - oy) / res)).astype(np.int32)
            valid = (c >= 0) & (c < w) & (r >= 0) & (r < h)
            if not valid.any():
                best[k] = 0.0
                continue
            best[k] = lik[r[valid], c[valid]].sum() / len(rng)
        return best

    # ---- coarse pass over free space -------------------------------------
    ys_f, xs_f = np.nonzero(free)
    step = max(1, int(round(0.15 / res)))
    sel = (ys_f % step == 0) & (xs_f % step == 0)
    ys_f, xs_f = ys_f[sel], xs_f[sel]
    cand_x = xs_f * res + ox
    cand_y = (h - 1 - ys_f) * res + oy
    yaws = np.arange(0, 2 * math.pi, math.radians(10), dtype=np.float32)
    print(f'coarse: {len(cand_x)} positions x {len(yaws)} headings '
          f'= {len(cand_x)*len(yaws)} poses')

    t0 = time.time()
    bx = by = byaw = 0.0
    bs = -1.0
    for yaw in yaws:
        s = score_batch(cand_x, cand_y, np.full(len(cand_x), yaw, dtype=np.float32))
        i = int(np.argmax(s))
        if s[i] > bs:
            bs, bx, by, byaw = float(s[i]), float(cand_x[i]), float(cand_y[i]), float(yaw)
    print(f'coarse best: x={bx:+.2f} y={by:+.2f} yaw={math.degrees(byaw):+.0f}deg '
          f'score={bs:.3f}   ({time.time()-t0:.1f}s)')

    # ---- fine refinement --------------------------------------------------
    for span, dstep, astep in ((0.20, 0.05, 3.0), (0.06, 0.02, 1.0)):
        xs, ys, yy = [], [], []
        for dx in np.arange(-span, span + 1e-6, dstep):
            for dy in np.arange(-span, span + 1e-6, dstep):
                for da in np.arange(-astep * 4, astep * 4 + 1e-6, astep):
                    xs.append(bx + dx)
                    ys.append(by + dy)
                    yy.append(byaw + math.radians(da))
        s = score_batch(np.array(xs, dtype=np.float32),
                        np.array(ys, dtype=np.float32),
                        np.array(yy, dtype=np.float32))
        i = int(np.argmax(s))
        if s[i] > bs:
            bs, bx, by, byaw = float(s[i]), xs[i], ys[i], yy[i]
    byaw = math.atan2(math.sin(byaw), math.cos(byaw))
    print(f'\nBEST POSE: x={bx:+.3f} y={by:+.3f} yaw={math.degrees(byaw):+.1f}deg')
    print(f'match score: {bs:.3f}  (1.0 = every beam exactly on a wall)')
    print('quality:', 'GOOD' if bs > 0.55 else 'MARGINAL' if bs > 0.35 else 'POOR')

    if args.publish:
        pub = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = n.get_clock().now().to_msg()
        msg.pose.pose.position.x = bx
        msg.pose.pose.position.y = by
        msg.pose.pose.orientation.z = math.sin(byaw / 2)
        msg.pose.pose.orientation.w = math.cos(byaw / 2)
        msg.pose.covariance[0] = 0.1
        msg.pose.covariance[7] = 0.1
        msg.pose.covariance[35] = 0.03
        for _ in range(4):
            pub.publish(msg)
            rclpy.spin_once(n, timeout_sec=0.15)
        print('published to /initialpose')

    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
