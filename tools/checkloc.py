#!/usr/bin/env python3
"""
checkloc.py — is AMCL's pose estimate actually correct?

AMCL reports a covariance, but a stationary robot never runs a filter update
(update_min_d/update_min_a), so that covariance is just the seed value — it
says nothing about whether the pose is right. Driving on a wrong pose means
the planner plans from the wrong place and the robot drives into things.

This checks it directly: take the live /scan, project every endpoint into the
map frame using the CURRENT map->base_footprint transform, and see what
fraction land on cells the map says are occupied. A well-localised robot puts
most returns on walls.

  >70%  good        — the scan lines up with the map
  40-70% marginal   — partially matched, nudge the pose estimate
  <40%  bad         — do not navigate; re-seed with 2D Pose Estimate
"""
import math
import sys
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

MAP_YAML = sys.argv[1] if len(sys.argv) > 1 else \
    '/home/damith-raspberry/maps/office_map.yaml'
TOL_CELLS = 2   # a hit within this many cells of occupied counts as matched


def load_map(path):
    meta = yaml.safe_load(open(path))
    import os
    img = os.path.join(os.path.dirname(path), meta['image'])
    f = open(img, 'rb')
    assert f.readline().strip() == b'P5'
    line = f.readline()
    while line.startswith(b'#'):
        line = f.readline()
    w, h = map(int, line.split())
    f.readline()
    data = f.read()
    return meta, w, h, data


def main():
    meta, w, h, data = load_map(MAP_YAML)
    res = meta['resolution']
    ox, oy = meta['origin'][0], meta['origin'][1]
    occ = [[data[r * w + c] < 64 for c in range(w)] for r in range(h)]

    def near_occupied(x, y):
        c = int((x - ox) / res)
        r = h - 1 - int((y - oy) / res)
        for dr in range(-TOL_CELLS, TOL_CELLS + 1):
            for dc in range(-TOL_CELLS, TOL_CELLS + 1):
                rr, cc = r + dr, c + dc
                if 0 <= rr < h and 0 <= cc < w and occ[rr][cc]:
                    return True
        return False

    def in_map(x, y):
        c = int((x - ox) / res)
        r = h - 1 - int((y - oy) / res)
        return 0 <= r < h and 0 <= c < w

    rclpy.init()
    n = Node('checkloc')
    buf = Buffer()
    TransformListener(buf, n)
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)
    scans = []
    n.create_subscription(LaserScan, '/scan', lambda m: scans.append(m), qos)

    t = time.time()
    while time.time() - t < 10 and not scans:
        rclpy.spin_once(n, timeout_sec=0.1)
    if not scans:
        print('no /scan received')
        return

    tf = None
    t = time.time()
    while time.time() - t < 10:
        rclpy.spin_once(n, timeout_sec=0.1)
        try:
            tf = buf.lookup_transform('map', 'laser', rclpy.time.Time())
            break
        except Exception:
            pass
    if tf is None:
        print('no map->laser transform')
        return

    tr = tf.transform.translation
    q = tf.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    print(f'laser in map: x={tr.x:+.3f} y={tr.y:+.3f} yaw={math.degrees(yaw):+.1f}deg')

    m = scans[-1]
    total = hit = outside = 0
    for i, r in enumerate(m.ranges):
        if not math.isfinite(r) or r < 0.15 or r > 8.0:
            continue
        a = m.angle_min + i * m.angle_increment + yaw
        x = tr.x + r * math.cos(a)
        y = tr.y + r * math.sin(a)
        if not in_map(x, y):
            outside += 1
            continue
        total += 1
        if near_occupied(x, y):
            hit += 1

    if total == 0:
        print('no usable returns landed inside the map')
        return
    pct = 100.0 * hit / total
    print(f'scan endpoints inside map : {total}   (outside: {outside})')
    print(f'landing on mapped walls   : {hit}  = {pct:.1f}%')
    verdict = ('GOOD — safe to navigate' if pct > 70 else
               'MARGINAL — nudge the pose estimate' if pct > 40 else
               'BAD — do NOT navigate, re-seed the pose')
    print(f'VERDICT: {verdict}')
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
