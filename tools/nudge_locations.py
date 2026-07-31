#!/usr/bin/env python3
"""
nudge_locations.py — move saved locations onto ground Nav2 will actually enter.

Why this is needed
------------------
A location is recorded by parking the robot there and reading TF, which proves
the robot FITS but not that Nav2 will drive to it. Cells within the robot's
0.30 m circumscribed radius of an obstacle are lethal whatever inflation_radius
says, so anything parked against a wall becomes an unreachable goal. The
delivery FSM navigates to the STORED pose and has no snapping of its own, so
such a location fails every leg that targets it — three nav retries, then
MISSION_FAILED, with nothing naming the goal as the cause.

A single cost-0 cell is NOT enough. RPP projects the footprint forward along the
path, so a free cell wedged against a wall still trips "collision ahead" on the
final approach. This therefore requires a clear DISC (default 0.25 m) around the
candidate, which also leaves the arriving robot room to rotate to its heading.

Headings are preserved untouched — only x/y move. locations.json is backed up
before anything is written, and each moved entry records where it came from.

location_manager caches locations in memory at startup, so RESTART THE MISSION
STACK after running this or the old poses stay live.
"""
import argparse
import json
import math
import os
import shutil
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

# The costmap is latched; a default subscription receives nothing and looks
# exactly like "Nav2 is not running".
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)


class Costmap(Node):
    def __init__(self):
        super().__init__('nudge_locations')
        self.grid = None
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._cb, LATCHED)

    def _cb(self, msg):
        self.grid = msg

    def wait(self, timeout=20.0):
        end = time.monotonic() + timeout
        while rclpy.ok() and self.grid is None:
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.monotonic() > end:
                raise SystemExit('ERROR: no /global_costmap/costmap — is Nav2 up?')
        return self.grid

    def cost(self, x, y):
        g = self.grid
        gx = int((x - g.info.origin.position.x) / g.info.resolution)
        gy = int((y - g.info.origin.position.y) / g.info.resolution)
        if not (0 <= gx < g.info.width and 0 <= gy < g.info.height):
            return None
        return g.data[gy * g.info.width + gx]

    def clear(self, x, y, r):
        res = self.grid.info.resolution
        steps = int(r / res)
        for dy in range(-steps, steps + 1):
            for dx in range(-steps, steps + 1):
                if math.hypot(dx * res, dy * res) > r:
                    continue
                if self.cost(x + dx * res, y + dy * res) != 0:
                    return False
        return True

    def nearest_clear(self, x, y, r, max_r):
        res = self.grid.info.resolution
        steps = int(max_r / res)
        best = None
        for dy in range(-steps, steps + 1):
            for dx in range(-steps, steps + 1):
                d = math.hypot(dx * res, dy * res)
                if d > max_r or (best and d >= best[0]):
                    continue
                nx, ny = x + dx * res, y + dy * res
                if self.clear(nx, ny, r):
                    best = (d, nx, ny)
        return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--file', default=os.path.expanduser('~/maps/locations.json'))
    ap.add_argument('--map-name', default='office_map_v2')
    ap.add_argument('--clearance', type=float, default=0.25)
    ap.add_argument('--max-move', type=float, default=1.00)
    ap.add_argument('--apply', action='store_true',
                    help='write the file; without this it only reports')
    args = ap.parse_args()

    with open(args.file) as f:
        data = json.load(f)
    locs = data.get(args.map_name)
    if not locs:
        raise SystemExit(f'no locations for map "{args.map_name}" in {args.file}')

    rclpy.init()
    cm = Costmap()
    try:
        g = cm.wait()
        print('costmap %dx%d @ %.3f m, clearance %.2f m\n'
              % (g.info.width, g.info.height, g.info.resolution, args.clearance))

        changed = 0
        for name in sorted(locs):
            e = locs[name]
            x, y = e['x'], e['y']
            c = cm.cost(x, y)
            if c == 0 and cm.clear(x, y, args.clearance):
                print('%-16s cost=%-4s OK — leaving alone' % (name, c))
                continue
            hit = cm.nearest_clear(x, y, args.clearance, args.max_move)
            if not hit:
                print('%-16s cost=%-4s NO clear cell within %.2f m — UNCHANGED'
                      % (name, c, args.max_move))
                continue
            d, nx, ny = hit
            print('%-16s cost=%-4s (%+.3f, %+.3f) -> (%+.3f, %+.3f)  moved %.2f m'
                  % (name, c, x, y, nx, ny, d))
            if args.apply:
                e.setdefault('surveyed_at', {'x': x, 'y': y})
                e['x'], e['y'] = round(nx, 4), round(ny, 4)
                e['nudged'] = ('moved %.2f m off the surveyed spot for %.2f m '
                               'Nav2 clearance' % (d, args.clearance))
            changed += 1

        if not args.apply:
            print('\n(dry run — pass --apply to write)')
            return

        if changed:
            bak = args.file + '.bak-' + time.strftime('%Y%m%d-%H%M%S')
            shutil.copy2(args.file, bak)
            tmp = args.file + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(data, f, indent=2, sort_keys=True)
            os.replace(tmp, args.file)   # atomic, like location_manager's own write
            print(f'\nwrote {args.file}  (backup: {bak})')
            print('RESTART the mission stack — location_manager caches these at startup.')
        else:
            print('\nnothing to change')
    finally:
        cm.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
