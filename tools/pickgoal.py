#!/usr/bin/env python3
"""
pickgoal.py — choose a nav goal with genuine CLEARANCE, not just reachability.

freespace.py answers "can the planner reach this cell", which is why its
suggestions came back at cost 81-97. That is nearly the inscribed threshold, so
RegulatedPurePursuit rejects the path with "detected collision ahead!" even
though a plan exists. Cells within the robot's 0.30 m circumscribed radius of
any obstacle are lethal no matter what inflation_radius is set to, so the fix
is to pick goals by cost, not distance.

Prints reachable cells sorted by cost (lowest = most open), with distance from
the robot, so a test goal can be chosen that the controller will actually
accept.

  /global_costmap/costmap is nav_msgs/OccupancyGrid scaled 0-100 with -1
  unknown -- NOT raw 0-254. Reading it with raw thresholds reports "0 lethal
  cells" on a costmap full of walls.
"""
import math
from collections import deque

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from tf2_ros import Buffer, TransformListener

MAX_TRAVERSABLE = 99      # inscribed on the 0-100 scale
GOOD_COST = 40            # comfortable margin for the controller


class PickGoal(Node):
    def __init__(self):
        super().__init__('pickgoal')
        self.grid = None
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._cb, qos)
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)

    def _cb(self, msg):
        self.grid = msg

    def run(self):
        import time
        end = time.time() + 10
        while rclpy.ok() and time.time() < end and self.grid is None:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.grid is None:
            print('no costmap received')
            return 1

        g = self.grid
        w, h, res = g.info.width, g.info.height, g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        data = g.data
        print('costmap %dx%d @%.3f m origin (%.2f, %.2f)' % (w, h, res, ox, oy))

        # robot pose
        rx = ry = None
        end = time.time() + 5
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            try:
                tf = self.buf.lookup_transform('map', 'base_footprint',
                                               rclpy.time.Time())
                rx = tf.transform.translation.x
                ry = tf.transform.translation.y
                break
            except Exception:
                continue
        if rx is None:
            print('no map->base_footprint TF')
            return 1

        cx = int((rx - ox) / res)
        cy = int((ry - oy) / res)
        print('robot at (%.2f, %.2f) cell (%d, %d) cost %d'
              % (rx, ry, cx, cy, data[cy * w + cx]))

        # flood fill over traversable cells
        seen = {(cx, cy)}
        q = deque([(cx, cy)])
        reach = []
        while q:
            x, y = q.popleft()
            c = data[y * w + x]
            reach.append((x, y, c))
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = x + dx, y + dy
                if not (0 <= nx < w and 0 <= ny < h) or (nx, ny) in seen:
                    continue
                v = data[ny * w + nx]
                if v < 0 or v > MAX_TRAVERSABLE:
                    continue
                seen.add((nx, ny))
                q.append((nx, ny))

        good = [(x, y, c) for x, y, c in reach if 0 <= c <= GOOD_COST]
        print('reachable %d cells, of which cost<=%d: %d'
              % (len(reach), GOOD_COST, len(good)))
        if not good:
            print('NO low-cost cells reachable — the space is too tight for '
                  'this footprint; nothing the controller will accept.')
            return 1

        scored = []
        for x, y, c in good:
            gx = ox + x * res
            gy = oy + y * res
            d = math.hypot(gx - rx, gy - ry)
            scored.append((d, c, gx, gy))

        # Constrain to a distance BAND, not just "far".
        #
        # This map has known boundary gaps, so a flood fill leaks out of the
        # room and finds wide-open cost-0 cells at the map edge. They are
        # perfectly traversable on paper and completely wrong in reality — the
        # robot would drive at a gap in the wall. Keeping goals within a couple
        # of metres keeps them inside the room that was actually surveyed.
        import sys
        dmin = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
        dmax = float(sys.argv[2]) if len(sys.argv) > 2 else 2.2

        band = [s for s in scored if dmin <= s[0] <= dmax]
        band.sort(key=lambda s: (s[1], -s[0]))
        print('\nbest low-cost goals %.1f-%.1f m away (cost, distance):'
              % (dmin, dmax))
        for d, c, gx, gy in band[:8]:
            print('  x=%+.2f y=%+.2f   %.2f m   cost %d' % (gx, gy, d, c))

        if band:
            d, c, gx, gy = band[0]
            print('\nRECOMMENDED:  --goal %.2f %.2f 0   (cost %d, %.2f m)'
                  % (gx, gy, c, d))
        else:
            print('  none in that band')
        return 0


def main():
    rclpy.init()
    n = PickGoal()
    try:
        rc = n.run()
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
    return rc


if __name__ == '__main__':
    raise SystemExit(main())
