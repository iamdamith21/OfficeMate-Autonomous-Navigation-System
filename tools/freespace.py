#!/usr/bin/env python3
"""Report where Nav2 can actually plan to, straight from the global costmap.

"No valid path found" usually means the goal sits in inflated space, not that
the planner is broken. This reads /global_costmap/costmap and /amcl_pose and
prints the genuinely plannable cells, plus a few suggested goals reachable
from where the robot is standing — so goals can be chosen from data instead of
guessed off a map's bounding box.
"""
import math
from collections import deque

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, \
    QoSReliabilityPolicy

# /global_costmap/costmap is a nav_msgs/OccupancyGrid, so nav2 rescales the
# internal 0-254 cost range down to 0-100 and uses -1 for unknown. Reading it
# with the raw 0-255 thresholds reports "0 lethal cells" on a costmap that is
# in fact full of walls. 99 is the inscribed-obstacle level on this scale:
# at or above it the robot's footprint is in collision.
MAX_TRAVERSABLE = 99


class FreeSpace(Node):
    def __init__(self):
        super().__init__('freespace')
        latched = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.grid = None
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._grid, latched)
        # AMCL only publishes /amcl_pose after it updates, which needs motion.
        # TF always has the current map->base_footprint, so use that instead.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _grid(self, m):
        self.grid = m

    @property
    def pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()).transform
            class P:
                pass
            p = P()
            p.position = t.translation
            p.orientation = t.rotation
            return p
        except Exception:
            return None


def main():
    rclpy.init()
    n = FreeSpace()
    for _ in range(200):
        rclpy.spin_once(n, timeout_sec=0.05)
        if n.grid and n.pose:
            break

    if not n.grid:
        print('no /global_costmap/costmap received')
        return
    g = n.grid
    w, h, res = g.info.width, g.info.height, g.info.resolution
    ox, oy = g.info.origin.position.x, g.info.origin.position.y
    d = g.data

    free = sum(1 for v in d if 0 <= v < MAX_TRAVERSABLE)
    lethal = sum(1 for v in d if v >= MAX_TRAVERSABLE)
    unknown = sum(1 for v in d if v < 0)
    print(f'costmap {w}x{h} @{res} m, origin ({ox:.2f}, {oy:.2f})')
    print(f'  traversable {free} cells = {free*res*res:.2f} m^2')
    print(f'  lethal/inflated {lethal}, unknown {unknown}')

    if not n.pose:
        print('no /amcl_pose — cannot compute reachability')
        return
    rx, ry = n.pose.position.x, n.pose.position.y
    ci = int((rx - ox) / res)
    cj = int((ry - oy) / res)
    print(f'\nrobot at ({rx:+.2f}, {ry:+.2f}) -> cell ({ci}, {cj})')
    if not (0 <= ci < w and 0 <= cj < h):
        print('  ROBOT IS OUTSIDE THE COSTMAP')
        return
    cost_here = d[cj * w + ci]
    print(f'  cost under robot: {cost_here}'
          f'{"  <-- LETHAL, planner cannot start here" if cost_here >= MAX_TRAVERSABLE or cost_here < 0 else ""}')

    # Flood fill from the robot over traversable cells.
    seen = bytearray(w * h)
    q = deque()
    # If the robot is in inflated space, start from the nearest traversable
    # cell instead, otherwise the fill is empty and tells us nothing.
    start = None
    for r in range(0, 25):
        for dj in range(-r, r + 1):
            for di in range(-r, r + 1):
                if max(abs(di), abs(dj)) != r:
                    continue
                i, j = ci + di, cj + dj
                if 0 <= i < w and 0 <= j < h:
                    v = d[j * w + i]
                    if 0 <= v < MAX_TRAVERSABLE:
                        start = (i, j)
                        break
            if start:
                break
        if start:
            break
    if not start:
        print('  no traversable cell within 25 cells of the robot')
        return
    if start != (ci, cj):
        sx = ox + (start[0] + 0.5) * res
        sy = oy + (start[1] + 0.5) * res
        print(f'  nearest traversable cell is ({sx:+.2f}, {sy:+.2f}), '
              f'{math.hypot(sx-rx, sy-ry):.2f} m away')

    q.append(start)
    seen[start[1] * w + start[0]] = 1
    reach = []
    while q:
        i, j = q.popleft()
        reach.append((i, j))
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            i2, j2 = i + di, j + dj
            if 0 <= i2 < w and 0 <= j2 < h and not seen[j2 * w + i2]:
                v = d[j2 * w + i2]
                if 0 <= v < MAX_TRAVERSABLE:
                    seen[j2 * w + i2] = 1
                    q.append((i2, j2))

    print(f'\nreachable from robot: {len(reach)} cells = '
          f'{len(reach)*res*res:.2f} m^2')

    # Suggest the farthest reachable cells — those make the most useful test
    # goals, and their spread shows how much room the robot really has.
    cand = []
    for (i, j) in reach:
        x = ox + (i + 0.5) * res
        y = oy + (j + 0.5) * res
        cand.append((math.hypot(x - rx, y - ry), x, y, d[j * w + i]))
    cand.sort(reverse=True)
    print('\nfarthest reachable points (good test goals):')
    for dist, x, y, c in cand[:6]:
        print(f'  x={x:+.2f} y={y:+.2f}   {dist:.2f} m away, cost {c}')
    mid = cand[len(cand) // 2]
    print(f'\nmid-range goal: x={mid[1]:+.2f} y={mid[2]:+.2f} ({mid[0]:.2f} m)')

    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
