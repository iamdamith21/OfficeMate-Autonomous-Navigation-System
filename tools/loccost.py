#!/usr/bin/env python3
"""
loccost.py — report the COSTMAP COST at every saved location.

    loccost.py                 # all saved locations
    loccost.py base_station    # just one

Why this is needed
------------------
A location is saved by parking the robot there, which proves the robot FITS but
says nothing about whether Nav2 will agree to drive to it. Nav2 refuses goals
in inflated space: cells within the robot's circumscribed radius (0.30 m here)
of any obstacle are LETHAL regardless of inflation_radius, and lowering
inflation does not help -- it only shrinks the decay skirt. The planner will
still happily produce a path to such a cell, and then the controller refuses it
every cycle with "RegulatedPurePursuitController detected collision ahead!" and
"Controller patience exceeded". Nothing in that message names the goal as the
problem.

A "base station" is exactly the kind of spot that gets parked against a wall,
so it is the most likely location to be unreachable.

Cost scale
----------
/global_costmap/costmap is a nav_msgs/OccupancyGrid scaled 0-100 with -1 for
unknown -- NOT the raw 0-254 costmap scale. Reading it with raw thresholds
reports "0 lethal cells" on a costmap full of walls.

  0        free
  1-98     inflated: the closer to 99 the more likely the controller balks
  99       inscribed / lethal-adjacent
  100      lethal
  -1       unknown
"""
import math
import sys

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from robot_interfaces.srv import ListLocations

# The costmap is published transient-local (latched); a default subscription
# receives nothing and looks like "no costmap".
LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


def verdict(cost):
    if cost < 0:
        return 'UNKNOWN — outside mapped area, planner will refuse'
    if cost == 0:
        return 'FREE — good goal'
    if cost <= 50:
        return 'lightly inflated — usually fine'
    if cost <= 98:
        return 'HEAVILY INFLATED — controller may refuse (collision ahead)'
    return 'LETHAL — Nav2 will not drive here'


class LocCost(Node):
    def __init__(self):
        super().__init__('loccost')
        self.grid = None
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                self._cb, LATCHED)
        self._list = self.create_client(ListLocations, '/list_locations')

    def _cb(self, msg):
        self.grid = msg

    def wait_grid(self, timeout=20.0):
        end = self.get_clock().now().nanoseconds * 1e-9 + timeout
        while rclpy.ok() and self.grid is None:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.get_clock().now().nanoseconds * 1e-9 > end:
                raise SystemExit(
                    'ERROR: no /global_costmap/costmap received.\n'
                    '       Is Nav2 running? (~/fw_testing/start_nav.sh)')
        return self.grid

    def locations(self):
        if not self._list.wait_for_service(timeout_sec=5.0):
            raise SystemExit('ERROR: /list_locations unavailable — is '
                             'location_manager running?')
        fut = self._list.call_async(ListLocations.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        r = fut.result()
        return list(zip(r.names, r.poses))

    def cost_at(self, x, y):
        g = self.grid
        gx = int((x - g.info.origin.position.x) / g.info.resolution)
        gy = int((y - g.info.origin.position.y) / g.info.resolution)
        if not (0 <= gx < g.info.width and 0 <= gy < g.info.height):
            return None, (gx, gy)
        return g.data[gy * g.info.width + gx], (gx, gy)

    def best_nearby(self, x, y, max_r=0.60):
        """Nearest cost-0 cell, so a bad location can be nudged not abandoned."""
        g = self.grid
        res = g.info.resolution
        best = None
        steps = int(max_r / res)
        for dy in range(-steps, steps + 1):
            for dx in range(-steps, steps + 1):
                d = math.hypot(dx * res, dy * res)
                if d > max_r:
                    continue
                c, _ = self.cost_at(x + dx * res, y + dy * res)
                if c == 0 and (best is None or d < best[0]):
                    best = (d, x + dx * res, y + dy * res)
        return best


def main():
    only = sys.argv[1] if len(sys.argv) > 1 else None
    rclpy.init()
    n = LocCost()
    try:
        g = n.wait_grid()
        print('costmap %dx%d @ %.3f m, origin (%.2f, %.2f)'
              % (g.info.width, g.info.height, g.info.resolution,
                 g.info.origin.position.x, g.info.origin.position.y))
        print('scale: 0 free, 1-98 inflated, 99 inscribed, 100 lethal, -1 unknown\n')

        rows = n.locations()
        if only:
            rows = [(nm, p) for nm, p in rows if nm == only]
            if not rows:
                raise SystemExit(f'no saved location named "{only}"')

        bad = []
        for name, p in sorted(rows):
            x, y = p.pose.position.x, p.pose.position.y
            cost, (gx, gy) = n.cost_at(x, y)
            if cost is None:
                print('%-20s x=%+.3f y=%+.3f  OFF-MAP (cell %d,%d)'
                      % (name, x, y, gx, gy))
                bad.append(name)
                continue
            print('%-20s x=%+.3f y=%+.3f  cost=%3d  %s'
                  % (name, x, y, cost, verdict(cost)))
            if cost != 0:
                bad.append(name)
                b = n.best_nearby(x, y)
                if b:
                    print('%-20s   nearest FREE cell: x=%+.3f y=%+.3f '
                          '(%.2f m away)' % ('', b[1], b[2], b[0]))
                else:
                    print('%-20s   no free cell within 0.60 m' % '')

        if bad:
            print('\nPROBLEM: %s not on free cells.' % ', '.join(sorted(bad)))
            print('Nav2 can refuse these even though the robot physically fits.')
            print('Re-save from a nearby free cell, or nudge the goal there.')
        else:
            print('\nAll locations sit on free cells.')
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
