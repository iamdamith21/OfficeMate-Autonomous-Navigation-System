#!/usr/bin/env python3
"""
navto.py — navigate to SAVED LOCATION NAMES through Nav2.

    navto.py base_station                                   # single goal
    navto.py base_station sender_desk:5 recipient_desk      # sequence
    navto.py --list
    navto.py base_station --dry                             # resolve only

A waypoint is `name` or `name:seconds`, where seconds is how long to WAIT
parked at that waypoint before moving on. So the two-leg delivery rehearsal is

    navto.py base_station sender_desk:5 recipient_desk

Why this exists instead of goto.py or deliver.py
------------------------------------------------
goto.py is not navigation: no planner, no costmap, no obstacle avoidance -- it
drives a straight line into whatever is in the way. deliver.py is the other
extreme: it runs the full DeliveryMission FSM with RFID, doors and the IR
compartment check, so it cannot be used to rehearse pure navigation.

This sits in between: real Nav2 goals (planner + costmaps + recovery), by name,
with nothing else in the loop. Names resolve through location_manager's
/get_location, the same service the web app uses, so a name means exactly the
same pose here as it does in a real mission.

Heading matters
---------------
The saved pose includes yaw, and it is sent as part of the goal -- the robot
arrives FACING the direction it was pointing when the location was saved, not
merely standing on the spot. That is what makes a dropoff repeatable.

Localisation first
------------------
Nav2 plans in the map frame, so a wrong AMCL pose sends the robot confidently
to the wrong place. AMCL's covariance is meaningless while stationary
(update_min_d), so it happily reports confidence on a completely wrong pose.
Run `checkloc.py <map.yaml>` first: >70% good, <40% do not navigate.
"""
import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from robot_interfaces.srv import GetLocation, ListLocations

# The costmap is published transient-local (latched); a default subscription
# receives nothing and looks like "no costmap".
LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


def yaw_deg(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def parse_waypoint(token):
    """`name` or `name:seconds` -> (name, wait_seconds)."""
    if ':' in token:
        name, _, secs = token.rpartition(':')
        try:
            return name, float(secs)
        except ValueError:
            raise SystemExit(f'bad wait time in "{token}" — expected name:seconds')
    return token, 0.0


class NavTo(Node):
    def __init__(self):
        super().__init__('navto')
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._get = self.create_client(GetLocation, '/get_location')
        self._list = self.create_client(ListLocations, '/list_locations')
        self.grid = None
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._grid_cb, LATCHED)

    def _grid_cb(self, msg):
        self.grid = msg

    # ---- costmap-aware goal snapping -------------------------------------

    def wait_grid(self, timeout=15.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and self.grid is None:
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.monotonic() > deadline:
                return None
        return self.grid

    def cost_at(self, x, y):
        """Cost on the 0-100 OccupancyGrid scale, -1 unknown, None off-map.

        /global_costmap/costmap is scaled 0-100, NOT the raw 0-254 costmap
        scale -- raw thresholds report "0 lethal cells" on a map full of walls.
        """
        g = self.grid
        gx = int((x - g.info.origin.position.x) / g.info.resolution)
        gy = int((y - g.info.origin.position.y) / g.info.resolution)
        if not (0 <= gx < g.info.width and 0 <= gy < g.info.height):
            return None
        return g.data[gy * g.info.width + gx]

    def clear(self, x, y, clearance):
        """True if (x,y) and everything within `clearance` is cost 0."""
        res = self.grid.info.resolution
        steps = int(clearance / res)
        for dy in range(-steps, steps + 1):
            for dx in range(-steps, steps + 1):
                if math.hypot(dx * res, dy * res) > clearance:
                    continue
                if self.cost_at(x + dx * res, y + dy * res) != 0:
                    return False
        return True

    def snap(self, name, pose, max_r=1.00, clearance=0.25):
        """Nudge a goal onto the nearest cell with real CLEARANCE around it.

        A saved location only proves the robot FITS there. Nav2 refuses goals
        inside the robot's circumscribed radius of an obstacle -- those cells
        are lethal whatever inflation_radius says -- and the failure surfaces
        as the controller reporting "collision ahead" every cycle, never as
        "your goal is unreachable".

        A single cost-0 cell is NOT enough, which cost 0.17 m of stall to
        learn: RPP projects the robot's footprint forward along the path
        (max_allowed_time_to_collision_up_to_carrot), so a free cell wedged
        against a wall still trips "collision ahead" on the final approach.
        Requiring a clear disc means the arriving robot has room to sit and to
        rotate to the goal heading. Heading is preserved; only x/y move.
        """
        if self.grid is None:
            return pose, None
        x0, y0 = pose.pose.position.x, pose.pose.position.y
        c = self.cost_at(x0, y0)
        if c == 0 and self.clear(x0, y0, clearance):
            return pose, None

        res = self.grid.info.resolution
        steps = int(max_r / res)
        best = None
        for dy in range(-steps, steps + 1):
            for dx in range(-steps, steps + 1):
                d = math.hypot(dx * res, dy * res)
                if d > max_r or (best and d >= best[0]):
                    continue
                nx, ny = x0 + dx * res, y0 + dy * res
                if self.clear(nx, ny, clearance):
                    best = (d, nx, ny)
        if best is None:
            print(f'     !! {name} cost={c}, no cell with {clearance:.2f} m '
                  f'clearance within {max_r:.2f} m — leaving as-is, Nav2 will '
                  f'likely refuse')
            return pose, None

        d, nx, ny = best
        pose.pose.position.x, pose.pose.position.y = nx, ny
        print(f'     snapped {name}: cost={c} -> clear cell {d:.2f} m away '
              f'(x={nx:+.3f} y={ny:+.3f}, {clearance:.2f} m clearance)')
        return pose, (c, d)

    # ---- name resolution -------------------------------------------------

    def _call(self, cli, srv_name, req):
        if not cli.wait_for_service(timeout_sec=5.0):
            raise SystemExit(
                f'ERROR: {srv_name} unavailable — is location_manager running?\n'
                '       ~/fw_testing/start_locations.sh')
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        return fut.result()

    def list_locations(self):
        r = self._call(self._list, '/list_locations', ListLocations.Request())
        print(r.message)
        for name, p in zip(r.names, r.poses):
            print('  %-20s x=%+.3f y=%+.3f yaw=%+.1fdeg'
                  % (name, p.pose.position.x, p.pose.position.y,
                     yaw_deg(p.pose.orientation)))
        return list(r.names)

    def resolve(self, name):
        r = self._call(self._get, '/get_location',
                       GetLocation.Request(name=name))
        if r is None or not r.success:
            msg = getattr(r, 'message', 'no response')
            raise SystemExit(f'ERROR: cannot resolve "{name}": {msg}')
        return r.pose

    # ---- navigation ------------------------------------------------------

    def wait_for_server(self, timeout=20.0):
        if not self.nav.wait_for_server(timeout_sec=timeout):
            raise SystemExit(
                'ERROR: navigate_to_pose action server not available.\n'
                '       Nav2 is not up (or still inactive). Start it with\n'
                '       ~/fw_testing/start_nav.sh and give it ~20 s.')

    def go(self, name, pose, timeout):
        """Send one goal and block until it settles. True if it succeeded."""
        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        print('  -> %s  x=%+.3f y=%+.3f yaw=%+.1fdeg'
              % (name, pose.pose.position.x, pose.pose.position.y,
                 yaw_deg(pose.pose.orientation)), flush=True)

        state = {'dist': None}

        def on_feedback(fb):
            state['dist'] = fb.feedback.distance_remaining

        send = self.nav.send_goal_async(goal, feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            print('     REJECTED by Nav2')
            return False

        result_fut = handle.get_result_async()
        start = time.monotonic()
        last_print = 0.0
        try:
            while rclpy.ok() and not result_fut.done():
                rclpy.spin_once(self, timeout_sec=0.1)
                elapsed = time.monotonic() - start
                if elapsed - last_print >= 2.0:
                    last_print = elapsed
                    d = state['dist']
                    d_s = f'{d:.2f} m' if d is not None else '(no feedback yet)'
                    print(f'     {elapsed:5.1f}s  remaining {d_s}', flush=True)
                if elapsed > timeout:
                    print(f'     TIMEOUT after {timeout:.0f}s — cancelling')
                    cancel = handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(self, cancel,
                                                     timeout_sec=5.0)
                    return False
        except KeyboardInterrupt:
            print('\n     interrupted — cancelling goal')
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel, timeout_sec=5.0)
            raise

        status = result_fut.result().status
        ok = status == GoalStatus.STATUS_SUCCEEDED
        label = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
        }.get(status, f'status={status}')
        print(f'     {label} in {time.monotonic()-start:.1f}s', flush=True)
        if not ok:
            print('     (ABORTED usually means the planner could not find a '
                  'path, or localisation drifted — re-run checkloc.py)')
        return ok


def main():
    ap = argparse.ArgumentParser(
        description='Navigate to saved location names via Nav2.')
    ap.add_argument('waypoints', nargs='*',
                    help='name or name:wait_seconds, in order')
    ap.add_argument('--list', action='store_true',
                    help='list saved locations and exit')
    ap.add_argument('--dry', action='store_true',
                    help='resolve names and print the plan, navigate nothing')
    ap.add_argument('--timeout', type=float, default=120.0,
                    help='per-goal timeout in seconds (default 120)')
    ap.add_argument('--snap', action='store_true',
                    help='nudge goals off inflated cells onto the nearest free '
                         'cell (does NOT modify saved locations)')
    args = ap.parse_args()

    if not args.waypoints and not args.list:
        ap.print_help()
        return 2

    rclpy.init()
    n = NavTo()
    rc = 0
    try:
        if args.list:
            n.list_locations()
            return 0

        legs = [parse_waypoint(t) for t in args.waypoints]

        # Resolve every name BEFORE moving, so a typo in the last waypoint
        # fails now rather than after the robot has already driven two legs.
        resolved = [(name, n.resolve(name), wait) for name, wait in legs]

        if args.snap:
            if n.wait_grid() is None:
                print('WARNING: no /global_costmap/costmap — cannot snap, '
                      'sending goals as saved.')
            else:
                print('\nchecking goals against the global costmap:')
                clean = True
                for i, (name, pose, wait) in enumerate(resolved):
                    pose, changed = n.snap(name, pose)
                    resolved[i] = (name, pose, wait)
                    if changed:
                        clean = False
                if clean:
                    print('     all goals already on free cells')

        print('\nPlan:')
        for i, (name, pose, wait) in enumerate(resolved, 1):
            w = f'  then wait {wait:g}s' if wait else ''
            print('  %d. %-20s x=%+.3f y=%+.3f yaw=%+.1fdeg%s'
                  % (i, name, pose.pose.position.x, pose.pose.position.y,
                     yaw_deg(pose.pose.orientation), w))

        if args.dry:
            print('\n--dry: nothing sent.')
            return 0

        n.wait_for_server()

        print(f'\n=== navigating {len(resolved)} leg(s) ===')
        for i, (name, pose, wait) in enumerate(resolved, 1):
            print(f'\n[leg {i}/{len(resolved)}] {name}')
            if not n.go(name, pose, args.timeout):
                print(f'\nSTOPPED at leg {i} ({name}) — not continuing.')
                rc = 1
                break
            if wait:
                print(f'     parked, waiting {wait:g}s ...', flush=True)
                end = time.monotonic() + wait
                while rclpy.ok() and time.monotonic() < end:
                    rclpy.spin_once(n, timeout_sec=0.1)
        else:
            print('\n=== all legs SUCCEEDED ===')
    except KeyboardInterrupt:
        print('\ninterrupted.')
        rc = 130
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
