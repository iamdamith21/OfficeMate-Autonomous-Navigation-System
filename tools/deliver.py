#!/usr/bin/env python3
"""
deliver.py — send a delivery mission by LOCATION NAME and narrate the FSM.

    python3 deliver.py --sender supervisor_desk --recipient office
    python3 deliver.py --sender supervisor_desk --recipient office --tag A1B2C3D4
    python3 deliver.py --sender supervisor_desk --recipient office --dry

Preflight first, because every step of this FSM fails by TIMING OUT rather than
erroring — a missing service or an unlocalised robot otherwise surfaces as
"WAIT_FOR_RFID timeout" minutes into a run.
"""
import argparse
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from robot_interfaces.action import DeliveryMission
from robot_interfaces.msg import MissionState
from robot_interfaces.srv import GetLocation


class Runner(Node):
    def __init__(self, args):
        super().__init__('deliver_cli')
        self.args = args
        self.t0 = time.time()
        self.last = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, DeliveryMission, 'deliver')
        self.loc = self.create_client(GetLocation, '/get_location')
        self.create_subscription(MissionState, '/mission_state', self._state, 10)
        self.create_subscription(String, '/rfid/tag', self._rfid, 10)
        self.create_subscription(Bool, '/compartment/occupied', self._ir, 10)
        self.create_subscription(String, '/doors/state', self._door, 10)
        self.occupied = None
        self.doors = None

    def _t(self):
        return '[%6.1fs]' % (time.time() - self.t0)

    def _state(self, m):
        key = (m.state_name, m.detail)
        if key != self.last:
            self.last = key
            d = (' — ' + m.detail) if m.detail else ''
            r = (' (retry %d)' % m.retry_count) if m.retry_count else ''
            print('%s FSM  %-22s%s%s' % (self._t(), m.state_name, d, r),
                  flush=True)

    def _rfid(self, m):
        print('%s RFID %s' % (self._t(), m.data), flush=True)

    def _ir(self, m):
        if m.data != self.occupied:
            self.occupied = m.data
            print('%s IR   %s' % (self._t(),
                                  'FILES PRESENT' if m.data else 'EMPTY'),
                  flush=True)

    def _door(self, m):
        if m.data != self.doors:
            self.doors = m.data
            print('%s DOOR %s' % (self._t(), m.data), flush=True)

    def spin_for(self, s):
        end = time.time() + s
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def resolve(self, name):
        req = GetLocation.Request()
        req.name = name
        fut = self.loc.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        r = fut.result()
        return r if (r and r.success) else None

    def preflight(self):
        ok = True
        print('== PREFLIGHT ==')

        end = time.time() + 5
        pose = None
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                pose = self.tf_buffer.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time())
                break
            except Exception:
                continue
        if pose is None:
            print('  FAIL  no map->base_footprint TF — AMCL not localised?')
            print('        globalloc.py ~/maps/office_map.yaml --publish')
            ok = False
        else:
            print('  ok    localised at x=%.2f y=%.2f'
                  % (pose.transform.translation.x, pose.transform.translation.y))

        if not self.loc.wait_for_service(timeout_sec=5.0):
            print('  FAIL  /get_location absent — run location_manager')
            return False, None
        print('  ok    location_manager')

        for label, name in (('sender', self.args.sender),
                            ('recipient', self.args.recipient),
                            ('base', self.args.base)):
            r = self.resolve(name)
            if r is None:
                print(f'  FAIL  unknown {label} location "{name}"')
                print('        loc.py list   /   loc.py save <name>')
                ok = False
            else:
                print('  ok    %-10s "%s" x=%+.2f y=%+.2f'
                      % (label, name, r.pose.pose.position.x,
                         r.pose.pose.position.y))

        if not self.client.wait_for_server(timeout_sec=5.0):
            print('  FAIL  `deliver` action server absent — run delivery_manager')
            ok = False
        else:
            print('  ok    deliver action server')

        self.spin_for(3.0)
        print('  %s doors report %s'
              % ('ok   ' if self.doors else 'WARN ', self.doors or 'nothing'))
        if self.occupied is None:
            print('  WARN  no /compartment/occupied yet (IR)')
        else:
            print('  ok    compartment %s'
                  % ('OCCUPIED' if self.occupied else 'EMPTY'))
            if self.occupied:
                print('        NOTE: WAIT_FOR_FILE passes instantly — empty it '
                      'first for a realistic run')
        return ok, pose

    def run(self):
        ok, _ = self.preflight()
        if not ok:
            return 1

        g = DeliveryMission.Goal()
        g.mission_id = self.args.id
        g.sender_location = self.args.sender
        g.recipient_location = self.args.recipient
        g.recipient_rfid = self.args.tag
        g.base_location = self.args.base

        print('\n== MISSION ==')
        print('  %s -> %s -> %s' % (self.args.base, self.args.sender,
                                    self.args.recipient))
        print('  tag %s' % (self.args.tag or '<any>'))
        if self.args.dry:
            print('\n(dry run — nothing sent)')
            return 0

        print('\n== RUNNING ==', flush=True)
        send = self.client.send_goal_async(g)
        while rclpy.ok() and not send.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        gh = send.result()
        if not gh.accepted:
            print('goal REJECTED')
            return 1

        fut = gh.get_result_async()
        deadline = time.time() + self.args.timeout
        while rclpy.ok() and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() > deadline:
                print('\nTIMEOUT — cancelling')
                gh.cancel_goal_async()
                self.spin_for(20.0)
                return 1

        w = fut.result()
        print('\n== RESULT ==')
        print('  status  %s' % ('SUCCEEDED'
                                if w.status == GoalStatus.STATUS_SUCCEEDED
                                else w.status))
        print('  success %s' % w.result.success)
        print('  message %s' % w.result.message)
        return 0 if w.result.success else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sender', required=True)
    ap.add_argument('--recipient', required=True)
    ap.add_argument('--base', default='base_station')
    ap.add_argument('--tag', default='', help='expected RFID UID; empty = any')
    ap.add_argument('--id', default='mission-1')
    ap.add_argument('--timeout', type=float, default=900.0)
    ap.add_argument('--dry', action='store_true')
    args = ap.parse_args()

    rclpy.init()
    n = Runner(args)
    try:
        rc = n.run()
    except KeyboardInterrupt:
        rc = 130
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
