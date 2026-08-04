#!/usr/bin/env python3
"""
loc.py — save and manage named delivery locations.

    python3 loc.py save base_station     # record where the robot is standing
    python3 loc.py list
    python3 loc.py get office
    python3 loc.py delete office

A thin client over the location_manager services, so it does exactly what the
web app's "Save Current Position" button will do. Saving an existing name
overwrites it — that is how you nudge a destination after re-mapping.
"""
import sys

import rclpy
from rclpy.node import Node

from robot_interfaces.srv import (DeleteLocation, GetLocation, ListLocations,
                                  SaveLocation)


def yaw_of(q):
    import math
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


class Loc(Node):
    def __init__(self):
        super().__init__('loc_cli')

    def call(self, cli_type, name, req):
        cli = self.create_client(cli_type, name)
        if not cli.wait_for_service(timeout_sec=5.0):
            print(f'ERROR: {name} unavailable — is location_manager running?')
            print('       ros2 run mission_manager location_manager')
            return None
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        return fut.result()


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    cmd = sys.argv[1]
    arg = sys.argv[2] if len(sys.argv) > 2 else ''

    rclpy.init()
    n = Loc()
    rc = 0
    try:
        if cmd == 'save':
            if not arg:
                print('usage: loc.py save <name>')
                return 2
            r = n.call(SaveLocation, '/save_location',
                       SaveLocation.Request(name=arg))
            if r is None:
                rc = 1
            else:
                print(('OK  ' if r.success else 'FAIL ') + r.message)
                rc = 0 if r.success else 1

        elif cmd == 'list':
            r = n.call(ListLocations, '/list_locations',
                       ListLocations.Request())
            if r is None:
                rc = 1
            else:
                print(r.message)
                for name, p in zip(r.names, r.poses):
                    print('  %-20s x=%+.3f y=%+.3f yaw=%+.1fdeg'
                          % (name, p.pose.position.x, p.pose.position.y,
                             yaw_of(p.pose.orientation)))
                if not r.names:
                    print('  (none yet — drive the robot somewhere and '
                          '`loc.py save <name>`)')

        elif cmd == 'get':
            r = n.call(GetLocation, '/get_location',
                       GetLocation.Request(name=arg))
            if r is None:
                rc = 1
            elif r.success:
                p = r.pose
                print('%s  x=%+.3f y=%+.3f yaw=%+.1fdeg'
                      % (arg, p.pose.position.x, p.pose.position.y,
                         yaw_of(p.pose.orientation)))
            else:
                print('FAIL ' + r.message)
                rc = 1

        elif cmd == 'delete':
            r = n.call(DeleteLocation, '/delete_location',
                       DeleteLocation.Request(name=arg))
            if r is None:
                rc = 1
            else:
                print(('OK  ' if r.success else 'FAIL ') + r.message)
                rc = 0 if r.success else 1
        else:
            print(__doc__)
            rc = 2
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
