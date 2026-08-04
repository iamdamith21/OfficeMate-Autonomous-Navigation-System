#!/usr/bin/env python3
"""
save_location.py — Location Manager for OfficeMate.

Records, lists, and deletes named robot pose coordinates directly from the
live TF tree (map -> base_footprint) into ~/maps/locations.json.

Usage:
  python3 ~/officemate_tools/save_location.py save <name> [map_name]
  python3 ~/officemate_tools/save_location.py list [map_name]
  python3 ~/officemate_tools/save_location.py delete <name> [map_name]

Examples:
  python3 ~/officemate_tools/save_location.py save base_station server_room
  python3 ~/officemate_tools/save_location.py save desk_1 server_room
  python3 ~/officemate_tools/save_location.py list server_room
"""
import argparse
import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

LOC_FILE = os.path.expanduser('~/maps/locations.json')
DEFAULT_MAP = 'server_room'


def yaw_deg(z, w):
    return math.degrees(math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z))


class LocationSaver(Node):
    def __init__(self):
        super().__init__('location_saver_cli')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_pose(self, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                t = self.tf_buffer.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time()
                )
                return t.transform
            except TransformException:
                pass
        return None


def load_locations():
    if os.path.exists(LOC_FILE):
        try:
            with open(LOC_FILE, 'r') as f:
                return json.load(f)
        except Exception:
            return {}
    return {}


def save_locations(data):
    os.makedirs(os.path.dirname(LOC_FILE), exist_ok=True)
    tmp = LOC_FILE + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(data, f, indent=2, sort_keys=True)
    os.replace(tmp, LOC_FILE)


def do_save(name, map_name):
    rclpy.init()
    saver = LocationSaver()
    try:
        print(f"Reading current robot pose from TF (map -> base_footprint)...")
        tf = saver.get_pose(timeout=5.0)
        if tf is None:
            print("ERROR: Could not lookup transform from 'map' to 'base_footprint'.")
            print("       Make sure SLAM or Navigation (AMCL) is running!")
            return 1

        x = round(tf.translation.x, 4)
        y = round(tf.translation.y, 4)
        qz = round(tf.rotation.z, 4)
        qw = round(tf.rotation.w, 4)
        heading = round(yaw_deg(qz, qw), 1)

        data = load_locations()
        data.setdefault(map_name, {})
        data[map_name][name] = {
            'x': x,
            'y': y,
            'z': qz,
            'w': qw,
            'yaw': heading
        }
        save_locations(data)

        print(f"\n✅ Location '{name}' SAVED successfully in map '{map_name}':")
        print(f"   Position    : x={x:+.4f} m, y={y:+.4f} m")
        print(f"   Orientation : yaw={heading:+.1f}°, quat(z={qz}, w={qw})")
        print(f"   Saved File  : {LOC_FILE}\n")
        return 0
    finally:
        saver.destroy_node()
        rclpy.try_shutdown()


def do_list(map_name):
    data = load_locations()
    locs = data.get(map_name, {})
    print(f"\nSaved locations for map '{map_name}' ({LOC_FILE}):")
    if not locs:
        print("  (no locations stored yet)")
        print(f"  Drive to a spot and run: python3 save_location.py save <name> {map_name}\n")
        return 0
    for name, p in sorted(locs.items()):
        print(f"  • {name:<20} x={p['x']:+.3f} m, y={p['y']:+.3f} m, yaw={p['yaw']:+.1f}°")
    print()
    return 0


def do_delete(name, map_name):
    data = load_locations()
    if map_name in data and name in data[map_name]:
        del data[map_name][name]
        save_locations(data)
        print(f"Deleted location '{name}' from map '{map_name}'.")
        return 0
    else:
        print(f"Location '{name}' not found in map '{map_name}'.")
        return 1


def main():
    parser = argparse.ArgumentParser(description="OfficeMate Location Manager CLI")
    subparsers = parser.add_subparsers(dest="command")

    save_p = subparsers.add_parser("save", help="Save current robot position as a named location")
    save_p.add_argument("name", help="Location name (e.g. base_station, desk_1)")
    save_p.add_argument("map_name", nargs="?", default=DEFAULT_MAP, help=f"Map name (default: {DEFAULT_MAP})")

    list_p = subparsers.add_parser("list", help="List all saved locations")
    list_p.add_argument("map_name", nargs="?", default=DEFAULT_MAP, help=f"Map name (default: {DEFAULT_MAP})")

    del_p = subparsers.add_parser("delete", help="Delete a saved location")
    del_p.add_argument("name", help="Location name to delete")
    del_p.add_argument("map_name", nargs="?", default=DEFAULT_MAP, help=f"Map name (default: {DEFAULT_MAP})")

    args = parser.parse_args()

    if args.command == "save":
        return do_save(args.name, args.map_name)
    elif args.command == "list":
        return do_list(args.map_name)
    elif args.command == "delete":
        return do_delete(args.name, args.map_name)
    else:
        parser.print_help()
        return 0


if __name__ == "__main__":
    sys.exit(main())
