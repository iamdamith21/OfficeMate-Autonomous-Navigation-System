#!/usr/bin/env python3
"""
location_manager — name the places the robot delivers to.

Drive the robot to a spot, call /save_location with a name, and the pose is
recorded into locations.json. Missions then reference "supervisor_desk"
instead of raw map coordinates, so the web app never stores coordinates and
re-surveying a room does not invalidate every saved destination.

ROS interface
  srvs : /save_location     robot_interfaces/SaveLocation
         /get_location      robot_interfaces/GetLocation
         /list_locations    robot_interfaces/ListLocations
         /delete_location   robot_interfaces/DeleteLocation
  subs : /amcl_pose         geometry_msgs/PoseWithCovarianceStamped
  pubs : /locations/markers visualization_msgs/MarkerArray  (latched, for RViz)

POSE SOURCE — TF first, /amcl_pose only as a fallback.
------------------------------------------------------
The obvious implementation subscribes to /amcl_pose and stores the last message.
That is subtly wrong for this exact use case: AMCL only runs an update cycle
after the robot has MOVED (update_min_d / update_min_a), so a robot standing
still at the spot you want to name may not have published a fresh pose for a
long time. Saving a location is done precisely when the robot is stationary.

The live map -> base_footprint transform is always current — it is what Nav2
itself steers on — so it is preferred. /amcl_pose is kept as a fallback for the
case where TF is momentarily unavailable, and the response says which was used.

Storage is JSON, keyed by MAP NAME so two surveys of different rooms cannot
collide:

    {"office_map": {"supervisor_desk": {"x":2.54,"y":4.81,
                                        "orientation":{"x":0,"y":0,
                                                       "z":0.71,"w":0.70},
                                        "saved":"2026-07-31T14:02:11"}}}

Written atomically (temp file + os.replace) so an interrupted write cannot
truncate the file and lose every location recorded so far.
"""
import datetime
import json
import math
import os
import tempfile

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from robot_interfaces.srv import (DeleteLocation, GetLocation, ListLocations,
                                  SaveLocation)


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LocationManager(Node):
    def __init__(self):
        super().__init__('location_manager')

        default_file = os.path.join(os.path.expanduser('~'), 'maps',
                                    'locations.json')
        self.declare_parameter('locations_file', default_file)
        self.declare_parameter('map_name', 'office_map')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('tf_timeout', 3.0)

        self.path = self.get_parameter('locations_file').value
        self.map_name = self.get_parameter('map_name').value
        self.global_frame = self.get_parameter('global_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.tf_timeout = self.get_parameter('tf_timeout').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._amcl = None
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._amcl_cb, 10)

        self.locations = self._load()

        cbg = ReentrantCallbackGroup()
        self.create_service(SaveLocation, '/save_location',
                            self._save_cb, callback_group=cbg)
        self.create_service(GetLocation, '/get_location',
                            self._get_cb, callback_group=cbg)
        self.create_service(ListLocations, '/list_locations',
                            self._list_cb, callback_group=cbg)
        self.create_service(DeleteLocation, '/delete_location',
                            self._delete_cb, callback_group=cbg)

        # Latched, so RViz shows every saved location the moment it subscribes
        # rather than only after the next change.
        self.marker_pub = self.create_publisher(
            MarkerArray, '/locations/markers',
            QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        self.get_logger().info(
            f'location_manager up — map "{self.map_name}", '
            f'{len(self.locations)} location(s) in {self.path}')
        if self.locations:
            self.get_logger().info(
                '  known: ' + ', '.join(sorted(self.locations)))
        self._publish_markers()

    def _amcl_cb(self, msg):
        self._amcl = msg.pose.pose

    # ── storage ─────────────────────────────────────────────────────────────
    def _load(self):
        if not os.path.exists(self.path):
            return {}
        try:
            with open(self.path) as f:
                return (json.load(f) or {}).get(self.map_name, {}) or {}
        except (ValueError, OSError) as e:
            self.get_logger().error(f'could not read {self.path}: {e}')
            return {}

    def _write(self):
        """Read-modify-write so other maps' entries survive, then replace
        atomically so an interrupted write cannot corrupt the file."""
        data = {}
        if os.path.exists(self.path):
            try:
                with open(self.path) as f:
                    data = json.load(f) or {}
            except (ValueError, OSError):
                data = {}
        data[self.map_name] = self.locations

        os.makedirs(os.path.dirname(self.path) or '.', exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=os.path.dirname(self.path) or '.',
                                   prefix='.locations-', suffix='.json')
        try:
            with os.fdopen(fd, 'w') as f:
                json.dump(data, f, indent=2, sort_keys=True)
            os.replace(tmp, self.path)
        except OSError:
            if os.path.exists(tmp):
                os.unlink(tmp)
            raise

    # ── pose ────────────────────────────────────────────────────────────────
    def _current_pose(self):
        """Returns (PoseStamped, source) or (None, reason)."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout))
            p = PoseStamped()
            p.header.frame_id = self.global_frame
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.position.x = tf.transform.translation.x
            p.pose.position.y = tf.transform.translation.y
            p.pose.orientation = tf.transform.rotation
            return p, 'tf'
        except Exception:
            pass

        if self._amcl is not None:
            p = PoseStamped()
            p.header.frame_id = self.global_frame
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose = self._amcl
            return p, 'amcl_pose (TF unavailable)'

        return None, (f'no {self.global_frame}->{self.robot_frame} TF and no '
                      f'/amcl_pose — is AMCL localised? '
                      f'run globalloc.py <map> --publish')

    def _to_pose(self, name):
        e = self.locations[name]
        p = PoseStamped()
        p.header.frame_id = self.global_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(e['x'])
        p.pose.position.y = float(e['y'])
        o = e['orientation']
        p.pose.orientation.x = float(o['x'])
        p.pose.orientation.y = float(o['y'])
        p.pose.orientation.z = float(o['z'])
        p.pose.orientation.w = float(o['w'])
        return p

    def _publish_markers(self):
        arr = MarkerArray()
        # DELETEALL first, so a deleted location's marker actually disappears
        # instead of lingering in RViz forever.
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        for i, name in enumerate(sorted(self.locations)):
            pose = self._to_pose(name).pose
            arrow = Marker()
            arrow.header.frame_id = self.global_frame
            arrow.ns = 'locations'
            arrow.id = i * 2
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose = pose
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.30, 0.06, 0.06
            (arrow.color.r, arrow.color.g,
             arrow.color.b, arrow.color.a) = 0.1, 0.8, 1.0, 0.9
            arr.markers.append(arrow)

            text = Marker()
            text.header.frame_id = self.global_frame
            text.ns = 'locations'
            text.id = i * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose = self._to_pose(name).pose
            text.pose.position.z += 0.35
            text.scale.z = 0.18
            (text.color.r, text.color.g,
             text.color.b, text.color.a) = 1.0, 1.0, 1.0, 0.95
            text.text = name
            arr.markers.append(text)
        self.marker_pub.publish(arr)

    # ── services ────────────────────────────────────────────────────────────
    def _save_cb(self, req, resp):
        name = req.name.strip()
        if not name:
            resp.success = False
            resp.message = 'name must not be empty'
            return resp

        pose, source = self._current_pose()
        if pose is None:
            resp.success = False
            resp.message = source
            self.get_logger().warn(f'save "{name}" refused: {source}')
            return resp

        existed = name in self.locations
        previous = self.locations.get(name)
        o = pose.pose.orientation
        self.locations[name] = {
            'x': round(float(pose.pose.position.x), 4),
            'y': round(float(pose.pose.position.y), 4),
            'orientation': {'x': round(float(o.x), 4),
                            'y': round(float(o.y), 4),
                            'z': round(float(o.z), 4),
                            'w': round(float(o.w), 4)},
            'saved': datetime.datetime.now().isoformat(timespec='seconds'),
        }
        try:
            self._write()
        except OSError as e:
            # Roll back in memory so the file and our state cannot diverge.
            if previous is None:
                del self.locations[name]
            else:
                self.locations[name] = previous
            resp.success = False
            resp.message = f'could not write {self.path}: {e}'
            return resp

        self._publish_markers()
        resp.success = True
        resp.pose = pose
        resp.message = (
            f'{"updated" if existed else "saved"} "{name}" at '
            f'x={pose.pose.position.x:.3f} y={pose.pose.position.y:.3f} '
            f'yaw={math.degrees(quat_to_yaw(o)):.1f}deg [{source}]')
        self.get_logger().info(resp.message)
        return resp

    def _get_cb(self, req, resp):
        if req.name not in self.locations:
            resp.success = False
            resp.message = (f'no location "{req.name}" — known: '
                            f'{", ".join(sorted(self.locations)) or "(none)"}')
            return resp
        resp.success = True
        resp.pose = self._to_pose(req.name)
        resp.message = 'ok'
        return resp

    def _list_cb(self, req, resp):
        names = sorted(self.locations)
        resp.success = True
        resp.names = names
        resp.poses = [self._to_pose(n) for n in names]
        resp.message = f'{len(names)} location(s) on map "{self.map_name}"'
        return resp

    def _delete_cb(self, req, resp):
        if req.name not in self.locations:
            resp.success = False
            resp.message = f'no location "{req.name}"'
            return resp
        removed = self.locations.pop(req.name)
        try:
            self._write()
        except OSError as e:
            self.locations[req.name] = removed
            resp.success = False
            resp.message = f'could not write {self.path}: {e}'
            return resp
        self._publish_markers()
        resp.success = True
        resp.message = f'deleted "{req.name}"'
        self.get_logger().info(resp.message)
        return resp


def main():
    rclpy.init()
    node = LocationManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
