#!/usr/bin/env python3
"""
delivery_manager — OfficeMate two-leg delivery mission FSM.

    BASE -> SENDER (load files) -> RECIPIENT (unload) -> BASE

Implemented as a state-dispatch loop: `_HANDLERS` maps a state to a method that
does the work and RETURNS THE NEXT STATE. Adding or re-routing a state is a
one-line change and every transition funnels through one place, which is what
makes the cancel/emergency-stop checks below possible at all.

The Mission Manager never touches motors. It only issues: navigate to
waypoint, open/close compartment, wait for RFID, wait for IR, return to base,
handle timeout, handle navigation failure.

States
------
  0 IDLE                    9 WAIT_FOR_RFID
  1 WAIT_FOR_REQUEST       10 VERIFY_RFID
  2 NAVIGATE_TO_SENDER     11 OPEN_RECIPIENT_DOOR
  3 ARRIVED_AT_SENDER      12 WAIT_FOR_FILE_REMOVAL
  4 OPEN_SENDER_DOOR       13 CLOSE_RECIPIENT_DOOR
  5 WAIT_FOR_FILE          14 RETURN_TO_BASE
  6 CLOSE_SENDER_DOOR      15 RETURN_TO_SENDER
  7 NAVIGATE_TO_RECIPIENT  16 MISSION_COMPLETE
  8 ARRIVED_AT_RECIPIENT   17 MISSION_FAILED

The three timeout branches (all `file_timeout` / `rfid_timeout`, default 15 s)
are the whole point of the design and are easy to get subtly wrong:

  WAIT_FOR_FILE          IR true  -> CLOSE_SENDER_DOOR (carry on)
                         timeout  -> close, RETURN_TO_BASE (nothing to deliver)
  WAIT_FOR_RFID          valid    -> VERIFY_RFID -> OPEN_RECIPIENT_DOOR
                         WRONG    -> keep waiting, do NOT fail
                         timeout  -> RETURN_TO_SENDER (give the files back)
  WAIT_FOR_FILE_REMOVAL  IR false -> CLOSE_RECIPIENT_DOOR -> RETURN_TO_BASE
                         timeout  -> close, RETURN_TO_SENDER

RETURN_TO_SENDER re-uses OPEN_SENDER_DOOR and WAIT_FOR_FILE_REMOVAL to hand the
files back, which is why `self.returning` exists: those two states must route
to RETURN_TO_BASE on the way back instead of onward to the recipient.

Locations are NAMES resolved through location_manager (/get_location) at
mission start, so the web app never handles coordinates. Resolving up front
means a typo'd destination fails immediately rather than halfway through.

Param `simulate` (default False) mocks nav/doors/RFID/IR so the whole FSM can
be dry-run with no hardware and no Nav2 — use it to verify routing after any
change to this file.
"""
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from robot_interfaces.action import DeliveryMission
from robot_interfaces.msg import MissionState
from robot_interfaces.srv import GetLocation

S = MissionState

NAMES = {
    S.IDLE: 'IDLE',
    S.WAIT_FOR_REQUEST: 'WAIT_FOR_REQUEST',
    S.NAVIGATE_TO_SENDER: 'NAVIGATE_TO_SENDER',
    S.ARRIVED_AT_SENDER: 'ARRIVED_AT_SENDER',
    S.OPEN_SENDER_DOOR: 'OPEN_SENDER_DOOR',
    S.WAIT_FOR_FILE: 'WAIT_FOR_FILE',
    S.CLOSE_SENDER_DOOR: 'CLOSE_SENDER_DOOR',
    S.NAVIGATE_TO_RECIPIENT: 'NAVIGATE_TO_RECIPIENT',
    S.ARRIVED_AT_RECIPIENT: 'ARRIVED_AT_RECIPIENT',
    S.WAIT_FOR_RFID: 'WAIT_FOR_RFID',
    S.VERIFY_RFID: 'VERIFY_RFID',
    S.OPEN_RECIPIENT_DOOR: 'OPEN_RECIPIENT_DOOR',
    S.WAIT_FOR_FILE_REMOVAL: 'WAIT_FOR_FILE_REMOVAL',
    S.CLOSE_RECIPIENT_DOOR: 'CLOSE_RECIPIENT_DOOR',
    S.RETURN_TO_BASE: 'RETURN_TO_BASE',
    S.RETURN_TO_SENDER: 'RETURN_TO_SENDER',
    S.MISSION_COMPLETE: 'MISSION_COMPLETE',
    S.MISSION_FAILED: 'MISSION_FAILED',
}


class DeliveryManager(Node):
    def __init__(self):
        super().__init__('delivery_manager')

        self.declare_parameter('simulate', False)
        self.declare_parameter('rfid_timeout', 15.0)
        self.declare_parameter('file_timeout', 15.0)
        self.declare_parameter('nav_retries', 3)
        self.declare_parameter('nav_timeout', 300.0)
        self.declare_parameter('base_location', 'base_station')
        self.declare_parameter('min_battery', 0.0)   # 0 disables the check
        # Test escape: the MFRC522 browns out at 2.65 V on the Mega 3.3 V rail
        # and resets before a card read can finish. With this false, WAIT_FOR_
        # RFID passes immediately — anyone can open the compartment.
        self.declare_parameter('require_rfid', True)

        self.simulate = self.get_parameter('simulate').value
        self.cbg = ReentrantCallbackGroup()

        # ── blackboard ──────────────────────────────────────────────────────
        self.state = S.IDLE
        self.state_since = time.time()
        self.mission_id = ''
        self.retry_count = 0
        self.returning = False        # on the give-the-files-back path
        self.file_present = False
        self.door_state = 'UNKNOWN'
        self.battery = None
        self.sender_pose = None
        self.recipient_pose = None
        self.base_pose = None
        self._last_tag = None
        self._cancel = threading.Event()

        self.state_pub = self.create_publisher(MissionState, '/mission_state', 10)

        # ── sensors ─────────────────────────────────────────────────────────
        self.create_subscription(String, '/rfid/tag', self._rfid_cb, 10,
                                 callback_group=self.cbg)
        self.create_subscription(Bool, '/compartment/occupied', self._ir_cb, 10,
                                 callback_group=self.cbg)
        self.create_subscription(String, '/doors/state', self._door_cb, 10,
                                 callback_group=self.cbg)

        # ── actuators / navigation / locations ──────────────────────────────
        self.open_cli = self.create_client(Trigger, '/doors/open',
                                           callback_group=self.cbg)
        self.close_cli = self.create_client(Trigger, '/doors/close',
                                            callback_group=self.cbg)
        self.loc_cli = self.create_client(GetLocation, '/get_location',
                                          callback_group=self.cbg)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                       callback_group=self.cbg)
        self._nav_goal_handle = None

        self.action_server = ActionServer(
            self, DeliveryMission, 'deliver',
            execute_callback=self._execute,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=self._cancel_cb,
            callback_group=self.cbg)

        # Republish state at 1 Hz so a web client that connects mid-mission
        # learns the current state without waiting for the next transition.
        self.create_timer(1.0, self._publish_state)

        self._set_state(S.IDLE, 'ready for missions')
        self.get_logger().info(
            f'delivery_manager up ({"SIMULATE" if self.simulate else "LIVE"}), '
            f'base="{self.get_parameter("base_location").value}", '
            f'timeouts file={self.get_parameter("file_timeout").value}s '
            f'rfid={self.get_parameter("rfid_timeout").value}s, '
            f'nav_retries={self.get_parameter("nav_retries").value}')

    # ── sensor callbacks ────────────────────────────────────────────────────
    def _rfid_cb(self, msg):
        self._last_tag = msg.data.strip().upper()

    def _ir_cb(self, msg):
        self.file_present = msg.data

    def _door_cb(self, msg):
        self.door_state = msg.data

    def _cancel_cb(self, goal_handle):
        self.get_logger().warn('cancel requested — will unwind at the next state')
        self._cancel.set()
        return CancelResponse.ACCEPT

    # ── state plumbing ──────────────────────────────────────────────────────
    def _set_state(self, state, detail=''):
        self.state = state
        self.state_since = time.time()
        self._publish_state(detail)
        self.get_logger().info(
            f'[FSM] {NAMES.get(state, state)}'
            f'{" - " + detail if detail else ""}')

    def _publish_state(self, detail=''):
        m = MissionState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.state = self.state
        m.state_name = NAMES.get(self.state, str(self.state))
        m.mission_id = self.mission_id
        m.detail = detail
        m.retry_count = self.retry_count
        m.file_present = self.file_present
        m.door_state = self.door_state
        m.seconds_in_state = float(time.time() - self.state_since)
        self.state_pub.publish(m)

    def _feedback(self, gh, detail=''):
        fb = DeliveryMission.Feedback()
        fb.state = self.state
        fb.state_name = NAMES.get(self.state, str(self.state))
        fb.detail = detail
        fb.retry_count = self.retry_count
        gh.publish_feedback(fb)

    def _wait_future(self, future, timeout=30.0):
        start = time.time()
        while rclpy.ok() and not future.done():
            if time.time() - start > timeout:
                return None
            time.sleep(0.05)
        return future.result() if future.done() else None

    # ── primitive actions ───────────────────────────────────────────────────
    def _resolve(self, name):
        """Name -> PoseStamped via location_manager."""
        if self.simulate:
            p = PoseStamped()
            p.header.frame_id = 'map'
            return p
        if not self.loc_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/get_location unavailable — is '
                                    'location_manager running?')
            return None
        req = GetLocation.Request()
        req.name = name
        res = self._wait_future(self.loc_cli.call_async(req), timeout=10.0)
        if res is None or not res.success:
            self.get_logger().error(
                f'location "{name}": {res.message if res else "no response"}')
            return None
        return res.pose

    def _navigate(self, pose, what):
        """Navigate with retries. Returns True, or False after nav_retries."""
        retries = self.get_parameter('nav_retries').value
        timeout = self.get_parameter('nav_timeout').value
        self.retry_count = 0

        while self.retry_count <= retries:
            if self._cancel.is_set():
                return False
            if self.simulate:
                time.sleep(0.5)
                return True

            if not self.nav_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('navigate_to_pose server unavailable')
            else:
                goal = NavigateToPose.Goal()
                goal.pose = pose
                send = self.nav_client.send_goal_async(goal)
                gh = self._wait_future(send, timeout=10.0)
                if gh is not None and gh.accepted:
                    self._nav_goal_handle = gh
                    res = self._wait_future(gh.get_result_async(),
                                            timeout=timeout)
                    self._nav_goal_handle = None
                    # STATUS_SUCCEEDED == 4. Anything else is a real failure;
                    # treating "got a result at all" as success (as an earlier
                    # version did) reports an aborted goal as a successful one.
                    if res is not None and res.status == 4:
                        return True
                    self.get_logger().warn(
                        f'{what}: nav result status='
                        f'{res.status if res else "timeout"}')
                else:
                    self.get_logger().warn(f'{what}: goal rejected')

            self.retry_count += 1
            if self.retry_count <= retries:
                self.get_logger().warn(
                    f'{what}: retry {self.retry_count}/{retries}')
                self._publish_state(f'nav retry {self.retry_count}/{retries}')
                time.sleep(2.0)
        return False

    def _door(self, opening):
        if self.simulate:
            time.sleep(0.3)
            self.door_state = 'OPEN' if opening else 'CLOSED'
            return True
        cli = self.open_cli if opening else self.close_cli
        name = '/doors/open' if opening else '/doors/close'
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'{name} unavailable')
            return False
        res = self._wait_future(cli.call_async(Trigger.Request()), timeout=20.0)
        if res is None or not res.success:
            self.get_logger().error(
                f'{name}: {res.message if res else "no response"}')
            return False
        return True

    def _wait_ir(self, want_present, timeout, label):
        """Wait for the IR to report present/absent. True if it happened,
        False on timeout or cancel."""
        if self.simulate:
            time.sleep(0.3)
            return True
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout:
            if self._cancel.is_set():
                return False
            if self.file_present == want_present:
                self.get_logger().info(f'{label}: IR satisfied after '
                                       f'{time.time() - start:.1f}s')
                return True
            self._publish_state(f'{label} {timeout - (time.time() - start):.0f}s left')
            time.sleep(0.1)
        self.get_logger().warn(f'{label}: TIMEOUT after {timeout:.0f}s')
        return False

    # ── state handlers: each returns the NEXT state ─────────────────────────
    def _h_navigate_to_sender(self):
        self._set_state(S.NAVIGATE_TO_SENDER, 'to sender')
        if not self._navigate(self.sender_pose, 'NAVIGATE_TO_SENDER'):
            return S.MISSION_FAILED
        return S.ARRIVED_AT_SENDER

    def _h_arrived_at_sender(self):
        self._set_state(S.ARRIVED_AT_SENDER)
        return S.OPEN_SENDER_DOOR

    def _h_open_sender_door(self):
        self._set_state(S.OPEN_SENDER_DOOR)
        if not self._door(True):
            return S.MISSION_FAILED
        # On the way back this door is opened to RETURN the files, so the next
        # thing to wait for is removal, not placement.
        return S.WAIT_FOR_FILE_REMOVAL if self.returning else S.WAIT_FOR_FILE

    def _h_wait_for_file(self):
        t = self.get_parameter('file_timeout').value
        self._set_state(S.WAIT_FOR_FILE, f'place files within {t:.0f}s')
        if not self._wait_ir(True, t, 'WAIT_FOR_FILE'):
            # Nothing was loaded — close up and go home. This is a normal
            # outcome, not a failure.
            self.returning = False
            self._set_state(S.CLOSE_SENDER_DOOR, 'nothing loaded')
            self._door(False)
            return S.RETURN_TO_BASE
        return S.CLOSE_SENDER_DOOR

    def _h_close_sender_door(self):
        self._set_state(S.CLOSE_SENDER_DOOR)
        if not self._door(False):
            return S.MISSION_FAILED
        return S.RETURN_TO_BASE if self.returning else S.NAVIGATE_TO_RECIPIENT

    def _h_navigate_to_recipient(self):
        self._set_state(S.NAVIGATE_TO_RECIPIENT, 'to recipient')
        if not self._navigate(self.recipient_pose, 'NAVIGATE_TO_RECIPIENT'):
            return S.MISSION_FAILED
        return S.ARRIVED_AT_RECIPIENT

    def _h_arrived_at_recipient(self):
        self._set_state(S.ARRIVED_AT_RECIPIENT)
        return S.WAIT_FOR_RFID

    def _h_wait_for_rfid(self):
        t = self.get_parameter('rfid_timeout').value
        if not self.get_parameter('require_rfid').value:
            self._set_state(S.WAIT_FOR_RFID, 'BYPASSED (require_rfid=false)')
            self.get_logger().warn(
                'RFID BYPASSED — opening the compartment without checking any tag')
            return S.OPEN_RECIPIENT_DOOR

        self._set_state(S.WAIT_FOR_RFID, f'scan tag within {t:.0f}s')
        # Discard anything scanned earlier in the mission, so only a tag
        # presented HERE counts.
        self._last_tag = None
        expected = (self.recipient_rfid or '').strip().upper()
        start = time.time()
        while rclpy.ok() and time.time() - start < t:
            if self._cancel.is_set():
                return S.RETURN_TO_BASE
            tag = self._last_tag
            if tag:
                self._set_state(S.VERIFY_RFID, f'got {tag}')
                if not expected or tag == expected:
                    self.get_logger().info(f'RFID accepted: {tag}')
                    return S.OPEN_RECIPIENT_DOOR
                # WRONG tag is NOT a failure — someone else may tap first.
                # Keep waiting on the same timer.
                self.get_logger().warn(
                    f'RFID {tag} != expected {expected}; still waiting')
                self._last_tag = None
                self._set_state(S.WAIT_FOR_RFID, f'wrong tag {tag}, waiting')
            self._publish_state(f'{t - (time.time() - start):.0f}s left')
            time.sleep(0.1)

        self.get_logger().warn('WAIT_FOR_RFID: TIMEOUT — returning to sender')
        return S.RETURN_TO_SENDER

    def _h_open_recipient_door(self):
        self._set_state(S.OPEN_RECIPIENT_DOOR)
        if not self._door(True):
            return S.MISSION_FAILED
        return S.WAIT_FOR_FILE_REMOVAL

    def _h_wait_for_file_removal(self):
        t = self.get_parameter('file_timeout').value
        self._set_state(S.WAIT_FOR_FILE_REMOVAL, f'take files within {t:.0f}s')
        taken = self._wait_ir(False, t, 'WAIT_FOR_FILE_REMOVAL')

        if self.returning:
            # Handing files back at the sender. Either way we close and go home.
            self._set_state(S.CLOSE_SENDER_DOOR,
                            'files returned' if taken else 'not collected')
            self._door(False)
            return S.RETURN_TO_BASE

        if not taken:
            # Recipient never took them — carry them back to the sender.
            self._set_state(S.CLOSE_RECIPIENT_DOOR, 'not collected')
            self._door(False)
            return S.RETURN_TO_SENDER
        return S.CLOSE_RECIPIENT_DOOR

    def _h_close_recipient_door(self):
        self._set_state(S.CLOSE_RECIPIENT_DOOR)
        if not self._door(False):
            return S.MISSION_FAILED
        return S.RETURN_TO_BASE

    def _h_return_to_sender(self):
        self._set_state(S.RETURN_TO_SENDER, 'undelivered — returning files')
        self.returning = True
        if not self._navigate(self.sender_pose, 'RETURN_TO_SENDER'):
            return S.MISSION_FAILED
        return S.OPEN_SENDER_DOOR

    def _h_return_to_base(self):
        self._set_state(S.RETURN_TO_BASE)
        if not self._navigate(self.base_pose, 'RETURN_TO_BASE'):
            return S.MISSION_FAILED
        return S.MISSION_COMPLETE

    _HANDLERS = {
        S.NAVIGATE_TO_SENDER: _h_navigate_to_sender,
        S.ARRIVED_AT_SENDER: _h_arrived_at_sender,
        S.OPEN_SENDER_DOOR: _h_open_sender_door,
        S.WAIT_FOR_FILE: _h_wait_for_file,
        S.CLOSE_SENDER_DOOR: _h_close_sender_door,
        S.NAVIGATE_TO_RECIPIENT: _h_navigate_to_recipient,
        S.ARRIVED_AT_RECIPIENT: _h_arrived_at_recipient,
        S.WAIT_FOR_RFID: _h_wait_for_rfid,
        S.OPEN_RECIPIENT_DOOR: _h_open_recipient_door,
        S.WAIT_FOR_FILE_REMOVAL: _h_wait_for_file_removal,
        S.CLOSE_RECIPIENT_DOOR: _h_close_recipient_door,
        S.RETURN_TO_SENDER: _h_return_to_sender,
        S.RETURN_TO_BASE: _h_return_to_base,
    }

    # ── mission execution ───────────────────────────────────────────────────
    def _execute(self, gh):
        goal = gh.request
        self.mission_id = goal.mission_id or 'mission'
        self.recipient_rfid = goal.recipient_rfid
        self.retry_count = 0
        self.returning = False
        self._cancel.clear()
        result = DeliveryMission.Result()

        def finish(state, ok, msg):
            self._set_state(state, msg)
            result.success = ok
            result.message = msg
            result.final_state = state
            if ok:
                gh.succeed()
            elif self._cancel.is_set():
                gh.canceled()
            else:
                gh.abort()
            self._set_state(S.IDLE, 'ready for missions')
            return result

        # WAIT_FOR_REQUEST — validate everything BEFORE moving. Every step of
        # this FSM fails by timing out, so an unresolvable location or a flat
        # battery must be caught here rather than three minutes in.
        self._set_state(S.WAIT_FOR_REQUEST,
                        f'{goal.sender_location} -> {goal.recipient_location}')
        self._feedback(gh)

        min_batt = self.get_parameter('min_battery').value
        if min_batt > 0.0 and self.battery is not None and self.battery < min_batt:
            return finish(S.MISSION_FAILED, False,
                          f'battery {self.battery:.0%} below {min_batt:.0%}')

        base_name = goal.base_location or self.get_parameter('base_location').value
        for label, name in (('sender', goal.sender_location),
                            ('recipient', goal.recipient_location),
                            ('base', base_name)):
            if not name:
                return finish(S.MISSION_FAILED, False,
                              f'{label} location not specified')
        self.sender_pose = self._resolve(goal.sender_location)
        self.recipient_pose = self._resolve(goal.recipient_location)
        self.base_pose = self._resolve(base_name)
        for label, pose, name in (('sender', self.sender_pose, goal.sender_location),
                                  ('recipient', self.recipient_pose, goal.recipient_location),
                                  ('base', self.base_pose, base_name)):
            if pose is None:
                return finish(S.MISSION_FAILED, False,
                              f'unknown {label} location "{name}"')

        # Dispatch loop.
        state = S.NAVIGATE_TO_SENDER
        while rclpy.ok():
            if self._cancel.is_set() and state not in (
                    S.RETURN_TO_BASE, S.MISSION_COMPLETE, S.MISSION_FAILED):
                # Unwind safely: shut the compartment, then head home.
                self.get_logger().warn('cancelling — closing door, returning to base')
                self._door(False)
                state = S.RETURN_TO_BASE

            if state == S.MISSION_COMPLETE:
                return finish(S.MISSION_COMPLETE, True, 'delivery complete')
            if state == S.MISSION_FAILED:
                # Best effort: get out of the way rather than block a corridor.
                return finish(S.MISSION_FAILED, False,
                              f'failed in {NAMES.get(self.state, self.state)}')

            handler = self._HANDLERS.get(state)
            if handler is None:
                return finish(S.MISSION_FAILED, False,
                              f'no handler for state {state}')
            state = handler(self)
            self._feedback(gh)

        return finish(S.MISSION_FAILED, False, 'shutdown')


def main():
    rclpy.init()
    node = DeliveryManager()
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
