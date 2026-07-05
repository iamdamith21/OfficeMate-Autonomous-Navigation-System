#!/usr/bin/env python3
"""
delivery_manager_node — OfficeMate delivery mission state machine.

Implements the mission FSM:

  INITIALIZING → IDLE → MISSION_RECEIVED → PLANNING → NAVIGATING → ARRIVED →
  VERIFY_RFID → OPEN_DOOR → WAIT_PACKAGE_REMOVAL → CLOSE_DOOR → RETURN_HOME →
  MISSION_DONE → IDLE

Entry point: the `deliver` action (robot_interfaces/DeliveryMission). Each
transition is published on /mission_state (robot_interfaces/MissionState) for
the web app. Wired to the real hardware_bridge interfaces:

  NAVIGATING / RETURN_HOME : Nav2 NavigateToPose action
  VERIFY_RFID              : /rfid/tag            (std_msgs/String)
  OPEN_DOOR / CLOSE_DOOR   : /doors/open /doors/close (std_srvs/Trigger)
  WAIT_PACKAGE_REMOVAL     : /compartment/occupied (std_msgs/Bool, IR)

Param `simulate` (default False): mock nav/doors/RFID/IR so the whole FSM can
be dry-run without hardware or Nav2 (used for verification / CI).
"""
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from robot_interfaces.action import DeliveryMission
from robot_interfaces.msg import MissionState


S = MissionState  # state constants live on the message


class DeliveryManager(Node):
    def __init__(self):
        super().__init__('delivery_manager')
        self.declare_parameter('simulate', False)
        self.declare_parameter('rfid_timeout', 30.0)
        self.declare_parameter('package_timeout', 120.0)
        self.simulate = self.get_parameter('simulate').value

        self.cbg = ReentrantCallbackGroup()

        # State publishing ----------------------------------------------------
        self.state = S.INITIALIZING
        self.mission_id = ''
        self.state_pub = self.create_publisher(MissionState, '/mission_state', 10)

        # Hardware / sensor inputs -------------------------------------------
        self._last_tag = None
        self._occupied = False
        self.create_subscription(String, '/rfid/tag', self._rfid_cb, 10, callback_group=self.cbg)
        self.create_subscription(Bool, '/compartment/occupied', self._ir_cb, 10, callback_group=self.cbg)

        # Actuators / navigation ---------------------------------------------
        self.open_cli = self.create_client(Trigger, '/doors/open', callback_group=self.cbg)
        self.close_cli = self.create_client(Trigger, '/doors/close', callback_group=self.cbg)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.cbg)

        # Mission entry point -------------------------------------------------
        self.action_server = ActionServer(
            self, DeliveryMission, 'deliver',
            execute_callback=self._execute,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda c: CancelResponse.ACCEPT,
            callback_group=self.cbg)

        self._set_state(S.IDLE, detail='ready for missions')
        self.get_logger().info(
            f'delivery_manager up ({"SIMULATE" if self.simulate else "LIVE"} mode)')

    # ── sensor callbacks ────────────────────────────────────────────────────
    def _rfid_cb(self, msg): self._last_tag = msg.data
    def _ir_cb(self, msg): self._occupied = msg.data

    # ── state helpers ───────────────────────────────────────────────────────
    _NAMES = {
        S.INITIALIZING: 'INITIALIZING', S.IDLE: 'IDLE',
        S.MISSION_RECEIVED: 'MISSION_RECEIVED', S.PLANNING: 'PLANNING',
        S.NAVIGATING: 'NAVIGATING', S.ARRIVED: 'ARRIVED',
        S.VERIFY_RFID: 'VERIFY_RFID', S.OPEN_DOOR: 'OPEN_DOOR',
        S.WAIT_PACKAGE_REMOVAL: 'WAIT_PACKAGE_REMOVAL', S.CLOSE_DOOR: 'CLOSE_DOOR',
        S.RETURN_HOME: 'RETURN_HOME', S.MISSION_DONE: 'MISSION_DONE',
        S.ERROR: 'ERROR',
    }

    def _set_state(self, state, detail=''):
        self.state = state
        name = self._NAMES.get(state, str(state))
        msg = MissionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = state
        msg.state_name = name
        msg.mission_id = self.mission_id
        msg.detail = detail
        self.state_pub.publish(msg)
        self.get_logger().info(f'[FSM] {name} {("- " + detail) if detail else ""}')

    def _feedback(self, goal_handle):
        fb = DeliveryMission.Feedback()
        fb.state = self.state
        fb.state_name = self._NAMES.get(self.state, str(self.state))
        goal_handle.publish_feedback(fb)

    def _wait_future(self, future, timeout=30.0):
        """Block (in this executor thread) until an async future completes."""
        start = time.time()
        while rclpy.ok() and not future.done():
            if time.time() - start > timeout:
                return None
            time.sleep(0.05)
        return future.result() if future.done() else None

    # ── mission execution ───────────────────────────────────────────────────
    def _execute(self, goal_handle):
        goal = goal_handle.request
        self.mission_id = goal.mission_id or 'mission'
        result = DeliveryMission.Result()

        def fail(msg):
            self._set_state(S.ERROR, detail=msg)
            result.success = False
            result.message = msg
            goal_handle.abort()
            return result

        # MISSION_RECEIVED
        self._set_state(S.MISSION_RECEIVED, detail=f'recipient={goal.recipient_id}')
        self._feedback(goal_handle)

        # PLANNING
        self._set_state(S.PLANNING)
        self._feedback(goal_handle)

        # NAVIGATING → dropoff
        self._set_state(S.NAVIGATING, detail='to dropoff')
        self._feedback(goal_handle)
        if not self._navigate(goal.dropoff_pose):
            return fail('navigation to dropoff failed')

        # ARRIVED
        self._set_state(S.ARRIVED)
        self._feedback(goal_handle)

        # VERIFY_RFID
        self._set_state(S.VERIFY_RFID, detail=f'expecting {goal.recipient_id}')
        self._feedback(goal_handle)
        if not self._verify_rfid(goal.recipient_id):
            return fail('RFID verification failed/timed out')

        # OPEN_DOOR
        self._set_state(S.OPEN_DOOR)
        self._feedback(goal_handle)
        if not self._door(self.open_cli, '/doors/open'):
            return fail('door open failed')

        # WAIT_PACKAGE_REMOVAL
        self._set_state(S.WAIT_PACKAGE_REMOVAL)
        self._feedback(goal_handle)
        if not self._wait_package_removed():
            return fail('package not removed in time')

        # CLOSE_DOOR
        self._set_state(S.CLOSE_DOOR)
        self._feedback(goal_handle)
        if not self._door(self.close_cli, '/doors/close'):
            return fail('door close failed')

        # RETURN_HOME
        self._set_state(S.RETURN_HOME)
        self._feedback(goal_handle)
        home = PoseStamped()
        home.header.frame_id = 'map'
        # home = base (pickup_pose if given, else map origin)
        if goal.pickup_pose.header.frame_id:
            home = goal.pickup_pose
        if not self._navigate(home):
            return fail('return home failed')

        # MISSION_DONE → IDLE
        self._set_state(S.MISSION_DONE, detail='delivered')
        self._feedback(goal_handle)
        goal_handle.succeed()
        result.success = True
        result.message = 'delivery complete'
        self._set_state(S.IDLE, detail='ready for missions')
        return result

    # ── step implementations (live + simulated) ─────────────────────────────
    def _navigate(self, pose):
        if self.simulate:
            time.sleep(1.0)
            return True
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 navigate_to_pose server unavailable')
            return False
        goal = NavigateToPose.Goal()
        goal.pose = pose
        send = self.nav_client.send_goal_async(goal)
        gh = self._wait_future(send, timeout=10.0)
        if gh is None or not gh.accepted:
            return False
        res = self._wait_future(gh.get_result_async(), timeout=300.0)
        return res is not None

    def _verify_rfid(self, expected):
        if self.simulate:
            time.sleep(0.5)
            return True
        timeout = self.get_parameter('rfid_timeout').value
        self._last_tag = None
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout:
            tag = self._last_tag
            if tag is not None and (not expected or tag == expected):
                self.get_logger().info(f'RFID matched: {tag}')
                return True
            time.sleep(0.1)
        return False

    def _door(self, client, name):
        if self.simulate:
            time.sleep(0.5)
            return True
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'{name} service unavailable')
            return False
        res = self._wait_future(client.call_async(Trigger.Request()), timeout=15.0)
        return res is not None and res.success

    def _wait_package_removed(self):
        if self.simulate:
            time.sleep(0.5)
            return True
        timeout = self.get_parameter('package_timeout').value
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout:
            if not self._occupied:
                return True
            time.sleep(0.1)
        return False


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
