"""Typing controller: turns a text string into arm keystrokes.

On /type_text (std_msgs/String):
  1. Move to the SCAN joint pose (camera over the table, looking at the
     keyboard) and wait for the detector's /keyboard/key_map.
  2. For each character: analytic IK (rover_arm_typing.ik) to hover above
     the key, descend to press depth, dwell, retract.
  3. Return to the scan pose and publish DONE on /typing_status.

Motion goes straight to /arm_controller/follow_joint_trajectory with
single-waypoint joint-space goals; durations are scaled from the largest
joint displacement.
"""

import json
import math
import threading

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from . import ik


class TypingController(Node):

    def __init__(self):
        super().__init__('typing_controller')
        self.declare_parameter('scan_pose', [0.0, 0.35, 0.85, 1.57, 0.0])
        self.declare_parameter('hover_height', 0.06)
        self.declare_parameter('press_clearance', 0.004)
        self.declare_parameter('dwell_sec', 0.4)
        self.declare_parameter('max_joint_vel', 0.8)
        self.declare_parameter('min_move_sec', 0.6)
        self.declare_parameter('key_map_timeout', 30.0)

        self.cb_group = ReentrantCallbackGroup()
        self.client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            callback_group=self.cb_group)

        self.key_map = None
        self.map_event = threading.Event()
        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, '/keyboard/key_map', self.on_key_map, latched,
            callback_group=self.cb_group)
        self.create_subscription(
            String, '/type_text', self.on_type_text, 10,
            callback_group=self.cb_group)
        self.status_pub = self.create_publisher(String, '/typing_status', 10)

        self.last_q = None
        self.busy = threading.Lock()
        self.get_logger().info('typing_controller ready; '
                               'publish text to /type_text')

    # ------------------------------------------------------------ helpers
    def status(self, text):
        self.get_logger().info(text)
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def on_key_map(self, msg):
        self.key_map = json.loads(msg.data)
        self.map_event.set()

    def move_to(self, q, extra_time=0.0):
        """Send a single-waypoint trajectory and block until done."""
        if self.last_q is None:
            dq = math.pi
        else:
            dq = max(abs(a - b) for a, b in zip(q, self.last_q))
        vmax = self.get_parameter('max_joint_vel').value
        dur = max(self.get_parameter('min_move_sec').value,
                  dq / vmax) + extra_time

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = ik.ARM_JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        pt.time_from_start.sec = int(dur)
        pt.time_from_start.nanosec = int((dur % 1.0) * 1e9)
        traj.points = [pt]
        goal.trajectory = traj

        # This method runs on a worker thread while the executor spins the
        # node, so plain polling on the futures is safe.
        send = self.client.send_goal_async(goal)
        while not send.done():
            threading.Event().wait(0.02)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('trajectory goal rejected')
            return False
        result = handle.get_result_async()
        while not result.done():
            threading.Event().wait(0.02)
        self.last_q = list(q)
        code = result.result().result.error_code
        if code != 0:
            self.get_logger().warn(f'trajectory finished with code {code}')
        return True

    # ------------------------------------------------------------ typing
    def on_type_text(self, msg):
        if not self.busy.acquire(blocking=False):
            self.get_logger().warn('already typing, ignoring request')
            return
        threading.Thread(
            target=self.run_typing, args=(msg.data,), daemon=True).start()

    def run_typing(self, text):
        try:
            self._type(text)
        except Exception as e:  # noqa: BLE001 - report and keep node alive
            self.get_logger().error(f'typing failed: {e}')
            self.status(f'ERROR {e}')
        finally:
            self.busy.release()

    def _type(self, text):
        self.status(f'TYPING "{text}"')
        if not self.client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError('arm_controller action server unavailable')

        scan = list(self.get_parameter('scan_pose').value)
        self.status('SCAN: moving to scan pose')
        self.move_to(scan, extra_time=0.5)

        timeout = self.get_parameter('key_map_timeout').value
        if self.key_map is None:
            self.status('SCAN: waiting for keyboard detection...')
        if not self.map_event.wait(timeout=timeout):
            raise RuntimeError('no keyboard detection within timeout')
        # Let the detector's sliding median settle before trusting the map.
        threading.Event().wait(1.5)
        key_map = dict(self.key_map)
        self.status(f'SCAN: keyboard detected ({len(key_map)} keys)')

        hover = self.get_parameter('hover_height').value
        press = self.get_parameter('press_clearance').value
        dwell = self.get_parameter('dwell_sec').value

        for ch in text:
            name = 'SPACE' if ch == ' ' else ch.upper()
            if name not in key_map:
                self.status(f'SKIP "{ch}": not on keyboard')
                continue
            x, y, z = key_map[name]
            # Hover is a transit pose: allow a small tool tilt so far keys
            # stay reachable despite the one-sided wrist-pitch limit.
            # The press itself stays (near-)vertical.
            q_hover = ik.solve_tool_down(x, y, z + hover, max_tilt=0.30)
            q_press = ik.solve_tool_down(x, y, z + press, max_tilt=0.05)
            if q_hover is None or q_press is None:
                self.status(f'SKIP "{ch}": target unreachable '
                            f'({x:.3f},{y:.3f},{z:.3f})')
                continue
            self.status(f'KEY "{name}" at ({x:.3f},{y:.3f},{z:.3f})')
            self.move_to(q_hover)
            self.move_to(q_press, extra_time=dwell / 2)
            threading.Event().wait(dwell)
            self.move_to(q_hover)

        self.status('DONE: returning to scan pose')
        self.move_to(scan)
        self.status(f'DONE "{text}"')


def main(args=None):
    rclpy.init(args=args)
    node = TypingController()
    executor = MultiThreadedExecutor(num_threads=4)
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
