"""Key-press validator: watches TF and emits typed characters.

Polls the base_link -> tool_tip transform; when the tip descends into a
small cylinder above a detected key (horizontal radius `xy_tol`, height
below `press_z` above the key surface), that key is emitted once on
/typed_keys.  The key re-arms when the tip retracts above `rearm_z`.

If /type_text was seen, prints a PASS/FAIL comparison once the same
number of characters has been typed.
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


class KeyPressValidator(Node):

    def __init__(self):
        super().__init__('key_press_validator')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tip_frame', 'tool_tip')
        self.declare_parameter('xy_tol', 0.02)
        self.declare_parameter('press_z', 0.025)
        self.declare_parameter('rearm_z', 0.05)

        self.key_map = {}
        self.armed = True
        self.expected = None
        self.typed = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, '/keyboard/key_map', self.on_key_map, latched)
        self.create_subscription(String, '/type_text', self.on_text, 10)
        self.typed_pub = self.create_publisher(String, '/typed_keys', 10)

        self.create_timer(1.0 / 30.0, self.tick)

    def on_key_map(self, msg):
        self.key_map = json.loads(msg.data)
        self.get_logger().info(f'validator got key map ({len(self.key_map)})')

    def on_text(self, msg):
        self.expected = msg.data
        self.typed = []
        self.get_logger().info(f'expecting "{self.expected}"')

    def tick(self):
        if not self.key_map:
            return
        base = self.get_parameter('base_frame').value
        tip = self.get_parameter('tip_frame').value
        try:
            tf = self.tf_buffer.lookup_transform(
                base, tip, rclpy.time.Time())
        except Exception:  # noqa: BLE001 - tf not up yet
            return
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        tz = tf.transform.translation.z

        xy_tol = self.get_parameter('xy_tol').value
        press_z = self.get_parameter('press_z').value
        rearm_z = self.get_parameter('rearm_z').value

        # Closest key horizontally.
        best, dist = None, 1e9
        for name, (kx, ky, kz) in self.key_map.items():
            d = math.hypot(tx - kx, ty - ky)
            if d < dist:
                best, dist = name, d
        if best is None:
            return
        kz = self.key_map[best][2]

        if self.armed and dist < xy_tol and tz < kz + press_z:
            self.armed = False
            ch = ' ' if best == 'SPACE' else best
            self.typed.append(ch)
            msg = String()
            msg.data = ch
            self.typed_pub.publish(msg)
            self.get_logger().info(f'PRESS "{best}" '
                                   f'(dxy={dist * 1000:.1f} mm)')
            self.check_done()
        elif not self.armed and tz > kz + rearm_z:
            self.armed = True

    def check_done(self):
        if self.expected is None:
            return
        want = self.expected.upper()
        if len(self.typed) >= len(want):
            got = ''.join(self.typed)[:len(want)]
            verdict = 'PASS' if got == want else 'FAIL'
            self.get_logger().info(
                f'{verdict}: expected "{want}" typed "{got}"')


def main(args=None):
    rclpy.init(args=args)
    node = KeyPressValidator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
