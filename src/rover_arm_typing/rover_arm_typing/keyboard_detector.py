"""Keyboard detector: ArUco corners + homography + depth -> 3D key map.

Pipeline per RGB frame:
  1. Detect the four corner ArUco markers (DICT_4X4_50 ids 0-3).
  2. Homography from the keyboard surface frame (metres, from
     key_layout.yaml) to image pixels using all detected marker corners.
  3. Project every key centre through the homography, sample a median
     depth patch, deproject through the camera intrinsics to 3D in the
     optical frame, then TF-transform into base_link.
  4. Median-filter over a sliding window of frames; once stable, publish:
       /keyboard/key_map   std_msgs/String (JSON {key: [x,y,z]}), latched
       /keyboard/markers   visualization_msgs/MarkerArray (RViz)
"""

import collections
import json
import os

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def image_to_bgr(msg):
    """sensor_msgs/Image -> BGR ndarray without cv_bridge (whose C
    extension is built against NumPy 1 and segfaults under NumPy 2)."""
    buf = np.frombuffer(msg.data, np.uint8)
    if msg.encoding in ('rgb8', 'bgr8'):
        img = buf.reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
        return img[:, :, ::-1].copy() if msg.encoding == 'rgb8' else img
    if msg.encoding == 'mono8':
        m = buf.reshape(msg.height, msg.step)[:, :msg.width]
        return cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
    raise ValueError(f'unsupported encoding {msg.encoding}')


def image_to_depth(msg):
    """sensor_msgs/Image (32FC1) -> float32 metres ndarray."""
    if msg.encoding != '32FC1':
        raise ValueError(f'unsupported depth encoding {msg.encoding}')
    buf = np.frombuffer(msg.data, np.float32)
    return buf.reshape(msg.height, msg.step // 4)[:, :msg.width]


def quat_to_matrix(qx, qy, qz, qw):
    x2, y2, z2 = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array([
        [1 - 2 * (y2 + z2), 2 * (xy - wz), 2 * (xz + wy)],
        [2 * (xy + wz), 1 - 2 * (x2 + z2), 2 * (yz - wx)],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (x2 + y2)],
    ])


class KeyboardDetector(Node):

    def __init__(self):
        super().__init__('keyboard_detector')
        self.declare_parameter('image_topic', '/wrist_camera/image')
        self.declare_parameter('depth_topic', '/wrist_camera/depth_image')
        self.declare_parameter('info_topic', '/wrist_camera/camera_info')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', '')   # '' -> use image header
        self.declare_parameter('stable_frames', 5)
        self.declare_parameter('depth_min', 0.15)
        self.declare_parameter('depth_max', 2.5)

        layout_path = os.path.join(
            get_package_share_directory('rover_arm_typing'),
            'config', 'key_layout.yaml')
        with open(layout_path) as f:
            layout = yaml.safe_load(f)
        self.keys = {k: np.array(v, dtype=float)
                     for k, v in layout['keys'].items()}
        msize = layout['aruco']['marker_size']
        half = msize / 2.0
        # Surface-frame corner coordinates for each marker id, in the
        # order cv2.aruco reports corners (TL, TR, BR, BL of the marker
        # as drawn; texture u->+x, v->+y).
        self.marker_obj = {}
        for mid, (cx, cy) in layout['aruco']['markers'].items():
            self.marker_obj[int(mid)] = np.array([
                [cx - half, cy - half],
                [cx + half, cy - half],
                [cx + half, cy + half],
                [cx - half, cy + half],
            ], dtype=np.float32)

        d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(
            d, cv2.aruco.DetectorParameters())

        self.depth = None
        self.info = None
        self.window = collections.deque(
            maxlen=int(self.get_parameter('stable_frames').value))
        self.published_map = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(
            String, '/keyboard/key_map', latched)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/keyboard/markers', 10)

        self.create_subscription(
            Image, self.get_parameter('image_topic').value,
            self.on_image, 10)
        self.create_subscription(
            Image, self.get_parameter('depth_topic').value,
            self.on_depth, 10)
        self.create_subscription(
            CameraInfo, self.get_parameter('info_topic').value,
            self.on_info, 10)
        self.get_logger().info(
            f'loaded layout with {len(self.keys)} keys from {layout_path}')

    def on_depth(self, msg):
        self.depth = msg

    def on_info(self, msg):
        self.info = msg

    def on_image(self, msg):
        if self.depth is None or self.info is None:
            return
        img = image_to_bgr(msg)
        corners, ids, _ = self.detector.detectMarkers(img)
        if ids is None:
            return
        obj_pts, img_pts = [], []
        for c, mid in zip(corners, ids.flatten()):
            if int(mid) in self.marker_obj:
                obj_pts.append(self.marker_obj[int(mid)])
                img_pts.append(c.reshape(4, 2))
        if len(obj_pts) < 2:      # need >= 8 correspondences for a stable H
            return
        obj = np.concatenate(obj_pts)
        pix = np.concatenate(img_pts).astype(np.float32)
        H, _ = cv2.findHomography(obj, pix, cv2.RANSAC, 3.0)
        if H is None:
            return

        depth = image_to_depth(self.depth)
        fx, fy = self.info.k[0], self.info.k[4]
        cx, cy = self.info.k[2], self.info.k[5]
        d_lo = self.get_parameter('depth_min').value
        d_hi = self.get_parameter('depth_max').value

        cam_pts = {}
        h_img, w_img = depth.shape[:2]
        for name, kxy in self.keys.items():
            p = H @ np.array([kxy[0], kxy[1], 1.0])
            u, v = p[0] / p[2], p[1] / p[2]
            ui, vi = int(round(u)), int(round(v))
            if not (2 <= ui < w_img - 2 and 2 <= vi < h_img - 2):
                continue
            patch = depth[vi - 2:vi + 3, ui - 2:ui + 3]
            vals = patch[np.isfinite(patch)]
            vals = vals[(vals > d_lo) & (vals < d_hi)]
            if vals.size < 3:
                continue
            z = float(np.median(vals))
            cam_pts[name] = np.array(
                [(u - cx) / fx * z, (v - cy) / fy * z, z])
        # Only accept frames where every key got a valid depth sample; a
        # partial map would make the typing controller skip characters.
        if len(cam_pts) < len(self.keys):
            return

        cam_frame = self.get_parameter('camera_frame').value \
            or msg.header.frame_id
        base = self.get_parameter('base_frame').value
        try:
            tf = self.tf_buffer.lookup_transform(
                base, cam_frame, rclpy.time.Time())
        except Exception as e:  # noqa: BLE001 - tf2 raises several types
            self.get_logger().warn(
                f'TF {base} <- {cam_frame} unavailable: {e}',
                throttle_duration_sec=5.0)
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat_to_matrix(q.x, q.y, q.z, q.w)
        T = np.array([t.x, t.y, t.z])

        frame_map = {n: (R @ p + T) for n, p in cam_pts.items()}
        self.window.append(frame_map)
        if len(self.window) < self.window.maxlen:
            return

        # Median over the window, per key present in every frame.
        common = set(self.window[0])
        for fm in self.window:
            common &= set(fm)
        stable = {
            n: np.median(np.stack([fm[n] for fm in self.window]), axis=0)
            for n in common
        }
        if not stable:
            return
        out = {n: [round(float(x), 4) for x in p] for n, p in stable.items()}
        payload = json.dumps(out, sort_keys=True)
        if payload != self.published_map:
            self.published_map = payload
            m = String()
            m.data = payload
            self.map_pub.publish(m)
            self.get_logger().info(
                f'published key map ({len(out)} keys), e.g. '
                + ', '.join(f'{k}:{v}' for k, v in list(out.items())[:2]))
        self.publish_markers(stable, base)

    def publish_markers(self, key_map, frame):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, (name, p) in enumerate(sorted(key_map.items())):
            for kind, mid in (('cube', i * 2), ('text', i * 2 + 1)):
                mk = Marker()
                mk.header.frame_id = frame
                mk.header.stamp = now
                mk.ns = 'keys'
                mk.id = mid
                mk.action = Marker.ADD
                mk.pose = Pose()
                mk.pose.position.x = float(p[0])
                mk.pose.position.y = float(p[1])
                mk.pose.position.z = float(p[2]) + (0.02 if kind == 'text'
                                                    else 0.0)
                mk.pose.orientation.w = 1.0
                if kind == 'cube':
                    mk.type = Marker.CUBE
                    mk.scale.x = mk.scale.y = 0.025
                    mk.scale.z = 0.004
                    mk.color.r, mk.color.g, mk.color.b, mk.color.a = \
                        0.1, 0.8, 0.2, 0.8
                else:
                    mk.type = Marker.TEXT_VIEW_FACING
                    mk.text = name
                    mk.scale.z = 0.02
                    mk.color.r = mk.color.g = mk.color.b = mk.color.a = 1.0
                arr.markers.append(mk)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
