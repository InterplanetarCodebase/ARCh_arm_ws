"""Generate the keyboard texture and its machine-readable key layout.

Draws a QWERTY keyboard face with four ArUco markers (DICT_4X4_50,
ids 0-3) in the corners, and writes:

  models/aruco_keyboard/materials/textures/keyboard.png
  config/key_layout.yaml

The layout file is the single source of truth shared by the texture, the
detector (homography model) and nothing else — key positions are
expressed in the *keyboard surface frame*: origin at the keyboard
centre, +x along the long axis (toward the right of the keyboard),
+y toward the space bar, units metres.

Run once during development (outputs are committed):
    python3 -m rover_arm_typing.texture_gen [output_package_root]
"""

import os
import sys

import cv2
import numpy as np
import yaml

# Physical size of the keyboard face (m) and raster scale (px/m).
KB_W = 0.50
KB_H = 0.20
SCALE = 3000

MARKER_SIZE = 0.055
# id -> centre in surface frame; ids fixed so the detector knows which
# physical corner each marker is.  Markers are deliberately large: the
# wrist camera reads them from ~0.5 m away.
MARKERS = {
    0: (-0.2175, -0.0675),   # top-left    (away from space bar)
    1: (0.2175, -0.0675),    # top-right
    2: (0.2175, 0.0675),     # bottom-right
    3: (-0.2175, 0.0675),    # bottom-left
}

KEY_SIZE = 0.032
ROWS = [
    ('QWERTYUIOP', -0.040, 0.0),
    ('ASDFGHJKL', 0.000, 0.019),
    ('ZXCVBNM', 0.040, 0.038),
]
SPACE_CENTER = (0.0, 0.062)
SPACE_W, SPACE_H = 0.150, 0.024
KEY_PITCH = 0.038


def build_key_map():
    keys = {}
    for chars, row_y, x_shift in ROWS:
        n = len(chars)
        for i, ch in enumerate(chars):
            x = (i - (n - 1) / 2.0) * KEY_PITCH + x_shift * 0
            keys[ch] = (round(x, 5), round(row_y, 5))
    keys['SPACE'] = SPACE_CENTER
    return keys


def to_px(x, y):
    return (int(round((x + KB_W / 2) * SCALE)),
            int(round((y + KB_H / 2) * SCALE)))


def generate(pkg_root):
    w_px, h_px = int(KB_W * SCALE), int(KB_H * SCALE)
    img = np.full((h_px, w_px, 3), 235, np.uint8)          # light face
    cv2.rectangle(img, (0, 0), (w_px - 1, h_px - 1), (40, 40, 40), 8)

    keys = build_key_map()
    half = int(KEY_SIZE / 2 * SCALE)
    for ch, (x, y) in keys.items():
        u, v = to_px(x, y)
        if ch == 'SPACE':
            hw, hh = int(SPACE_W / 2 * SCALE), int(SPACE_H / 2 * SCALE)
            cv2.rectangle(img, (u - hw, v - hh), (u + hw, v + hh),
                          (60, 60, 60), -1)
            cv2.rectangle(img, (u - hw, v - hh), (u + hw, v + hh),
                          (20, 20, 20), 3)
        else:
            cv2.rectangle(img, (u - half, v - half), (u + half, v + half),
                          (60, 60, 60), -1)
            cv2.rectangle(img, (u - half, v - half), (u + half, v + half),
                          (20, 20, 20), 3)
            ts = cv2.getTextSize(ch, cv2.FONT_HERSHEY_SIMPLEX, 1.6, 4)[0]
            cv2.putText(img, ch, (u - ts[0] // 2, v + ts[1] // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.6, (230, 230, 230), 4,
                        cv2.LINE_AA)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    m_px = int(MARKER_SIZE * SCALE)
    for mid, (mx, my) in MARKERS.items():
        marker = cv2.aruco.generateImageMarker(aruco_dict, mid, m_px)
        u, v = to_px(mx, my)
        # White quiet zone around the marker so detection stays reliable.
        pad = m_px // 6
        cv2.rectangle(img, (u - m_px // 2 - pad, v - m_px // 2 - pad),
                      (u + m_px // 2 + pad, v + m_px // 2 + pad),
                      (255, 255, 255), -1)
        img[v - m_px // 2:v - m_px // 2 + m_px,
            u - m_px // 2:u - m_px // 2 + m_px] = \
            cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)

    tex_path = os.path.join(pkg_root, 'models', 'aruco_keyboard',
                            'materials', 'textures', 'keyboard.png')
    os.makedirs(os.path.dirname(tex_path), exist_ok=True)
    # gz-sim/ogre2 maps a box's top-face UVs rotated 90deg relative to the
    # model x/y axes (verified empirically against detected marker
    # positions), so pre-rotate the texture to compensate and keep the
    # painted keys aligned with the layout coordinates.
    cv2.imwrite(tex_path, cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE))

    layout = {
        'keyboard': {'width': KB_W, 'height': KB_H},
        'aruco': {
            'dictionary': 'DICT_4X4_50',
            'marker_size': MARKER_SIZE,
            'markers': {int(k): [float(v[0]), float(v[1])]
                        for k, v in MARKERS.items()},
        },
        'keys': {k: [float(v[0]), float(v[1])] for k, v in keys.items()},
    }
    layout_path = os.path.join(pkg_root, 'config', 'key_layout.yaml')
    os.makedirs(os.path.dirname(layout_path), exist_ok=True)
    with open(layout_path, 'w') as f:
        yaml.safe_dump(layout, f, sort_keys=True)

    print(f'texture: {tex_path} ({w_px}x{h_px})')
    print(f'layout:  {layout_path} ({len(keys)} keys)')


def main(args=None):
    root = sys.argv[1] if len(sys.argv) > 1 else os.getcwd()
    generate(root)


if __name__ == '__main__':
    main()
