import math

import pytest

from rover_arm_typing import ik


def grid_targets():
    """Targets across (and beyond) the tool-down workspace."""
    targets = []
    for r in (0.55, 0.65, 0.75, 0.85, 0.95):
        for z in (0.30, 0.35, 0.42, 0.50):
            for yaw_deg in (-40, -15, 0, 20, 45):
                yaw = math.radians(yaw_deg)
                targets.append((r * math.cos(yaw), r * math.sin(yaw), z))
    return targets


def test_ik_fk_roundtrip():
    solved = 0
    for x, y, z in grid_targets():
        q = ik.solve_tool_down(x, y, z)
        if q is None:
            continue
        solved += 1
        fx, fy, fz, tool_angle = ik.fk_tool_tip(q)
        assert abs(fx - x) < 1e-6, (x, y, z, q)
        assert abs(fy - y) < 1e-6
        assert abs(fz - z) < 1e-6
        # Tool must point straight down.
        assert abs(tool_angle - math.pi) < 1e-9
        # Joint limits respected.
        for qi in q[1:4]:
            assert -1e-6 <= qi <= 1.57 + 1e-6
    # A healthy share of the keyboard-region grid must be reachable.
    assert solved >= 20, f'only {solved} targets solvable'


def test_keyboard_region_reachable():
    """Every key of the keyboard as placed in typing_world.sdf must be
    reachable at hover and press heights.

    World mapping (see typing_world.sdf): key (kx, ky) ->
    x = 0.78 - ky, y = -kx, z = 0.35.
    """
    import os

    import yaml
    layout_path = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'key_layout.yaml')
    with open(layout_path) as f:
        layout = yaml.safe_load(f)
    for name, (kx, ky) in layout['keys'].items():
        wx, wy, wz = 0.78 - ky, -kx, 0.35
        # Same tilt allowances the typing controller uses.
        for dz, tilt in ((0.004, 0.05), (0.06, 0.30)):
            q = ik.solve_tool_down(wx, wy, wz + dz, max_tilt=tilt)
            assert q is not None, \
                f'key {name} unreachable at ({wx:.3f},{wy:.3f},{wz + dz:.3f})'


def test_unreachable_rejected():
    assert ik.solve_tool_down(2.0, 0.0, 0.3) is None      # too far
    assert ik.solve_tool_down(0.05, 0.0, 1.2) is None     # too close/high
