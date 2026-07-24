"""Closed-form kinematics for the ARCh 5-DOF rover arm.

Geometry (from rover_arm_urdf.urdf — every joint stacks along +Z at the
zero pose):

    base_link --0.043--> j1 (base roll, Z) --0.0875--> j2 (shoulder pitch, Y)
    --0.4725--> j3 (elbow pitch, Y) --0.6025--> j4 (wrist pitch, Y)
    --0.030--> j5 (wrist roll, Z) --0.123--> tool_tip

Pitch limits are one-sided (0..pi/2), so the arm always bends forward,
toward +X of the yawed base frame.

The arm has 5 DOF, so a full 6-DOF pose is over-constrained.  For typing
we need position (3) plus a straight-down tool axis (2) = exactly 5
constraints, which this solver handles in closed form:

    yaw  = atan2(y, x)
    q2,q3 = planar 2R to the wrist-pitch pivot (which sits directly above
            the tool tip by L3 when the tool points down)
    q4   = pi - q2 - q3     (makes the tool axis vertical, pointing down)
    q5   = 0                (wrist roll is free; keep the camera aligned)
"""

import math

# Link constants (metres).
Z_SHOULDER = 0.043 + 0.0875          # base_link -> shoulder pitch pivot
L_UPPER = 0.4725                     # shoulder -> elbow
L_FOREARM = 0.6025                   # elbow -> wrist pitch
L_TOOL = 0.030 + 0.123               # wrist pitch pivot -> tool_tip

ARM_JOINT_NAMES = [
    'joint_1_base_roll',
    'joint_2_shoulder_pitch',
    'joint_3_elbow_pitch',
    'joint_4_wrist_pitch',
    'joint_5_wrist_roll',
]

PITCH_LO, PITCH_HI = 0.0, 1.57       # URDF limits for q2, q3, q4
_EPS = 1e-6


def _solve_plane(r, z, tilt):
    """Planar solve for a tool axis tilted `tilt` rad back from straight
    down (tilt=0 -> vertical).  Returns (q2, q3, q4) unchecked, or None
    if the wrist point is geometrically unreachable."""
    a4 = math.pi - tilt
    rw = r - L_TOOL * math.sin(tilt)
    zw = z + L_TOOL * math.cos(tilt) - Z_SHOULDER
    d2 = rw * rw + zw * zw
    d = math.sqrt(d2)

    if d > L_UPPER + L_FOREARM - _EPS:
        return None                                   # too far
    if d < abs(L_UPPER - L_FOREARM) + _EPS:
        return None                                   # too close

    c3 = (d2 - L_UPPER**2 - L_FOREARM**2) / (2.0 * L_UPPER * L_FOREARM)
    c3 = max(-1.0, min(1.0, c3))
    q3 = math.acos(c3)                                # elbow-forward branch

    # Angle of the shoulder->wrist line from vertical (+Z), then subtract
    # the interior triangle angle.
    phi = math.atan2(rw, zw)
    q2 = phi - math.atan2(L_FOREARM * math.sin(q3),
                          L_UPPER + L_FOREARM * math.cos(q3))
    q4 = a4 - q2 - q3
    return q2, q3, q4


def solve_tool_down(x, y, z, max_tilt=0.0):
    """IK for tool_tip at (x, y, z) in base_link frame, tool pointing
    straight down.  Returns [q1..q5] or None.

    If the wrist-pitch limit blocks a perfectly vertical tool (common for
    far targets at hover height), the tool is allowed to lean back toward
    the base by up to `max_tilt` rad — useful for transit/hover poses
    where a couple of degrees of tilt is irrelevant.
    """
    yaw = math.atan2(y, x)
    r = math.hypot(x, y)

    tilt = 0.0
    sol = None
    for _ in range(4):
        cand = _solve_plane(r, z, tilt)
        if cand is None:
            return None
        q2, q3, q4 = cand
        if q4 <= PITCH_HI + _EPS:
            sol = cand
            break
        needed = q4 - PITCH_HI + 0.005
        if tilt + needed > max_tilt + _EPS:
            return None
        tilt += needed
    if sol is None:
        return None

    q2, q3, q4 = sol
    for q in (q2, q3, q4):
        if q < PITCH_LO - _EPS or q > PITCH_HI + _EPS:
            return None

    return [yaw,
            min(max(q2, PITCH_LO), PITCH_HI),
            min(max(q3, PITCH_LO), PITCH_HI),
            min(max(q4, PITCH_LO), PITCH_HI),
            0.0]


def fk_tool_tip(q):
    """Forward kinematics of tool_tip in base_link.  Returns (x, y, z,
    tool_angle) where tool_angle is the tool axis angle from +Z (pi ==
    pointing straight down)."""
    yaw, q2, q3, q4 = q[0], q[1], q[2], q[3]
    a2 = q2
    a3 = q2 + q3
    a4 = q2 + q3 + q4
    r = (L_UPPER * math.sin(a2)
         + L_FOREARM * math.sin(a3)
         + L_TOOL * math.sin(a4))
    z = (Z_SHOULDER
         + L_UPPER * math.cos(a2)
         + L_FOREARM * math.cos(a3)
         + L_TOOL * math.cos(a4))
    return (r * math.cos(yaw), r * math.sin(yaw), z, a4)
