"""Small BEV geometry helpers (rotated rectangles) without external dependencies."""

import numpy as np


def corners_bev(center, size, yaw):
    """4x2 corners (counter-clockwise) of a BEV rectangle."""
    hl, hw = size[0] / 2.0, size[1] / 2.0
    c, s = np.cos(yaw), np.sin(yaw)
    local = np.array([[hl, hw], [-hl, hw], [-hl, -hw], [hl, -hw]])
    rot = np.array([[c, -s], [s, c]])
    return local @ rot.T + np.asarray(center[:2])


def _clip(subject, a, b):
    """Sutherland-Hodgman: clip polygon by the half-plane left of edge a->b."""
    out = []
    n = len(subject)
    if n == 0:
        return out

    def side(p):
        return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])

    for i in range(n):
        p, q = subject[i], subject[(i + 1) % n]
        sp, sq = side(p), side(q)
        if sp >= 0:
            out.append(p)
        if (sp >= 0) != (sq >= 0):
            t = sp / (sp - sq)
            out.append(p + t * (q - p))
    return out


def polygon_area(poly):
    if len(poly) < 3:
        return 0.0
    p = np.asarray(poly)
    x, y = p[:, 0], p[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def bev_iou(box_a, box_b):
    """box = (center, size, yaw); returns BEV IoU of the two rotated rectangles."""
    ca = corners_bev(*box_a)
    cb = corners_bev(*box_b)
    inter = list(ca)
    for i in range(4):
        inter = _clip(inter, cb[i], cb[(i + 1) % 4])
        if not inter:
            return 0.0
    ai = polygon_area(inter)
    aa = box_a[1][0] * box_a[1][1]
    ab = box_b[1][0] * box_b[1][1]
    union = aa + ab - ai
    return float(ai / union) if union > 0 else 0.0


def points_in_bev_box(xy, center, size, yaw, margin=0.0):
    d = xy - np.asarray(center[:2])
    c, s = np.cos(-yaw), np.sin(-yaw)
    x = c * d[:, 0] - s * d[:, 1]
    y = s * d[:, 0] + c * d[:, 1]
    return (np.abs(x) <= size[0] / 2 + margin) & (np.abs(y) <= size[1] / 2 + margin)


def points_in_box(xyz, center, size, yaw, z_bottom_skip=0.0):
    inside = points_in_bev_box(xyz[:, :2], center, size, yaw)
    z0 = center[2] - size[2] / 2 + z_bottom_skip
    z1 = center[2] + size[2] / 2
    return inside & (xyz[:, 2] >= z0) & (xyz[:, 2] <= z1)


def orientation_error_mod90(yaw_a, yaw_b):
    """Footprint orientation error in [0, 45] deg (rectangles are symmetric under 90 deg
    rotation once length/width are allowed to swap)."""
    d = np.abs((yaw_a - yaw_b) % (np.pi / 2))
    return float(np.degrees(min(d, np.pi / 2 - d)))


def heading_error_mod180(yaw_gt, yaw_pred):
    """Heading error in [0, 180] deg: unlike `orientation_error_mod90`, a box pointing
    backwards (or sideways) counts as wrong. Used for moving objects, whose direction of
    travel is known."""
    d = abs((yaw_gt - yaw_pred + np.pi) % (2 * np.pi) - np.pi)
    return float(np.degrees(d))
