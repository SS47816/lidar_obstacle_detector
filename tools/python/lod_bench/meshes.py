"""Low-poly 3D models of the object classes (car, truck, bus, ...).

Each model is built from convex parts at a typical real-world size, then normalised to the
unit cube [-0.5, 0.5]^3 (x forward, y left, z up), so a viewer scales it to a box with
`scale = box size` (RViz MESH_RESOURCE markers, the demo renderer).

    python -I meshes.py OUT_DIR     # writes OUT_DIR/<class>.stl (binary STL)
"""

import os
import struct
import sys

import numpy as np
from scipy.spatial import ConvexHull


def _hull(points):
    """Outward-oriented triangles (N x 3 x 3) of the convex hull of `points`."""
    pts = np.asarray(points, dtype=float)
    hull = ConvexHull(pts)
    tris = pts[hull.simplices]
    centre = pts.mean(0)
    normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    flip = np.einsum("ij,ij->i", normals, tris.mean(1) - centre) < 0
    tris[flip] = tris[flip][:, ::-1]
    return tris


def _box(x0, x1, y0, y1, z0, z1):
    return [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]


def _frustum(bottom, top):
    """Hull of two axis-aligned rectangles (x0, x1, y_half, z) stacked on each other."""
    (bx0, bx1, by, bz), (tx0, tx1, ty, tz) = bottom, top
    return [(x, s * by, bz) for x in (bx0, bx1) for s in (-1, 1)] + [
        (x, s * ty, tz) for x in (tx0, tx1) for s in (-1, 1)
    ]


def _cylinder_y(cx, cy, cz, radius, width, segments=12):
    """Wheel: cylinder along y."""
    a = np.linspace(0, 2 * np.pi, segments, endpoint=False)
    ring = np.stack([cx + radius * np.cos(a), cz + radius * np.sin(a)], 1)
    return [(x, cy + s * width / 2, z) for x, z in ring for s in (-1, 1)]


def _sphere(cx, cy, cz, r, n=8):
    pts = []
    for i in range(1, n):
        phi = np.pi * i / n
        for j in range(n):
            th = 2 * np.pi * j / n
            pts.append(
                (
                    cx + r * np.sin(phi) * np.cos(th),
                    cy + r * np.sin(phi) * np.sin(th),
                    cz + r * np.cos(phi),
                )
            )
    return pts + [(cx, cy, cz + r), (cx, cy, cz - r)]


def _wheels(xs, y, z, r, w):
    return [_cylinder_y(x, s * y, z, r, w) for x in xs for s in (-1, 1)]


def _rider(x, top):
    """Seated person: torso and head."""
    return [
        _frustum((x - 0.25, x + 0.05, 0.2, top - 0.75), (x - 0.12, x + 0.08, 0.17, top - 0.3)),
        _sphere(x, 0, top - 0.13, 0.13),
    ]


def car():
    return [
        # body with a sloped hood and boot
        [(x, s * 0.95, 0.25) for x in (-2.3, 2.3) for s in (-1, 1)]
        + [
            (x, s * 0.93, z)
            for x, z in ((2.3, 0.5), (2.15, 0.82), (-2.3, 0.45), (-2.2, 0.92))
            for s in (-1, 1)
        ],
        _frustum((-1.55, 0.95, 0.86, 0.85), (-1.05, 0.2, 0.72, 1.6)),  # cabin
        *_wheels((1.45, -1.45), 0.8, 0.34, 0.34, 0.26),
    ]


def truck():
    return [
        _box(-4.0, 4.0, -1.1, 1.1, 0.5, 0.95),  # chassis
        # cab with a sloped windscreen
        [
            (x, s * 1.2, z)
            for x, z in ((2.35, 0.6), (4.0, 0.6), (4.0, 1.9), (3.7, 3.0), (2.35, 3.0))
            for s in (-1, 1)
        ],
        _box(-4.0, 2.2, -1.25, 1.25, 0.95, 3.4),  # cargo box
        *_wheels((3.1, -1.9, -3.1), 1.0, 0.5, 0.5, 0.35),
    ]


def bus():
    body = [(x, s * 1.3, z) for x in (-5.5, 5.5) for s in (-1, 1) for z in (0.4, 2.9)]
    body += [(x, s * 1.15, 3.2) for x in (-5.3, 5.3) for s in (-1, 1)]  # rounded roof edge
    return [body, *_wheels((3.7, -3.2), 1.15, 0.5, 0.5, 0.35)]


def trailer():
    return [_box(-3.0, 3.0, -1.25, 1.25, 1.0, 3.6), *_wheels((-1.6, -2.7), 1.0, 0.5, 0.5, 0.35)]


def pedestrian():
    parts = [
        _box(-0.12, 0.12, 0.04, 0.2, 0.0, 0.85),  # legs
        _box(-0.12, 0.12, -0.2, -0.04, 0.0, 0.85),
        _frustum((-0.14, 0.14, 0.2, 0.85), (-0.12, 0.12, 0.24, 1.45)),  # torso
        _sphere(0.0, 0.0, 1.6, 0.13),
        _box(-0.07, 0.07, 0.24, 0.32, 0.8, 1.42),  # arms
        _box(-0.07, 0.07, -0.32, -0.24, 0.8, 1.42),
    ]
    return parts


def bicycle():
    return [
        _cylinder_y(0.55, 0, 0.34, 0.34, 0.06),
        _cylinder_y(-0.55, 0, 0.34, 0.34, 0.06),
        _frustum((-0.55, 0.55, 0.03, 0.32), (-0.2, 0.45, 0.03, 0.75)),  # frame
        *_rider(-0.1, 1.75),
    ]


def motorcycle():
    return [
        _cylinder_y(0.75, 0, 0.32, 0.32, 0.14),
        _cylinder_y(-0.75, 0, 0.32, 0.32, 0.14),
        _frustum((-0.75, 0.8, 0.2, 0.3), (-0.55, 0.55, 0.16, 0.9)),  # body
        *_rider(-0.2, 1.65),
    ]


def traffic_cone():
    a = np.linspace(0, 2 * np.pi, 12, endpoint=False)
    cone = [(0.16 * np.cos(t), 0.16 * np.sin(t), 0.05) for t in a] + [(0, 0, 0.8)]
    return [_box(-0.2, 0.2, -0.2, 0.2, 0.0, 0.05), cone]


def barrier():
    return [_frustum((-1.0, 1.0, 0.25, 0.0), (-1.0, 1.0, 0.1, 1.0))]


MODELS = {
    "car": car,
    "truck": truck,
    "construction_vehicle": truck,
    "bus": bus,
    "trailer": trailer,
    "pedestrian": pedestrian,
    "bicycle": bicycle,
    "motorcycle": motorcycle,
    "traffic_cone": traffic_cone,
    "barrier": barrier,
}


def build(name):
    """Triangles (N x 3 x 3) of a class model, normalised to the unit cube."""
    tris = np.concatenate([_hull(p) for p in MODELS[name]()])
    lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
    return (tris - (lo + hi) / 2) / (hi - lo)


def write_stl(path, tris):
    normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    with open(path, "wb") as f:
        f.write(b"lidar_obstacle_detector low-poly model".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for n, t in zip(normals, tris):
            f.write(struct.pack("<12fH", *n, *t.reshape(-1), 0))


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "meshes"
    os.makedirs(out, exist_ok=True)
    for name in MODELS:
        tris = build(name)
        write_stl(os.path.join(out, f"{name}.stl"), tris)
        print(f"{name}: {len(tris)} triangles")


if __name__ == "__main__":
    main()
