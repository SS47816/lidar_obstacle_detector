"""Render the showcase video: one or more scenarios, each a 3D chase view of a run with the vector
map, class models for recognised objects, 2.5D pillars for everything else, predicted paths and
a front-camera inset with the projected boxes.

    python -I render_showcase.py --out demo.gif \
        --scenario SEQ_DIR RUN_DIR START FRAMES "Title" "feature one|feature two" \
        [--scenario ...] [--width 960] [--fps 10]

SEQ_DIR needs map.json / camera.jsonl for the map and the inset (prepare_av2.py --with-map
--with-camera); RUN_DIR is a `lod_benchmark --dump-every 1` output.
"""

import argparse
import json
import os
import shutil
import sys
import tempfile
from multiprocessing import Pool

import matplotlib

matplotlib.use("Agg")
import matplotlib.image as mpimg  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib import patheffects  # noqa: E402
from matplotlib.collections import LineCollection, PolyCollection  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import read_sequence  # noqa: E402
from render_demo import encode, points_in_boxes, track_color  # noqa: E402

import meshes  # noqa: E402

BG = "#0a0e16"
TEXT = "#eef2f8"
MUTED = "#8d97aa"
ACCENT = "#36d1c4"
GROUND_RGB = np.array([0.06, 0.36, 0.40])
PILLAR_RGB = np.array([0.50, 0.55, 0.65])
NEAR = 2.0  # [m] near clipping plane of the chase camera
VIEW_RANGE = 42.0  # [m]
LIGHT = np.array([-0.35, 0.45, 1.0]) / np.linalg.norm([-0.35, 0.45, 1.0])

CLASS_RGB = {
    "car": "#3f9eff",
    "truck": "#ff9a26",
    "construction_vehicle": "#ff9a26",
    "trailer": "#ff9a26",
    "bus": "#ffd133",
    "pedestrian": "#ff5a73",
    "bicycle": "#4de680",
    "motorcycle": "#4de680",
    "barrier": "#c8ccd4",
    "traffic_cone": "#ff7b3a",
}
MESHES = {name: meshes.build(name) for name in meshes.MODELS}


def rgb(c):
    return np.array(matplotlib.colors.to_rgb(c))


def object_rgb(b):
    c = CLASS_RGB.get(b.get("label", "unknown"))
    return rgb(c) if c else np.asarray(track_color(b["id"]))


# ----------------------------------------------------------------------------------------
# Camera
# ----------------------------------------------------------------------------------------
class Camera:
    def __init__(self, eye, target, f, cx, cy):
        fwd = np.asarray(target, float) - eye
        fwd /= np.linalg.norm(fwd)
        right = np.cross(fwd, [0, 0, 1.0])
        right /= np.linalg.norm(right)
        up = np.cross(right, fwd)
        self.eye = np.asarray(eye, float)
        self.rot = np.stack([right, -up, fwd])
        self.f, self.cx, self.cy = f, cx, cy

    def to_cam(self, xyz):
        return (np.asarray(xyz) - self.eye) @ self.rot.T

    def uv(self, cam):
        return np.c_[
            self.f * cam[..., 0] / cam[..., 2] + self.cx,
            self.f * cam[..., 1] / cam[..., 2] + self.cy,
        ]

    def clip_polygon(self, cam):
        """Sutherland-Hodgman against the near plane; camera-frame vertices in, out."""
        out = []
        n = len(cam)
        for i in range(n):
            a, b = cam[i], cam[(i + 1) % n]
            ina, inb = a[2] >= NEAR, b[2] >= NEAR
            if ina:
                out.append(a)
            if ina != inb:
                t = (NEAR - a[2]) / (b[2] - a[2])
                out.append(a + t * (b - a))
        return np.asarray(out)

    def clip_polyline(self, cam):
        """Visible pieces of a camera-frame polyline."""
        pieces, cur = [], []
        for i in range(len(cam) - 1):
            a, b = cam[i], cam[i + 1]
            if a[2] < NEAR and b[2] < NEAR:
                if len(cur) > 1:
                    pieces.append(np.asarray(cur))
                cur = []
                continue
            if a[2] < NEAR:
                a = a + (NEAR - a[2]) / (b[2] - a[2]) * (b - a)
            if b[2] < NEAR:
                b = b + (NEAR - b[2]) / (a[2] - b[2]) * (a - b)
                cur += [a, b]
                pieces.append(np.asarray(cur))
                cur = []
                continue
            if not cur:
                cur = [a]
            cur.append(b)
        if len(cur) > 1:
            pieces.append(np.asarray(cur))
        return pieces


# ----------------------------------------------------------------------------------------
# Data
# ----------------------------------------------------------------------------------------
class Scenario:
    def __init__(self, seq_dir, run_dir, start, frames, title, features):
        self.seq_dir, self.run_dir = seq_dir, run_dir
        self.meta, self.frames, _ = read_sequence(seq_dir)
        self.sensor_height = self.meta.get("sensor_height", 1.7)
        with open(os.path.join(run_dir, "results.jsonl")) as f:
            self.results = {r["idx"]: r for r in map(json.loads, f)}
        self.idxs = list(range(start, min(len(self.frames), start + frames)))
        self.title, self.features = title, features
        self.map = None
        path = os.path.join(seq_dir, "map.json")
        if os.path.exists(path):
            with open(path) as f:
                self.map = json.load(f)
        self.camera = {}
        path = os.path.join(seq_dir, "camera.jsonl")
        if os.path.exists(path):
            with open(path) as f:
                self.camera = {r["idx"]: r for r in map(json.loads, f)}

    def pose(self, i):
        return np.asarray(self.frames[i]["pose"]).reshape(4, 4)

    def clouds(self, i):
        base = os.path.join(self.run_dir, "clouds", f"{self.frames[i]['idx']:06d}")
        g = np.fromfile(base + "_ground.bin", np.float32).reshape(-1, 3)
        o = np.fromfile(base + "_obstacle.bin", np.float32).reshape(-1, 3)
        return g, o

    def pillars(self, i):
        path = os.path.join(self.run_dir, "clouds", f"{self.frames[i]['idx']:06d}_pillars.bin")
        if not os.path.exists(path):
            return np.zeros((0, 5))
        p = np.fromfile(path, np.int16).reshape(-1, 5).astype(float)
        p[:, :4] /= 100.0
        return p

    def ms(self, i):
        lo = max(0, i - 9)
        return np.mean([self.results[k]["timing_ms"]["total"] for k in range(lo, i + 1)])


# ----------------------------------------------------------------------------------------
# Scene layers
# ----------------------------------------------------------------------------------------
def draw_map(ax, cam, sc, inv):
    if sc.map is None:
        return

    def local(pts):
        p = np.asarray(pts, float)
        return p @ inv[:3, :3].T + inv[:3, 3]

    def near(p):
        return np.hypot(p[:, 0], p[:, 1]).min() < VIEW_RANGE + 15

    polys, colours = [], []
    for area in sc.map["drivable"]:
        p = local(area)
        if near(p):
            c = cam.clip_polygon(cam.to_cam(p))
            if len(c) >= 3:
                polys.append(cam.uv(c))
                colours.append((0.10, 0.13, 0.19, 1.0))
    for cw in sc.map["crosswalks"]:
        p = local(cw)
        if near(p):
            c = cam.clip_polygon(cam.to_cam(p))
            if len(c) >= 3:
                polys.append(cam.uv(c))
                colours.append((0.55, 0.60, 0.70, 0.30))
    ax.add_collection(PolyCollection(polys, facecolors=colours, edgecolors="none", zorder=1))

    segs, cols, widths, styles = [], [], [], []
    for lane in sc.map["lanes"]:
        mark = lane["mark"]
        if mark == "NONE" or lane["intersection"]:
            continue
        p = local(lane["points"])
        if not near(p):
            continue
        yellow = "YELLOW" in mark
        for piece in cam.clip_polyline(cam.to_cam(p)):
            segs.append(cam.uv(piece))
            cols.append((0.95, 0.80, 0.30, 0.55) if yellow else (0.85, 0.89, 0.96, 0.35))
            widths.append(1.0)
            styles.append((0, (6, 6)) if "DASHED" in mark else "solid")
    ax.add_collection(
        LineCollection(segs, colors=cols, linewidths=widths, linestyles=styles, zorder=2)
    )


def pillar_polygons(cam, pillars, res):
    """Visible faces of the pillars: (camera-frame polygons, colours)."""
    out, cols = [], []
    if not len(pillars):
        return out, cols
    keep = np.hypot(pillars[:, 0], pillars[:, 1]) < VIEW_RANGE
    h = res / 2
    for x, y, z0, z1, observed in pillars[keep]:
        z1 = max(z1, z0 + 0.15)
        bottom = np.array(
            [[x - h, y - h, z0], [x + h, y - h, z0], [x + h, y + h, z0], [x - h, y + h, z0]]
        )
        top = bottom.copy()
        top[:, 2] = z1
        shade = 1.0 if observed else 0.8
        faces = [(top, np.array([0, 0, 1.0]), 1.25)]
        for k, n in enumerate(([0, -1, 0], [1, 0, 0], [0, 1, 0], [-1, 0, 0])):
            a, b = k, (k + 1) % 4
            faces.append(
                (np.array([bottom[a], bottom[b], top[b], top[a]]), np.array(n, float), 0.6)
            )
        centre = np.array([x, y, (z0 + z1) / 2])
        for face, n, light in faces:
            if np.dot(n, centre - cam.eye) >= 0:  # facing away
                continue
            c = cam.clip_polygon(cam.to_cam(face))
            if len(c) >= 3:
                out.append(c)
                cols.append((*np.clip(PILLAR_RGB * light * shade, 0, 1), 0.88))
    return out, cols


def mesh_polygons(cam, b, colour, alpha=1.0):
    tris = MESHES[b["label"]] * np.asarray(b["size"])
    c, s = np.cos(b["yaw"]), np.sin(b["yaw"])
    r = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    tris = tris @ r.T + np.asarray(b["center"])
    normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-9)
    front = np.einsum("ij,ij->i", normals, tris.mean(1) - cam.eye) < 0
    out, cols = [], []
    for t, n in zip(tris[front], normals[front]):
        cc = cam.clip_polygon(cam.to_cam(t))
        if len(cc) >= 3:
            shade = 0.38 + 0.62 * max(0.0, float(n @ LIGHT))
            out.append(cc)
            cols.append((*np.clip(colour * shade, 0, 1), alpha))
    return out, cols


def box_edges(b):
    (cx, cy, cz), (length, width, height), yaw = b["center"], b["size"], b["yaw"]
    c, s = np.cos(yaw), np.sin(yaw)
    xs = np.array([1, -1, -1, 1]) * length / 2
    ys = np.array([1, 1, -1, -1]) * width / 2
    x, y = cx + c * xs - s * ys, cy + s * xs + c * ys
    cr = np.vstack(
        [np.c_[x, y, np.full(4, cz - height / 2)], np.c_[x, y, np.full(4, cz + height / 2)]]
    )
    idx = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4)]
    idx += [(0, 4), (1, 5), (2, 6), (3, 7)]
    return cr, idx


# ----------------------------------------------------------------------------------------
# Frame
# ----------------------------------------------------------------------------------------
def render(job):
    sc, k, i, path, opt = job
    W, H = opt["size"]
    dpi = 100
    fig = plt.figure(figsize=(W / dpi, H / dpi), dpi=dpi, facecolor=BG)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(H, 0)
    ax.axis("off")
    ax.set_facecolor(BG)
    cam = Camera([-17.0, 0.0, 12.0], [16.0, 0.0, -3.0], 0.6 * W, W * 0.40, H * 0.40)

    pose = sc.pose(i)
    inv = np.linalg.inv(pose)
    res = sc.results[sc.frames[i]["idx"]]
    boxes = [b for b in res["boxes"] if np.hypot(*b["center"][:2]) < VIEW_RANGE]
    ground_z = -sc.sensor_height

    draw_map(ax, cam, sc, inv)

    # points: ground teal (optional), object points in the object colour; the rest is shown as
    # pillars
    g, o = sc.clouds(i)
    if not opt["ground_points"]:
        g = g[:0]
    owner = points_in_boxes(o, boxes)
    pts = np.vstack([g, o[owner >= 0]])
    col = np.vstack(
        [
            np.tile(GROUND_RGB, (len(g), 1)),
            np.array([object_rgb(boxes[j]) for j in owner[owner >= 0]]).reshape(-1, 3),
        ]
    )
    c = cam.to_cam(pts)
    vis = (c[:, 2] > 6.0) & (np.hypot(pts[:, 0], pts[:, 1]) < VIEW_RANGE)
    uv = cam.uv(c[vis])
    size = np.clip(50.0 / c[vis, 2], 0.2, 2.5)
    size[int(vis[: len(g)].sum()) :] *= 1.8  # object points
    ax.scatter(uv[:, 0], uv[:, 1], s=size, c=col[vis], linewidths=0, zorder=3)

    # predicted paths: fading ribbons on the ground
    for b in boxes:
        if "path" not in b:
            continue
        p = np.r_[[b["center"][:2]], np.asarray(b["path"])]
        z = b["center"][2] - b["size"][2] / 2 + 0.05
        cc = cam.to_cam(np.c_[p, np.full(len(p), z)])
        colour = object_rgb(b)
        for j in range(len(cc) - 1):
            seg = cam.clip_polyline(cc[j : j + 2])
            if not seg:
                continue
            a = 1.0 - j / len(cc)
            s = cam.uv(seg[0])
            ax.plot(
                s[:, 0],
                s[:, 1],
                color=colour,
                alpha=0.85 * a,
                lw=4.0 * a + 1,
                zorder=4,
                solid_capstyle="round",
            )
        tip = cam.clip_polyline(cc[-2:])
        if tip:
            s = cam.uv(tip[0])
            ax.scatter(s[-1:, 0], s[-1:, 1], s=14, color=colour, alpha=0.5, zorder=4)

    # depth-sorted solids: pillars, object models, the ego car
    polys, cols = pillar_polygons(cam, sc.pillars(i), opt["pillar_res"])
    for b in boxes:
        if b.get("label", "unknown") in MESHES:
            p, cc = mesh_polygons(cam, b, object_rgb(b))
            polys += p
            cols += cc
    ego = {
        "label": "car",
        "center": [0.2, 0.0, ground_z + 0.78],
        "size": [4.8, 1.95, 1.56],
        "yaw": 0.0,
    }
    p, cc = mesh_polygons(cam, ego, np.array([0.92, 0.94, 0.98]))
    polys += p
    cols += cc
    if polys:
        depth = np.array([q[:, 2].mean() for q in polys])
        order = np.argsort(-depth)
        ax.add_collection(
            PolyCollection(
                [cam.uv(polys[j]) for j in order],
                facecolors=[cols[j] for j in order],
                edgecolors=[cols[j] for j in order],
                linewidths=0.3,
                zorder=5,
            )
        )

    # unrecognised objects (clustering mode): wireframe boxes
    for b in boxes:
        if b.get("label", "unknown") in MESHES:
            continue
        cr, idx = box_edges(b)
        cc = cam.to_cam(cr)
        if cc[:, 2].min() < NEAR:
            continue
        uvb = cam.uv(cc)
        colour = object_rgb(b)
        ax.add_collection(
            LineCollection([uvb[[a, e]] for a, e in idx], colors=[colour], lw=1.2, zorder=6)
        )

    # labels, nearest first, skipping those that would overlap
    stroke = [patheffects.withStroke(linewidth=2.6, foreground=BG)]
    placed = []
    for b in sorted(boxes, key=lambda b: np.hypot(*b["center"][:2])):
        d = np.hypot(*b["center"][:2])
        speed = np.hypot(*b["velocity"]) if "velocity" in b else 0.0
        if d > 30 or (speed < 1.5 and d > 16):
            continue
        top = np.array(b["center"]) + [0, 0, b["size"][2] / 2 + 0.4]
        tc = cam.to_cam(top[None])
        if tc[0, 2] < 6.0:
            continue
        t = cam.uv(tc)[0]
        if any(abs(t[0] - q[0]) < 80 and abs(t[1] - q[1]) < 13 for q in placed):
            continue
        placed.append(t)
        name = b.get("label", "unknown")
        text = f"{name if name != 'unknown' else 'object'} {b['id']}"
        if speed > 1.5:
            text += f" · {speed * 3.6:.0f} km/h"
        ax.text(
            t[0],
            t[1],
            text,
            color="white",
            fontsize=7.2,
            ha="center",
            va="bottom",
            fontweight="bold",
            path_effects=stroke,
            zorder=8,
        )

    draw_inset(fig, sc, i, boxes)
    draw_hud(ax, fig, sc, k, i, boxes, opt)
    fig.savefig(path, dpi=dpi, facecolor=BG)
    plt.close(fig)


def draw_inset(fig, sc, i, boxes):
    camr = sc.camera.get(sc.frames[i]["idx"])
    if camr is None:
        return
    img = mpimg.imread(camr["image"])
    fx, fy, cx, cy = camr["K"]
    w, h = camr["size"]
    top = int(max(0, cy - 0.29 * w))  # landscape band around the horizon
    bottom = int(min(h, top + 0.58 * w))
    img = img[top:bottom:2, ::2]
    x0, w_ax = 0.645, 0.34
    h_ax = w_ax * (img.shape[0] / img.shape[1]) * fig.get_figwidth() / fig.get_figheight()
    ax = fig.add_axes([x0, 0.975 - h_ax, w_ax, h_ax])
    ax.imshow(img, extent=(0, w, bottom, top), interpolation="bilinear")
    ax.set_xlim(0, w)
    ax.set_ylim(bottom, top)
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_color("#3a4458")
        s.set_linewidth(1.2)
    cam_T_world = np.linalg.inv(np.asarray(camr["world_T_camera"]).reshape(4, 4))
    cam_T_bench = cam_T_world @ sc.pose(i)
    for b in boxes:
        cr, idx = box_edges(b)
        pc = cr @ cam_T_bench[:3, :3].T + cam_T_bench[:3, 3]
        if pc[:, 2].min() < 1.0:
            continue
        u = fx * pc[:, 0] / pc[:, 2] + cx
        v = fy * pc[:, 1] / pc[:, 2] + cy
        if u.max() < 0 or u.min() > w or v.max() < top or v.min() > bottom:
            continue
        segs = [np.c_[u[[a, e]], v[[a, e]]] for a, e in idx]
        ax.add_collection(LineCollection(segs, colors=[object_rgb(b)], lw=1.1))
    ax.text(
        w * 0.015,
        top + (bottom - top) * 0.06,
        "front camera (for reference only)",
        color="white",
        fontsize=7,
        va="top",
        alpha=0.85,
        path_effects=[patheffects.withStroke(linewidth=2, foreground="black")],
    )


def draw_hud(ax, fig, sc, k, i, boxes, opt):
    W, H = opt["size"]
    ax.text(26, 40, "lidar_obstacle_detector", color=TEXT, fontsize=20, fontweight="bold", zorder=9)
    ax.text(26, 66, opt["subtitle"], color=MUTED, fontsize=10.5, zorder=9)
    n_moving = sum(1 for b in boxes if "velocity" in b and np.hypot(*b["velocity"]) > 1.5)
    stats = f"{sc.ms(i):4.1f} ms / scan   ·   {len(boxes)} tracked   ·   {n_moving} moving"
    ax.text(26, 92, stats, color=ACCENT, fontsize=10.5, family="monospace", zorder=9)

    # scenario caption, bottom left, on a dark panel
    y = H - 30 - 24 * len(sc.features)
    panel = dict(
        boxstyle="round,pad=0.6,rounding_size=0.4", facecolor=BG, edgecolor="#2a3245", alpha=0.82
    )
    ax.add_patch(
        matplotlib.patches.FancyBboxPatch((14, y - 36), 470, H - y + 18, zorder=8.5, **panel)
    )
    ax.text(
        26,
        y - 10,
        sc.title,
        color=TEXT,
        fontsize=15,
        fontweight="bold",
        zorder=9,
        path_effects=[patheffects.withStroke(linewidth=4, foreground=BG)],
    )
    for n, feat in enumerate(sc.features):
        ax.text(
            30,
            y + 20 + 24 * n,
            "▸ " + feat,
            color=TEXT,
            fontsize=11,
            zorder=9,
            path_effects=[patheffects.withStroke(linewidth=4, foreground=BG)],
        )

    # legend, bottom right
    items = [
        ("car", CLASS_RGB["car"]),
        ("truck / bus", CLASS_RGB["truck"]),
        ("pedestrian", CLASS_RGB["pedestrian"]),
        ("cyclist", CLASS_RGB["bicycle"]),
        ("unknown → 2.5D pillars", PILLAR_RGB),
        ("predicted path (3 s)", "#ffffff"),
    ]
    ax.add_patch(matplotlib.patches.FancyBboxPatch((W - 690, H - 64), 676, 54, zorder=8.5, **panel))
    x = W - 26
    for label, colour in reversed(items):
        t = ax.text(x, H - 24, label, color=MUTED, fontsize=9, ha="right", va="center", zorder=9)
        fig.canvas.draw_idle()
        ext = t.get_window_extent(renderer=fig.canvas.get_renderer())
        x = x - ext.width - 12
        ax.scatter([x + 2], [H - 24], s=40, color=colour, marker="s", zorder=9)
        x -= 18
    ax.text(W - 26, H - 48, opt["credit"], color="#5d6679", fontsize=8, ha="right", zorder=9)

    # scenario title card fading out at the start of each scenario
    if k < 12:
        a = 1.0 if k < 6 else (12 - k) / 6
        ax.text(
            W * 0.40,
            H * 0.5,
            sc.title,
            color=TEXT,
            fontsize=30,
            fontweight="bold",
            ha="center",
            va="center",
            alpha=a,
            zorder=10,
            path_effects=[patheffects.withStroke(linewidth=8, foreground=BG, alpha=a)],
        )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--scenario",
        nargs=6,
        action="append",
        required=True,
        metavar=("SEQ", "RUN", "START", "FRAMES", "TITLE", "FEATURES"),
    )
    ap.add_argument("--out", required=True, nargs="+", help=".gif and / or .mp4")
    ap.add_argument("--width", type=int, default=960, help="GIF width (MP4: full size)")
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--gif-fps", type=int, default=8, help="GIF frame rate (drops frames)")
    ap.add_argument("--gif-colors", type=int, default=128)
    ap.add_argument("--subtitle", default="")
    ap.add_argument("--credit", default="")
    ap.add_argument("--pillar-res", type=float, default=0.2)
    ap.add_argument("--no-ground-points", action="store_true", help="smaller GIFs")
    ap.add_argument("--jobs", type=int, default=8)
    ap.add_argument("--keep-frames")
    args = ap.parse_args()

    opt = {
        "size": (1600, 900),
        "subtitle": args.subtitle,
        "credit": args.credit,
        "pillar_res": args.pillar_res,
        "ground_points": not args.no_ground_points,
    }
    jobs, n = [], 0
    frame_dir = args.keep_frames or tempfile.mkdtemp(prefix="lod_showcase_")
    os.makedirs(frame_dir, exist_ok=True)
    for seq, run, start, frames, title, feats in args.scenario:
        sc = Scenario(
            os.path.expanduser(seq),
            os.path.expanduser(run),
            int(start),
            int(frames),
            title,
            [f for f in feats.split("|") if f],
        )
        for k, i in enumerate(sc.idxs):
            jobs.append((sc, k, i, os.path.join(frame_dir, f"{n:05d}.png"), opt))
            n += 1
    with Pool(args.jobs) as pool:
        for m, _ in enumerate(pool.imap(render, jobs)):
            if m % 20 == 0:
                print(f"rendered {m + 1}/{len(jobs)}", flush=True)
    for out in args.out:
        if out.endswith(".gif"):
            encode(
                frame_dir,
                out,
                args.fps,
                args.width,
                args.gif_fps,
                args.gif_colors,
                "bayer:bayer_scale=4",
            )
        else:
            encode(frame_dir, out, args.fps, opt["size"][0])
        print("wrote", out, f"({os.path.getsize(out) / 1e6:.1f} MB)")
    if not args.keep_frames:
        shutil.rmtree(frame_dir)


if __name__ == "__main__":
    main()
