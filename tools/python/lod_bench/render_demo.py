"""Render demo videos / GIFs from lod_benchmark results.

    # 3D chase-camera view of one run
    python -I render_demo.py chase --seq SEQ_DIR --run RUN_DIR --out demo.gif
    # bird's-eye side-by-side comparison of two runs on the same sequence
    python -I render_demo.py compare --seq SEQ_DIR --run RUN_DIR --run-b RUN_B_DIR --out cmp.gif

RUN_DIR is a `lod_benchmark --out` directory (results.jsonl). Frames are rendered with
matplotlib in parallel, then encoded with ffmpeg (GIF with an optimised palette, or MP4).
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from multiprocessing import Pool

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib import colors as mcolors  # noqa: E402
from matplotlib import patheffects  # noqa: E402
from scipy.spatial import cKDTree  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import load_points, read_sequence  # noqa: E402

BG = "#0b0f17"
TEXT = "#e8edf5"
MUTED = "#8a94a6"
TRAIL_LEN = 10  # frames
GROUND_RGB = np.array([0.05, 0.32, 0.36])  # dim teal "road"
OTHER_RGB = np.array([0.42, 0.45, 0.52])  # non-ground, not an object (walls, poles, ...)
STRUCT_RGB = np.array([0.62, 0.64, 0.70])  # structures (reported apart from objects)


def track_color(track_id):
    """Stable, saturated colour per track id (golden-ratio hue walk)."""
    hue = (track_id * 0.618033988749895) % 1.0
    return mcolors.hsv_to_rgb((hue, 0.72, 1.0))


def box_corners(b):
    """8 x 3 corners of an upright box: bottom face then top face."""
    (cx, cy, cz), (length, width, height), yaw = b["center"], b["size"], b["yaw"]
    c, s = np.cos(yaw), np.sin(yaw)
    xs = np.array([1, -1, -1, 1]) * length / 2
    ys = np.array([1, 1, -1, -1]) * width / 2
    x = cx + c * xs - s * ys
    y = cy + s * xs + c * ys
    bottom = np.stack([x, y, np.full(4, cz - height / 2)], 1)
    top = np.stack([x, y, np.full(4, cz + height / 2)], 1)
    return np.vstack([bottom, top])


EDGES = [
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 0),
    (4, 5),
    (5, 6),
    (6, 7),
    (7, 4),
    (0, 4),
    (1, 5),
    (2, 6),
    (3, 7),
]


def points_in_boxes(xyz, boxes):
    """Index of the box containing each point (-1 if none)."""
    label = np.full(len(xyz), -1)
    for k, b in enumerate(boxes):
        (cx, cy, cz), (length, width, height), yaw = b["center"], b["size"], b["yaw"]
        r = max(length, width) / 2 + 0.2
        near = (np.abs(xyz[:, 0] - cx) < r) & (np.abs(xyz[:, 1] - cy) < r)
        idx = np.nonzero(near)[0]
        if not len(idx):
            continue
        d = xyz[idx, :2] - (cx, cy)
        c, s = np.cos(-yaw), np.sin(-yaw)
        u = c * d[:, 0] - s * d[:, 1]
        v = s * d[:, 0] + c * d[:, 1]
        z = xyz[idx, 2]
        inside = (
            (np.abs(u) <= length / 2 + 0.1)
            & (np.abs(v) <= width / 2 + 0.1)
            & (z >= cz - height / 2 + 0.05)
            & (z <= cz + height / 2 + 0.1)
        )
        label[idx[inside]] = k
    return label


class Sequence:
    def __init__(self, seq_dir, run_dirs, roi):
        self.seq_dir = seq_dir
        _, self.frames, _ = read_sequence(seq_dir)
        self.poses = [np.asarray(f["pose"]).reshape(4, 4) for f in self.frames]
        self.runs = []
        self.run_dirs = run_dirs
        for run in run_dirs:
            with open(os.path.join(run, "results.jsonl")) as f:
                self.runs.append({r["idx"]: r for r in map(json.loads, f)})
        self.roi = roi

    def run_dir(self, run):
        return self.run_dirs[run]

    def ground_mask(self, run, idx, pts):
        """True for raw points whose nearest processed point was classified as ground."""
        clouds = os.path.join(self.run_dirs[run], "clouds", f"{self.frames[idx]['idx']:06d}")
        if not os.path.exists(clouds + "_ground.bin"):
            return None
        g = np.fromfile(clouds + "_ground.bin", np.float32).reshape(-1, 3)
        o = np.fromfile(clouds + "_obstacle.bin", np.float32).reshape(-1, 3)
        proc = np.vstack([g, o])
        if not len(proc):
            return None
        is_g = np.r_[np.ones(len(g), bool), np.zeros(len(o), bool)]
        d, nn = cKDTree(proc).query(pts, distance_upper_bound=0.5)
        out = np.zeros(len(pts), bool)
        ok = np.isfinite(d)
        out[ok] = is_g[nn[ok]]
        return out

    def structures(self, run, idx):
        return self.runs[run][idx].get("structures", [])

    def occupancy(self, run, idx):
        r = self.runs[run][idx]
        meta = r.get("occupancy")
        if not meta:
            return None
        path = os.path.join(
            self.run_dirs[run], "clouds", f"{self.frames[idx]['idx']:06d}_occupancy.bin"
        )
        data = np.fromfile(path, np.int8).reshape(meta["height"], meta["width"])
        return data, meta

    def boxes(self, run, idx):
        r = self.runs[run][idx]
        return [
            b
            for b in r["boxes"]
            if abs(b["center"][0]) < self.roi and abs(b["center"][1]) < self.roi
        ]

    def trails(self, run, idx):
        """id -> (N, 2) past centres (world frame -> current sensor frame)."""
        hist = {}
        for k in range(max(0, idx - TRAIL_LEN), idx + 1):
            pose = self.poses[k]
            for b in self.boxes(run, k):
                w = pose[:3, :3] @ np.asarray(b["center"]) + pose[:3, 3]
                hist.setdefault(b["id"], []).append(w)
        inv = np.linalg.inv(self.poses[idx])
        out = {}
        for tid, pts in hist.items():
            p = np.asarray(pts) @ inv[:3, :3].T + inv[:3, 3]
            out[tid] = p
        return out

    def ids_issued(self, run, idx):
        ids = set()
        for k in range(0, idx + 1):
            ids.update(b["id"] for b in self.runs[run][k]["boxes"])
        return len(ids)

    def ms(self, run, idx):
        lo = max(0, idx - 9)
        return np.mean([self.runs[run][k]["timing_ms"]["total"] for k in range(lo, idx + 1)])


def colour_points(seq, run, idx, pts, boxes):
    """Per-point RGB: object points in their track colour, structures light grey,
    ground teal, other non-ground grey. Returns (rgb, owner) with owner = box index."""
    owner = points_in_boxes(pts, boxes)
    struct = points_in_boxes(pts, seq.structures(run, idx)) >= 0
    ground = seq.ground_mask(run, idx, pts)
    h = np.clip((pts[:, 2] + 1.8) / 3.0, 0, 1)
    if ground is None:  # no dumps (e.g. v1): shade by height
        rgb = plt.cm.cool(0.15 + 0.5 * h)[:, :3] * (0.35 + 0.45 * h)[:, None]
    else:
        rgb = np.where(ground[:, None], GROUND_RGB, OTHER_RGB * (0.6 + 0.4 * h)[:, None])
        rgb[struct & ~ground] = STRUCT_RGB
    for k, b in enumerate(boxes):
        rgb[owner == k] = track_color(b["id"])
    return rgb, owner


# ----------------------------------------------------------------------------------------
# 3D chase camera
# ----------------------------------------------------------------------------------------
def look_at(eye, target):
    fwd = target - eye
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, [0, 0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    return np.stack([right, -up, fwd])  # rows: camera x (right), y (down), z (forward)


def project(xyz, eye, rot, f, cx, cy):
    cam = (xyz - eye) @ rot.T
    ok = cam[:, 2] > 0.5
    uv = np.full((len(xyz), 2), np.nan)
    uv[ok, 0] = f * cam[ok, 0] / cam[ok, 2] + cx
    uv[ok, 1] = f * cam[ok, 1] / cam[ok, 2] + cy
    return uv, cam[:, 2]


def render_chase(job):
    seq, idx, path, opt = job
    W, H = opt["size"]
    dpi = 100
    fig = plt.figure(figsize=(W / dpi, H / dpi), dpi=dpi, facecolor=BG)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(H, 0)
    ax.axis("off")
    ax.set_facecolor(BG)

    eye = np.array([-17.0, 0.0, 9.5])
    rot = look_at(eye, np.array([16.0, 0.0, -2.5]))
    f, cx, cy = 0.62 * W, W / 2, H * 0.40

    pts = load_points(seq.seq_dir, seq.frames[idx]["idx"])[:, :3]
    keep = (np.abs(pts[:, 0]) < seq.roi) & (np.abs(pts[:, 1]) < seq.roi) & (pts[:, 2] < 1.5)
    pts = pts[keep]
    boxes = seq.boxes(0, idx)
    col, owner = colour_points(seq, 0, idx, pts, boxes)

    uv, depth = project(pts, eye, rot, f, cx, cy)
    vis = (
        np.isfinite(uv[:, 0])
        & (depth > 7.0)  # points right in front of the camera would blow up
        & (uv[:, 0] > -5)
        & (uv[:, 0] < W + 5)
        & (uv[:, 1] > -5)
        & (uv[:, 1] < H + 5)
    )
    size = np.clip(55.0 / np.maximum(depth, 1.0), 0.15, 3.0)
    size[owner >= 0] *= 2.2
    order = np.argsort(-depth[vis])
    ax.scatter(
        uv[vis, 0][order], uv[vis, 1][order], s=size[vis][order], c=col[vis][order], linewidths=0
    )

    # trails on the ground
    ground_z = -opt["sensor_height"] + 0.05
    for tid, tr in seq.trails(0, idx).items():
        if len(tr) < 3:
            continue
        p = np.c_[tr[:, :2], np.full(len(tr), ground_z)]
        t_uv, t_depth = project(p, eye, rot, f, cx, cy)
        c = track_color(tid)
        for i in range(len(t_uv) - 1):
            seg = t_uv[i : i + 2]
            # skip segments close to / behind the camera, and jumps (re-used ids)
            if min(t_depth[i : i + 2]) < 6 or not np.all(np.isfinite(seg)):
                continue
            if np.linalg.norm(tr[i + 1, :2] - tr[i, :2]) > 4.0:
                continue
            a = (i + 1) / len(t_uv)
            ax.plot(seg[:, 0], seg[:, 1], color=c, alpha=0.65 * a, lw=2.2 * a + 0.4)

    # ego vehicle
    ego = {"center": [0.1, 0.0, -opt["sensor_height"] + 0.75], "size": [4.9, 1.95, 1.5], "yaw": 0.0}
    e_uv, _ = project(box_corners(ego), eye, rot, f, cx, cy)
    for face in ([0, 1, 2, 3], [4, 5, 6, 7], [1, 2, 6, 5], [0, 3, 7, 4], [2, 3, 7, 6]):
        ax.fill(e_uv[face, 0], e_uv[face, 1], color="#c9d2e3", alpha=0.22, lw=0)
    for i, j in EDGES:
        ax.plot(e_uv[[i, j], 0], e_uv[[i, j], 1], color="white", lw=1.1, alpha=0.8)

    # boxes, ids, speeds
    for b in sorted(boxes, key=lambda b: -np.hypot(*b["center"][:2])):
        cuv, cdepth = project(box_corners(b), eye, rot, f, cx, cy)
        if not np.all(np.isfinite(cuv)) or cdepth.min() < 7.0:
            continue
        c = track_color(b["id"])
        for i, j in EDGES:
            ax.plot(
                cuv[[i, j], 0], cuv[[i, j], 1], color=c, lw=4.5, alpha=0.18, solid_capstyle="round"
            )
            ax.plot(cuv[[i, j], 0], cuv[[i, j], 1], color=c, lw=1.3, alpha=0.95)
        dist = np.hypot(*b["center"][:2])
        speed = np.hypot(*b["velocity"]) if "velocity" in b else 0.0
        if speed > 2.5:  # direction of travel, drawn on the ground in front of the box
            yaw = b["yaw"]
            fwd = np.array([np.cos(yaw), np.sin(yaw)])
            if np.dot(fwd, np.asarray(b["velocity"][:2])) < 0:
                fwd = -fwd
            gz = -opt["sensor_height"] + 0.1
            base = np.asarray(b["center"][:2]) + fwd * b["size"][0] * 0.5
            tip = base + fwd * min(2.5, 0.35 * speed)
            a_uv, a_depth = project(np.array([[*base, gz], [*tip, gz]]), eye, rot, f, cx, cy)
            if np.all(np.isfinite(a_uv)) and a_depth.min() > 7.0:
                ax.annotate(
                    "",
                    xy=a_uv[1],
                    xytext=a_uv[0],
                    arrowprops=dict(arrowstyle="-|>", color=c, lw=1.6, mutation_scale=9),
                )
        if (speed > 2.5 and dist < 32) or (dist < 14 and max(b["size"][:2]) > 0.4):
            top = cuv[4:].mean(0)
            label = f"{b['id']}" + (f" · {speed * 3.6:.0f} km/h" if speed > 1.5 else "")
            ax.text(
                top[0],
                top[1] - 8,
                label,
                color="white",
                fontsize=7.5,
                ha="center",
                va="bottom",
                fontweight="bold",
                path_effects=[patheffects.withStroke(linewidth=2.5, foreground=BG)],
            )

    # HUD
    ax.text(22, 34, "lidar_obstacle_detector", color=TEXT, fontsize=19, fontweight="bold")
    ax.text(22, 60, opt["subtitle"], color=MUTED, fontsize=10.5)
    n_moving = sum(
        1 for b in boxes if b.get("velocity") is not None and np.hypot(*b["velocity"]) > 2.5
    )
    hud = f"{seq.ms(0, idx):4.1f} ms / scan    {len(boxes)} obstacles    {n_moving} moving"
    ax.text(W - 22, 34, hud, color=TEXT, fontsize=11, ha="right", family="monospace")
    ax.text(W - 22, H - 16, opt["credit"], color=MUTED, fontsize=8, ha="right")
    fig.savefig(path, dpi=dpi, facecolor=BG)
    plt.close(fig)


# ----------------------------------------------------------------------------------------
# Bird's-eye comparison
# ----------------------------------------------------------------------------------------
def draw_bev(ax, seq, run, idx, title, opt):
    ax.set_facecolor(BG)
    R = opt["bev_range"]
    ax.set_xlim(-R * 0.62, R * 0.62)
    ax.set_ylim(-R * 0.45, R)
    ax.set_aspect("equal")
    ax.axis("off")
    pts = load_points(seq.seq_dir, seq.frames[idx]["idx"])[:, :3]
    keep = (np.abs(pts[:, 0]) < R + 5) & (np.abs(pts[:, 1]) < R + 5) & (pts[:, 2] < 1.5)
    pts = pts[keep]
    boxes = seq.boxes(run, idx)
    col, owner = colour_points(seq, run, idx, pts, boxes)
    occ = seq.occupancy(run, idx)
    if occ is not None:  # free = faint teal, occupied = faint amber, unknown = clear
        data, meta = occ
        rgba = np.zeros(data.shape + (4,))
        rgba[data == 0] = (0.1, 0.55, 0.6, 0.16)
        rgba[data == 100] = (1.0, 0.62, 0.2, 0.45)
        img = np.transpose(rgba, (1, 0, 2))[:, ::-1]  # rows = x (up), cols = -y (right)
        x0, y0 = meta["origin"]
        res = meta["resolution"]
        ax.imshow(
            img,
            origin="lower",
            interpolation="nearest",
            zorder=0,
            extent=(-(y0 + meta["height"] * res), -y0, x0, x0 + meta["width"] * res),
        )
    # x forward is up: plot (-y, x)
    ax.scatter(-pts[:, 1], pts[:, 0], s=np.where(owner >= 0, 1.4, 0.35), c=col, linewidths=0)
    for tid, tr in seq.trails(run, idx).items():
        if len(tr) < 2:
            continue
        c = track_color(tid)
        for i in range(len(tr) - 1):
            if np.linalg.norm(tr[i + 1, :2] - tr[i, :2]) > 4.0:
                continue
            a = (i + 1) / len(tr)
            ax.plot(-tr[i : i + 2, 1], tr[i : i + 2, 0], color=c, alpha=0.7 * a, lw=1.8 * a + 0.3)
    for b in boxes:
        cr = box_corners(b)[:4]
        poly = np.vstack([cr, cr[:1]])
        c = track_color(b["id"])
        ax.fill(-poly[:, 1], poly[:, 0], color=c, alpha=0.18, lw=0)
        ax.plot(-poly[:, 1], poly[:, 0], color=c, lw=1.2)
    ego = box_corners({"center": [0.1, 0, 0], "size": [4.9, 1.95, 1], "yaw": 0.0})[:4]
    ego = np.vstack([ego, ego[:1]])
    ax.fill(-ego[:, 1], ego[:, 0], color="white", alpha=0.9)
    ax.set_xlim(-R * 0.62, R * 0.62)  # imshow may have changed the limits
    ax.set_ylim(-R * 0.45, R)
    # header band above the panel
    x0 = ax.get_position().x0 + 0.02
    fig = ax.figure
    fig.text(x0, 0.905, title, color=TEXT, fontsize=15, fontweight="bold", va="center")
    fig.text(
        x0 + 0.17, 0.905, f"{seq.ms(run, idx):.1f} ms / scan", color=MUTED, fontsize=11, va="center"
    )
    fig.text(
        x0 + 0.29,
        0.905,
        f"track IDs issued: {seq.ids_issued(run, idx)}",
        color=MUTED,
        fontsize=11,
        va="center",
    )


def render_compare(job):
    seq, idx, path, opt = job
    W, H = opt["size"]
    dpi = 100
    fig = plt.figure(figsize=(W / dpi, H / dpi), dpi=dpi, facecolor=BG)
    a1 = fig.add_axes([0.0, 0.0, 0.5, 0.87])
    a2 = fig.add_axes([0.5, 0.0, 0.5, 0.87])
    draw_bev(a1, seq, 1, idx, opt["title_b"], opt)
    draw_bev(a2, seq, 0, idx, opt["title_a"], opt)
    fig.add_artist(plt.Line2D([0.5, 0.5], [0.02, 0.93], color="#283041", lw=1.5))
    fig.text(0.5, 0.965, opt["subtitle"], color=TEXT, fontsize=12, ha="center", va="center")
    fig.text(
        0.99,
        0.015,
        opt["credit"],
        color=MUTED,
        fontsize=8,
        ha="right",
        bbox=dict(facecolor=BG, edgecolor="none", alpha=0.85, pad=3),
    )
    fig.savefig(path, dpi=dpi, facecolor=BG)
    plt.close(fig)


def encode(frame_dir, out, fps, width):
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        sys.exit("ffmpeg not found; frames are in " + frame_dir)
    pattern = os.path.join(frame_dir, "%05d.png")
    if out.endswith(".gif"):
        vf = f"fps={fps},scale={width}:-1:flags=lanczos"
        pal = os.path.join(frame_dir, "palette.png")
        subprocess.run(
            [
                ffmpeg,
                "-y",
                "-loglevel",
                "error",
                "-framerate",
                str(fps),
                "-i",
                pattern,
                "-vf",
                vf + ",palettegen=max_colors=192:stats_mode=diff",
                pal,
            ],
            check=True,
        )
        subprocess.run(
            [
                ffmpeg,
                "-y",
                "-loglevel",
                "error",
                "-framerate",
                str(fps),
                "-i",
                pattern,
                "-i",
                pal,
                "-lavfi",
                vf + "[x];[x][1:v]paletteuse=dither=sierra2_4a:diff_mode=rectangle",
                "-loop",
                "0",
                out,
            ],
            check=True,
        )
    else:
        subprocess.run(
            [
                ffmpeg,
                "-y",
                "-loglevel",
                "error",
                "-framerate",
                str(fps),
                "-i",
                pattern,
                "-vf",
                f"scale={width}:-2",
                "-c:v",
                "libx264",
                "-pix_fmt",
                "yuv420p",
                "-crf",
                "20",
                out,
            ],
            check=True,
        )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["chase", "compare"])
    ap.add_argument("--seq", required=True)
    ap.add_argument("--run", required=True, help="lod_benchmark output dir (shown on the right)")
    ap.add_argument("--run-b", help="second run for `compare` (shown on the left)")
    ap.add_argument("--out", required=True, help=".gif or .mp4")
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--frames", type=int, default=0, help="0 = to the end")
    ap.add_argument("--step", type=int, default=1, help="use every n-th scan")
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--width", type=int, default=960, help="output width in pixels")
    ap.add_argument("--subtitle", default="")
    ap.add_argument("--title-a", default="v2")
    ap.add_argument("--title-b", default="v1")
    ap.add_argument("--credit", default="")
    ap.add_argument("--sensor-height", type=float, default=1.7)
    ap.add_argument("--jobs", type=int, default=8)
    ap.add_argument("--keep-frames", help="also keep the PNG frames in this directory")
    args = ap.parse_args()

    runs = [args.run] + ([args.run_b] if args.run_b else [])
    if args.mode == "compare" and not args.run_b:
        sys.exit("compare needs --run-b")
    seq = Sequence(os.path.expanduser(args.seq), runs, roi=40.0)
    end = (
        len(seq.frames)
        if not args.frames
        else min(len(seq.frames), args.start + args.frames * args.step)
    )
    idxs = list(range(args.start, end, args.step))
    opt = {
        "size": (1600, 900) if args.mode == "chase" else (1600, 860),
        "subtitle": args.subtitle,
        "title_a": args.title_a,
        "title_b": args.title_b,
        "credit": args.credit,
        "sensor_height": args.sensor_height,
        "bev_range": 38.0,
    }
    frame_dir = args.keep_frames or tempfile.mkdtemp(prefix="lod_demo_")
    os.makedirs(frame_dir, exist_ok=True)
    jobs = [(seq, i, os.path.join(frame_dir, f"{k:05d}.png"), opt) for k, i in enumerate(idxs)]
    fn = render_chase if args.mode == "chase" else render_compare
    with Pool(args.jobs) as pool:
        for k, _ in enumerate(pool.imap(fn, jobs)):
            if k % 20 == 0:
                print(f"rendered {k + 1}/{len(jobs)}", flush=True)
    encode(frame_dir, args.out, args.fps, args.width)
    print("wrote", args.out, f"({os.path.getsize(args.out) / 1e6:.1f} MB)")
    if not args.keep_frames:
        shutil.rmtree(frame_dir)


if __name__ == "__main__":
    main()
