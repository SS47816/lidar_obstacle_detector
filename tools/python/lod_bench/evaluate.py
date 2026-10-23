"""Evaluate lod_benchmark results against benchmark-format ground truth.

The detector is class-agnostic and also (correctly) reports walls, poles, vegetation, ...
that have no labels, so plain precision / MOTA would mostly measure "how much unlabeled
stuff is there". We therefore report:

* detection: recall of obstacle-class GT (vehicle/pedestrian/cyclist, >= min_pts points,
  centre in the eval ROI) at BEV centre distance <= 1 m / 2 m and at BEV IoU >= 0.3, and the
  coverage: GT reported either as a box or by at least one 2.5D pillar in its footprint
  (unrecognised objects are not missed when they are pillars)
* segmentation: over-segmentation (GT split into >= 2 predicted boxes), its rate on large
  vehicles (> 7 m), and under-segmentation (GT merged with another GT into one predicted box)
* boxes: mean BEV IoU and orientation error (mod 90 deg, vehicles) of centre matches, and the
  heading error of moving vehicles (front/back counts), and the box jitter: frame-to-frame shape
  change (1 - BEV IoU with the centres aligned) of a track matched to the same GT
* pillars: flicker, the share of static pillar cells (outside GT objects, within 25 m) that
  appear or disappear between consecutive annotated frames
* tracking: FP-filtered CLEAR/ID metrics (predictions further than 2 m from every GT are
  ignored): MOTA*, IDF1, ID switches, fragmentations, mostly-tracked ratio, and the
  velocity error of matched tracks
* ground: per-point precision/recall/IoU vs. lidarseg labels (where available) and the share
  of object points (inside obstacle GT boxes, >= 0.2 m above the box bottom) labelled ground
* runtime: per-stage mean / p95 [ms]

    python -I evaluate.py --data ~/datasets/lod_bench/nuscenes --results RUN_DIR --out metrics.json
"""

import argparse
import glob
import json
import os
import sys
from collections import defaultdict

import motmetrics as mm
import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import OBSTACLE_CLASSES, load_points, read_sequence  # noqa: E402
from geometry import (  # noqa: E402
    bev_iou,
    heading_error_mod180,
    orientation_error_mod90,
    points_in_bev_box,
    points_in_box,
)

MATCH_DIST = 2.0  # [m] association gate (as in the nuScenes tracking benchmark)
PILLAR_RES = 0.2  # [m] pillar cell size (occupancy_resolution)
FLICKER_RANGE = 25.0  # [m]

# Learned detector labels -> evaluation classes ("unknown" from clustering is not scored)
LABEL_TO_CLS = {
    "car": "vehicle",
    "truck": "vehicle",
    "bus": "vehicle",
    "trailer": "vehicle",
    "construction_vehicle": "vehicle",
    "pedestrian": "pedestrian",
    "motorcycle": "cyclist",
    "bicycle": "cyclist",
}


def load_cfg(path):
    cfg = {}
    with open(path) as f:
        for line in f:
            line = line.split("#")[0].strip()
            if line:
                k, v = line.split("=", 1)
                cfg[k.strip()] = float(v)
    return cfg


def in_roi(c, cfg):
    return (
        cfg["roi_min_x"] <= c[0] <= cfg["roi_max_x"]
        and cfg["roi_min_y"] <= c[1] <= cfg["roi_max_y"]
    )


def to_box(o):
    return (np.asarray(o["center"]), np.asarray(o["size"]), o["yaw"])


def match(gt, pred, cost_fn, max_cost):
    """Hungarian matching; returns list of (gi, pi, cost) with cost <= max_cost."""
    if not gt or not pred:
        return []
    cost = np.full((len(gt), len(pred)), 1e6)
    for i, g in enumerate(gt):
        for j, p in enumerate(pred):
            c = cost_fn(g, p)
            if c is not None and c <= max_cost:
                cost[i, j] = c
    rows, cols = linear_sum_assignment(cost)
    return [(i, j, cost[i, j]) for i, j in zip(rows, cols) if cost[i, j] <= max_cost]


def center_dist(g, p):
    return float(np.hypot(g["center"][0] - p["center"][0], g["center"][1] - p["center"][1]))


def iou_cost(g, p):
    if center_dist(g, p) > 0.5 * (np.hypot(*g["size"][:2]) + np.hypot(*p["size"][:2])):
        return None
    return 1.0 - bev_iou(to_box(g), to_box(p))


def world_center(o, pose):
    c = np.asarray(o["center"])
    return pose[:3, :3] @ c + pose[:3, 3]


def load_pillars(res_dir, idx):
    """N x 5: x, y, z_min, z_max (sensor frame), observed."""
    path = os.path.join(res_dir, "clouds", f"{idx:06d}_pillars.bin")
    if not os.path.exists(path):
        return np.zeros((0, 5))
    p = np.fromfile(path, np.int16).reshape(-1, 5).astype(np.float64)
    p[:, :4] /= 100.0
    return p


def pillar_cells(pillars, pose, origin, objs):
    """Keys (tracking-frame cells) of static pillars within FLICKER_RANGE, outside GT objects."""
    if not len(pillars):
        return set()
    xy = pillars[:, :2]
    keep = np.hypot(xy[:, 0], xy[:, 1]) <= FLICKER_RANGE
    for o in objs:
        keep &= ~points_in_bev_box(xy, o["center"], o["size"], o["yaw"], margin=0.5)
    w = (pose[:3, :3] @ np.c_[xy[keep], np.zeros(keep.sum())].T).T + pose[:3, 3] - origin
    return set(map(tuple, np.floor(w[:, :2] / PILLAR_RES).astype(np.int64)))


def evaluate_sequence(seq_dir, res_dir, cfg, min_pts, stats, acc):
    meta, frames, gt = read_sequence(seq_dir)
    with open(os.path.join(res_dir, "results.jsonl")) as f:
        results = {r["idx"]: r for r in map(json.loads, f)}
    gt_ids = {}  # motmetrics wants numeric ids
    origin = np.asarray(frames[0]["pose"]).reshape(4, 4)[:3, 3] if frames else np.zeros(3)
    prev_cells, prev_pose = None, None
    prev_shape = {}  # GT id -> (track id, size, yaw) in the previous annotated frame

    for fr in frames:
        idx = fr["idx"]
        res = results.get(idx)
        if res is None:
            continue
        for k, v in res["timing_ms"].items():
            stats["timing"][k].append(v)
        if idx not in gt:
            continue
        objs = gt[idx]
        targets = [
            o
            for o in objs
            if o["cls"] in OBSTACLE_CLASSES and o["num_pts"] >= min_pts and in_roi(o["center"], cfg)
        ]
        preds = [p for p in res["boxes"] if in_roi(p["center"], cfg)]
        stats["frames"] += 1
        stats["n_gt"] += len(targets)
        stats["n_pred"] += len(preds)
        for o in targets:
            stats["n_gt_cls"][o["cls"]] += 1

        # --- coverage: a box or 2.5D pillars --------------------------------
        pillars = load_pillars(res_dir, idx)
        pc_all = np.array([p["center"][:2] for p in preds]) if preds else np.zeros((0, 2))
        for g in targets:
            covered = (
                len(pc_all)
                and points_in_bev_box(pc_all, g["center"], g["size"], g["yaw"], margin=0.5).any()
            )
            if not covered and len(pillars):
                covered = points_in_bev_box(
                    pillars[:, :2], g["center"], g["size"], g["yaw"], margin=0.2
                ).any()
            stats["covered"] += int(bool(covered))
        stats["n_pillars"] += len(pillars)
        pose = np.asarray(fr["pose"]).reshape(4, 4)
        if "pillars" in res or prev_cells is not None:
            cells = pillar_cells(pillars, pose, origin, objs)
            if prev_cells is not None and np.linalg.norm(pose[:3, 3] - prev_pose[:3, 3]) < 5.0:
                # only cells within range of both sensor positions
                def near(c, p):
                    ctr = (np.array(c) + 0.5) * PILLAR_RES + origin[:2] - p[:2, 3]
                    return np.hypot(*ctr) <= FLICKER_RANGE - 1.0

                a = {c for c in prev_cells if near(c, pose)}
                b = {c for c in cells if near(c, prev_pose)}
                if a | b:
                    stats["flicker_changed"] += len(a ^ b)
                    stats["flicker_total"] += len(a | b)
            prev_cells, prev_pose = cells, pose

        # --- detection recall & box quality ---------------------------------
        cm = match(targets, preds, center_dist, MATCH_DIST)
        shape = {}
        for gi, pi, _ in cm:
            g, p = targets[gi], preds[pi]
            shape[g["id"]] = (p["id"], np.asarray(p["size"]), p["yaw"])
            last = prev_shape.get(g["id"])
            if last is not None and last[0] == p["id"]:
                iou = bev_iou(
                    (np.zeros(3), last[1], last[2]), (np.zeros(3), shape[g["id"]][1], p["yaw"])
                )
                stats["jitter"].append(1.0 - iou)
        prev_shape = shape
        stats["n_structures"] += len(res.get("structures", []))
        for o in targets:
            if o["size"][0] > 7.0:
                stats["n_gt_large"] += 1
        for gi, pi, d in cm:
            g, p = targets[gi], preds[pi]
            if d <= 1.0 and g["size"][0] > 7.0:
                stats["hit_1m_large"] += 1
            pred_cls = LABEL_TO_CLS.get(p.get("label", "unknown"))
            if pred_cls is not None:
                stats["labelled"] += 1
                stats["label_correct"] += int(pred_cls == g["cls"])
            v = g.get("velocity")
            if g["cls"] == "vehicle" and v is not None and np.hypot(*v) > 2.0:
                stats["heading_err"].append(heading_error_mod180(g["yaw"], p["yaw"]))
            stats["hit_2m"] += 1
            if d <= 1.0:
                stats["hit_1m"] += 1
                stats["hit_1m_cls"][g["cls"]] += 1
            stats["box_iou"].append(bev_iou(to_box(g), to_box(p)))
            if g["cls"] == "vehicle":
                stats["yaw_err"].append(orientation_error_mod90(g["yaw"], p["yaw"]))
        im = match(targets, preds, iou_cost, 0.7)
        stats["hit_iou03"] += len(im)

        # --- over / under segmentation --------------------------------------
        if preds:
            pc = np.array([p["center"][:2] for p in preds])
            for g in targets:
                inside = points_in_bev_box(pc, g["center"], g["size"], g["yaw"], margin=0.3)
                if inside.sum() >= 2:
                    stats["over_seg"] += 1
                    if g["size"][0] > 7.0:
                        stats["over_seg_large"] += 1
            if len(targets) >= 2:
                gc = np.array([g["center"][:2] for g in targets])
                merged = np.zeros(len(targets), bool)
                for p in preds:
                    inside = points_in_bev_box(gc, p["center"], p["size"], p["yaw"])
                    if inside.sum() >= 2:
                        merged |= inside
                stats["under_seg"] += int(merged.sum())

        # --- tracking (FP-filtered) -----------------------------------------
        if targets and preds:
            gc = np.array([g["center"][:2] for g in targets])
            pc = np.array([p["center"][:2] for p in preds])
            dmin = np.min(np.linalg.norm(pc[:, None, :] - gc[None, :, :], axis=2), axis=1)
            tracked = [p for p, d in zip(preds, dmin) if d <= MATCH_DIST]
        else:
            tracked = []
        dist = np.full((len(targets), len(tracked)), np.nan)
        for i, g in enumerate(targets):
            for j, p in enumerate(tracked):
                d = center_dist(g, p)
                if d <= MATCH_DIST:
                    dist[i, j] = d
        acc.update(
            [gt_ids.setdefault(g["id"], len(gt_ids)) for g in targets],
            [int(p["id"]) for p in tracked],
            dist,
        )
        for gi, pi, _ in match(targets, tracked, center_dist, MATCH_DIST):
            g, p = targets[gi], tracked[pi]
            if g.get("velocity") is not None and "velocity" in p:
                stats["vel_err"].append(
                    float(np.hypot(*(np.subtract(p["velocity"], g["velocity"]))))
                )

        # --- ground segmentation --------------------------------------------
        if res.get("dumped"):
            name = f"{idx:06d}"
            ground = np.fromfile(
                os.path.join(res_dir, "clouds", name + "_ground.bin"), np.float32
            ).reshape(-1, 3)
            obst = np.fromfile(
                os.path.join(res_dir, "clouds", name + "_obstacle.bin"), np.float32
            ).reshape(-1, 3)
            raw = load_points(seq_dir, idx)[:, :3]
            roi = (
                (raw[:, 0] >= cfg["roi_min_x"])
                & (raw[:, 0] <= cfg["roi_max_x"])
                & (raw[:, 1] >= cfg["roi_min_y"])
                & (raw[:, 1] <= cfg["roi_max_y"])
                & (raw[:, 2] >= cfg["roi_min_z"])
                & (raw[:, 2] <= cfg["roi_max_z"])
            )
            proc = np.vstack([ground, obst]) if len(ground) + len(obst) else np.zeros((0, 3))
            is_ground_proc = np.r_[np.ones(len(ground), bool), np.zeros(len(obst), bool)]
            pred_ground = np.zeros(len(raw), bool)
            if len(proc):
                dd, nn = cKDTree(proc).query(raw, distance_upper_bound=0.5)
                ok = np.isfinite(dd)
                pred_ground[ok] = is_ground_proc[nn[ok]]
            if fr["ground_labels"]:
                lab = np.fromfile(os.path.join(seq_dir, fr["ground_labels"]), np.uint8)
                valid = roi & (lab != 255)
                gtg = lab == 1
                stats["g_tp"] += int((pred_ground & gtg & valid).sum())
                stats["g_fp"] += int((pred_ground & ~gtg & valid).sum())
                stats["g_fn"] += int((~pred_ground & gtg & valid).sum())
            for g in targets:
                m = points_in_box(raw, g["center"], g["size"], g["yaw"], z_bottom_skip=0.2) & roi
                stats["obj_pts"] += int(m.sum())
                stats["obj_pts_lost"] += int((m & pred_ground).sum())


def summarize(stats, acc_list, names):
    out = {}
    n_gt = max(stats["n_gt"], 1)
    out["frames"] = stats["frames"]
    out["gt_objects"] = stats["n_gt"]
    out["pred_per_frame"] = stats["n_pred"] / max(stats["frames"], 1)
    out["recall@1m"] = stats["hit_1m"] / n_gt
    out["recall@2m"] = stats["hit_2m"] / n_gt
    for c in OBSTACLE_CLASSES:
        out[f"recall@1m_{c}"] = stats["hit_1m_cls"][c] / max(stats["n_gt_cls"][c], 1)
    out["recall@iou0.3"] = stats["hit_iou03"] / n_gt
    out["coverage"] = stats["covered"] / n_gt
    out["pillars_per_frame"] = stats["n_pillars"] / max(stats["frames"], 1)
    if stats["flicker_total"]:
        out["pillar_flicker"] = stats["flicker_changed"] / stats["flicker_total"]
    out["large_split_rate"] = stats["over_seg_large"] / max(stats["n_gt_large"], 1)
    if stats["jitter"]:
        out["box_jitter"] = float(np.mean(stats["jitter"]))
    out["over_seg_rate"] = stats["over_seg"] / n_gt
    out["under_seg_rate"] = stats["under_seg"] / n_gt
    out["mean_bev_iou"] = float(np.mean(stats["box_iou"])) if stats["box_iou"] else 0.0
    out["vehicle_yaw_err_deg"] = float(np.median(stats["yaw_err"])) if stats["yaw_err"] else None
    out["vehicle_yaw_err_deg_mean"] = float(np.mean(stats["yaw_err"])) if stats["yaw_err"] else None
    out["vel_err_mps"] = float(np.median(stats["vel_err"])) if stats["vel_err"] else None
    out["moving_heading_err_deg"] = (
        float(np.mean(stats["heading_err"])) if stats["heading_err"] else None
    )
    out["recall@1m_large"] = stats["hit_1m_large"] / max(stats["n_gt_large"], 1)
    out["structures_per_frame"] = stats["n_structures"] / max(stats["frames"], 1)
    if stats["labelled"]:
        out["label_accuracy"] = stats["label_correct"] / stats["labelled"]

    mh = mm.metrics.create()
    summary = mh.compute_many(
        acc_list,
        names=names,
        metrics=[
            "mota",
            "idf1",
            "num_switches",
            "num_fragmentations",
            "mostly_tracked",
            "num_unique_objects",
            "recall",
        ],
        generate_overall=True,
    )
    ov = summary.loc["OVERALL"]
    out["mota*"] = float(ov["mota"])
    out["idf1*"] = float(ov["idf1"])
    out["id_switches"] = int(ov["num_switches"])
    out["fragmentations"] = int(ov["num_fragmentations"])
    out["mostly_tracked"] = float(ov["mostly_tracked"]) / max(float(ov["num_unique_objects"]), 1)

    tp, fp, fn = stats["g_tp"], stats["g_fp"], stats["g_fn"]
    if tp + fp + fn:
        out["ground_precision"] = tp / max(tp + fp, 1)
        out["ground_recall"] = tp / max(tp + fn, 1)
        out["ground_iou"] = tp / max(tp + fp + fn, 1)
    if stats["obj_pts"]:
        out["object_pts_lost_to_ground"] = stats["obj_pts_lost"] / stats["obj_pts"]

    out["timing_ms"] = {
        k: {"mean": float(np.mean(v)), "p95": float(np.percentile(v, 95))}
        for k, v in stats["timing"].items()
    }
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="benchmark dataset dir (one subdir per sequence)")
    ap.add_argument("--results", required=True, help="run dir (one subdir per sequence)")
    ap.add_argument(
        "--cfg",
        default=os.path.join(os.path.dirname(__file__), "..", "..", "configs", "bench_common.cfg"),
    )
    ap.add_argument("--min-pts", type=int, default=5)
    ap.add_argument("--out", help="write metrics json here")
    args = ap.parse_args()
    cfg = load_cfg(args.cfg)

    stats = defaultdict(int)
    for k in ("box_iou", "yaw_err", "vel_err", "heading_err", "jitter"):
        stats[k] = []
    stats["timing"] = defaultdict(list)
    stats["n_gt_cls"] = defaultdict(int)
    stats["hit_1m_cls"] = defaultdict(int)

    accs, names = [], []
    for seq in sorted(glob.glob(os.path.join(os.path.expanduser(args.data), "*"))):
        name = os.path.basename(seq)
        res_dir = os.path.join(args.results, name)
        if not os.path.exists(os.path.join(res_dir, "results.jsonl")):
            continue
        acc = mm.MOTAccumulator(auto_id=True)
        evaluate_sequence(seq, res_dir, cfg, args.min_pts, stats, acc)
        accs.append(acc)
        names.append(name)
    if not accs:
        sys.exit("no results found")
    out = summarize(stats, accs, names)
    out["sequences"] = len(accs)
    text = json.dumps(out, indent=2)
    print(text)
    if args.out:
        with open(args.out, "w") as f:
            f.write(text + "\n")


if __name__ == "__main__":
    main()
