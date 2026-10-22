"""Shared helpers for the lidar_obstacle_detector benchmark format.

Benchmark sequence layout (one directory per sequence)::

    meta.json            dataset / sequence info
    frames.jsonl         one line per lidar frame: idx, t [s], pose (world_T_bench, 4x4
                         row-major), annotated (bool), ground_labels (relative path or null)
    points/XXXXXX.bin    float32 N x 4 (x, y, z, intensity), KITTI-style
    labels/XXXXXX.bin    uint8 N, per-point ground label: 1 ground, 0 non-ground, 255 ignore
    gt.jsonl             one line per annotated frame: idx, objects[]

All points and boxes are expressed in the "bench" frame: x forward, y left, z up, with the
origin at the (top) lidar mounting position and the orientation of the ego vehicle. This keeps
the ROI parameters comparable across datasets.
"""

import json
import os

import numpy as np

# Coarse classes used by the evaluation. The detector itself is class-agnostic.
OBSTACLE_CLASSES = ("vehicle", "pedestrian", "cyclist")


def quat_to_rot(w, x, y, z):
    """Rotation matrix from a (w, x, y, z) unit quaternion."""
    n = np.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def make_tf(rot, trans):
    tf = np.eye(4)
    tf[:3, :3] = rot
    tf[:3, 3] = trans
    return tf


def yaw_of(rot):
    """Heading of the x-axis of a rotation matrix projected on the xy-plane."""
    return float(np.arctan2(rot[1, 0], rot[0, 0]))


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class SequenceWriter:
    def __init__(self, out_dir, meta):
        self.out_dir = out_dir
        os.makedirs(os.path.join(out_dir, "points"), exist_ok=True)
        os.makedirs(os.path.join(out_dir, "labels"), exist_ok=True)
        with open(os.path.join(out_dir, "meta.json"), "w") as f:
            json.dump(meta, f, indent=2)
        self._frames = open(os.path.join(out_dir, "frames.jsonl"), "w")
        self._gt = open(os.path.join(out_dir, "gt.jsonl"), "w")
        self.num_frames = 0

    def add_frame(self, points, t, pose, objects=None, ground_labels=None):
        """points: (N, 4) float array in the bench frame; pose: 4x4 world_T_bench."""
        idx = self.num_frames
        points.astype(np.float32).tofile(
            os.path.join(self.out_dir, "points", f"{idx:06d}.bin")
        )
        label_path = None
        if ground_labels is not None:
            assert len(ground_labels) == len(points)
            label_path = f"labels/{idx:06d}.bin"
            ground_labels.astype(np.uint8).tofile(
                os.path.join(self.out_dir, label_path)
            )
        rec = {
            "idx": idx,
            "t": float(t),
            "pose": [float(v) for v in np.asarray(pose).reshape(-1)],
            "annotated": objects is not None,
            "ground_labels": label_path,
        }
        self._frames.write(json.dumps(rec) + "\n")
        if objects is not None:
            self._gt.write(json.dumps({"idx": idx, "objects": objects}) + "\n")
        self.num_frames += 1

    def close(self):
        self._frames.close()
        self._gt.close()


def box_record(track_id, category, cls, center, size_lwh, yaw, num_pts, velocity):
    return {
        "id": str(track_id),
        "category": category,
        "cls": cls,
        "center": [float(v) for v in center],
        "size": [float(v) for v in size_lwh],
        "yaw": float(wrap_angle(yaw)),
        "num_pts": int(num_pts),
        "velocity": None if velocity is None else [float(v) for v in velocity[:2]],
    }


def read_sequence(seq_dir):
    """Load meta, frames and gt (dict idx -> objects) of a benchmark sequence."""
    with open(os.path.join(seq_dir, "meta.json")) as f:
        meta = json.load(f)
    with open(os.path.join(seq_dir, "frames.jsonl")) as f:
        frames = [json.loads(line) for line in f]
    gt = {}
    with open(os.path.join(seq_dir, "gt.jsonl")) as f:
        for line in f:
            rec = json.loads(line)
            gt[rec["idx"]] = rec["objects"]
    return meta, frames, gt


def load_points(seq_dir, idx):
    return np.fromfile(
        os.path.join(seq_dir, "points", f"{idx:06d}.bin"), dtype=np.float32
    ).reshape(-1, 4)
