"""Convert nuScenes (e.g. v1.0-mini + lidarseg) into the benchmark format.

Every LIDAR_TOP sweep (20 Hz) is exported so trackers run at the real sensor rate; boxes and
per-point ground labels are attached to the 2 Hz keyframes only.

    python -I prepare_nuscenes.py --root ~/datasets/nuscenes --version v1.0-mini \
        --out ~/datasets/lod_bench/nuscenes
"""

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import SequenceWriter, box_record, make_tf, quat_to_rot, yaw_of  # noqa: E402

GROUND_PREFIX = "flat."
IGNORE_SEG = {"noise", "vehicle.ego"}


def coarse_class(name):
    if name.startswith("human.pedestrian"):
        return "pedestrian"
    if name in ("vehicle.bicycle", "vehicle.motorcycle"):
        return "cyclist"
    if name.startswith("vehicle."):
        return "vehicle"
    return "other"


def load_table(root, version, name):
    with open(os.path.join(root, version, name + ".json")) as f:
        return json.load(f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--version", default="v1.0-mini")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    root = os.path.expanduser(args.root)

    tables = {
        n: load_table(root, args.version, n)
        for n in (
            "scene",
            "sample",
            "sample_data",
            "sample_annotation",
            "instance",
            "category",
            "calibrated_sensor",
            "sensor",
            "ego_pose",
        )
    }
    by_token = {n: {r["token"]: r for r in t} for n, t in tables.items()}
    lidarseg = {}
    if os.path.exists(os.path.join(root, args.version, "lidarseg.json")):
        for r in load_table(root, args.version, "lidarseg"):
            lidarseg[r["sample_data_token"]] = r["filename"]

    # lidarseg index -> ground(1) / non-ground(0) / ignore(255)
    seg_map = np.full(256, 255, dtype=np.uint8)
    for c in tables["category"]:
        if "index" not in c:
            continue
        if c["name"] in IGNORE_SEG:
            continue
        seg_map[c["index"]] = 1 if c["name"].startswith(GROUND_PREFIX) else 0

    anns_by_sample = {}
    for a in tables["sample_annotation"]:
        anns_by_sample.setdefault(a["sample_token"], []).append(a)

    def ann_velocity(ann):
        """Global-frame velocity via neighbouring annotations (as in the devkit)."""
        prev = by_token["sample_annotation"].get(ann["prev"]) if ann["prev"] else None
        nxt = by_token["sample_annotation"].get(ann["next"]) if ann["next"] else None
        a, b = prev or ann, nxt or ann
        if a is b:
            return None
        ta = by_token["sample"][a["sample_token"]]["timestamp"] * 1e-6
        tb = by_token["sample"][b["sample_token"]]["timestamp"] * 1e-6
        if tb - ta <= 0 or tb - ta > 1.5:
            return None
        return (np.array(b["translation"]) - np.array(a["translation"])) / (tb - ta)

    for scene in tables["scene"]:
        first = by_token["sample"][scene["first_sample_token"]]
        sd_token = next(
            sd["token"]
            for sd in tables["sample_data"]
            if sd["sample_token"] == first["token"]
            and by_token["sensor"][
                by_token["calibrated_sensor"][sd["calibrated_sensor_token"]]["sensor_token"]
            ]["channel"]
            == "LIDAR_TOP"
        )
        cs = by_token["calibrated_sensor"][
            by_token["sample_data"][sd_token]["calibrated_sensor_token"]
        ]
        r_cs = quat_to_rot(*cs["rotation"])
        t_cs = np.array(cs["translation"])
        ego_T_bench = make_tf(np.eye(3), t_cs)

        out_dir = os.path.join(os.path.expanduser(args.out), scene["name"])
        writer = SequenceWriter(
            out_dir,
            {
                "dataset": "nuscenes",
                "version": args.version,
                "sequence": scene["name"],
                "description": scene["description"],
                "sensor_height": float(t_cs[2]),
                "lidar": "Velodyne HDL-32E",
            },
        )
        while sd_token:
            sd = by_token["sample_data"][sd_token]
            pts = np.fromfile(os.path.join(root, sd["filename"]), dtype=np.float32).reshape(-1, 5)
            xyz = pts[:, :3] @ r_cs.T  # lidar -> bench (rotation only)
            points = np.hstack([xyz, pts[:, 3:4]])

            ep = by_token["ego_pose"][sd["ego_pose_token"]]
            world_T_bench = make_tf(quat_to_rot(*ep["rotation"]), ep["translation"]) @ ego_T_bench
            bench_T_world = np.linalg.inv(world_T_bench)

            objects, labels = None, None
            if sd["is_key_frame"]:
                objects = []
                for a in anns_by_sample.get(sd["sample_token"], []):
                    cat = by_token["category"][
                        by_token["instance"][a["instance_token"]]["category_token"]
                    ]["name"]
                    r_world = quat_to_rot(*a["rotation"])
                    center = (
                        bench_T_world[:3, :3] @ np.array(a["translation"]) + bench_T_world[:3, 3]
                    )
                    yaw = yaw_of(bench_T_world[:3, :3] @ r_world)
                    width, length, height = a["size"]
                    objects.append(
                        box_record(
                            a["instance_token"],
                            cat,
                            coarse_class(cat),
                            center,
                            (length, width, height),
                            yaw,
                            a["num_lidar_pts"],
                            ann_velocity(a),
                        )
                    )
                if sd_token in lidarseg:
                    seg = np.fromfile(os.path.join(root, lidarseg[sd_token]), dtype=np.uint8)
                    labels = seg_map[seg]
            writer.add_frame(points, sd["timestamp"] * 1e-6, world_T_bench, objects, labels)
            sd_token = sd["next"]
        writer.close()
        print(f"{scene['name']}: {writer.num_frames} frames -> {out_dir}")


if __name__ == "__main__":
    main()
