"""Convert Argoverse 2 sensor logs into the benchmark format.

Fetch logs without an account, e.g. (lidar + annotations + poses only, ~150 MB per log):

    s5cmd --no-sign-request cp "s3://argoverse/datasets/av2/sensor/val/<log>/sensors/lidar/*" ...

    python -I prepare_av2.py --root ~/datasets/av2/sensor/val --out ~/datasets/lod_bench/av2
"""

import argparse
import glob
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import SequenceWriter, box_record, make_tf, quat_to_rot, yaw_of  # noqa: E402

VEHICLE = {
    "REGULAR_VEHICLE", "LARGE_VEHICLE", "BUS", "BOX_TRUCK", "TRUCK", "TRUCK_CAB",
    "VEHICULAR_TRAILER", "SCHOOL_BUS", "ARTICULATED_BUS", "RAILED_VEHICLE",
}
PEDESTRIAN = {"PEDESTRIAN", "WHEELCHAIR", "STROLLER", "OFFICIAL_SIGNALER"}
# Ridden bicycles are covered by the BICYCLIST cuboid; the separate BICYCLE box -> "other".
CYCLIST = {"BICYCLIST", "MOTORCYCLIST", "WHEELED_RIDER", "MOTORCYCLE"}


def coarse_class(name):
    if name in VEHICLE:
        return "vehicle"
    if name in PEDESTRIAN:
        return "pedestrian"
    if name in CYCLIST:
        return "cyclist"
    return "other"


def track_velocities(ann, poses):
    """World-frame velocity per annotation row from its track's neighbouring positions."""
    vel = np.full((len(ann), 2), np.nan)
    centers = np.zeros((len(ann), 3))
    for i, row in enumerate(ann.itertuples()):
        tf = poses[row.timestamp_ns]
        centers[i] = tf[:3, :3] @ np.array([row.tx_m, row.ty_m, row.tz_m]) + tf[:3, 3]
    ann = ann.assign(_cx=centers[:, 0], _cy=centers[:, 1], _row=np.arange(len(ann)))
    for _, g in ann.groupby("track_uuid"):
        g = g.sort_values("timestamp_ns")
        t = g.timestamp_ns.to_numpy() * 1e-9
        xy = g[["_cx", "_cy"]].to_numpy()
        rows = g._row.to_numpy()
        for k in range(len(g)):
            a, b = max(k - 1, 0), min(k + 1, len(g) - 1)
            if a != b and 0 < t[b] - t[a] < 0.5:
                vel[rows[k]] = (xy[b] - xy[a]) / (t[b] - t[a])
    return vel


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True, help="directory containing AV2 log folders")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    for log_dir in sorted(glob.glob(os.path.join(os.path.expanduser(args.root), "*"))):
        log = os.path.basename(log_dir)
        cal = pd.read_feather(os.path.join(log_dir, "calibration", "egovehicle_SE3_sensor.feather"))
        up = cal[cal.sensor_name == "up_lidar"].iloc[0]
        t_up = np.array([up.tx_m, up.ty_m, up.tz_m])
        ego_T_bench = make_tf(np.eye(3), t_up)

        pose_df = pd.read_feather(os.path.join(log_dir, "city_SE3_egovehicle.feather"))
        poses = {
            int(r.timestamp_ns): make_tf(quat_to_rot(r.qw, r.qx, r.qy, r.qz), [r.tx_m, r.ty_m, r.tz_m])
            for r in pose_df.itertuples()
        }
        ann = pd.read_feather(os.path.join(log_dir, "annotations.feather")).reset_index(drop=True)
        ann["vx"], ann["vy"] = np.nan, np.nan
        ann[["vx", "vy"]] = track_velocities(ann, poses)
        ann_by_t = dict(tuple(ann.groupby("timestamp_ns")))

        out_dir = os.path.join(os.path.expanduser(args.out), log)
        writer = SequenceWriter(
            out_dir,
            {
                "dataset": "av2",
                "sequence": log,
                "sensor_height": float(t_up[2]),
                "lidar": "2x Velodyne VLP-32C (stacked)",
            },
        )
        for path in sorted(glob.glob(os.path.join(log_dir, "sensors", "lidar", "*.feather"))):
            ts = int(os.path.basename(path).split(".")[0])
            df = pd.read_feather(path)
            xyz = df[["x", "y", "z"]].to_numpy(np.float32) - t_up.astype(np.float32)
            points = np.hstack([xyz, df[["intensity"]].to_numpy(np.float32)])
            world_T_bench = poses[ts] @ ego_T_bench

            objects = []
            for r in ann_by_t.get(ts, pd.DataFrame()).itertuples():
                rot = quat_to_rot(r.qw, r.qx, r.qy, r.qz)
                vel = None if np.isnan(r.vx) else (r.vx, r.vy)
                objects.append(
                    box_record(
                        r.track_uuid,
                        r.category,
                        coarse_class(r.category),
                        np.array([r.tx_m, r.ty_m, r.tz_m]) - t_up,
                        (r.length_m, r.width_m, r.height_m),
                        yaw_of(rot),
                        r.num_interior_pts,
                        vel,
                    )
                )
            writer.add_frame(points, ts * 1e-9, world_T_bench, objects, None)
        writer.close()
        print(f"{log}: {writer.num_frames} frames -> {out_dir}")


if __name__ == "__main__":
    main()
