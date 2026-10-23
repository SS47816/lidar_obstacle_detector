"""Convert Argoverse 2 sensor logs into the benchmark format.

Fetch logs without an account, e.g. (lidar + annotations + poses only, ~150 MB per log):

    s5cmd --no-sign-request cp "s3://argoverse/datasets/av2/sensor/val/<log>/sensors/lidar/*" ...

    python -I prepare_av2.py --root ~/datasets/av2/sensor/val --out ~/datasets/lod_bench/av2

For the showcase renderer, also fetch `<log>/map/*` and `<log>/sensors/cameras/ring_front_center/*`
and add `--with-map --with-camera` (`--extras-only` writes just these for converted logs):
map.json (lane boundaries, crosswalks, drivable area in the world frame) and camera.jsonl (per
frame: the nearest front-camera image, intrinsics and world_T_camera).
"""

import argparse
import bisect
import glob
import json
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import SequenceWriter, box_record, make_tf, quat_to_rot, yaw_of  # noqa: E402

VEHICLE = {
    "REGULAR_VEHICLE",
    "LARGE_VEHICLE",
    "BUS",
    "BOX_TRUCK",
    "TRUCK",
    "TRUCK_CAB",
    "VEHICULAR_TRAILER",
    "SCHOOL_BUS",
    "ARTICULATED_BUS",
    "RAILED_VEHICLE",
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


def write_map(log_dir, out_dir):
    """Vector map (world = AV2 city frame) for drawing; False if the log has no map."""
    paths = glob.glob(os.path.join(log_dir, "map", "log_map_archive_*.json"))
    if not paths:
        return False
    with open(paths[0]) as f:
        m = json.load(f)

    def line(pts):
        return [[round(p["x"], 2), round(p["y"], 2), round(p.get("z", 0.0), 2)] for p in pts]

    lanes = []
    for lane in m["lane_segments"].values():
        for side in ("left", "right"):
            lanes.append(
                {
                    "points": line(lane[f"{side}_lane_boundary"]),
                    "mark": lane[f"{side}_lane_mark_type"],
                    "intersection": lane["is_intersection"],
                }
            )
    crosswalks = [
        line(c["edge1"]) + line(c["edge2"])[::-1] for c in m["pedestrian_crossings"].values()
    ]
    drivable = [line(d["area_boundary"]) for d in m["drivable_areas"].values()]
    with open(os.path.join(out_dir, "map.json"), "w") as f:
        json.dump({"lanes": lanes, "crosswalks": crosswalks, "drivable": drivable}, f)
    return True


def write_camera(log_dir, out_dir, cal, pose_df, lidar_stamps, name="ring_front_center"):
    """Per lidar frame: nearest image of `name`, intrinsics and world_T_camera."""
    images = sorted(glob.glob(os.path.join(log_dir, "sensors", "cameras", name, "*.jpg")))
    if not images:
        return False
    img_ts = [int(os.path.basename(p).split(".")[0]) for p in images]
    intr = pd.read_feather(os.path.join(log_dir, "calibration", "intrinsics.feather"))
    k = intr[intr.sensor_name == name].iloc[0]
    c = cal[cal.sensor_name == name].iloc[0]
    ego_T_cam = make_tf(quat_to_rot(c.qw, c.qx, c.qy, c.qz), [c.tx_m, c.ty_m, c.tz_m])
    pose_ts = pose_df.timestamp_ns.to_numpy()
    with open(os.path.join(out_dir, "camera.jsonl"), "w") as f:
        for idx, ts in enumerate(lidar_stamps):
            j = bisect.bisect_left(img_ts, ts)
            j = min((j - 1, j), key=lambda i: abs(img_ts[i] - ts) if 0 <= i < len(img_ts) else 1e30)
            r = pose_df.iloc[int(np.argmin(np.abs(pose_ts - img_ts[j])))]
            world_T_ego = make_tf(quat_to_rot(r.qw, r.qx, r.qy, r.qz), [r.tx_m, r.ty_m, r.tz_m])
            rec = {
                "idx": idx,
                "image": os.path.abspath(images[j]),
                "dt": (img_ts[j] - ts) * 1e-9,
                "K": [k.fx_px, k.fy_px, k.cx_px, k.cy_px],
                "size": [int(k.width_px), int(k.height_px)],
                "world_T_camera": (world_T_ego @ ego_T_cam).reshape(-1).tolist(),
            }
            f.write(json.dumps(rec) + "\n")
    return True


def convert(log_dir, out_dir, cal, lidar_paths, lidar_stamps):
    log = os.path.basename(log_dir)
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

    writer = SequenceWriter(
        out_dir,
        {
            "dataset": "av2",
            "sequence": log,
            "sensor_height": float(t_up[2]),
            "lidar": "2x Velodyne VLP-32C (stacked)",
        },
    )
    for path, ts in zip(lidar_paths, lidar_stamps):
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True, help="directory containing AV2 log folders")
    ap.add_argument("--out", required=True)
    ap.add_argument("--with-map", action="store_true", help="also write map.json")
    ap.add_argument("--with-camera", action="store_true", help="also write camera.jsonl")
    ap.add_argument("--extras-only", action="store_true", help="only write map / camera")
    args = ap.parse_args()

    for log_dir in sorted(glob.glob(os.path.join(os.path.expanduser(args.root), "*"))):
        log = os.path.basename(log_dir)
        out_dir = os.path.join(os.path.expanduser(args.out), log)
        cal = pd.read_feather(os.path.join(log_dir, "calibration", "egovehicle_SE3_sensor.feather"))
        lidar_paths = sorted(glob.glob(os.path.join(log_dir, "sensors", "lidar", "*.feather")))
        lidar_stamps = [int(os.path.basename(p).split(".")[0]) for p in lidar_paths]
        if not args.extras_only:
            convert(log_dir, out_dir, cal, lidar_paths, lidar_stamps)
        if not os.path.isdir(out_dir):
            continue
        if args.with_map and write_map(log_dir, out_dir):
            print(f"{log}: map.json")
        if args.with_camera:
            pose_df = pd.read_feather(os.path.join(log_dir, "city_SE3_egovehicle.feather"))
            if write_camera(log_dir, out_dir, cal, pose_df, lidar_stamps):
                print(f"{log}: camera.jsonl")


if __name__ == "__main__":
    main()
