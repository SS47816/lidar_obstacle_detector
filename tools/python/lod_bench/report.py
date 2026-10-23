"""Collect metrics.json files of several runs into markdown tables.

    python -I report.py --runs ~/datasets/lod_bench/runs --names legacy_aabb fast accurate \
        [--labels "classic" "fast" "accurate"] [--out docs/benchmarks_table.md]
"""

import argparse
import json
import os

COLUMNS = [
    ("recall@1m", "Recall@1m", "{:.3f}"),
    ("recall@2m", "Recall@2m", "{:.3f}"),
    ("recall@iou0.3", "Recall@IoU.3", "{:.3f}"),
    ("coverage", "Coverage", "{:.1%}"),
    ("recall@1m_large", "Recall@1m >7m", "{:.3f}"),
    ("over_seg_rate", "Over-seg", "{:.1%}"),
    ("large_split_rate", "Split >7m", "{:.1%}"),
    ("under_seg_rate", "Under-seg", "{:.1%}"),
    ("mean_bev_iou", "BEV IoU", "{:.3f}"),
    ("box_jitter", "Box jitter", "{:.3f}"),
    ("vehicle_yaw_err_deg_mean", "Yaw err [deg]", "{:.1f}"),
    ("moving_heading_err_deg", "Moving heading err [deg]", "{:.1f}"),
    ("idf1*", "IDF1*", "{:.3f}"),
    ("mota*", "MOTA*", "{:.3f}"),
    ("id_switches", "ID sw.", "{:d}"),
    ("vel_err_mps", "Vel. err [m/s]", "{:.2f}"),
    ("ground_iou", "Ground IoU", "{:.3f}"),
    ("object_pts_lost_to_ground", "Obj. pts lost", "{:.1%}"),
    ("pillar_flicker", "Pillar flicker", "{:.1%}"),
    ("label_accuracy", "Label acc.", "{:.1%}"),
]


def fmt(metrics, key, spec):
    v = metrics.get(key)
    if v is None:
        return "–"
    return spec.format(v)


def table(runs_dir, dataset, names, labels):
    cols = [c for c in COLUMNS]
    lines = [
        "| Pipeline | " + " | ".join(c[1] for c in cols) + " | Runtime mean / p95 [ms] |",
        "|---|" + "---|" * (len(cols) + 1),
    ]
    for name, label in zip(names, labels):
        path = os.path.join(runs_dir, dataset, name, "metrics.json")
        if not os.path.exists(path):
            continue
        with open(path) as f:
            m = json.load(f)
        t = m.get("timing_ms", {}).get("total")
        row = [fmt(m, c[0], c[2]) for c in cols]
        runtime = f"{t['mean']:.1f} / {t['p95']:.1f}" if t else "–"
        lines.append(f"| {label} | " + " | ".join(row) + f" | {runtime} |")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", required=True)
    ap.add_argument("--names", nargs="+", required=True)
    ap.add_argument("--labels", nargs="+")
    ap.add_argument("--datasets", nargs="+", default=["nuscenes", "av2"])
    ap.add_argument("--out")
    args = ap.parse_args()
    labels = args.labels or args.names
    runs = os.path.expanduser(args.runs)
    parts = []
    for ds in args.datasets:
        parts.append(f"### {ds}\n\n" + table(runs, ds, args.names, labels) + "\n")
    text = "\n".join(parts)
    print(text)
    if args.out:
        with open(args.out, "w") as f:
            f.write(text)


if __name__ == "__main__":
    main()
