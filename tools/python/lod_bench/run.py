"""Run lod_benchmark over every sequence of a benchmark dataset, then evaluate.

    python -I run.py --data ~/datasets/lod_bench/nuscenes --runs ~/datasets/lod_bench/runs \
        --name fast --preset fast [--set key=value ...] [--jobs 1]

Use --jobs 1 (default) for runtime numbers; parallel jobs distort per-frame timings.
"""

import argparse
import glob
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True)
    ap.add_argument("--runs", required=True)
    ap.add_argument("--name", required=True)
    ap.add_argument("--preset", default="fast")
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--dump-every", type=int, default=5)
    ap.add_argument("--bin", default=os.path.join(REPO, "build", "core", "lod_benchmark"))
    ap.add_argument("--cfg", default=os.path.join(REPO, "tools", "configs", "bench_common.cfg"))
    args = ap.parse_args()

    data = os.path.expanduser(args.data)
    dataset = os.path.basename(os.path.normpath(data))
    run_dir = os.path.join(os.path.expanduser(args.runs), dataset, args.name)
    seqs = sorted(d for d in glob.glob(os.path.join(data, "*")) if os.path.isdir(d))

    def run(seq):
        cmd = [
            args.bin,
            "--seq",
            seq,
            "--out",
            os.path.join(run_dir, os.path.basename(seq)),
            "--preset",
            args.preset,
            "--config",
            args.cfg,
            "--dump-every",
            str(args.dump_every),
        ]
        for kv in args.set:
            cmd += ["--set", kv]
        return subprocess.run(cmd, capture_output=True, text=True)

    with ThreadPoolExecutor(max_workers=args.jobs) as ex:
        for seq, r in zip(seqs, ex.map(run, seqs)):
            if r.returncode != 0:
                sys.exit(f"{seq} failed:\n{r.stderr}")
    subprocess.run(
        [
            sys.executable,
            "-I",
            os.path.join(HERE, "evaluate.py"),
            "--data",
            data,
            "--results",
            run_dir,
            "--cfg",
            args.cfg,
            "--out",
            os.path.join(run_dir, "metrics.json"),
        ],
        check=True,
    )


if __name__ == "__main__":
    main()
