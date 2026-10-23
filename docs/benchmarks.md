# Benchmarks

All numbers on this page come from `tools/` in this repository and can be reproduced with the
commands at the end.

## Setup

**Data.**

| | Argoverse 2 (sensor, val) | nuScenes (v1.0-mini) |
|---|---|---|
| Sequences | 15 logs (every 10th of the 150 val logs) | all 10 scenes |
| Lidar | 2 × 32-beam (stacked), 10 Hz, ~95k points | 32-beam, 20 Hz, ~35k points |
| Scans processed | 2,350 (all annotated) | 3,931 (404 annotated keyframes) |
| Ground truth objects scored | 52,774 | 6,610 |

Both datasets are converted into one frame: x forward, y left, z up, origin at the (top) lidar.
Every pipeline uses the same region of interest, ±40 m in x and y and -2.5 m to 1 m in z
(`tools/configs/bench_common.cfg`). All other parameters are the pipeline's own defaults.

**Ground truth.** The detector is class-agnostic, so it is scored against vehicles, pedestrians
and cyclists that have at least 5 lidar points and a centre inside the ROI. Static object
classes such as cones and barriers are not scored.

**Metrics.** The detector also (correctly) reports walls, poles and vegetation, which have no
labels. Plain precision and MOTA would mostly measure how much unlabeled structure a scene
contains, so we report:

- **Recall@1m / @2m**: share of GT objects with a predicted box centre within 1 m / 2 m in
  bird's-eye view (one-to-one Hungarian matching).
- **Recall@IoU.3**: share of GT objects matched with BEV IoU ≥ 0.3.
- **Over-seg**: GT objects that contain ≥ 2 predicted box centres. **Under-seg**: GT objects
  merged with another GT object into one predicted box.
- **BEV IoU** and **yaw error** (vehicles; footprint orientation modulo 90°, mean) of
  centre-matched boxes.
- **IDF1\* / MOTA\* / ID switches**: CLEAR / ID metrics (py-motmetrics, 2 m gate) after removing
  predictions that are more than 2 m from every scored GT object. nuScenes is scored on its 2 Hz
  keyframes, AV2 on every scan. **Velocity error**: median, in the world frame (the benchmark
  passes the ego pose, as `tracking_frame` does on a robot).
- **Ground IoU** against nuScenes lidarseg labels (`flat.*` = ground). **Obj. pts lost**: share
  of points inside scored GT boxes (≥ 0.2 m above the box bottom) labelled as ground.
- **Runtime**: wall-clock per scan on an Intel i9-12900K, one process at a time. Box fitting
  uses OpenMP; every other stage is single-threaded.

## Results

The v1 rows run the original v1 code, kept in `legacy/`. Its only changes are a guard against
the v1 out-of-bounds read and no per-frame printing. v2 `legacy` is the new library configured
like v1, so the difference between those rows comes from the bug fixes and the new tracker alone.

### nuscenes

| Pipeline | Recall@1m | Recall@2m | Recall@IoU.3 | Recall@1m >7m | Over-seg | Under-seg | BEV IoU | Yaw err [deg] | Moving heading err [deg] | IDF1* | MOTA* | ID sw. | Vel. err [m/s] | Ground IoU | Obj. pts lost | Structures / frame | Runtime mean / p95 [ms] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| v1 (AABB boxes) | 0.376 | 0.558 | 0.308 | 0.264 | 5.3% | 9.1% | 0.348 | 9.0 | 101.4 | 0.351 | 0.268 | 1476 | – | 0.907 | 5.5% | 0.0 | 6.6 / 8.3 |
| v1 (PCA boxes) | 0.372 | 0.553 | 0.270 | 0.295 | 5.3% | 8.8% | 0.307 | 12.1 | 112.5 | 0.354 | 0.269 | 1460 | – | 0.907 | 5.5% | 0.0 | 6.6 / 8.3 |
| v2 `legacy` | 0.375 | 0.555 | 0.305 | 0.264 | 5.2% | 9.2% | 0.348 | 9.0 | 101.9 | 0.568 | 0.422 | 425 | 0.43 | 0.908 | 5.4% | 0.0 | 6.7 / 8.5 |
| v2 `fast` | 0.415 | 0.548 | 0.357 | 0.384 | 3.4% | 11.1% | 0.426 | 4.0 | 15.9 | 0.585 | 0.442 | 363 | 0.41 | 0.921 | 6.4% | 0.6 | 2.7 / 3.2 |
| v2 `accurate` | 0.408 | 0.547 | 0.344 | 0.339 | 4.8% | 7.9% | 0.408 | 4.1 | 18.5 | 0.568 | 0.418 | 390 | 0.41 | 0.921 | 6.4% | 0.3 | 6.1 / 8.9 |

### av2

| Pipeline | Recall@1m | Recall@2m | Recall@IoU.3 | Recall@1m >7m | Over-seg | Under-seg | BEV IoU | Yaw err [deg] | Moving heading err [deg] | IDF1* | MOTA* | ID sw. | Vel. err [m/s] | Ground IoU | Obj. pts lost | Structures / frame | Runtime mean / p95 [ms] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| v1 (AABB boxes) | 0.534 | 0.743 | 0.423 | 0.348 | 15.9% | 28.2% | 0.370 | 6.2 | 89.6 | 0.204 | 0.293 | 11822 | – | – | 25.4% | 0.0 | 27.9 / 36.1 |
| v1 (PCA boxes) | 0.531 | 0.744 | 0.400 | 0.294 | 16.1% | 26.2% | 0.344 | 14.5 | 97.3 | 0.200 | 0.286 | 12223 | – | – | 25.4% | 0.0 | 27.8 / 35.1 |
| v2 `legacy` | 0.531 | 0.742 | 0.420 | 0.356 | 15.9% | 28.1% | 0.368 | 6.2 | 90.1 | 0.506 | 0.422 | 4653 | 0.56 | – | 26.3% | 0.0 | 28.8 / 37.4 |
| v2 `fast` | 0.661 | 0.818 | 0.617 | 0.543 | 5.3% | 9.7% | 0.512 | 5.9 | 13.9 | 0.725 | 0.709 | 1725 | 0.35 | – | 4.8% | 1.9 | 7.5 / 8.9 |
| v2 `accurate` | 0.697 | 0.849 | 0.628 | 0.510 | 9.4% | 8.6% | 0.508 | 5.7 | 14.4 | 0.710 | 0.676 | 1749 | 0.32 | – | 4.8% | 1.2 | 31.8 / 50.8 |


**Takeaways**

- **Tracking**: the new tracker alone (v1 → v2 `legacy`) cuts ID switches by 61–71 % (IDF1\* 0.35 → 0.57
  on nuScenes, 0.20 → 0.51 on AV2).
  With the full `fast` preset, IDF1\* goes from 0.35 to 0.59 (nuScenes) and from 0.20 to 0.73 (AV2),
  and ID switches drop by 75–85 %.
- **Ground segmentation** is the largest single gain on AV2's hilly Pittsburgh logs: RANSAC's
  single plane labels 25 % of object points as ground; Patchwork++ with refinement, under 5 %.
- **Detection**: recall@1m 0.53 → 0.66 (`fast`) / 0.70 (`accurate`) on AV2 and 0.38 → 0.42 on
  nuScenes, with under-segmentation on AV2 down from 28 % to about 10 %.
- **Speed**: `fast` is 2.4× (nuScenes) to 3.7× (AV2) faster than v1, including the occupancy grid
  (0.5–1.2 ms). `accurate` is about as slow as v1 (24 ms of its 32 ms on AV2 go to KD-tree
  clustering, including re-splitting oversized clusters) and is the next optimisation target.
- **nuScenes recall is limited by sparsity.** Almost half of its objects have fewer than 20 points,
  so `cluster_min_size` is the limit; the sparse-lidar tuning raises recall@1m to 0.57.

## Ablations

Each row replaces one component of `fast` with an alternative. These runs used 12 parallel
jobs, so runtimes are not reported.

### nuscenes

| Pipeline | Recall@1m | Recall@2m | Recall@IoU.3 | Recall@1m >7m | Over-seg | Under-seg | BEV IoU | Yaw err [deg] | Moving heading err [deg] | IDF1* | MOTA* | ID sw. | Vel. err [m/s] | Ground IoU | Obj. pts lost | Structures / frame |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `fast` (all v2.1 defaults) | 0.415 | 0.548 | 0.357 | 0.384 | 3.4% | 11.1% | 0.426 | 4.0 | 15.9 | 0.585 | 0.442 | 363 | 0.41 | 0.921 | 6.4% | 0.6 |
| no heading from motion | 0.415 | 0.548 | 0.355 | 0.376 | 3.4% | 11.1% | 0.424 | 4.2 | 100.4 | 0.583 | 0.441 | 366 | 0.41 | 0.921 | 6.4% | 0.6 |
| no shape completion | 0.385 | 0.542 | 0.307 | 0.334 | 3.3% | 10.3% | 0.380 | 4.1 | 16.1 | 0.570 | 0.434 | 380 | 0.40 | 0.921 | 6.4% | 0.6 |
| no height-gap split | 0.406 | 0.542 | 0.352 | 0.402 | 3.0% | 11.8% | 0.427 | 4.1 | 16.4 | 0.579 | 0.436 | 385 | 0.41 | 0.921 | 6.4% | 0.7 |
| no oversized-cluster split | 0.398 | 0.526 | 0.342 | 0.373 | 2.1% | 10.7% | 0.433 | 3.8 | 15.9 | 0.579 | 0.444 | 336 | 0.41 | 0.921 | 6.4% | 1.3 |
| structures kept as objects | 0.415 | 0.548 | 0.357 | 0.381 | 3.4% | 11.5% | 0.425 | 4.0 | 15.9 | 0.585 | 0.443 | 360 | 0.41 | 0.921 | 6.4% | 0.0 |
| object_max_length 12 m | 0.417 | 0.551 | 0.356 | 0.366 | 3.5% | 10.3% | 0.423 | 4.1 | 15.7 | 0.583 | 0.439 | 384 | 0.41 | 0.921 | 6.4% | 1.0 |
| object_max_length 20 m | 0.415 | 0.547 | 0.357 | 0.379 | 3.4% | 11.3% | 0.426 | 4.0 | 15.9 | 0.585 | 0.442 | 360 | 0.41 | 0.921 | 6.4% | 0.3 |
| ground: constrained RANSAC, no refine | 0.418 | 0.577 | 0.362 | 0.379 | 3.9% | 12.0% | 0.409 | 4.5 | 17.3 | 0.592 | 0.451 | 426 | 0.45 | 0.914 | 5.6% | 0.9 |
| ground: v1 RANSAC, no refine | 0.419 | 0.580 | 0.363 | 0.379 | 4.1% | 12.3% | 0.406 | 4.6 | 17.9 | 0.590 | 0.451 | 443 | 0.46 | 0.908 | 5.4% | 0.9 |
| no elevation-map refinement | 0.401 | 0.567 | 0.352 | 0.352 | 3.9% | 12.3% | 0.399 | 4.9 | 17.1 | 0.564 | 0.432 | 489 | 0.51 | 0.889 | 2.8% | 0.7 |
| clustering: adaptive Euclidean | 0.407 | 0.546 | 0.343 | 0.334 | 4.8% | 7.9% | 0.407 | 4.1 | 18.2 | 0.568 | 0.418 | 387 | 0.41 | 0.921 | 6.4% | 0.3 |
| boxes: axis-aligned | 0.412 | 0.546 | 0.370 | 0.355 | 3.4% | 11.6% | 0.420 | 7.1 | 17.9 | 0.584 | 0.438 | 391 | 0.40 | 0.921 | 6.4% | 0.6 |
| boxes: PCA | 0.396 | 0.543 | 0.326 | 0.360 | 3.4% | 11.3% | 0.379 | 10.5 | 14.4 | 0.579 | 0.433 | 369 | 0.40 | 0.921 | 6.4% | 0.6 |
| tracking cost: centre distance | 0.413 | 0.548 | 0.353 | 0.366 | 3.3% | 11.2% | 0.423 | 4.1 | 16.3 | 0.575 | 0.435 | 397 | 0.41 | 0.921 | 6.4% | 0.6 |
| no tracking | 0.385 | 0.542 | 0.307 | 0.334 | 3.3% | 10.4% | 0.380 | 4.2 | 107.5 | 0.154 | 0.079 | 2732 | – | 0.921 | 6.4% | 0.6 |

### av2

| Pipeline | Recall@1m | Recall@2m | Recall@IoU.3 | Recall@1m >7m | Over-seg | Under-seg | BEV IoU | Yaw err [deg] | Moving heading err [deg] | IDF1* | MOTA* | ID sw. | Vel. err [m/s] | Ground IoU | Obj. pts lost | Structures / frame |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `fast` (all v2.1 defaults) | 0.661 | 0.818 | 0.617 | 0.543 | 5.3% | 9.7% | 0.512 | 5.9 | 13.9 | 0.725 | 0.709 | 1725 | 0.35 | – | 4.8% | 1.9 |
| no heading from motion | 0.660 | 0.818 | 0.616 | 0.535 | 5.3% | 9.7% | 0.510 | 6.5 | 77.9 | 0.724 | 0.709 | 1742 | 0.35 | – | 4.8% | 1.9 |
| no shape completion | 0.652 | 0.821 | 0.588 | 0.479 | 5.3% | 9.0% | 0.495 | 5.9 | 14.3 | 0.724 | 0.712 | 1663 | 0.35 | – | 4.8% | 1.9 |
| no height-gap split | 0.636 | 0.801 | 0.599 | 0.529 | 4.7% | 9.8% | 0.506 | 6.1 | 14.1 | 0.718 | 0.704 | 1842 | 0.35 | – | 4.8% | 2.1 |
| no oversized-cluster split | 0.637 | 0.783 | 0.600 | 0.506 | 4.8% | 8.0% | 0.521 | 5.9 | 13.9 | 0.717 | 0.689 | 1586 | 0.34 | – | 4.8% | 3.2 |
| structures kept as objects | 0.662 | 0.821 | 0.618 | 0.542 | 5.4% | 13.2% | 0.511 | 5.9 | 13.9 | 0.727 | 0.710 | 1761 | 0.35 | – | 4.8% | 0.0 |
| object_max_length 12 m | 0.664 | 0.820 | 0.619 | 0.547 | 5.3% | 8.9% | 0.513 | 5.9 | 14.1 | 0.728 | 0.711 | 1717 | 0.35 | – | 4.8% | 2.6 |
| object_max_length 20 m | 0.661 | 0.818 | 0.618 | 0.530 | 5.3% | 10.0% | 0.512 | 5.9 | 13.8 | 0.725 | 0.708 | 1726 | 0.35 | – | 4.8% | 1.7 |
| ground: constrained RANSAC, no refine | 0.449 | 0.658 | 0.400 | 0.421 | 6.3% | 11.7% | 0.388 | 9.3 | 26.6 | 0.508 | 0.484 | 4420 | 0.62 | – | 25.4% | 4.0 |
| ground: v1 RANSAC, no refine | 0.409 | 0.633 | 0.358 | 0.354 | 6.3% | 12.3% | 0.359 | 10.1 | 32.1 | 0.459 | 0.441 | 5301 | 0.74 | – | 26.3% | 4.9 |
| no elevation-map refinement | 0.646 | 0.819 | 0.612 | 0.490 | 5.5% | 10.7% | 0.489 | 6.4 | 14.9 | 0.702 | 0.704 | 1858 | 0.38 | – | 2.0% | 2.1 |
| clustering: adaptive Euclidean | 0.691 | 0.848 | 0.618 | 0.510 | 9.9% | 8.3% | 0.500 | 5.7 | 15.2 | 0.706 | 0.664 | 1832 | 0.32 | – | 4.8% | 1.1 |
| boxes: axis-aligned | 0.665 | 0.816 | 0.637 | 0.515 | 5.2% | 10.6% | 0.515 | 5.5 | 15.2 | 0.729 | 0.713 | 1677 | 0.34 | – | 4.8% | 1.9 |
| boxes: PCA | 0.656 | 0.816 | 0.611 | 0.438 | 5.3% | 9.1% | 0.487 | 10.4 | 13.7 | 0.726 | 0.709 | 1740 | 0.34 | – | 4.8% | 1.9 |
| tracking cost: centre distance | 0.660 | 0.818 | 0.617 | 0.504 | 5.3% | 9.6% | 0.513 | 5.9 | 14.7 | 0.707 | 0.707 | 1917 | 0.35 | – | 4.8% | 1.9 |
| no tracking | 0.652 | 0.821 | 0.588 | 0.479 | 5.3% | 9.0% | 0.495 | 6.5 | 92.4 | 0.085 | 0.028 | 37969 | – | – | 4.8% | 1.9 |

- **Heading from motion** cuts the heading error of moving vehicles from 78° to 14° (AV2) and from
  100° to 16° (nuScenes), at no cost elsewhere. Stationary or unstably tracked objects keep the
  L-shape yaw.
- **Shape completion** improves BEV IoU (+0.017 AV2, +0.046 nuScenes) and recall@IoU.3 (+0.03 /
  +0.05), at the cost of about 4 % more ID switches on AV2.
- **The height-gap split and the oversized-cluster split** each add about 0.025 recall@1m on AV2. The
  height gap also removes 120 ID switches. The oversized split makes more objects to track, so it
  adds about 140 switches. `object_max_length` between 12 and 20 m makes little difference.
- **Ground**: on flat nuScenes, constrained RANSAC is on par with Patchwork++. On AV2's slopes
  Patchwork++ is decisive (recall@1m 0.45 → 0.66), and the refinement step helps on both datasets.
- **Boxes**: L-shape halves the vehicle yaw error on nuScenes compared with axis-aligned boxes
  (4.0° vs 7.1°). On AV2, where most vehicles are lane-aligned, axis-aligned boxes are slightly
  better (5.5° vs 5.9°). PCA is the worst everywhere.
- **GIoU association** (the default) has 9–10 % fewer ID switches than centre distance.

## Parameter sensitivity

These one-at-a-time changes from the preset defaults also used 12 parallel jobs, so runtimes
are not reported.

### nuscenes

| Pipeline | Recall@1m | Recall@2m | Recall@IoU.3 | Recall@1m >7m | Over-seg | Under-seg | BEV IoU | Yaw err [deg] | Moving heading err [deg] | IDF1* | MOTA* | ID sw. | Vel. err [m/s] | Ground IoU | Obj. pts lost | Structures / frame |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `accurate`: preset defaults | 0.408 | 0.547 | 0.344 | 0.339 | 4.8% | 7.9% | 0.408 | 4.1 | 18.5 | 0.568 | 0.418 | 390 | 0.41 | 0.921 | 6.4% | 0.3 |
| `accurate`: cluster_range_gain=0 | 0.407 | 0.546 | 0.343 | 0.334 | 4.8% | 7.9% | 0.407 | 4.1 | 18.2 | 0.568 | 0.418 | 387 | 0.41 | 0.921 | 6.4% | 0.3 |
| `accurate`: cluster_range_gain=0.04 | 0.440 | 0.580 | 0.382 | 0.420 | 3.8% | 9.4% | 0.425 | 4.1 | 15.2 | 0.594 | 0.455 | 393 | 0.41 | 0.921 | 6.4% | 0.5 |
| `accurate`: cluster_min_size=3 | 0.607 | 0.849 | 0.362 | 0.392 | 23.7% | 8.0% | 0.288 | 6.4 | 22.9 | 0.565 | 0.314 | 973 | 0.39 | 0.921 | 6.4% | 0.3 |
| `accurate`: cluster_min_size=5 | 0.538 | 0.756 | 0.364 | 0.368 | 12.9% | 8.0% | 0.327 | 5.3 | 19.5 | 0.613 | 0.451 | 677 | 0.38 | 0.921 | 6.4% | 0.3 |
| `accurate`: cluster_threshold=0.4 | 0.379 | 0.507 | 0.284 | 0.287 | 11.2% | 3.5% | 0.357 | 4.0 | 22.6 | 0.506 | 0.289 | 414 | 0.40 | 0.921 | 6.4% | 0.2 |
| `accurate`: cluster_threshold=0.8 | 0.419 | 0.549 | 0.362 | 0.399 | 3.2% | 12.7% | 0.432 | 3.9 | 16.4 | 0.579 | 0.436 | 371 | 0.40 | 0.921 | 6.4% | 0.2 |
| `accurate`: voxel_grid_size=0.1 | 0.439 | 0.596 | 0.361 | 0.355 | 6.0% | 8.4% | 0.391 | 4.2 | 18.2 | 0.585 | 0.433 | 437 | 0.38 | 0.918 | 8.5% | 0.3 |
| `fast`: preset defaults | 0.415 | 0.548 | 0.357 | 0.384 | 3.4% | 11.1% | 0.426 | 4.0 | 15.9 | 0.585 | 0.442 | 363 | 0.41 | 0.921 | 6.4% | 0.6 |
| `fast`: cluster_cell_size=0.15 | 0.414 | 0.544 | 0.361 | 0.423 | 2.7% | 13.2% | 0.435 | 4.0 | 14.4 | 0.585 | 0.454 | 361 | 0.42 | 0.921 | 6.4% | 0.6 |
| `fast`: cluster_cell_size=0.35 | 0.412 | 0.555 | 0.346 | 0.358 | 3.8% | 8.8% | 0.407 | 4.1 | 17.2 | 0.585 | 0.446 | 393 | 0.41 | 0.921 | 6.4% | 0.8 |
| `fast`: cluster_range_gain=0 | 0.415 | 0.547 | 0.356 | 0.381 | 3.4% | 11.1% | 0.426 | 4.0 | 16.2 | 0.584 | 0.442 | 360 | 0.40 | 0.921 | 6.4% | 0.6 |
| `fast`: cluster_range_gain=0.04 | 0.436 | 0.569 | 0.378 | 0.441 | 2.9% | 12.3% | 0.436 | 4.1 | 15.6 | 0.600 | 0.464 | 388 | 0.42 | 0.921 | 6.4% | 0.8 |
| `fast`: cluster_min_size=3 | 0.589 | 0.835 | 0.374 | 0.415 | 16.4% | 11.4% | 0.310 | 6.3 | 18.6 | 0.634 | 0.476 | 811 | 0.39 | 0.921 | 6.4% | 0.5 |
| `fast`: cluster_min_size=5 | 0.534 | 0.754 | 0.377 | 0.394 | 8.7% | 11.5% | 0.345 | 5.3 | 17.9 | 0.648 | 0.533 | 637 | 0.39 | 0.921 | 6.4% | 0.5 |
| `fast`: ground_refine_threshold=0.15 | 0.411 | 0.549 | 0.354 | 0.376 | 3.5% | 11.3% | 0.424 | 4.1 | 15.6 | 0.581 | 0.439 | 373 | 0.42 | 0.921 | 4.9% | 0.6 |
| `fast`: ground_refine_threshold=0.3 | 0.410 | 0.541 | 0.349 | 0.381 | 3.4% | 10.6% | 0.422 | 4.0 | 16.8 | 0.580 | 0.435 | 365 | 0.41 | 0.908 | 10.1% | 0.5 |
| `fast`: cluster_threshold=0.4 | 0.407 | 0.554 | 0.326 | 0.329 | 6.6% | 5.5% | 0.379 | 4.3 | 18.4 | 0.566 | 0.401 | 410 | 0.42 | 0.921 | 6.4% | 0.4 |
| `fast`: cluster_threshold=0.8 | 0.402 | 0.534 | 0.353 | 0.405 | 2.7% | 17.1% | 0.434 | 4.2 | 15.8 | 0.575 | 0.436 | 361 | 0.45 | 0.921 | 6.4% | 0.5 |
| `fast`: voxel_grid_size=0.1 | 0.435 | 0.590 | 0.368 | 0.394 | 4.1% | 11.8% | 0.407 | 4.4 | 15.7 | 0.597 | 0.463 | 419 | 0.39 | 0.918 | 8.5% | 0.6 |
| `fast`: cluster_min_size=5, cluster_range_gain=0.04 | 0.550 | 0.764 | 0.406 | 0.449 | 7.6% | 12.7% | 0.363 | 5.2 | 17.0 | 0.655 | 0.552 | 674 | 0.43 | 0.921 | 6.4% | 0.7 |
| `accurate`: cluster_min_size=5, cluster_range_gain=0.04 | 0.572 | 0.791 | 0.408 | 0.439 | 9.9% | 9.7% | 0.348 | 5.3 | 18.7 | 0.648 | 0.521 | 626 | 0.39 | 0.921 | 6.4% | 0.4 |

### av2

| Pipeline | Recall@1m | Recall@2m | Recall@IoU.3 | Recall@1m >7m | Over-seg | Under-seg | BEV IoU | Yaw err [deg] | Moving heading err [deg] | IDF1* | MOTA* | ID sw. | Vel. err [m/s] | Ground IoU | Obj. pts lost | Structures / frame |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `accurate`: preset defaults | 0.697 | 0.849 | 0.628 | 0.510 | 9.4% | 8.6% | 0.508 | 5.7 | 14.4 | 0.710 | 0.676 | 1749 | 0.32 | – | 4.8% | 1.2 |
| `accurate`: cluster_range_gain=0 | 0.691 | 0.848 | 0.618 | 0.510 | 9.9% | 8.3% | 0.500 | 5.7 | 15.2 | 0.706 | 0.664 | 1832 | 0.32 | – | 4.8% | 1.1 |
| `accurate`: cluster_range_gain=0.04 | 0.675 | 0.817 | 0.633 | 0.614 | 5.0% | 13.2% | 0.526 | 5.6 | 12.7 | 0.716 | 0.697 | 1744 | 0.34 | – | 4.8% | 2.0 |
| `accurate`: cluster_min_size=3 | 0.765 | 0.925 | 0.630 | 0.546 | 27.8% | 8.5% | 0.461 | 6.8 | 17.3 | 0.599 | 0.415 | 3104 | 0.37 | – | 4.8% | 1.2 |
| `accurate`: cluster_min_size=5 | 0.741 | 0.902 | 0.631 | 0.529 | 18.9% | 8.5% | 0.478 | 6.3 | 15.7 | 0.662 | 0.571 | 2458 | 0.34 | – | 4.8% | 1.2 |
| `accurate`: cluster_threshold=0.4 | 0.710 | 0.863 | 0.626 | 0.447 | 16.8% | 6.7% | 0.493 | 5.8 | 16.3 | 0.640 | 0.519 | 2165 | 0.34 | – | 4.8% | 0.4 |
| `accurate`: cluster_threshold=0.8 | 0.690 | 0.847 | 0.632 | 0.521 | 6.6% | 9.8% | 0.506 | 5.8 | 13.2 | 0.694 | 0.643 | 1970 | 0.34 | – | 4.8% | 0.3 |
| `accurate`: voxel_grid_size=0.1 | 0.711 | 0.868 | 0.646 | 0.518 | 11.6% | 8.7% | 0.505 | 6.1 | 14.3 | 0.699 | 0.655 | 1862 | 0.32 | – | 4.0% | 1.4 |
| `fast`: preset defaults | 0.661 | 0.818 | 0.617 | 0.543 | 5.3% | 9.7% | 0.512 | 5.9 | 13.9 | 0.725 | 0.709 | 1725 | 0.35 | – | 4.8% | 1.9 |
| `fast`: cluster_cell_size=0.15 | 0.660 | 0.814 | 0.617 | 0.523 | 4.7% | 10.2% | 0.514 | 5.9 | 13.2 | 0.725 | 0.714 | 1631 | 0.35 | – | 4.8% | 2.0 |
| `fast`: cluster_cell_size=0.35 | 0.662 | 0.816 | 0.613 | 0.534 | 6.6% | 8.3% | 0.514 | 5.8 | 14.8 | 0.715 | 0.690 | 1869 | 0.34 | – | 4.8% | 2.2 |
| `fast`: cluster_range_gain=0 | 0.660 | 0.819 | 0.613 | 0.540 | 5.6% | 9.4% | 0.510 | 5.9 | 14.0 | 0.725 | 0.707 | 1712 | 0.34 | – | 4.8% | 1.9 |
| `fast`: cluster_range_gain=0.04 | 0.626 | 0.777 | 0.604 | 0.565 | 3.0% | 14.5% | 0.518 | 5.9 | 12.4 | 0.711 | 0.693 | 1805 | 0.36 | – | 4.8% | 2.7 |
| `fast`: cluster_min_size=3 | 0.714 | 0.890 | 0.620 | 0.578 | 17.1% | 9.6% | 0.472 | 6.7 | 15.9 | 0.667 | 0.598 | 2767 | 0.38 | – | 4.8% | 1.9 |
| `fast`: cluster_min_size=5 | 0.697 | 0.866 | 0.620 | 0.545 | 10.9% | 9.6% | 0.487 | 6.4 | 15.0 | 0.706 | 0.671 | 2238 | 0.36 | – | 4.8% | 1.9 |
| `fast`: ground_refine_threshold=0.15 | 0.663 | 0.820 | 0.622 | 0.546 | 5.4% | 9.8% | 0.514 | 5.8 | 14.0 | 0.724 | 0.709 | 1735 | 0.34 | – | 4.1% | 2.0 |
| `fast`: ground_refine_threshold=0.3 | 0.654 | 0.809 | 0.608 | 0.539 | 5.1% | 9.3% | 0.510 | 5.9 | 13.9 | 0.722 | 0.703 | 1690 | 0.34 | – | 6.8% | 1.9 |
| `fast`: cluster_threshold=0.4 | 0.686 | 0.840 | 0.626 | 0.531 | 9.6% | 8.2% | 0.511 | 5.8 | 15.4 | 0.707 | 0.668 | 1946 | 0.34 | – | 4.8% | 1.6 |
| `fast`: cluster_threshold=0.8 | 0.645 | 0.805 | 0.603 | 0.547 | 3.6% | 13.6% | 0.504 | 5.9 | 12.3 | 0.717 | 0.709 | 1939 | 0.37 | – | 4.8% | 1.6 |
| `fast`: voxel_grid_size=0.1 | 0.670 | 0.831 | 0.628 | 0.528 | 6.6% | 10.0% | 0.508 | 6.3 | 14.6 | 0.721 | 0.705 | 1780 | 0.34 | – | 4.0% | 2.1 |
| `fast`: cluster_min_size=5, cluster_range_gain=0.04 | 0.652 | 0.816 | 0.609 | 0.569 | 5.8% | 14.3% | 0.501 | 6.3 | 13.1 | 0.704 | 0.684 | 2166 | 0.38 | – | 4.8% | 2.6 |
| `accurate`: cluster_min_size=5, cluster_range_gain=0.04 | 0.705 | 0.859 | 0.639 | 0.622 | 9.4% | 13.1% | 0.505 | 6.0 | 13.7 | 0.695 | 0.656 | 2139 | 0.35 | – | 4.8% | 2.0 |

**Recommendations.** For sparse lidars (≤ 32 beams), `cluster_min_size: 5` and
`cluster_range_gain: 0.04` give large recall gains on nuScenes (see the `sparse` rows). On the
denser AV2 setup they cause more merging, so keep the defaults there. `ground_refine_threshold` between 0.15 and 0.3 m makes little
difference.

## Learned detector (CUDA-CenterPoint, GPU)

`detector_method: centerpoint` runs NVIDIA's CUDA-CenterPoint. `hybrid` adds the clusters that it
does not cover. All rows use single sweeps on an RTX 3090 Ti and the same evaluation as above,
plus label accuracy: the share of matched objects whose predicted class agrees with the ground
truth's class.

The model was trained on nuScenes, so nuScenes is scored only on the two mini **val** scenes;
the other mini scenes belong to its training split. Argoverse 2 is a different lidar setup and
city that the network has never seen.

### Argoverse 2

| Pipeline | Recall@1m | Recall@2m | Recall@IoU.3 | Recall@1m >7m | Over-seg | Under-seg | BEV IoU | Yaw err [deg] | Moving heading err [deg] | IDF1* | MOTA* | ID sw. | Vel. err [m/s] | Ground IoU | Obj. pts lost | Structures / frame | Label acc. | Runtime mean / p95 [ms] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| clustering (`fast`) | 0.661 | 0.818 | 0.617 | 0.543 | 5.3% | 9.7% | 0.512 | 5.9 | 13.9 | 0.725 | 0.709 | 1725 | 0.35 | – | 4.8% | 1.9 | – | 7.7 / 9.3 |
| `centerpoint` | 0.876 | 0.904 | 0.871 | 0.767 | 5.3% | 2.0% | 0.695 | 2.8 | 4.3 | 0.822 | 0.787 | 1534 | 0.16 | – | 4.8% | 0.0 | 98.4% | 15.1 / 16.6 |
| `hybrid` | 0.905 | 0.950 | 0.890 | 0.777 | 6.1% | 4.1% | 0.675 | 3.1 | 4.6 | 0.812 | 0.778 | 1804 | 0.18 | – | 4.8% | 1.9 | 97.5% | 17.3 / 19.8 |

### nuScenes val (scene-0103, scene-0916)

| Pipeline | Recall@1m | Recall@2m | Recall@IoU.3 | Recall@1m >7m | Over-seg | Under-seg | BEV IoU | Yaw err [deg] | Moving heading err [deg] | IDF1* | MOTA* | ID sw. | Vel. err [m/s] | Ground IoU | Obj. pts lost | Structures / frame | Label acc. | Runtime mean / p95 [ms] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| clustering (`fast`) | 0.378 | 0.508 | 0.320 | 0.733 | 1.6% | 16.6% | 0.429 | 4.0 | 19.0 | 0.550 | 0.426 | 135 | 0.45 | 0.928 | 5.3% | 0.4 | – | 2.8 / 3.5 |
| `centerpoint` | 0.926 | 0.957 | 0.911 | 1.000 | 10.5% | 1.0% | 0.696 | 3.7 | 5.1 | 0.803 | 0.675 | 146 | 0.17 | 0.928 | 5.3% | 0.0 | 97.9% | 10.1 / 11.3 |
| `hybrid` | 0.930 | 0.966 | 0.911 | 1.000 | 10.5% | 2.2% | 0.691 | 3.7 | 5.1 | 0.797 | 0.651 | 163 | 0.18 | 0.928 | 5.3% | 0.4 | 97.9% | 11.5 / 14.0 |


- **Detection quality**: CenterPoint far exceeds model-free clustering on the classes it knows.
  Recall@1m is 0.93 vs 0.38 on nuScenes val and 0.88 vs 0.66 on AV2, and BEV IoU is about 0.70 vs
  0.43–0.51. Velocity error is 0.16–0.17 m/s.
- **Speed**: it is slower, 10–15 ms per scan in total of which about 9 ms is in the network,
  against 3–8 ms for clustering on the CPU.
- **Hybrid** keeps clusters that the network does not explain: around 70 extra unknown obstacles
  per AV2 frame, such as poles, walls and debris. That buys a little more recall (0.91 on AV2) for
  slightly more ID switches.
- **Class-agnostic NMS** across CenterPoint's class heads and **flipping learned headings to match
  the direction of travel** were both necessary:
  - ID switches on AV2 dropped from 2,655 to 1,534.
  - Moving-vehicle heading error dropped from 17° to 4–5°.

## Reproduce

```bash
pixi run -e core build
pixi run -e core python -I tools/python/lod_bench/prepare_av2.py --root ~/datasets/av2/sensor/val --out ~/datasets/lod_bench/av2
pixi run -e core python -I tools/python/lod_bench/prepare_nuscenes.py --root ~/datasets/nuscenes --out ~/datasets/lod_bench/nuscenes
# one row: --pipeline legacy for v1, --preset legacy|fast|accurate for v2, --set key=value to override
pixi run -e core python -I tools/python/lod_bench/run.py --data ~/datasets/lod_bench/av2 \
    --runs ~/datasets/lod_bench/runs --name fast --preset fast --jobs 1
pixi run -e core python -I tools/python/lod_bench/report.py --runs ~/datasets/lod_bench/runs --names fast
```

The AV2 logs are fetched without an account (about 150 MB of lidar per log):
`s5cmd --no-sign-request cp "s3://argoverse/datasets/av2/sensor/val/<log>/sensors/lidar/*" ...`
plus `annotations.feather`, `city_SE3_egovehicle.feather` and `calibration/`. The dataset is
licensed CC BY-NC-SA 4.0. nuScenes requires a free account.
