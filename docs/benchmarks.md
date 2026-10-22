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

| Pipeline | Recall@1m | Recall@2m | Recall@IoU.3 | Over-seg | Under-seg | BEV IoU | Yaw err [deg] | IDF1* | MOTA* | ID sw. | Vel. err [m/s] | Ground IoU | Obj. pts lost | Runtime mean / p95 [ms] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| v1 (AABB boxes) | 0.376 | 0.558 | 0.308 | 5.3% | 9.1% | 0.348 | 9.0 | 0.351 | 0.268 | 1476 | – | 0.907 | 5.5% | 6.6 / 8.3 |
| v1 (PCA boxes) | 0.372 | 0.553 | 0.270 | 5.3% | 8.8% | 0.307 | 12.1 | 0.354 | 0.269 | 1460 | – | 0.907 | 5.5% | 6.6 / 8.3 |
| v2 `legacy` | 0.375 | 0.555 | 0.305 | 5.2% | 9.2% | 0.348 | 9.0 | 0.556 | 0.417 | 455 | 0.45 | 0.908 | 5.4% | 6.6 / 8.5 |
| v2 `fast` | 0.359 | 0.513 | 0.293 | 1.9% | 13.9% | 0.391 | 4.0 | 0.558 | 0.426 | 394 | 0.43 | 0.921 | 6.4% | 2.2 / 2.8 |
| v2 `accurate` | 0.379 | 0.535 | 0.295 | 4.4% | 7.9% | 0.365 | 4.1 | 0.552 | 0.415 | 416 | 0.41 | 0.921 | 6.4% | 5.1 / 7.1 |

### av2

| Pipeline | Recall@1m | Recall@2m | Recall@IoU.3 | Over-seg | Under-seg | BEV IoU | Yaw err [deg] | IDF1* | MOTA* | ID sw. | Vel. err [m/s] | Ground IoU | Obj. pts lost | Runtime mean / p95 [ms] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| v1 (AABB boxes) | 0.534 | 0.743 | 0.423 | 15.9% | 28.2% | 0.370 | 6.2 | 0.204 | 0.293 | 11822 | – | – | 25.4% | 27.9 / 36.1 |
| v1 (PCA boxes) | 0.531 | 0.744 | 0.400 | 16.1% | 26.2% | 0.344 | 14.5 | 0.200 | 0.286 | 12223 | – | – | 25.4% | 27.8 / 35.1 |
| v2 `legacy` | 0.531 | 0.742 | 0.420 | 15.9% | 28.1% | 0.368 | 6.2 | 0.484 | 0.416 | 4987 | 0.58 | – | 26.3% | 27.9 / 35.9 |
| v2 `fast` | 0.603 | 0.773 | 0.555 | 4.2% | 16.5% | 0.493 | 6.7 | 0.691 | 0.686 | 1760 | 0.36 | – | 4.8% | 6.1 / 7.6 |
| v2 `accurate` | 0.659 | 0.825 | 0.581 | 8.5% | 12.2% | 0.489 | 6.4 | 0.688 | 0.673 | 1736 | 0.32 | – | 4.8% | 24.0 / 34.5 |


**Takeaways**

- **Tracking**: the new tracker alone (v1 → v2 `legacy`) raises IDF1\* from 0.35 to 0.56
  (nuScenes) and from 0.20 to 0.48 (AV2), and cuts ID switches by 58–69 %.
- **Ground segmentation** is the largest single gain on AV2's hilly Pittsburgh logs: RANSAC's
  single plane labels 25 % of object points as ground; Patchwork++ with refinement, under 5 %.
- **Speed**: `fast` is 3.0× (nuScenes) to 4.6× (AV2) faster than v1, mostly from the BEV-grid
  clusterer (0.15–0.4 ms instead of 3.5–18 ms).
- **nuScenes recall barely moves with the default presets.** Almost half of its objects have
  fewer than 20 points, so `cluster_min_size` is the limit there; see the sensitivity table
  below and the sparse-lidar rows in the README (recall@1m up to 0.53).

## Ablations

Each row replaces one component of `fast` with an alternative. These runs used 12 parallel
jobs, so runtimes are not reported.

### nuscenes

| Pipeline | Recall@1m | Recall@2m | Recall@IoU.3 | Over-seg | Under-seg | BEV IoU | Yaw err [deg] | IDF1* | MOTA* | ID sw. | Vel. err [m/s] | Ground IoU | Obj. pts lost |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `fast` (Patchwork++ · refine · BEV grid · L-shape · KF, centre cost) | 0.359 | 0.513 | 0.293 | 1.9% | 13.9% | 0.391 | 4.0 | 0.558 | 0.426 | 394 | 0.43 | 0.921 | 6.4% |
| ground: constrained RANSAC, no refine | 0.362 | 0.532 | 0.296 | 2.1% | 16.3% | 0.380 | 4.4 | 0.556 | 0.437 | 450 | 0.48 | 0.914 | 5.6% |
| ground: v1 RANSAC (30 it., unconstrained), no refine | 0.359 | 0.530 | 0.294 | 2.1% | 16.5% | 0.379 | 4.5 | 0.553 | 0.434 | 458 | 0.49 | 0.908 | 5.4% |
| no elevation-map refinement | 0.344 | 0.522 | 0.288 | 2.2% | 16.1% | 0.374 | 5.1 | 0.521 | 0.414 | 518 | 0.55 | 0.889 | 2.8% |
| clustering: adaptive Euclidean | 0.378 | 0.534 | 0.294 | 4.4% | 7.9% | 0.365 | 4.1 | 0.552 | 0.414 | 417 | 0.41 | 0.921 | 6.4% |
| boxes: axis-aligned | 0.352 | 0.512 | 0.303 | 1.9% | 14.0% | 0.380 | 9.0 | 0.553 | 0.423 | 393 | 0.44 | 0.921 | 6.4% |
| boxes: PCA | 0.349 | 0.509 | 0.275 | 1.9% | 13.7% | 0.348 | 12.1 | 0.551 | 0.425 | 393 | 0.43 | 0.921 | 6.4% |
| tracking cost: BEV GIoU | 0.359 | 0.513 | 0.293 | 1.9% | 13.9% | 0.391 | 4.0 | 0.555 | 0.430 | 372 | 0.43 | 0.921 | 6.4% |
| no tracking | 0.359 | 0.513 | 0.293 | 1.9% | 13.9% | 0.391 | 4.0 | 0.159 | 0.122 | 2428 | – | 0.921 | 6.4% |

### av2

| Pipeline | Recall@1m | Recall@2m | Recall@IoU.3 | Over-seg | Under-seg | BEV IoU | Yaw err [deg] | IDF1* | MOTA* | ID sw. | Vel. err [m/s] | Ground IoU | Obj. pts lost |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `fast` (Patchwork++ · refine · BEV grid · L-shape · KF, centre cost) | 0.603 | 0.773 | 0.555 | 4.2% | 16.5% | 0.493 | 6.7 | 0.691 | 0.686 | 1760 | 0.36 | – | 4.8% |
| ground: constrained RANSAC, no refine | 0.370 | 0.543 | 0.323 | 3.4% | 31.7% | 0.402 | 9.5 | 0.467 | 0.436 | 3372 | 0.60 | – | 25.4% |
| ground: v1 RANSAC (30 it., unconstrained), no refine | 0.322 | 0.496 | 0.275 | 3.2% | 35.1% | 0.375 | 10.2 | 0.409 | 0.377 | 4143 | 0.73 | – | 26.3% |
| no elevation-map refinement | 0.583 | 0.765 | 0.546 | 4.0% | 18.7% | 0.484 | 7.1 | 0.665 | 0.674 | 1931 | 0.40 | – | 2.0% |
| clustering: adaptive Euclidean | 0.653 | 0.824 | 0.569 | 9.1% | 11.9% | 0.480 | 6.4 | 0.682 | 0.662 | 1834 | 0.32 | – | 4.8% |
| boxes: axis-aligned | 0.599 | 0.772 | 0.567 | 4.2% | 17.4% | 0.494 | 5.8 | 0.695 | 0.686 | 1738 | 0.35 | – | 4.8% |
| boxes: PCA | 0.597 | 0.772 | 0.551 | 4.2% | 17.2% | 0.460 | 12.1 | 0.692 | 0.686 | 1739 | 0.35 | – | 4.8% |
| tracking cost: BEV GIoU | 0.603 | 0.773 | 0.555 | 4.2% | 16.5% | 0.493 | 6.7 | 0.709 | 0.688 | 1630 | 0.35 | – | 4.8% |
| no tracking | 0.603 | 0.773 | 0.555 | 4.2% | 16.5% | 0.493 | 6.7 | 0.099 | 0.074 | 34147 | – | – | 4.8% |

- Patchwork++ and the refinement matter most on AV2. Constrained RANSAC beats the v1
  unconstrained RANSAC, but both lose about a quarter of the object points on slopes.
- Adaptive Euclidean clustering (`accurate`) has the best recall and under-segmentation, at
  3.5–18 ms instead of 0.15–0.4 ms.
- L-shape fitting halves the vehicle yaw error on nuScenes (9.0° → 4.0°). On AV2, where most
  vehicles are aligned with the ego lane, axis-aligned boxes are slightly better (5.8° vs 6.7°);
  PCA is the worst everywhere.
- GIoU association reduces ID switches by 6–7 % on both datasets. It is available as
  `track_cost: giou`.

## Parameter sensitivity

These one-at-a-time changes from the preset defaults also used 12 parallel jobs, so runtimes
are not reported.

### nuscenes

| Pipeline | Recall@1m | Recall@2m | Recall@IoU.3 | Over-seg | Under-seg | BEV IoU | Yaw err [deg] | IDF1* | MOTA* | ID sw. | Vel. err [m/s] | Ground IoU | Obj. pts lost |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `accurate`: preset defaults | 0.379 | 0.535 | 0.295 | 4.4% | 7.9% | 0.366 | 4.1 | 0.552 | 0.415 | 415 | 0.41 | 0.921 | 6.7% |
| `accurate`: cluster_range_gain=0 | 0.378 | 0.534 | 0.294 | 4.4% | 7.9% | 0.365 | 4.1 | 0.552 | 0.414 | 417 | 0.41 | 0.921 | 6.7% |
| `accurate`: cluster_range_gain=0.04 | 0.405 | 0.569 | 0.325 | 3.4% | 9.6% | 0.378 | 4.2 | 0.581 | 0.456 | 400 | 0.42 | 0.921 | 6.7% |
| `accurate`: cluster_min_size=3 | 0.579 | 0.833 | 0.310 | 23.3% | 8.1% | 0.255 | 6.7 | 0.569 | 0.346 | 1000 | 0.43 | 0.921 | 6.7% |
| `accurate`: cluster_min_size=5 | 0.504 | 0.739 | 0.308 | 12.3% | 8.1% | 0.287 | 5.6 | 0.587 | 0.461 | 744 | 0.40 | 0.921 | 6.7% |
| `accurate`: cluster_min_size=5, voxel_grid_size=0.1 | 0.523 | 0.755 | 0.320 | 14.1% | 8.7% | 0.290 | 5.6 | 0.586 | 0.439 | 801 | 0.40 | 0.918 | 9.4% |
| `accurate`: cluster_threshold=0.4 | 0.353 | 0.502 | 0.239 | 11.1% | 3.4% | 0.308 | 4.2 | 0.479 | 0.281 | 493 | 0.45 | 0.921 | 6.7% |
| `accurate`: cluster_threshold=0.8 | 0.377 | 0.525 | 0.311 | 2.3% | 13.7% | 0.394 | 4.0 | 0.571 | 0.442 | 344 | 0.40 | 0.921 | 6.7% |
| `accurate`: voxel_grid_size=0.1 | 0.405 | 0.582 | 0.309 | 5.5% | 8.6% | 0.352 | 4.2 | 0.575 | 0.443 | 435 | 0.38 | 0.918 | 9.4% |
| `fast`: preset defaults | 0.359 | 0.513 | 0.293 | 1.9% | 13.9% | 0.391 | 4.0 | 0.558 | 0.426 | 394 | 0.43 | 0.921 | 6.7% |
| `fast`: cluster_cell_size=0.15 | 0.361 | 0.508 | 0.300 | 1.5% | 15.2% | 0.400 | 4.0 | 0.562 | 0.432 | 365 | 0.44 | 0.921 | 6.7% |
| `fast`: cluster_cell_size=0.35 | 0.372 | 0.537 | 0.298 | 3.4% | 9.9% | 0.369 | 4.1 | 0.563 | 0.430 | 425 | 0.42 | 0.921 | 6.7% |
| `fast`: cluster_range_gain=0 | 0.358 | 0.512 | 0.293 | 1.9% | 13.9% | 0.391 | 4.0 | 0.558 | 0.426 | 389 | 0.43 | 0.921 | 6.7% |
| `fast`: cluster_range_gain=0.04 | 0.376 | 0.531 | 0.314 | 1.6% | 15.5% | 0.402 | 4.1 | 0.577 | 0.451 | 363 | 0.44 | 0.921 | 6.7% |
| `fast`: cluster_min_size=3 | 0.518 | 0.779 | 0.306 | 13.7% | 14.1% | 0.282 | 6.5 | 0.609 | 0.493 | 817 | 0.43 | 0.921 | 6.7% |
| `fast`: cluster_min_size=5 | 0.465 | 0.703 | 0.305 | 6.6% | 14.1% | 0.312 | 5.3 | 0.611 | 0.524 | 667 | 0.42 | 0.921 | 6.7% |
| `fast`: cluster_min_size=5, voxel_grid_size=0.1 | 0.473 | 0.710 | 0.310 | 8.0% | 15.3% | 0.312 | 5.5 | 0.612 | 0.521 | 666 | 0.42 | 0.918 | 9.4% |
| `fast`: ground_refine_threshold=0.15 | 0.358 | 0.513 | 0.291 | 1.9% | 14.1% | 0.390 | 4.1 | 0.553 | 0.427 | 387 | 0.44 | 0.921 | 5.1% |
| `fast`: ground_refine_threshold=0.3 | 0.356 | 0.508 | 0.289 | 1.9% | 13.2% | 0.387 | 3.8 | 0.554 | 0.427 | 385 | 0.44 | 0.909 | 10.6% |
| `fast`: cluster_threshold=0.4 | 0.374 | 0.541 | 0.272 | 5.6% | 6.3% | 0.334 | 4.3 | 0.533 | 0.387 | 496 | 0.42 | 0.921 | 6.7% |
| `fast`: cluster_threshold=0.8 | 0.340 | 0.478 | 0.285 | 0.7% | 21.9% | 0.411 | 4.1 | 0.547 | 0.421 | 320 | 0.47 | 0.921 | 6.7% |
| `fast`: voxel_grid_size=0.1 | 0.376 | 0.546 | 0.299 | 2.5% | 15.1% | 0.377 | 4.2 | 0.564 | 0.449 | 408 | 0.43 | 0.918 | 9.4% |
| `fast`: cluster_min_size=5, cluster_range_gain=0.04 | 0.475 | 0.707 | 0.327 | 5.5% | 15.7% | 0.330 | 5.3 | 0.623 | 0.545 | 647 | 0.45 | 0.921 | 6.4% |
| `accurate`: cluster_min_size=5, cluster_range_gain=0.04 | 0.533 | 0.774 | 0.342 | 9.3% | 9.9% | 0.306 | 5.6 | 0.625 | 0.530 | 697 | 0.42 | 0.921 | 6.4% |

### av2

| Pipeline | Recall@1m | Recall@2m | Recall@IoU.3 | Over-seg | Under-seg | BEV IoU | Yaw err [deg] | IDF1* | MOTA* | ID sw. | Vel. err [m/s] | Ground IoU | Obj. pts lost |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `accurate`: preset defaults | 0.659 | 0.825 | 0.581 | 8.5% | 12.2% | 0.489 | 6.4 | 0.688 | 0.673 | 1739 | 0.33 | – | 5.3% |
| `accurate`: cluster_range_gain=0 | 0.653 | 0.824 | 0.569 | 9.1% | 11.9% | 0.480 | 6.4 | 0.682 | 0.662 | 1834 | 0.32 | – | 5.3% |
| `accurate`: cluster_range_gain=0.04 | 0.628 | 0.763 | 0.584 | 3.6% | 20.3% | 0.527 | 6.2 | 0.692 | 0.679 | 1504 | 0.33 | – | 5.3% |
| `accurate`: cluster_min_size=3 | 0.733 | 0.900 | 0.582 | 27.2% | 12.2% | 0.442 | 7.5 | 0.598 | 0.455 | 2805 | 0.38 | – | 5.3% |
| `accurate`: cluster_min_size=5 | 0.706 | 0.876 | 0.582 | 18.0% | 12.2% | 0.459 | 7.0 | 0.652 | 0.584 | 2335 | 0.34 | – | 5.3% |
| `accurate`: cluster_min_size=5, voxel_grid_size=0.1 | 0.711 | 0.882 | 0.598 | 20.1% | 9.5% | 0.462 | 7.3 | 0.637 | 0.562 | 2322 | 0.35 | – | 4.2% |
| `accurate`: cluster_threshold=0.4 | 0.675 | 0.847 | 0.581 | 16.1% | 9.4% | 0.467 | 6.5 | 0.630 | 0.580 | 2161 | 0.35 | – | 5.3% |
| `accurate`: cluster_threshold=0.8 | 0.633 | 0.791 | 0.576 | 4.8% | 16.2% | 0.504 | 6.4 | 0.701 | 0.697 | 1532 | 0.34 | – | 5.3% |
| `accurate`: voxel_grid_size=0.1 | 0.673 | 0.841 | 0.598 | 10.7% | 9.5% | 0.488 | 6.7 | 0.675 | 0.658 | 1746 | 0.32 | – | 4.2% |
| `fast`: preset defaults | 0.603 | 0.773 | 0.555 | 4.2% | 16.5% | 0.493 | 6.7 | 0.691 | 0.686 | 1761 | 0.36 | – | 5.3% |
| `fast`: cluster_cell_size=0.15 | 0.593 | 0.761 | 0.548 | 3.5% | 17.8% | 0.496 | 6.7 | 0.691 | 0.687 | 1615 | 0.36 | – | 5.3% |
| `fast`: cluster_cell_size=0.35 | 0.619 | 0.797 | 0.564 | 6.0% | 14.3% | 0.486 | 6.7 | 0.687 | 0.684 | 1938 | 0.36 | – | 5.3% |
| `fast`: cluster_range_gain=0 | 0.602 | 0.775 | 0.551 | 4.4% | 16.2% | 0.490 | 6.7 | 0.691 | 0.685 | 1770 | 0.35 | – | 5.3% |
| `fast`: cluster_range_gain=0.04 | 0.545 | 0.683 | 0.522 | 1.7% | 27.6% | 0.519 | 6.5 | 0.655 | 0.630 | 1581 | 0.38 | – | 5.3% |
| `fast`: cluster_min_size=3 | 0.653 | 0.839 | 0.555 | 14.6% | 16.5% | 0.454 | 7.5 | 0.642 | 0.611 | 2568 | 0.39 | – | 5.3% |
| `fast`: cluster_min_size=5 | 0.634 | 0.817 | 0.555 | 9.0% | 16.5% | 0.468 | 7.2 | 0.669 | 0.664 | 2174 | 0.37 | – | 5.3% |
| `fast`: cluster_min_size=5, voxel_grid_size=0.1 | 0.634 | 0.817 | 0.565 | 10.1% | 12.5% | 0.470 | 7.6 | 0.666 | 0.654 | 2124 | 0.37 | – | 4.2% |
| `fast`: ground_refine_threshold=0.15 | 0.604 | 0.774 | 0.558 | 4.2% | 16.8% | 0.495 | 6.6 | 0.692 | 0.687 | 1753 | 0.36 | – | 4.6% |
| `fast`: ground_refine_threshold=0.3 | 0.596 | 0.767 | 0.547 | 4.1% | 15.9% | 0.489 | 6.7 | 0.687 | 0.681 | 1756 | 0.36 | – | 7.4% |
| `fast`: cluster_threshold=0.4 | 0.639 | 0.815 | 0.575 | 8.6% | 12.3% | 0.483 | 6.6 | 0.669 | 0.662 | 1958 | 0.36 | – | 5.3% |
| `fast`: cluster_threshold=0.8 | 0.544 | 0.694 | 0.509 | 1.8% | 25.1% | 0.503 | 6.6 | 0.662 | 0.641 | 1591 | 0.39 | – | 5.3% |
| `fast`: voxel_grid_size=0.1 | 0.609 | 0.781 | 0.564 | 5.0% | 12.5% | 0.492 | 7.1 | 0.686 | 0.685 | 1733 | 0.35 | – | 4.2% |
| `fast`: cluster_min_size=5, cluster_range_gain=0.04 | 0.561 | 0.712 | 0.523 | 3.3% | 27.6% | 0.501 | 6.8 | 0.658 | 0.637 | 1803 | 0.39 | – | 4.8% |
| `accurate`: cluster_min_size=5, cluster_range_gain=0.04 | 0.651 | 0.800 | 0.585 | 7.4% | 20.4% | 0.506 | 6.5 | 0.682 | 0.662 | 1746 | 0.35 | – | 4.8% |

**Recommendations.** For sparse lidars (≤ 32 beams), `cluster_min_size: 5` and
`cluster_range_gain: 0.04` give large recall gains (nuScenes recall@1m 0.36 → 0.48 with `fast`,
0.38 → 0.53 with `accurate`). On the denser AV2 setup they cause more merging (under-seg 16 → 28 %),
so keep the defaults there. `ground_refine_threshold` between 0.15 and 0.3 m makes little
difference.

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
