# Benchmarks

All numbers on this page come from `tools/` in this repository and can be reproduced with the
commands at the end.

## Setup

**Data.**

| | Argoverse 2 (sensor, val) | nuScenes (v1.0-mini) |
|---|---|---|
| Sequences | 15 logs (every 10th of the 150 val logs) | all 10 scenes; 2 **val** scenes for the GPU detector |
| Lidar | 2 × 32-beam (stacked), 10 Hz, ~95k points | 32-beam, 20 Hz, ~35k points |
| Scans processed | 2,350 (all annotated) | 3,931 (404 annotated keyframes) |
| Ground truth objects scored | 52,774 | 6,610 |

Both datasets are converted into one frame: x forward, y left, z up, origin at the (top) lidar.
Every run uses the same region of interest, ±40 m in x and y and -2.5 m to 1 m in z
(`tools/configs/bench_common.cfg`). All other parameters are the preset's defaults. The ego pose
is passed to the pipeline as `tracking_frame` would be on a robot.

**Ground truth.** Detection is scored class-agnostically against vehicles, pedestrians and
cyclists with at least 5 lidar points and a centre inside the ROI. Static classes such as cones
and barriers are not scored.

**Metrics.** The detector also (correctly) reports walls, poles and vegetation, which have no
labels. Plain precision and MOTA would mostly measure how much unlabelled structure a scene
contains, so we report:

- **Recall@1m / @2m**: share of GT objects with a predicted box centre within 1 m / 2 m in
  bird's-eye view (one-to-one Hungarian matching). **Recall@IoU.3**: matched with BEV IoU ≥ 0.3.
- **Coverage**: share of GT objects reported at all, by a box centre in the footprint (+0.5 m) or
  by at least one 2.5D pillar in it (+0.2 m). This is the safety-relevant number: an object that is
  covered is never driven into, even when it is not recognised.
- **Over-seg**: GT objects that contain ≥ 2 box centres. **Split >7m**: the same for vehicles
  longer than 7 m (trucks, buses). **Under-seg**: GT objects merged with another GT object into
  one box.
- **BEV IoU**, **yaw error** (vehicles, modulo 90°) and **moving heading error** (vehicles faster
  than 2 m/s, front and back count) of centre-matched boxes.
- **Box jitter**: frame-to-frame shape change of a track matched to the same GT object,
  1 − BEV IoU of the two boxes with their centres aligned (size and orientation flicker).
- **Pillar flicker**: share of static pillar cells (outside GT objects, within 25 m) that appear
  or disappear between consecutive annotated scans.
- **IDF1\* / MOTA\* / ID switches**: CLEAR / ID metrics (py-motmetrics, 2 m gate) after removing
  predictions more than 2 m from every scored GT object. nuScenes is scored on its 2 Hz
  keyframes, AV2 on every scan. **Velocity error**: median, in the world frame.
- **Ground IoU** against nuScenes lidarseg labels (`flat.*` = ground). **Obj. pts lost**: share
  of points inside scored GT boxes (≥ 0.2 m above the box bottom) labelled as ground.
- **Label acc.** (learned modes): share of matched objects whose class agrees with the GT class.
- **Runtime**: wall-clock per scan on an Intel i9-12900K, pinned to one core and fully
  single-threaded (), plus an RTX 3090 Ti for the learned modes. The
  workstation was shared during the measurements, so expect about ±10%. With OpenMP, L-shape box
  fitting drops from about 2.9 ms to 0.5 ms per AV2 scan.

## Clustering presets (CPU)

`legacy` is the classic pipeline (unconstrained RANSAC ground, fixed-radius Euclidean
clustering, axis-aligned boxes) on the current tracker. It has no pillars or occupancy, which is
why its coverage is low.

### Argoverse 2

| Pipeline | Recall@1m | Recall@2m | Recall@IoU.3 | Coverage | Recall@1m >7m | Over-seg | Split >7m | Under-seg | BEV IoU | Box jitter | Yaw err [deg] | Moving heading err [deg] | IDF1* | MOTA* | ID sw. | Vel. err [m/s] | Ground IoU | Obj. pts lost | Pillar flicker | Label acc. | Runtime mean / p95 [ms] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `legacy` (classic) | 0.531 | 0.742 | 0.420 | 72.7% | 0.356 | 15.9% | 63.3% | 28.1% | 0.368 | 0.327 | 6.2 | 90.1 | 0.506 | 0.422 | 4653 | 0.56 | – | 26.3% | – | – | 30.5 / 41.1 |
| `fast` | 0.661 | 0.818 | 0.617 | 99.5% | 0.543 | 5.3% | 31.7% | 9.7% | 0.512 | 0.217 | 5.9 | 13.9 | 0.725 | 0.709 | 1725 | 0.35 | – | 4.8% | 13.4% | – | 10.4 / 12.8 |
| `accurate` | 0.697 | 0.849 | 0.628 | 99.5% | 0.510 | 9.4% | 47.4% | 8.6% | 0.508 | 0.216 | 5.7 | 14.4 | 0.710 | 0.676 | 1749 | 0.32 | – | 4.8% | 13.4% | – | 32.2 / 49.1 |

### nuScenes

| Pipeline | Recall@1m | Recall@2m | Recall@IoU.3 | Coverage | Recall@1m >7m | Over-seg | Split >7m | Under-seg | BEV IoU | Box jitter | Yaw err [deg] | Moving heading err [deg] | IDF1* | MOTA* | ID sw. | Vel. err [m/s] | Ground IoU | Obj. pts lost | Pillar flicker | Label acc. | Runtime mean / p95 [ms] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `legacy` (classic) | 0.375 | 0.555 | 0.305 | 58.1% | 0.264 | 5.2% | 28.5% | 9.2% | 0.348 | 0.328 | 9.0 | 101.9 | 0.568 | 0.422 | 425 | 0.43 | 0.908 | 5.4% | – | – | 6.7 / 8.5 |
| `fast` | 0.415 | 0.548 | 0.357 | 98.3% | 0.384 | 3.4% | 14.1% | 11.1% | 0.426 | 0.292 | 4.0 | 15.9 | 0.585 | 0.442 | 363 | 0.41 | 0.921 | 6.4% | 36.9% | – | 3.4 / 3.9 |
| `accurate` | 0.408 | 0.547 | 0.344 | 98.3% | 0.339 | 4.8% | 25.3% | 7.9% | 0.408 | 0.300 | 4.1 | 18.5 | 0.568 | 0.418 | 390 | 0.41 | 0.921 | 6.4% | 36.9% | – | 6.8 / 9.7 |

- **Ground segmentation** is the largest single gain on AV2's hilly Pittsburgh logs. A single
  RANSAC plane labels about 25% of object points as ground; Patchwork++ with refinement, under 5%.
- **Coverage** of `fast` and `accurate` is 98–99.5%: almost every labelled object is either a box
  or occupied by pillars.
- **`accurate`** has the best recall@1m on AV2 but is 2–3× slower, almost all of it in
  KD-tree clustering.
- **nuScenes recall is limited by sparsity.** Almost half of its objects have fewer than 20
  points. The sparse-lidar tuning under [Parameter sensitivity](#parameter-sensitivity) raises
  recall@1m to 0.57.

## Detector modes (GPU)

`learning` runs NVIDIA's CUDA-CenterPoint, `fused` adds 2.5D pillars for everything it does not
recognise, and `clustering` is the CPU detector, timed on the same machine. The model was
trained on nuScenes, so nuScenes is scored only on the two mini **val** scenes (scene-0103 and
scene-0916); the other mini scenes belong to its training split. Argoverse 2 is a lidar setup
and city the network has never seen. The single-sweep rows ran in parallel with other jobs, so
their runtimes are not reported.

### Argoverse 2

| Pipeline | Recall@1m | Recall@2m | Recall@IoU.3 | Coverage | Recall@1m >7m | Over-seg | Split >7m | Under-seg | BEV IoU | Box jitter | Yaw err [deg] | Moving heading err [deg] | IDF1* | MOTA* | ID sw. | Vel. err [m/s] | Ground IoU | Obj. pts lost | Pillar flicker | Label acc. | Runtime mean / p95 [ms] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `clustering` (`fast`) | 0.661 | 0.818 | 0.617 | 99.5% | 0.543 | 5.3% | 31.7% | 9.7% | 0.512 | 0.217 | 5.9 | 13.9 | 0.725 | 0.709 | 1725 | 0.35 | – | 4.8% | 13.4% | – | 10.4 / 12.8 |
| `learning`, 1 sweep | 0.876 | 0.904 | 0.871 | 91.1% | 0.766 | 5.3% | 4.5% | 2.0% | 0.695 | 0.118 | 2.8 | 4.3 | 0.822 | 0.786 | 1535 | 0.16 | – | 4.8% | – | 98.4% | – |
| `learning`, 10 sweeps | 0.913 | 0.939 | 0.912 | 94.7% | 0.683 | 4.7% | 4.4% | 2.3% | 0.701 | 0.087 | 2.3 | 2.4 | 0.863 | 0.844 | 1024 | 0.15 | – | 4.8% | – | 99.0% | 23.7 / 27.0 |
| `fused` | 0.913 | 0.939 | 0.912 | 99.0% | 0.683 | 4.7% | 4.4% | 2.3% | 0.701 | 0.087 | 2.3 | 2.4 | 0.863 | 0.844 | 1034 | 0.15 | – | 4.8% | 14.8% | 99.0% | 25.5 / 29.1 |

### nuScenes val

| Pipeline | Recall@1m | Recall@2m | Recall@IoU.3 | Coverage | Recall@1m >7m | Over-seg | Split >7m | Under-seg | BEV IoU | Box jitter | Yaw err [deg] | Moving heading err [deg] | IDF1* | MOTA* | ID sw. | Vel. err [m/s] | Ground IoU | Obj. pts lost | Pillar flicker | Label acc. | Runtime mean / p95 [ms] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `clustering` (`fast`) | 0.378 | 0.508 | 0.320 | 99.1% | 0.733 | 1.6% | 13.3% | 16.6% | 0.429 | 0.309 | 4.0 | 19.0 | 0.550 | 0.426 | 135 | 0.45 | 0.928 | 5.3% | 41.4% | – | 3.5 / 4.6 |
| `learning`, 1 sweep | 0.926 | 0.957 | 0.911 | 95.2% | 1.000 | 10.5% | 10.0% | 1.0% | 0.696 | 0.181 | 3.7 | 5.1 | 0.800 | 0.675 | 144 | 0.17 | 0.928 | 5.3% | – | 97.9% | – |
| `learning`, 10 sweeps | 0.981 | 0.988 | 0.964 | 99.0% | 1.000 | 8.3% | 6.7% | 0.9% | 0.737 | 0.140 | 2.6 | 1.7 | 0.878 | 0.803 | 77 | 0.14 | 0.928 | 5.3% | – | 99.6% | 14.9 / 16.3 |
| `fused` | 0.981 | 0.988 | 0.964 | 99.8% | 1.000 | 8.3% | 6.7% | 0.9% | 0.737 | 0.140 | 2.6 | 1.7 | 0.877 | 0.801 | 82 | 0.14 | 0.928 | 5.3% | 41.6% | 99.6% | 16.0 / 17.9 |

- **Multi-sweep input** (10 ego-motion-compensated scans of the last 0.5 s, as in training) is a
  large gain:
  - recall@1m 0.88 → 0.91 on AV2 and 0.93 → 0.98 on nuScenes val
  - ID switches −33% / −47%
  - moving-vehicle heading error 4.3° → 2.4° and 5.1° → 1.7°

  The one loss is on articulated vehicles: a moving tractor and trailer are more often merged
  into one box, which lowers recall@1m on objects longer than 7 m on AV2.
- **`fused` = `learning` + coverage.** Boxes and tracks are identical. The pillars raise coverage
  from 94.7% to 99.0% on AV2 and from 99.0% to 99.8% on nuScenes val: the objects the network
  misses are still reported as occupied space.
- **CenterPoint vs clustering**: recall@1m 0.91 vs 0.66 on AV2, BEV IoU about 0.70 vs 0.51, and
  less than half the box jitter. On AV2 a scan takes about 15 ms more than clustering (about 20 ms
  of it in the network, with 10 sweeps).

## Stability and tuning

These runs used parallel jobs, so runtimes are not reported.

### Argoverse 2

| Pipeline | Recall@1m | Recall@2m | Recall@IoU.3 | Coverage | Recall@1m >7m | Over-seg | Split >7m | Under-seg | BEV IoU | Box jitter | Yaw err [deg] | Moving heading err [deg] | IDF1* | MOTA* | ID sw. | Vel. err [m/s] | Ground IoU | Obj. pts lost | Pillar flicker | Label acc. |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `fast`, pillar_memory=0 | 0.661 | 0.818 | 0.617 | 98.1% | 0.543 | 5.3% | 31.7% | 9.7% | 0.512 | 0.217 | 5.9 | 13.9 | 0.725 | 0.709 | 1725 | 0.35 | – | 4.8% | 37.3% | – |
| `fast` (pillar_memory=2) | 0.661 | 0.818 | 0.617 | 99.5% | 0.543 | 5.3% | 31.7% | 9.7% | 0.512 | 0.217 | 5.9 | 13.9 | 0.725 | 0.709 | 1725 | 0.35 | – | 4.8% | 13.4% | – |
| `fast`, track-guided, margin 0.3 m | 0.614 | 0.775 | 0.585 | 99.5% | 0.491 | 3.5% | 25.3% | 13.0% | 0.511 | 0.210 | 5.9 | 13.2 | 0.719 | 0.701 | 1392 | 0.35 | – | 4.8% | 13.4% | – |
| `fast`, track-guided (margin 0) | 0.648 | 0.807 | 0.612 | 99.5% | 0.501 | 4.3% | 24.4% | 10.5% | 0.514 | 0.213 | 5.8 | 13.4 | 0.730 | 0.716 | 1523 | 0.35 | – | 4.8% | 13.4% | – |
| `fused`, fused_margin=0.6 | 0.913 | 0.939 | 0.912 | 99.0% | 0.683 | 4.7% | 4.4% | 2.3% | 0.701 | 0.087 | 2.3 | 2.4 | 0.861 | 0.844 | 1043 | 0.15 | – | 4.8% | 14.8% | 99.0% |
| `fused`, fused_min_overlap=0.3 | 0.913 | 0.939 | 0.912 | 99.0% | 0.684 | 4.7% | 4.5% | 2.3% | 0.701 | 0.087 | 2.3 | 2.4 | 0.860 | 0.844 | 1037 | 0.15 | – | 4.8% | 14.9% | 99.0% |

### nuScenes

| Pipeline | Recall@1m | Recall@2m | Recall@IoU.3 | Coverage | Recall@1m >7m | Over-seg | Split >7m | Under-seg | BEV IoU | Box jitter | Yaw err [deg] | Moving heading err [deg] | IDF1* | MOTA* | ID sw. | Vel. err [m/s] | Ground IoU | Obj. pts lost | Pillar flicker | Label acc. |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `fast`, pillar_memory=0 | 0.415 | 0.548 | 0.357 | 95.8% | 0.384 | 3.4% | 14.1% | 11.1% | 0.426 | 0.292 | 4.0 | 15.9 | 0.585 | 0.442 | 363 | 0.41 | 0.921 | 6.4% | 57.0% | – |
| `fast` (pillar_memory=2) | 0.415 | 0.548 | 0.357 | 98.3% | 0.384 | 3.4% | 14.1% | 11.1% | 0.426 | 0.292 | 4.0 | 15.9 | 0.585 | 0.442 | 363 | 0.41 | 0.921 | 6.4% | 36.9% | – |
| `fast`, track-guided, margin 0.3 m | 0.374 | 0.506 | 0.332 | 98.3% | 0.332 | 2.3% | 12.3% | 14.2% | 0.430 | 0.285 | 4.0 | 16.5 | 0.565 | 0.424 | 320 | 0.43 | 0.921 | 6.4% | 36.9% | – |

- **Pillar fusion** (`pillar_memory: 2.0`, the default) cuts pillar flicker from 37% to 13% on
  AV2 and from 57% to 37% on nuScenes, and raises coverage (98.1% → 99.5% on AV2), with no effect on
  the boxes.
- **Track-guided clustering** (`cluster_track_guided`, off by default) uses the predicted
  footprints of tracked vehicles to re-join fragments. On AV2 it cuts split trucks from 32% to
  24% and ID switches by 12%, but costs 1.3 points of recall@1m, because it occasionally joins
  objects that really are separate. With a footprint margin of 0.3 m the side effects are larger.
  Turn it on for truck-heavy scenes.
- **`fused` thresholds** (`fused_margin`, `fused_min_overlap`) make almost no difference, so the
  defaults (0.3 m, 0.5) stay.

## Ablations

Each row replaces one component of `fast` with an alternative. These runs used 12 parallel
jobs, so runtimes are not reported.

### nuscenes

| Pipeline | Recall@1m | Recall@2m | Recall@IoU.3 | Recall@1m >7m | Over-seg | Under-seg | BEV IoU | Yaw err [deg] | Moving heading err [deg] | IDF1* | MOTA* | ID sw. | Vel. err [m/s] | Ground IoU | Obj. pts lost | Structures / frame |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `fast` (preset defaults) | 0.415 | 0.548 | 0.357 | 0.384 | 3.4% | 11.1% | 0.426 | 4.0 | 15.9 | 0.585 | 0.442 | 363 | 0.41 | 0.921 | 6.4% | 0.6 |
| no heading from motion | 0.415 | 0.548 | 0.355 | 0.376 | 3.4% | 11.1% | 0.424 | 4.2 | 100.4 | 0.583 | 0.441 | 366 | 0.41 | 0.921 | 6.4% | 0.6 |
| no shape completion | 0.385 | 0.542 | 0.307 | 0.334 | 3.3% | 10.3% | 0.380 | 4.1 | 16.1 | 0.570 | 0.434 | 380 | 0.40 | 0.921 | 6.4% | 0.6 |
| no height-gap split | 0.406 | 0.542 | 0.352 | 0.402 | 3.0% | 11.8% | 0.427 | 4.1 | 16.4 | 0.579 | 0.436 | 385 | 0.41 | 0.921 | 6.4% | 0.7 |
| no oversized-cluster split | 0.398 | 0.526 | 0.342 | 0.373 | 2.1% | 10.7% | 0.433 | 3.8 | 15.9 | 0.579 | 0.444 | 336 | 0.41 | 0.921 | 6.4% | 1.3 |
| structures kept as objects | 0.415 | 0.548 | 0.357 | 0.381 | 3.4% | 11.5% | 0.425 | 4.0 | 15.9 | 0.585 | 0.443 | 360 | 0.41 | 0.921 | 6.4% | 0.0 |
| object_max_length 12 m | 0.417 | 0.551 | 0.356 | 0.366 | 3.5% | 10.3% | 0.423 | 4.1 | 15.7 | 0.583 | 0.439 | 384 | 0.41 | 0.921 | 6.4% | 1.0 |
| object_max_length 20 m | 0.415 | 0.547 | 0.357 | 0.379 | 3.4% | 11.3% | 0.426 | 4.0 | 15.9 | 0.585 | 0.442 | 360 | 0.41 | 0.921 | 6.4% | 0.3 |
| ground: constrained RANSAC, no refine | 0.418 | 0.577 | 0.362 | 0.379 | 3.9% | 12.0% | 0.409 | 4.5 | 17.3 | 0.592 | 0.451 | 426 | 0.45 | 0.914 | 5.6% | 0.9 |
| ground: unconstrained RANSAC, no refine | 0.419 | 0.580 | 0.363 | 0.379 | 4.1% | 12.3% | 0.406 | 4.6 | 17.9 | 0.590 | 0.451 | 443 | 0.46 | 0.908 | 5.4% | 0.9 |
| no elevation-map refinement | 0.401 | 0.567 | 0.352 | 0.352 | 3.9% | 12.3% | 0.399 | 4.9 | 17.1 | 0.564 | 0.432 | 489 | 0.51 | 0.889 | 2.8% | 0.7 |
| clustering: adaptive Euclidean | 0.407 | 0.546 | 0.343 | 0.334 | 4.8% | 7.9% | 0.407 | 4.1 | 18.2 | 0.568 | 0.418 | 387 | 0.41 | 0.921 | 6.4% | 0.3 |
| boxes: axis-aligned | 0.412 | 0.546 | 0.370 | 0.355 | 3.4% | 11.6% | 0.420 | 7.1 | 17.9 | 0.584 | 0.438 | 391 | 0.40 | 0.921 | 6.4% | 0.6 |
| boxes: PCA | 0.396 | 0.543 | 0.326 | 0.360 | 3.4% | 11.3% | 0.379 | 10.5 | 14.4 | 0.579 | 0.433 | 369 | 0.40 | 0.921 | 6.4% | 0.6 |
| tracking cost: centre distance | 0.413 | 0.548 | 0.353 | 0.366 | 3.3% | 11.2% | 0.423 | 4.1 | 16.3 | 0.575 | 0.435 | 397 | 0.41 | 0.921 | 6.4% | 0.6 |
| no tracking | 0.385 | 0.542 | 0.307 | 0.334 | 3.3% | 10.4% | 0.380 | 4.2 | 107.5 | 0.154 | 0.079 | 2732 | – | 0.921 | 6.4% | 0.6 |

### av2

| Pipeline | Recall@1m | Recall@2m | Recall@IoU.3 | Recall@1m >7m | Over-seg | Under-seg | BEV IoU | Yaw err [deg] | Moving heading err [deg] | IDF1* | MOTA* | ID sw. | Vel. err [m/s] | Ground IoU | Obj. pts lost | Structures / frame |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `fast` (preset defaults) | 0.661 | 0.818 | 0.617 | 0.543 | 5.3% | 9.7% | 0.512 | 5.9 | 13.9 | 0.725 | 0.709 | 1725 | 0.35 | – | 4.8% | 1.9 |
| no heading from motion | 0.660 | 0.818 | 0.616 | 0.535 | 5.3% | 9.7% | 0.510 | 6.5 | 77.9 | 0.724 | 0.709 | 1742 | 0.35 | – | 4.8% | 1.9 |
| no shape completion | 0.652 | 0.821 | 0.588 | 0.479 | 5.3% | 9.0% | 0.495 | 5.9 | 14.3 | 0.724 | 0.712 | 1663 | 0.35 | – | 4.8% | 1.9 |
| no height-gap split | 0.636 | 0.801 | 0.599 | 0.529 | 4.7% | 9.8% | 0.506 | 6.1 | 14.1 | 0.718 | 0.704 | 1842 | 0.35 | – | 4.8% | 2.1 |
| no oversized-cluster split | 0.637 | 0.783 | 0.600 | 0.506 | 4.8% | 8.0% | 0.521 | 5.9 | 13.9 | 0.717 | 0.689 | 1586 | 0.34 | – | 4.8% | 3.2 |
| structures kept as objects | 0.662 | 0.821 | 0.618 | 0.542 | 5.4% | 13.2% | 0.511 | 5.9 | 13.9 | 0.727 | 0.710 | 1761 | 0.35 | – | 4.8% | 0.0 |
| object_max_length 12 m | 0.664 | 0.820 | 0.619 | 0.547 | 5.3% | 8.9% | 0.513 | 5.9 | 14.1 | 0.728 | 0.711 | 1717 | 0.35 | – | 4.8% | 2.6 |
| object_max_length 20 m | 0.661 | 0.818 | 0.618 | 0.530 | 5.3% | 10.0% | 0.512 | 5.9 | 13.8 | 0.725 | 0.708 | 1726 | 0.35 | – | 4.8% | 1.7 |
| ground: constrained RANSAC, no refine | 0.449 | 0.658 | 0.400 | 0.421 | 6.3% | 11.7% | 0.388 | 9.3 | 26.6 | 0.508 | 0.484 | 4420 | 0.62 | – | 25.4% | 4.0 |
| ground: unconstrained RANSAC, no refine | 0.409 | 0.633 | 0.358 | 0.354 | 6.3% | 12.3% | 0.359 | 10.1 | 32.1 | 0.459 | 0.441 | 5301 | 0.74 | – | 26.3% | 4.9 |
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

## Reproduce

```bash
pixi run -e core build                    # or: pixi run -e gpu gpu-build (learned modes)
pixi run -e core python -I tools/python/lod_bench/prepare_av2.py --root ~/datasets/av2/sensor/val --out ~/datasets/lod_bench/av2
pixi run -e core python -I tools/python/lod_bench/prepare_nuscenes.py --root ~/datasets/nuscenes --out ~/datasets/lod_bench/nuscenes
# one row: --preset legacy|fast|accurate, --set key=value to override (e.g. detector_method=fused)
pixi run -e core python -I tools/python/lod_bench/run.py --data ~/datasets/lod_bench/av2 \
    --runs ~/datasets/lod_bench/runs --name fast --preset fast --jobs 1
pixi run -e core python -I tools/python/lod_bench/report.py --runs ~/datasets/lod_bench/runs --names fast
```

The AV2 logs are fetched without an account (about 150 MB of lidar per log):
`s5cmd --no-sign-request cp "s3://argoverse/datasets/av2/sensor/val/<log>/sensors/lidar/*" ...`
plus `annotations.feather`, `city_SE3_egovehicle.feather` and `calibration/`. For the showcase
video, also fetch `map/` and `sensors/cameras/ring_front_center/`. The dataset is licensed
CC BY-NC-SA 4.0. nuScenes requires a free account.
