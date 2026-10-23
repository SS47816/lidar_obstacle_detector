# Contributing

Thanks for helping improve lidar_obstacle_detector! Bug reports, tuning tips for new sensors,
benchmarks and pull requests are all welcome.

## Getting started

The quickest way to a working build is [pixi](https://pixi.sh). It needs no system ROS and no
root access:

```bash
pixi run -e core test      # ROS-agnostic library + unit tests
pixi run -e jazzy test     # ROS 2 Jazzy package (colcon)
pixi run -e noetic build   # ROS 1 Noetic package (catkin)
pixi run -e gpu gpu-test   # with CUDA-CenterPoint (needs an NVIDIA GPU, driver >= 570)
```

## Layout

| Path | What lives there |
|---|---|
| `core/` | the ROS-agnostic C++ library: every algorithm, one parameter table (`core/src/config.cpp`) |
| `ros1/`, `ros2/` | thin ROS wrappers (topics, parameters, message conversion) |
| `test/` | GoogleTest unit tests, run in all three builds |
| `tools/` | benchmark CLI, dataset converters, evaluation and demo renderers |
| `third_party/` | vendored Patchwork++ (BSD-2) and NVIDIA CUDA-CenterPoint (MIT), see `VENDORED.md` |

A new parameter goes into `core/include/lidar_obstacle_detector/config.hpp`, the table in
`core/src/config.cpp` and `cfg/obstacle_detector.cfg` (ROS 1). ROS 2 picks it up from the table
automatically.

## Before opening a pull request

- `pre-commit run --all-files` (clang-format 19, ruff) and the unit tests pass.
- New behaviour comes with a unit test.
- If the change affects detection or tracking quality, run the benchmark on at least one dataset
  and put the before / after numbers in the PR. Tune on whole datasets, not single scenes.

## Reporting a problem with your data

A short bag (10-20 s) and your lidar model and mounting height let us reproduce most issues.
For questions about tuning, use [Discussions](https://github.com/SS47816/lidar_obstacle_detector/discussions).
