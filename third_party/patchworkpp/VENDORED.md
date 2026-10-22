# Vendored: Patchwork++

- Upstream: https://github.com/url-kaist/patchwork-plusplus
- Commit: 3e6903a1d5537a4cc2ace897b0bbb98a92d6014c (v1.4.1, 2026-05-23)
- License: BSD-2-Clause (see `LICENSE`), compatible with this package's MIT license.
- Files: `cpp/patchworkpp/{include,src}` and `cpp/common/{include,src}` only.

Vendored (instead of `FetchContent`) so that catkin/colcon/buildfarm builds work offline.
Upstream requests C++20 but the code is valid C++17; we build it as C++17.

Local changes:
- `patchworkpp.h`: constructor banner is printed only when `params.verbose` is set.

Citation: S. Lee, H. Lim, H. Myung, "Patchwork++: Fast and Robust Ground Segmentation Solving
Partial Under-Segmentation Using 3D Point Cloud", IROS 2022.
