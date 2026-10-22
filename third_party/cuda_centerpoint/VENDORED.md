# Vendored: NVIDIA CUDA-CenterPoint and 3D sparse convolution parser

- Upstream: https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution
- Commit: 7c1623f234d5a9794fd862c9456f68f87ae7b900 (2026-07-28)
- License: MIT (`LICENSE.md`); `onnx/*.proto` are from ONNX, Apache-2.0.
- Files:
  - `src/`, `include/`: `CUDA-CenterPoint/{src,include}` (voxelization, decode and NMS kernels)
  - `spconv/include`: `libraries/3DSparseConvolution/libspconv/include`
  - `spconv/parser`: `libraries/3DSparseConvolution/libspconv/parser` (ONNX loader for libspconv)
  - `onnx/`: `libraries/3DSparseConvolution/src/onnx/*.proto` (compiled with protoc at build time)

Not vendored: the prebuilt `libspconv.so` (NVIDIA binary, ~40-65 MB) and the ONNX models
(`centerpoint.scn.onnx`, `rpn_centerhead_sim.onnx`, trained on nuScenes). CMake downloads them
from the same commit and checks their SHA-256 (`cmake/CenterPointAssets.cmake`).

Local changes:
- `src/tensorrt.cpp`, `include/tensorrt.hpp`: ported from TensorRT 8.5 to the TensorRT 10 API
  (tensor names, `enqueueV3`, smart-pointer ownership). The engine is built from the ONNX model
  with the TensorRT builder (FP16, half-precision linear I/O as the original `trtexec` call) and
  the serialized plan is cached.
- `src/centerpoint.cpp`, `include/centerpoint.h`: model directory and engine plan path are
  arguments; uses the current libspconv API (`input()->features().reference(...)`,
  `forward(stream)`); throws instead of `abort()` when a model fails to load.
