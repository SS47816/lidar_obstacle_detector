# Finds TensorRT (headers, nvinfer, nvonnxparser).
#
# Searches TENSORRT_ROOT, the system (NVIDIA apt packages / JetPack), the active
# conda / pixi environment and the `tensorrt_libs` directory of the PyPI wheels
# (which ship versioned libraries only, e.g. libnvinfer.so.10).
#
# Defines TensorRT_FOUND, TensorRT_VERSION and the imported target TensorRT::TensorRT.

set(_trt_hints ${TENSORRT_ROOT} $ENV{TENSORRT_ROOT} $ENV{CONDA_PREFIX} /usr /usr/local/tensorrt)
file(GLOB _trt_pip_dirs $ENV{CONDA_PREFIX}/lib/python3*/site-packages/tensorrt_libs)

find_path(TensorRT_INCLUDE_DIR NvInfer.h
  HINTS ${_trt_hints}
  PATH_SUFFIXES include include/tensorrt include/x86_64-linux-gnu include/aarch64-linux-gnu)
find_library(TensorRT_LIBRARY NAMES nvinfer libnvinfer.so.10 libnvinfer.so.11
  HINTS ${_trt_hints} ${_trt_pip_dirs}
  PATH_SUFFIXES lib lib64 lib/x86_64-linux-gnu lib/aarch64-linux-gnu)
find_library(TensorRT_ONNX_LIBRARY NAMES nvonnxparser libnvonnxparser.so.10 libnvonnxparser.so.11
  HINTS ${_trt_hints} ${_trt_pip_dirs}
  PATH_SUFFIXES lib lib64 lib/x86_64-linux-gnu lib/aarch64-linux-gnu)

if(TensorRT_INCLUDE_DIR AND EXISTS "${TensorRT_INCLUDE_DIR}/NvInferVersion.h")
  file(STRINGS "${TensorRT_INCLUDE_DIR}/NvInferVersion.h" _trt_ver REGEX "#define (NV_TENSORRT_MAJOR|TRT_MAJOR_ENTERPRISE|NV_TENSORRT_MINOR|TRT_MINOR_ENTERPRISE) ")
  string(REGEX MATCH "(NV_TENSORRT_MAJOR|TRT_MAJOR_ENTERPRISE) ([0-9]+)" _ "${_trt_ver}")
  set(_major ${CMAKE_MATCH_2})
  string(REGEX MATCH "(NV_TENSORRT_MINOR|TRT_MINOR_ENTERPRISE) ([0-9]+)" _ "${_trt_ver}")
  set(TensorRT_VERSION "${_major}.${CMAKE_MATCH_2}")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TensorRT
  REQUIRED_VARS TensorRT_INCLUDE_DIR TensorRT_LIBRARY TensorRT_ONNX_LIBRARY
  VERSION_VAR TensorRT_VERSION)

if(TensorRT_FOUND AND NOT TARGET TensorRT::TensorRT)
  add_library(TensorRT::TensorRT INTERFACE IMPORTED)
  set_target_properties(TensorRT::TensorRT PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${TensorRT_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "${TensorRT_LIBRARY};${TensorRT_ONNX_LIBRARY}")
endif()
mark_as_advanced(TensorRT_INCLUDE_DIR TensorRT_LIBRARY TensorRT_ONNX_LIBRARY)
