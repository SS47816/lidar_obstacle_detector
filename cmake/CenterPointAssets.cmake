# Downloads the CUDA-CenterPoint assets that are not vendored: NVIDIA's prebuilt
# libspconv (per CPU architecture and CUDA major version) and the nuScenes ONNX
# models, pinned to one Lidar_AI_Solution commit and verified by SHA-256.
#
# Sets LOD_SPCONV_LIBRARY and LOD_CENTERPOINT_MODEL_BUILD_DIR.

set(_lais_commit 7c1623f234d5a9794fd862c9456f68f87ae7b900)
set(_raw   "https://raw.githubusercontent.com/NVIDIA-AI-IOT/Lidar_AI_Solution/${_lais_commit}")
set(_media "https://media.githubusercontent.com/media/NVIDIA-AI-IOT/Lidar_AI_Solution/${_lais_commit}")
set(_assets "${CMAKE_BINARY_DIR}/centerpoint_assets")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(_arch aarch64)
else()
  set(_arch x86_64)
endif()
if(CUDAToolkit_VERSION_MAJOR EQUAL 11)
  set(_cuda cuda11.4)
elseif(CUDAToolkit_VERSION_MAJOR EQUAL 12)
  set(_cuda cuda12.8)
else()
  set(_cuda cuda13.0)
endif()

# variant -> url base (git-lfs files come from the media host) and sha256
set(_spconv_x86_64_cuda11.4  ${_media} c4becb480390416ba99aacd6ef8640ecd4719e3abeaf7057ee36a2dfafabd253)
set(_spconv_x86_64_cuda12.8  ${_media} 5e4e58136e7962f23c4e68a869ba0f3ada20502996eefd2a616293d34bc2ae9d)
set(_spconv_x86_64_cuda13.0  ${_media} ce71e330d9fb218a798ccc37c9c42692f9a2c6bb0d48c53bfe7cdf874322e396)
set(_spconv_aarch64_cuda11.4 ${_media} 55f3c765fb90056ab381882aacf581dfeb408abfd58fa1ee3f3d7d8d402149fa)
set(_spconv_aarch64_cuda12.8 ${_raw}   8fc56961f4bf2785193a358a3b068b9cd5844390a2bad460e8311cc7f5d5a51a)
set(_spconv_aarch64_cuda13.0 ${_raw}   e95bb4252c6a6276c1c1a424a7092c43a68d21e2d701273d3e379ee7d5452189)

set(_variant ${_arch}_${_cuda})
list(GET _spconv_${_variant} 0 _base)
list(GET _spconv_${_variant} 1 _sha)
set(LOD_SPCONV_LIBRARY "${_assets}/${_variant}/libspconv.so")
if(NOT EXISTS "${LOD_SPCONV_LIBRARY}")
  message(STATUS "Downloading libspconv (${_variant})")
  file(DOWNLOAD "${_base}/libraries/3DSparseConvolution/libspconv/lib/${_variant}/libspconv.so"
    "${LOD_SPCONV_LIBRARY}" EXPECTED_HASH SHA256=${_sha} TLS_VERIFY ON SHOW_PROGRESS)
endif()

set(LOD_CENTERPOINT_MODEL_BUILD_DIR "${_assets}/models")
foreach(_model
    "centerpoint.scn.onnx;d89f25e289c79b28d0d1f147866974732c9f8e7bc2a8cebbc737b8578c6d3e07"
    "rpn_centerhead_sim.onnx;d2044da8a1536b5287940d9167346e50aa348daa7b0f1ffdd351aca13941a32c")
  list(GET _model 0 _name)
  list(GET _model 1 _sha)
  if(NOT EXISTS "${LOD_CENTERPOINT_MODEL_BUILD_DIR}/${_name}")
    message(STATUS "Downloading CenterPoint model ${_name}")
    file(DOWNLOAD "${_raw}/CUDA-CenterPoint/model/${_name}" "${LOD_CENTERPOINT_MODEL_BUILD_DIR}/${_name}"
      EXPECTED_HASH SHA256=${_sha} TLS_VERIFY ON)
  endif()
endforeach()
