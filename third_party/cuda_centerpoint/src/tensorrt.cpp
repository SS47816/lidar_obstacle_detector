// TensorRT 10 port of CUDA-CenterPoint's tensorrt.cpp (MIT, NVIDIA): builds the
// engine from ONNX (replacing trtexec) and caches the serialized plan.
#include "tensorrt.hpp"

#include <cuda_runtime.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>

#include "NvInfer.h"
#include "NvOnnxParser.h"

namespace TensorRT {

static class Logger : public nvinfer1::ILogger {
 public:
  void log(Severity severity, const char *msg) noexcept override {
    if (severity <= Severity::kWARNING) std::cerr << "[TensorRT] " << msg << std::endl;
  }
} gLogger_;

static std::vector<char> load_file(const std::string &file) {
  std::ifstream in(file, std::ios::binary | std::ios::ate);
  if (!in) return {};
  std::vector<char> data(static_cast<size_t>(in.tellg()));
  in.seekg(0);
  in.read(data.data(), static_cast<std::streamsize>(data.size()));
  return data;
}

static bool build_plan(const std::string &onnx, const std::string &plan) {
  std::unique_ptr<nvinfer1::IBuilder> builder(nvinfer1::createInferBuilder(gLogger_));
  std::unique_ptr<nvinfer1::INetworkDefinition> network(builder->createNetworkV2(0));
  std::unique_ptr<nvonnxparser::IParser> parser(nvonnxparser::createParser(*network, gLogger_));
  if (!parser->parseFromFile(onnx.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
    std::cerr << "Failed to parse " << onnx << std::endl;
    return false;
  }
  std::unique_ptr<nvinfer1::IBuilderConfig> config(builder->createBuilderConfig());
  config->setFlag(nvinfer1::BuilderFlag::kFP16);
  config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 4ull << 30);
  // as `trtexec --inputIOFormats=fp16:chw --outputIOFormats=fp16:chw`
  const auto linear = 1U << static_cast<int>(nvinfer1::TensorFormat::kLINEAR);
  for (int i = 0; i < network->getNbInputs(); ++i) {
    network->getInput(i)->setType(nvinfer1::DataType::kHALF);
    network->getInput(i)->setAllowedFormats(linear);
  }
  for (int i = 0; i < network->getNbOutputs(); ++i) {
    network->getOutput(i)->setType(nvinfer1::DataType::kHALF);
    network->getOutput(i)->setAllowedFormats(linear);
  }
  std::unique_ptr<nvinfer1::IHostMemory> serialized(builder->buildSerializedNetwork(*network, *config));
  if (!serialized) return false;
  std::ofstream out(plan, std::ios::binary);
  out.write(static_cast<const char *>(serialized->data()), static_cast<std::streamsize>(serialized->size()));
  return true;
}

class EngineImpl : public Engine {
 public:
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  std::vector<std::string> io_names_;

  bool load(const std::string &onnx, const std::string &plan) {
    auto data = load_file(plan);
    if (data.empty()) {
      std::cerr << "Building TensorRT engine from " << onnx << " (one-off, ~1-2 min)..." << std::endl;
      if (!build_plan(onnx, plan)) return false;
      data = load_file(plan);
    }
    runtime_.reset(nvinfer1::createInferRuntime(gLogger_));
    engine_.reset(runtime_->deserializeCudaEngine(data.data(), data.size()));
    if (!engine_) return false;
    context_.reset(engine_->createExecutionContext());
    for (int i = 0; i < engine_->getNbIOTensors(); ++i) io_names_.emplace_back(engine_->getIOTensorName(i));
    return context_ != nullptr;
  }

  int64_t getBindingNumel(const std::string &name) override {
    const auto d = engine_->getTensorShape(name.c_str());
    return std::accumulate(d.d, d.d + d.nbDims, int64_t{1}, std::multiplies<int64_t>());
  }
  std::vector<int64_t> getBindingDims(const std::string &name) override {
    const auto d = engine_->getTensorShape(name.c_str());
    return std::vector<int64_t>(d.d, d.d + d.nbDims);
  }
  // Buffers in I/O tensor order (inputs first), as with the old binding indices
  bool forward(const std::initializer_list<void *> &buffers, void *stream = nullptr) override {
    size_t i = 0;
    for (void *b : buffers) context_->setTensorAddress(io_names_.at(i++).c_str(), b);
    return context_->enqueueV3(static_cast<cudaStream_t>(stream));
  }
  void print() override {
    for (const auto &n : io_names_) {
      const auto d = engine_->getTensorShape(n.c_str());
      std::cout << (engine_->getTensorIOMode(n.c_str()) == nvinfer1::TensorIOMode::kINPUT ? "in  " : "out ")
                << n << " [";
      for (int k = 0; k < d.nbDims; ++k) std::cout << d.d[k] << (k + 1 < d.nbDims ? "x" : "");
      std::cout << "]\n";
    }
  }
};

std::shared_ptr<Engine> load(const std::string &onnx, const std::string &plan) {
  std::shared_ptr<EngineImpl> impl(new EngineImpl());
  if (!impl->load(onnx, plan)) impl.reset();
  return impl;
}

}  // namespace TensorRT
