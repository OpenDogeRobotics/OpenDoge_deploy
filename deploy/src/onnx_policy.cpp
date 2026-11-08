#include "opendoge_deploy/policy.hpp"

#ifdef OPENDOGE_HAS_ONNX

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

namespace opendoge
{
namespace
{
class OnnxPolicy final : public Policy
{
public:
  OnnxPolicy()
  : env_(ORT_LOGGING_LEVEL_WARNING, "opendoge_deploy"),
    memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
  {
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetInterOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  }

  bool load(const std::string & path, std::string & error) override
  {
    try {
      session_ = std::make_unique<Ort::Session>(env_, path.c_str(), session_options_);
      auto input_name = session_->GetInputNameAllocated(0, allocator_);
      auto output_name = session_->GetOutputNameAllocated(0, allocator_);
      input_name_ = input_name.get();
      output_name_ = output_name.get();
      if (!validateIo(error)) {
        session_.reset();
        return false;
      }
    } catch (const std::exception & exc) {
      error = exc.what();
      return false;
    }
    return true;
  }

  bool infer(const std::array<double, kObsDim> & obs, std::array<double, kNumJoints> & action, std::string & error) override
  {
    if (!session_) {
      error = "ONNX session is not loaded";
      return false;
    }

    std::array<float, kObsDim> input{};
    for (std::size_t i = 0; i < kObsDim; ++i) {
      input[i] = static_cast<float>(obs[i]);
    }
    std::array<int64_t, 2> input_shape{1, static_cast<int64_t>(kObsDim)};
    auto tensor = Ort::Value::CreateTensor<float>(
      memory_info_, input.data(), input.size(), input_shape.data(), input_shape.size());

    const char * input_names[] = {input_name_.c_str()};
    const char * output_names[] = {output_name_.c_str()};
    try {
      auto outputs = session_->Run(
        Ort::RunOptions{nullptr}, input_names, &tensor, 1, output_names, 1);
      const auto * out = outputs[0].GetTensorData<float>();
      for (std::size_t i = 0; i < kNumJoints; ++i) {
        const double value = static_cast<double>(out[i]);
        if (!std::isfinite(value)) {
          error = "ONNX output contains NaN or Inf";
          return false;
        }
        // Policy output is intentionally unbounded (training applies no clip).
        action[i] = value;
      }
    } catch (const std::exception & exc) {
      error = exc.what();
      return false;
    }
    return true;
  }

private:
  Ort::Env env_;
  Ort::SessionOptions session_options_;
  Ort::AllocatorWithDefaultOptions allocator_;
  Ort::MemoryInfo memory_info_;
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_;
  std::string output_name_;

  static std::string shapeString(const std::vector<int64_t> & shape)
  {
    std::ostringstream ss;
    ss << "[";
    for (std::size_t i = 0; i < shape.size(); ++i) {
      if (i != 0) {
        ss << ",";
      }
      ss << shape[i];
    }
    ss << "]";
    return ss.str();
  }

  bool validateIo(std::string & error)
  {
    const auto input_type_info = session_->GetInputTypeInfo(0);
    const auto output_type_info = session_->GetOutputTypeInfo(0);
    const auto input_info = input_type_info.GetTensorTypeAndShapeInfo();
    const auto output_info = output_type_info.GetTensorTypeAndShapeInfo();
    const auto input_type = input_info.GetElementType();
    const auto output_type = output_info.GetElementType();
    const auto input_shape = input_info.GetShape();
    const auto output_shape = output_info.GetShape();

    if (input_type != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
      error = "ONNX input must be float tensor";
      return false;
    }
    if (output_type != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
      error = "ONNX output must be float tensor";
      return false;
    }
    if (input_shape.size() != 2 || input_shape[1] != static_cast<int64_t>(kObsDim)) {
      error = "ONNX input shape must be [batch," + std::to_string(kObsDim) + "], got " + shapeString(input_shape);
      return false;
    }
    if (output_shape.size() != 2 || output_shape[1] != static_cast<int64_t>(kNumJoints)) {
      error = "ONNX output shape must be [batch,12], got " + shapeString(output_shape);
      return false;
    }
    return true;
  }
};
}  // namespace

std::unique_ptr<Policy> makeOnnxPolicy()
{
  return std::make_unique<OnnxPolicy>();
}

}  // namespace opendoge

#endif
