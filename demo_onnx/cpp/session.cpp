#include <onnxruntime_cxx_api.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace py = pybind11;
namespace {
struct TensorInfo {
  std::string name;
  std::vector<int64_t> shape;
  std::string dtype;
  ONNXTensorElementDataType element_type;
};

std::string DtypeName(ONNXTensorElementDataType type) {
  switch (type) {
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:
    return "tensor(float)";
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE:
    return "tensor(double)";
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16:
    return "tensor(float16)";
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_BFLOAT16:
    return "tensor(bfloat16)";
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT8:
    return "tensor(int8)";
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8:
    return "tensor(uint8)";
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT16:
    return "tensor(int16)";
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16:
    return "tensor(uint16)";
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32:
    return "tensor(int32)";
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT32:
    return "tensor(uint32)";
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64:
    return "tensor(int64)";
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT64:
    return "tensor(uint64)";
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_BOOL:
    return "tensor(bool)";
  case ONNX_TENSOR_ELEMENT_DATA_TYPE_STRING:
    return "tensor(string)";
  default:
    return "tensor(unsupported)";
  }
}

class Session {
public:
  Session(const std::string &path, int intra_threads, int inter_threads)
      : env_(ORT_LOGGING_LEVEL_WARNING, "sdk_demo_onnx"),
        intra_threads_(intra_threads), inter_threads_(inter_threads) {
    try {
      Ort::SessionOptions options;
      options.SetIntraOpNumThreads(intra_threads);
      options.SetInterOpNumThreads(inter_threads);
      // Other options retain the ORT defaults used by the existing demos.
      session_ = std::make_unique<Ort::Session>(env_, path.c_str(), options);
      Ort::AllocatorWithDefaultOptions allocator;
      for (size_t i = 0; i < session_->GetInputCount(); ++i) {
        auto name = session_->GetInputNameAllocated(i, allocator);
        auto info = session_->GetInputTypeInfo(i);
        if (info.GetONNXType() != ONNX_TYPE_TENSOR) {
          throw py::value_error(std::string("model input '") + name.get() +
                                "' is not a tensor");
        }
        auto type = info.GetTensorTypeAndShapeInfo();
        inputs_.push_back({name.get(), type.GetShape(),
                           DtypeName(type.GetElementType()),
                           type.GetElementType()});
      }
      for (size_t i = 0; i < session_->GetOutputCount(); ++i) {
        auto name = session_->GetOutputNameAllocated(i, allocator);
        auto info = session_->GetOutputTypeInfo(i);
        if (info.GetONNXType() != ONNX_TYPE_TENSOR) {
          throw py::value_error(std::string("model output '") + name.get() +
                                "' is not a tensor");
        }
        auto type = info.GetTensorTypeAndShapeInfo();
        outputs_.push_back({name.get(), type.GetShape(),
                            DtypeName(type.GetElementType()),
                            type.GetElementType()});
      }
      auto meta = session_->GetModelMetadata();
      for (const auto &key :
           meta.GetCustomMetadataMapKeysAllocated(allocator)) {
        auto value =
            meta.LookupCustomMetadataMapAllocated(key.get(), allocator);
        metadata_.emplace(key.get(), value ? value.get() : "");
      }
    } catch (const Ort::Exception &error) {
      throw std::runtime_error("ONNX load '" + path + "': " + error.what());
    }
  }

  py::array_t<float> Run(const std::string &input_name,
                         const std::string &output_name,
                         const py::array &values) {
    const TensorInfo &input = Find(inputs_, input_name, "input");
    const TensorInfo &output = Find(outputs_, output_name, "output");
    if (input.element_type != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        output.element_type != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        !values.dtype().equal(py::dtype::of<float>())) {
      throw py::value_error("run requires float32 input and output tensors");
    }
    if (!input.shape.empty() &&
        static_cast<size_t>(values.ndim()) != input.shape.size()) {
      throw py::value_error("input rank differs from the model");
    }
    std::vector<int64_t> shape;
    for (py::ssize_t i = 0; i < values.ndim(); ++i) {
      if (!input.shape.empty() && input.shape[i] >= 0 &&
          values.shape(i) != input.shape[i]) {
        throw py::value_error("input shape differs from the model");
      }
      shape.push_back(values.shape(i));
    }
    auto contiguous = py::array_t<float, py::array::c_style>::ensure(values);
    if (!contiguous)
      throw py::value_error("cannot copy input to a dense tensor");
    std::vector<float> snapshot(static_cast<size_t>(contiguous.size()));
    if (!snapshot.empty()) {
      std::memcpy(snapshot.data(), contiguous.data(),
                  snapshot.size() * sizeof(float));
    }
    std::vector<Ort::Value> result;
    try {
      auto memory =
          Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
      auto tensor = Ort::Value::CreateTensor<float>(
          memory, snapshot.data(), snapshot.size(), shape.data(), shape.size());
      const char *in = input_name.c_str();
      const char *out = output_name.c_str();
      py::gil_scoped_release release;
      result =
          session_->Run(Ort::RunOptions{nullptr}, &in, &tensor, 1, &out, 1);
    } catch (const Ort::Exception &error) {
      throw std::runtime_error("ONNX run '" + input_name + "' -> '" +
                               output_name + "': " + error.what());
    }
    auto actual_shape = result.at(0).GetTensorTypeAndShapeInfo().GetShape();
    std::vector<py::ssize_t> numpy_shape(actual_shape.begin(),
                                         actual_shape.end());
    py::array_t<float> copied(numpy_shape);
    if (copied.size() != 0) {
      std::memcpy(copied.mutable_data(), result[0].GetTensorData<float>(),
                  static_cast<size_t>(copied.size()) * sizeof(float));
    }
    return copied;
  }

  const std::vector<TensorInfo> &inputs() const { return inputs_; }
  const std::vector<TensorInfo> &outputs() const { return outputs_; }
  const std::map<std::string, std::string> &metadata() const {
    return metadata_;
  }
  int intra_threads() const { return intra_threads_; }
  int inter_threads() const { return inter_threads_; }

private:
  static const TensorInfo &Find(const std::vector<TensorInfo> &tensors,
                                const std::string &name,
                                const char *direction) {
    auto it = std::find_if(
        tensors.begin(), tensors.end(),
        [&](const TensorInfo &tensor) { return tensor.name == name; });
    if (it == tensors.end()) {
      throw py::value_error(std::string("unknown ") + direction +
                            " name: " + name);
    }
    return *it;
  }
  Ort::Env env_;
  std::unique_ptr<Ort::Session> session_;
  std::vector<TensorInfo> inputs_;
  std::vector<TensorInfo> outputs_;
  std::map<std::string, std::string> metadata_;
  const int intra_threads_;
  const int inter_threads_;
};
} // namespace

PYBIND11_MODULE(_native, module) {
  const std::string version = OrtGetApiBase()->GetVersionString();
  if (version != "1.20.1") {
    throw py::import_error("demo ONNX backend requires ORT 1.20.1; loaded " +
                           version);
  }
  module.attr("ORT_VERSION") = version;
  py::class_<TensorInfo>(module, "TensorInfo")
      .def_readonly("name", &TensorInfo::name)
      .def_readonly("shape", &TensorInfo::shape)
      .def_readonly("dtype", &TensorInfo::dtype);
  py::class_<Session>(module, "Session")
      .def(py::init<const std::string &, int, int>(), py::arg("path"),
           py::kw_only(), py::arg("intra_threads"), py::arg("inter_threads"))
      .def_property_readonly("inputs", &Session::inputs)
      .def_property_readonly("outputs", &Session::outputs)
      .def_property_readonly("metadata", &Session::metadata)
      .def_property_readonly("intra_threads", &Session::intra_threads)
      .def_property_readonly("inter_threads", &Session::inter_threads)
      .def("run", &Session::Run, py::arg("input_name"), py::arg("output_name"),
           py::arg("values").noconvert());
}
