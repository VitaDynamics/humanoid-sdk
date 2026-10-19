// Offline oracle: compile against the unmodified baseline header and ORT 1.20.1.
#include <algorithm>
#include <array>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include "onnxruntime_cxx_api.h"
#include "src/application/locomotion/humanoid/models/vt_human_obs.h"

template <class T> std::vector<T> Read(int n) {
  std::vector<T> v(n);
  for (auto& x : v) std::cin >> x;
  return v;
}
template <class T> void Write(const std::vector<T>& v) {
  std::cout.write(reinterpret_cast<const char*>(v.data()), v.size() * sizeof(T));
}

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  int frames, history, dim, n, terms;
  float obs_clip, action_clip;
  std::cin >> frames >> history >> dim >> n >> terms >> obs_clip >> action_clip;
  std::vector<std::string> names(terms);
  std::map<std::string, double> scales;
  for (auto& name : names) { double scale; std::cin >> name >> scale; scales[name] = scale; }
  const auto defaults = Read<double>(n), action_scale = Read<double>(n);
  const auto lower = Read<double>(n), upper = Read<double>(n);
  Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "velocity-mode-parity");
  Ort::SessionOptions options;
  options.SetIntraOpNumThreads(1);
  options.SetInterOpNumThreads(1);
  Ort::Session session(env, argv[1], options);
  const char* input_names[] = {"obs"};
  const char* output_names[] = {"actions"};
  std::array<int64_t, 2> shape{1, history * dim};
  const auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  vita::humanoid::VtHumanObsHistory buffer(history, dim);
  std::vector<float> last_action(n, 0.0f);
  for (int f = 0; f < frames; ++f) {
    const auto gyro = Read<double>(3), quat = Read<double>(4), cmd = Read<double>(3);
    const auto q = Read<double>(n), dq = Read<double>(n);
    auto frame = vita::humanoid::BuildVtHumanFrame(names, scales,
        vita::humanoid::VtHumanObservationScaleOwner::kRuntime, gyro.data(), quat.data(),
        cmd.data(), q, dq, defaults, last_action);
    for (auto& value : frame) value = std::clamp(value, -obs_clip, obs_clip);
    auto obs = buffer.PushAndFlatten(frame);
    auto input = Ort::Value::CreateTensor<float>(memory, obs.data(), obs.size(), shape.data(), 2);
    auto outputs = session.Run(Ort::RunOptions{nullptr}, input_names, &input, 1, output_names, 1);
    const float* data = outputs[0].GetTensorData<float>();
    std::vector<float> raw(data, data + n);
    for (int i = 0; i < n; ++i) last_action[i] = std::clamp(raw[i], -action_clip, action_clip);
    const auto targets = vita::humanoid::DecodeAndClampVtHumanAction(last_action, action_scale, defaults, lower, upper);
    Write(obs); Write(raw); Write(targets);
  }
  return std::cin.fail() ? 3 : 0;
}
