// The one definition of "the golden run", shared by the capture tool that produced this
// phase's fixtures and the equivalence test that checks against them.
//
// Sharing it is the point: if the capture and the check could disagree about the scene, the
// timestep, the number of steps or the sampling cadence, a fixture mismatch would be
// ambiguous - a real regression and a drifted harness would look the same. They cannot
// disagree, because there is one copy of each number, here.
#ifndef PERCEPTION_TESTS_ADAPTER_ORACLE_GOLDEN_RUN_H_
#define PERCEPTION_TESTS_ADAPTER_ORACLE_GOLDEN_RUN_H_

#include <cstdio>
#include <filesystem>
#include <string>

#include <mujoco/mujoco.h>

#include "dpcbf/dynamic_obstacles.h"

namespace oracle_golden {

// 600 physics steps at the G1 scene's 0.002 s timestep = 1.2 s of simulated time, sampled
// every 20 steps (0.04 s) for 30 samples. Long enough that the obstacles have travelled
// (0.8 m/s x 1.2 s = up to ~0.96 m, several arena-wall reflections across the population)
// and that the uncontrolled robot has genuinely moved, which is what makes this a moving
// scene rather than a static one. Short enough to keep the fixture reviewable.
inline constexpr int kSteps = 600;
inline constexpr int kSampleEvery = 20;

// Exact decimal-free rendering of a double. printf("%.17g") round-trips in practice but
// "%a" removes the question entirely, and -0.0 / denormals stay distinguishable.
inline std::string HexDouble(double value) {
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%a", value);
  return buffer;
}

// Builds the model the way simulate/src/main.cc's LoadModel does: parse the scene, let
// DynamicObstacleManager add its mocap cylinders to the spec, then compile. Loading the
// compiled XML directly (as the P4 bring-up tests do) would produce a scene with no
// obstacles at all, which is precisely the thing under test here.
inline bool BuildScene(const std::filesystem::path& repository_root,
                       dpcbf::DynamicObstacleManager& obstacles, mjModel*& model,
                       mjData*& data, std::string& error) {
  model = nullptr;
  data = nullptr;

  const std::filesystem::path config = repository_root / "dpcbf/config/dpcbf_config.yaml";
  const std::filesystem::path scene =
      repository_root / "src/assets/robots/unitree_g1/xmls/scene_g1.xml";
  if (!std::filesystem::exists(config) || !std::filesystem::exists(scene)) {
    error = "expected " + config.string() + " and " + scene.string() + " to exist";
    return false;
  }

  try {
    obstacles.LoadConfig(config);
  } catch (const std::exception& thrown) {
    error = std::string("DynamicObstacleManager::LoadConfig threw: ") + thrown.what();
    return false;
  }

  char load_error[1024] = "";
  mjSpec* spec = mj_parseXML(scene.string().c_str(), nullptr, load_error, sizeof(load_error));
  if (spec == nullptr) {
    error = std::string("mj_parseXML failed: ") + load_error;
    return false;
  }
  try {
    obstacles.AddToSpec(spec);
  } catch (const std::exception& thrown) {
    mj_deleteSpec(spec);
    error = std::string("AddToSpec threw: ") + thrown.what();
    return false;
  }
  model = mj_compile(spec, nullptr);
  if (model == nullptr) {
    error = std::string("mj_compile failed: ") + mjs_getError(spec);
    mj_deleteSpec(spec);
    return false;
  }
  mj_deleteSpec(spec);

  data = mj_makeData(model);
  if (data == nullptr) {
    error = "mj_makeData failed";
    mj_deleteModel(model);
    model = nullptr;
    return false;
  }
  if (!obstacles.BindModel(model, data)) {
    error = "DynamicObstacleManager::BindModel failed";
    mj_deleteData(data);
    mj_deleteModel(model);
    data = nullptr;
    model = nullptr;
    return false;
  }
  mj_forward(model, data);
  return true;
}

}  // namespace oracle_golden

#endif  // PERCEPTION_TESTS_ADAPTER_ORACLE_GOLDEN_RUN_H_
