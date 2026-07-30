#include "perception/adapters/dpcbf/paired_filter_probe.h"

#include <chrono>
#include <exception>
#include <filesystem>

// THE ONLY TRANSLATION UNIT IN THIS PHASE THAT REACHES THE FILTER ITSELF. Everything above the
// pimpl is dpcbf-free by construction; see the header essay.
#include "dpcbf/dpcbf_safety_filter.h"
#include "perception/adapters/dpcbf/oracle_to_dpcbf.h"
#include "perception/adapters/dpcbf/safety_to_dpcbf.h"

namespace perception::adapters::dpcbf {
namespace {

::dpcbf::RobotState ToDpcbf(const ProbeRobotState& robot) {
  ::dpcbf::RobotState state;
  state.x = robot.x;
  state.y = robot.y;
  state.phi = robot.phi;
  state.sagittal_velocity = robot.sagittal_velocity;
  state.lateral_velocity = robot.lateral_velocity;
  return state;
}

::dpcbf::VelocityCommand ToDpcbf(const ProbeCommand& command) {
  ::dpcbf::VelocityCommand out;
  out.sagittal = command.sagittal;
  out.lateral = command.lateral;
  out.yaw_rate = command.yaw_rate;
  return out;
}

}  // namespace

struct PairedFilterProbe::Impl {
  ::dpcbf::DpcbfSafetyFilter oracle_arm;
  ::dpcbf::DpcbfSafetyFilter estimated_arm;
  std::vector<::dpcbf::ObstacleState> scratch;
  std::string error;
  bool loaded = false;

  // One arm's half of a paired sample. Kept private because it is the only place a
  // dpcbf::SafetyFilterResult is ever read.
  static void Run(::dpcbf::DpcbfSafetyFilter& filter, const ProbeRobotState& robot,
                  const ProbeCommand& desired,
                  const std::vector<::dpcbf::ObstacleState>& obstacles, ProbeOutcome* out) {
    const auto start = std::chrono::steady_clock::now();
    const ::dpcbf::SafetyFilterResult result =
        filter.Filter(ToDpcbf(robot), ToDpcbf(desired), obstacles);
    const auto end = std::chrono::steady_clock::now();

    out->command.sagittal = result.command.sagittal;
    out->command.lateral = result.command.lateral;
    out->command.yaw_rate = result.command.yaw_rate;
    out->active_constraints = result.active_constraints;
    out->active_dpcbf_constraints = result.active_dpcbf_constraints;
    out->active_ecbf_constraints = result.active_ecbf_constraints;
    out->solved = result.solved;
    out->obstacles_supplied = static_cast<int>(obstacles.size());
    out->filter_wall_us = std::chrono::duration<double, std::micro>(end - start).count();
  }
};

PairedFilterProbe::PairedFilterProbe() : impl_(std::make_unique<Impl>()) {}
PairedFilterProbe::~PairedFilterProbe() = default;
PairedFilterProbe::PairedFilterProbe(PairedFilterProbe&&) noexcept = default;
PairedFilterProbe& PairedFilterProbe::operator=(PairedFilterProbe&&) noexcept = default;

const char* PairedFilterProbe::Load(const std::string& dpcbf_config_path, double control_dt_s) {
  impl_->loaded = false;
  if (!(control_dt_s > 0.0)) {
    impl_->error = "PairedFilterProbe::Load: control_dt_s must be > 0";
    return impl_->error.c_str();
  }
  try {
    // Both arms from the SAME file and the SAME control step. Any difference the comparison
    // then reports is attributable to the obstacle sets, which is the entire point.
    const std::filesystem::path path(dpcbf_config_path);
    impl_->oracle_arm.LoadConfig(path);
    impl_->estimated_arm.LoadConfig(path);
    impl_->oracle_arm.Initialize(control_dt_s);
    impl_->estimated_arm.Initialize(control_dt_s);
  } catch (const std::exception& thrown) {
    impl_->error = std::string("PairedFilterProbe::Load: ") + thrown.what();
    return impl_->error.c_str();
  }
  impl_->loaded = true;
  return nullptr;
}

bool PairedFilterProbe::loaded() const { return impl_->loaded; }

const char* PairedFilterProbe::FilterOracle(const ProbeRobotState& robot,
                                            const ProbeCommand& desired,
                                            const std::vector<core::OracleObstacleState>& oracle,
                                            ProbeOutcome* out) {
  if (!impl_->loaded) return "PairedFilterProbe: Load() has not succeeded";
  if (out == nullptr) return "PairedFilterProbe: out must not be null";
  // The P3 conversion, verbatim: total, order-preserving, uncapped. The oracle arm has to be
  // the path the live system actually runs, or the comparison has two unknowns in it.
  ToDpcbfObstacleStates(oracle, impl_->scratch);
  Impl::Run(impl_->oracle_arm, robot, desired, impl_->scratch, out);
  return nullptr;
}

const char* PairedFilterProbe::FilterEstimated(const ProbeRobotState& robot,
                                               const ProbeCommand& desired,
                                               const std::vector<core::SafetyObstacle>& estimated,
                                               const DpcbfAdapterParams& params, ProbeOutcome* out,
                                               DpcbfAdapterStats* adapter_stats) {
  if (!impl_->loaded) return "PairedFilterProbe: Load() has not succeeded";
  if (out == nullptr) return "PairedFilterProbe: out must not be null";
  if (const char* reason =
          ToDpcbfObstacleStates(estimated, params, impl_->scratch, adapter_stats)) {
    return reason;
  }
  Impl::Run(impl_->estimated_arm, robot, desired, impl_->scratch, out);
  return nullptr;
}

}  // namespace perception::adapters::dpcbf
