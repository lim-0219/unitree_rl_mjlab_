// PairedFilterProbe - two independent DpcbfSafetyFilter instances, one fed the oracle and one
// fed the estimated path, commands compared and discarded. Integration ladder steps 4 and 5.
//
// =========================================================================================
// WHY THIS CLASS EXISTS AT ALL, RATHER THAN THE EVALUATOR OWNING TWO FILTERS
// =========================================================================================
// The duplicate-instance comparison needs `DpcbfSafetyFilter` itself, not just the
// `ObstacleState` type. Dependency rule §8 lets `integration` NAME the adapter's output type
// but forbids it from touching dpcbf headers, so `integration/Evaluator` cannot hold a filter.
//
// THIS HEADER THEREFORE NAMES NO DPCBF TYPE. Not one - not ObstacleState, not RobotState, not
// VelocityCommand, not SafetyFilterResult. The filters live behind a pimpl whose translation
// unit is the only place `dpcbf/dpcbf_safety_filter.h` is reached, and everything crossing
// this boundary is a plain POD declared below. That is what lets `integration` call the
// comparison without the include audit ever finding a dpcbf header on its side of the seam.
//
// THE TWO INSTANCES ARE SAFE. DpcbfSafetyFilter is pimpl'd and owns its OSQP workspace, so two
// of them share nothing (doc §16, confirmed by P3's own end-to-end equivalence test, which
// already ran two real instances side by side). This is the same pattern, comparing oracle
// against ESTIMATED rather than oracle-old-code against oracle-new-code.
//
// =========================================================================================
// WHAT THIS PROBE DELIBERATELY DOES NOT DO
// =========================================================================================
// It does not advance the robot. Every call is `(one robot state, one desired command, two
// obstacle sets) -> two outcomes`, and the caller owns whatever trajectory policy it wants.
// That keeps the side-effect-free promise of ladder step 4/5 structural: there is no state in
// here that an estimated command could steer, so "commands discarded" is a property of the
// type rather than a discipline the caller has to keep.
#ifndef PERCEPTION_ADAPTERS_DPCBF_PAIRED_FILTER_PROBE_H_
#define PERCEPTION_ADAPTERS_DPCBF_PAIRED_FILTER_PROBE_H_

#include <memory>
#include <string>
#include <vector>

#include "perception/adapters/dpcbf/adapter_params.h"
#include "perception/core/contracts/obstacles.h"

namespace perception::adapters::dpcbf {

// Mirrors dpcbf::RobotState field for field, without naming it. Same units and frame: world
// XY in metres, heading in radians, body-frame velocities in m/s.
struct ProbeRobotState {
  double x = 0.0;
  double y = 0.0;
  double phi = 0.0;
  double sagittal_velocity = 0.0;
  double lateral_velocity = 0.0;
};

// Mirrors dpcbf::VelocityCommand.
struct ProbeCommand {
  double sagittal = 0.0;
  double lateral = 0.0;
  double yaw_rate = 0.0;
};

// The subset of dpcbf::SafetyFilterResult a paired comparison needs. `decay_variables`,
// `slack_variables` and `selected_obstacles` are deliberately not carried: they are per-QP
// internals whose lengths depend on the constraint set, so a "difference" between two arms
// with different obstacle counts would not be a comparison of like with like.
struct ProbeOutcome {
  ProbeCommand command;
  int active_constraints = 0;
  int active_dpcbf_constraints = 0;
  int active_ecbf_constraints = 0;
  bool solved = false;

  // How many states this arm actually handed the filter, after the adapter's gates.
  int obstacles_supplied = 0;

  // Wall time of the Filter() call alone, microseconds. Feeds the §12 latency block.
  double filter_wall_us = 0.0;
};

class PairedFilterProbe {
 public:
  PairedFilterProbe();
  ~PairedFilterProbe();
  PairedFilterProbe(PairedFilterProbe&&) noexcept;
  PairedFilterProbe& operator=(PairedFilterProbe&&) noexcept;
  PairedFilterProbe(const PairedFilterProbe&) = delete;
  PairedFilterProbe& operator=(const PairedFilterProbe&) = delete;

  // Loads BOTH instances from the same dpcbf config and initializes them with the same
  // control step, so any difference between the arms is attributable to the obstacles alone.
  // Returns nullptr, or an error string owned by this object (valid until the next call).
  const char* Load(const std::string& dpcbf_config_path, double control_dt_s);

  bool loaded() const;

  // The oracle arm: ground truth in, through the P3 conversion, unchanged and uncapped.
  const char* FilterOracle(const ProbeRobotState& robot, const ProbeCommand& desired,
                           const std::vector<core::OracleObstacleState>& oracle,
                           ProbeOutcome* out);

  // The estimated arm: SafetyObstacles through the P11 conversion, with the adapter's gates.
  const char* FilterEstimated(const ProbeRobotState& robot, const ProbeCommand& desired,
                              const std::vector<core::SafetyObstacle>& estimated,
                              const DpcbfAdapterParams& params, ProbeOutcome* out,
                              DpcbfAdapterStats* adapter_stats = nullptr);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace perception::adapters::dpcbf

#endif  // PERCEPTION_ADAPTERS_DPCBF_PAIRED_FILTER_PROBE_H_
