// ObstacleSourceSelector - the one place that decides what DPCBF is fed.
//
// Before this class the decision was not a decision: the simulator's axis-filter lambda
// converted DynamicObstacleManager::Snapshot() inline and that was the only possible source.
// The oracle path is permanent by design (it is the regression baseline and the fallback of
// last resort, ladder step 9), so the point of this class is to make the choice explicit and
// testable while leaving mode 1 numerically untouched.
//
// THIS PHASE IMPLEMENTS MODE 1 AND NOTHING ELSE. Every other mode is present in the enum, is
// parsed, and then hard-fails at Bind() with a message naming the phase that owns it. That is
// deliberate over a half-built shadow path: a mode that silently degrades to oracle is
// indistinguishable from a mode that works, which is exactly the confusion a safety subsystem
// cannot afford.
//
// NO THREAD, NO QUEUE, NO RUNNER. In oracle mode the selector holds a pointer to the obstacle
// manager and calls it synchronously on whatever thread asks. The perception thread, the SPSC
// queue and the PerceptionFrame double buffer are P12.
#ifndef PERCEPTION_INTEGRATION_OBSTACLE_SOURCE_SELECTOR_H_
#define PERCEPTION_INTEGRATION_OBSTACLE_SOURCE_SELECTOR_H_

#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "perception/adapters/dpcbf/oracle_to_dpcbf.h"
#include "perception/adapters/mujoco/oracle_provider_mj.h"
#include "perception/integration/perception_config.h"

namespace perception::integration {

// The obstacle-source mode. Members are the closed set perception.mode.name accepts; the
// comments map them onto the nine-step integration ladder in the architecture doc, which is
// coarser than it looks - steps 7, 8 and 9 are `estimated` plus configuration
// (scenario_allowlist, which mode is the shipped default, and the fact that oracle never
// stops being selectable), not distinct code paths.
enum class ObstacleSourceMode {
  kOracle,             // Ladder 1. IMPLEMENTED (this phase).
  kShadow,             // Ladder 2-3: pipeline runs, output logged, never consumed. P12.
  kCompare,            // Ladder 4-5: both evaluated, duplicate filter instance. P11 + P12.
  kEstimatedFallback,  // Ladder 6-7: estimated drives live, per-frame oracle fallback. P15.
  kEstimated,          // Ladder 8-9: estimated is the default source. P16.
};

const char* ToString(ObstacleSourceMode mode);

// Maps perception.mode.name onto the enum. Returns false on an unrecognised name rather than
// defaulting: the config loader validates the same closed set, so a name that reaches here
// and is not recognised means the loader grew a mode this class was never taught, and
// treating that as "oracle" would put an unreviewed mode silently on the safe path.
bool ParseObstacleSourceMode(const std::string& name, ObstacleSourceMode& out,
                             std::string& error);

class ObstacleSourceSelector {
 public:
  ObstacleSourceSelector() = default;

  // Wires the selector for the configured mode.
  //
  // `manager` is the live obstacle oracle. It is taken here, at startup, rather than at model
  // bind, because the oracle obstacle leg needs no mjModel: DynamicObstacleManager populates
  // its obstacle list in AddToSpec, before compile, and Snapshot() is valid from that moment
  // on. Binding early means the axis-filter lambda can never observe a half-bound selector,
  // whatever order the physics and bridge threads happen to start in.
  //
  // Returns false with a human-readable `error` for any mode this phase does not implement.
  // Callers are expected to treat that as fatal at startup - the same discipline as the
  // extrinsic guard - rather than discovering it from a filter callback.
  bool Bind(const PerceptionConfig& config, const ::dpcbf::DynamicObstacleManager* manager,
            std::string& error);

  // Resolves the robot base body for the oracle's robot-pose leg. Optional: the obstacle leg
  // that feeds DPCBF does not need it, so a failure here is not a failure of mode 1. Called
  // from each model-load site, mirroring DynamicObstacleManager::BindModel.
  bool BindModel(const mjModel* model, std::string& error);

  // THE CALL THAT REPLACED THE INLINE CONVERSION.
  //
  // In oracle mode: Snapshot() -> core::OracleObstacleState (adapters/mujoco/oracle_provider)
  // -> dpcbf::ObstacleState (adapters/dpcbf/oracle_to_dpcbf), complete and in order, ready to
  // hand to DpcbfSafetyFilter::Filter() unchanged.
  //
  // `now_s` is MuJoCo sim time and is used only to stamp the intermediate
  // OracleObstacleStates - ground truth is never stale, so no staleness gate applies in mode
  // 1. It becomes load-bearing at mode 6, where the stamp is what fallback triggers on.
  //
  // Returns by value, matching the local the lambda used to build, so the call site's object
  // lifetime and aliasing are exactly what they were.
  //
  // WHAT IS DELIBERATELY *NOT* APPLIED HERE. perception.dpcbf_adapter.max_obstacles (20) and
  // .drop_invalid (true) are real, validated config today, and they are NOT consulted in
  // oracle mode. They belong to the P11 SafetyObstacle adapter on the estimated path. The
  // shipped dpcbf_config.yaml runs 90 obstacles, so honouring max_obstacles here would drop
  // 70 of them and change which constraints the QP sees - a large behaviour change wearing
  // the costume of a completeness fix. If a later phase wants these applied to the oracle
  // path, that is a deliberate decision with its own acceptance test, not tidying.
  std::vector<::dpcbf::ObstacleState> GetObstacleStates(double now_s);

  // The intermediate the call above routed through, for dumps, diagnostics and the P11
  // evaluator. Valid after GetObstacleStates.
  const std::vector<core::OracleObstacleState>& last_oracle_states() const {
    return oracle_states_;
  }

  // Ground-truth robot pose from the same source. Requires BindModel; not used by mode 1's
  // filter path (see the main.cc hook comment).
  adapters::mujoco::OracleRobotPose SampleRobotPose(const mjModel* model,
                                                    const mjData* data) const {
    return oracle_.SampleRobot(model, data);
  }

  bool bound() const { return bound_; }
  ObstacleSourceMode mode() const { return mode_; }
  const adapters::mujoco::OracleProviderMj& oracle() const { return oracle_; }

  // How many obstacle states the last call produced, and how many the source offered. Equal
  // by construction in oracle mode; exposed so a test can assert that rather than trust it.
  std::size_t last_source_count() const { return oracle_states_.size(); }
  std::size_t last_emitted_count() const { return last_emitted_count_; }

 private:
  ObstacleSourceMode mode_ = ObstacleSourceMode::kOracle;
  bool bound_ = false;

  // Copied at Bind, because BindModel happens later (per model load) and needs base_body.
  // A copy rather than a pointer: PerceptionConfig is immutable after load, but a dangling
  // reference to it would be a silent hazard for a saving of 48 bytes.
  FramesConfig frames_;

  adapters::mujoco::OracleProviderMj oracle_;

  // Reused across calls so the steady state does not allocate more than the returned vector.
  std::vector<core::OracleObstacleState> oracle_states_;
  std::size_t last_emitted_count_ = 0;
};

}  // namespace perception::integration

#endif  // PERCEPTION_INTEGRATION_OBSTACLE_SOURCE_SELECTOR_H_
