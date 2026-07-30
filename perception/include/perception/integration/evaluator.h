// Evaluator - oracle-to-estimate comparison, and the paired DPCBF command comparison.
// Architecture doc §6 ("association by nearest-center, error stats, JSON manifests in the
// dpcbf_rollout_evaluator output style"), §12's DPCBF block, ladder steps 4 and 5.
//
// =========================================================================================
// "FORBIDDEN FROM FEEDING ANYTHING BACK INTO THE ESTIMATOR" - ENFORCED, NOT PROMISED
// =========================================================================================
// §8 names `evaluator -> feeding estimator` a forbidden edge. Three structural facts make it
// unreachable rather than merely discouraged, and they are worth stating because "the class
// does not currently do that" is a claim about code that changes:
//
//   1. THE LINK SET. `perception_evaluator` links `perception_contracts` and
//      `perception_diagnostics`. It does NOT link `perception_core`. SegmentCircleDetector,
//      KfCircleTracker and SafetyStateGenerator are not merely unused here - they are not
//      linkable from this target, so there is no estimator object in reach to feed. This is
//      the same enforcement-by-link-set that keeps core off yaml-cpp and MuJoCo.
//   2. THE SIGNATURES. Every input is a `const&`. Nothing is taken by pointer-to-mutable, so
//      no call can write through an argument into a stage's state.
//   3. THE STORAGE. This class keeps accumulated SCALARS. It stores no pointer, reference or
//      handle to anything it was shown, so nothing survives a call to be reached later.
//
// The one dpcbf-adjacent thing it does hold is `adapters::dpcbf::ProbeOutcome`, which is a
// plain POD from a header that names no dpcbf type (see paired_filter_probe.h). Integration
// therefore still never reaches a dpcbf header, which is the other half of §8's rule.
//
// =========================================================================================
// ASSOCIATION: GLOBAL NEAREST-CENTRE, GREEDY, DETERMINISTIC
// =========================================================================================
// §6 asks for "association by nearest-center". Taken literally, per-estimate nearest-centre is
// not a matching at all - two estimates can claim one oracle obstacle - so what is implemented
// is the smallest well-defined thing that IS a matching: score every (oracle, estimate) pair
// inside `association_gate_m`, sort by distance, and accept greedily while both sides are
// unclaimed. Ties are broken by (oracle id, estimated id) so the result is replayable rather
// than dependent on sort stability.
//
// THE GATE IS GENEROUS ON PURPOSE, and it has to be: P10 measured centre errors up to 0.178 m
// on the shipped pipeline, and an association gate near that size would make the association
// itself part of the measurement. Same rule P8's, P9's and P10's harnesses already follow.
//
// =========================================================================================
// WHAT THE CLEARANCE AND COLLISION NUMBERS ACTUALLY MEAN AT THIS PHASE
// =========================================================================================
// §12 asks for "min-clearance difference >= -0.05 m" and "collision count = 0". Both are
// closed-loop quantities: they are properties of where the robot ENDED UP, and at ladder step
// 4/5 the estimated arm's commands are discarded by definition, so the robot ends up in
// exactly the same place in both arms and the honest difference is identically zero.
//
// Reporting zero would be true and worthless. What is reported instead is an OPEN-LOOP
// LOOKAHEAD PROXY: from the shared robot state, hold each arm's filtered command for
// `clearance_lookahead_s`, advance the true obstacles over the same interval at their true
// velocities, and take the minimum true clearance. It answers "would acting on the estimated
// command have taken the robot nearer a real obstacle than acting on the oracle command", which
// is the question §12 is reaching for and the strongest form of it available without a closed
// loop. It is NOT a campaign min-clearance. P15 measures that; P12 is what first makes it
// possible. Every report of these two numbers must carry that qualifier.
#ifndef PERCEPTION_INTEGRATION_EVALUATOR_H_
#define PERCEPTION_INTEGRATION_EVALUATOR_H_

#include <cstdint>
#include <string>
#include <vector>

#include "perception/adapters/dpcbf/paired_filter_probe.h"
#include "perception/core/contracts/obstacles.h"

namespace perception::integration {

struct EvaluatorParams {
  // Largest centre distance at which an oracle obstacle and an estimate may be called the
  // same object. See the header essay on why this is generous.
  double association_gate_m = 1.0;

  // dpcbf_config.yaml `robot.r_rob`. Passed in rather than read here: `perception_evaluator`
  // does not link yaml-cpp (only `perception_config` may), and a second parser for the DPCBF
  // config would be a second thing to keep in step with it.
  double robot_radius_m = 0.30;

  // A command counts as an intervention when the filter moved it this far from the desired
  // one, in the (sagittal, lateral) plane.
  double intervention_threshold_mps = 1e-3;

  // The open-loop lookahead horizon for the clearance proxy. See the header essay. One second
  // is the same horizon dpcbf_visualizer already uses to draw its relative-velocity arrows.
  double clearance_lookahead_s = 1.0;

  const char* Validate() const;
};

// One accepted oracle/estimate correspondence.
struct MatchedPair {
  double stamp_s = 0.0;
  std::uint32_t estimated_id = 0;
  std::int32_t oracle_id = -1;

  double center_error_m = 0.0;
  double velocity_error_mps = 0.0;

  // Two radius errors, because there are two radii and they answer different questions.
  // `radius_true_error_m` is what the ESTIMATOR believes minus the truth, i.e. the tracked,
  // enlargement-inclusive radius error (see the CORRECTION in contracts/obstacles.h - this is
  // NOT an estimate of the object's real size and is expected to be positive).
  // `radius_inflated_error_m` is what the QP was FED minus the truth: the total inflation the
  // filter actually sees, which is the number §2 of the P11 brief asks for a verdict on.
  double radius_true_error_m = 0.0;
  double radius_inflated_error_m = 0.0;
};

// Per-frame association census.
struct AssociationStats {
  std::int32_t oracle_in = 0;
  std::int32_t estimated_in = 0;
  std::int32_t matched = 0;
  std::int32_t oracle_unmatched = 0;    // Ground truth with no estimate: a miss.
  std::int32_t estimated_unmatched = 0; // An estimate with no ground truth: a spurious object.

  // Both sides are accounted for exactly once, the same invariant ScanStats, DetectionStats,
  // SafetyStats and DpcbfAdapterStats all assert.
  bool IsBalanced() const {
    return oracle_in == matched + oracle_unmatched &&
           estimated_in == matched + estimated_unmatched;
  }
};

// Accumulated estimation error over every matched pair the evaluator has been shown.
struct ErrorSummary {
  std::int64_t frames = 0;
  std::int64_t matched_pairs = 0;
  std::int64_t oracle_unmatched = 0;
  std::int64_t estimated_unmatched = 0;
  std::int64_t oracle_seen = 0;
  std::int64_t estimated_seen = 0;

  double center_rmse_m = 0.0;
  double center_worst_m = 0.0;
  double velocity_rmse_mps = 0.0;
  double velocity_worst_mps = 0.0;

  double radius_true_bias_m = 0.0;       // Mean signed error.
  double radius_inflated_bias_m = 0.0;   // Mean signed error: the mean total inflation seen.
  double radius_inflated_worst_m = 0.0;  // Largest single over-statement fed to the QP.

  double DetectionRate() const {
    return oracle_seen > 0 ? static_cast<double>(matched_pairs) / static_cast<double>(oracle_seen)
                           : 0.0;
  }
  double SpuriousRate() const {
    return estimated_seen > 0
               ? static_cast<double>(estimated_unmatched) / static_cast<double>(estimated_seen)
               : 0.0;
  }
};

// Accumulated difference between the two filter arms.
struct FilterDeltaSummary {
  std::int64_t samples = 0;

  // §12's "command-delta RMSE vs oracle <= 0.1 m/s". Over the (sagittal, lateral) plane, in
  // m/s; the yaw channel is rad/s and is kept separate rather than summed into a norm whose
  // units would be meaningless.
  double command_rmse_mps = 0.0;
  double command_worst_mps = 0.0;
  double yaw_rmse_rps = 0.0;
  double yaw_worst_rps = 0.0;

  std::int64_t oracle_solved = 0;
  std::int64_t estimated_solved = 0;
  std::int64_t oracle_interventions = 0;
  std::int64_t estimated_interventions = 0;

  double oracle_active_constraints_sum = 0.0;
  double estimated_active_constraints_sum = 0.0;
  std::int64_t oracle_obstacles_sum = 0;
  std::int64_t estimated_obstacles_sum = 0;

  // The open-loop lookahead proxy - see the header essay. `worst` is the most NEGATIVE
  // (estimated - oracle) clearance difference over all samples, i.e. the sample on which
  // acting on the estimate would have come closest to being less safe than the oracle.
  double clearance_delta_worst_m = 0.0;
  double clearance_delta_sum_m = 0.0;
  double oracle_min_clearance_worst_m = 0.0;
  double estimated_min_clearance_worst_m = 0.0;
  std::int64_t oracle_lookahead_collisions = 0;
  std::int64_t estimated_lookahead_collisions = 0;

  double filter_wall_us_sum = 0.0;
  double filter_wall_us_worst = 0.0;

  double QpFeasibility(std::int64_t solved) const {
    return samples > 0 ? static_cast<double>(solved) / static_cast<double>(samples) : 0.0;
  }
  double InterventionRate(std::int64_t interventions) const {
    return samples > 0 ? static_cast<double>(interventions) / static_cast<double>(samples) : 0.0;
  }
};

// Reads both paths, writes neither. See the header essay.
class Evaluator {
 public:
  explicit Evaluator(const EvaluatorParams& params);

  // One frame of oracle-vs-estimate association. `matched` may be null; when it is not, it is
  // CLEARED and filled with this frame's correspondences. `stats` may be null.
  // Returns nullptr or a STATIC reason string.
  const char* ObserveFrame(double stamp_s,
                           const std::vector<core::OracleObstacleState>& oracle,
                           const std::vector<core::SafetyObstacle>& estimated,
                           std::vector<MatchedPair>* matched = nullptr,
                           AssociationStats* stats = nullptr);

  // One paired filter sample. `truth` is the ground-truth obstacle set at this instant and is
  // used ONLY for the clearance proxy - both arms are scored against the same real geometry,
  // so an arm cannot buy clearance by inflating its own radii.
  const char* ObserveFilterPair(const adapters::dpcbf::ProbeRobotState& robot,
                                const adapters::dpcbf::ProbeCommand& desired,
                                const std::vector<core::OracleObstacleState>& truth,
                                const adapters::dpcbf::ProbeOutcome& oracle,
                                const adapters::dpcbf::ProbeOutcome& estimated);

  const ErrorSummary& errors() const { return errors_; }
  const FilterDeltaSummary& deltas() const { return deltas_; }
  const EvaluatorParams& params() const { return params_; }

  // A run manifest in the dpcbf_rollout_evaluator / run_manifest.json style (doc §2.1, §14):
  // pretty two-space JSON, every metric named, plus the qualifiers a reader needs in order not
  // to quote a lookahead proxy as a campaign measurement.
  std::string ToJsonManifest(const std::string& run_name) const;

  void Reset();

 private:
  // Squared-error accumulators, kept apart from the reported RMSEs so the summary stays a
  // plain value type that a caller can copy and diff.
  double center_sq_sum_ = 0.0;
  double velocity_sq_sum_ = 0.0;
  double radius_true_sum_ = 0.0;
  double radius_inflated_sum_ = 0.0;
  double command_sq_sum_ = 0.0;
  double yaw_sq_sum_ = 0.0;

  EvaluatorParams params_;
  ErrorSummary errors_;
  FilterDeltaSummary deltas_;

  // Scratch for the association, reserved on construction. A per-frame std::vector here would
  // be exactly the hidden allocation risk R14 exists to forbid.
  struct Candidate {
    double distance = 0.0;
    std::int32_t oracle_index = 0;
    std::int32_t estimated_index = 0;
  };
  std::vector<Candidate> candidates_;
  std::vector<std::uint8_t> oracle_claimed_;
  std::vector<std::uint8_t> estimated_claimed_;
};

}  // namespace perception::integration

#endif  // PERCEPTION_INTEGRATION_EVALUATOR_H_
