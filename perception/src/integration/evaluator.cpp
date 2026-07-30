#include "perception/integration/evaluator.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "perception/core/diagnostics/json_value.h"

namespace perception::integration {
namespace {

using core::diagnostics::JsonTextWriter;

// The minimum true clearance the robot would have, `horizon_s` from now, if it held `command`
// and every obstacle held its own velocity. Clearance is measured against the TRUE radii, so
// neither arm can buy margin by inflating what it reports.
//
// The motion model is the unicycle-with-lateral DPCBF itself assumes: RobotState carries a
// heading plus body-frame sagittal/lateral velocities, and VelocityCommand is expressed in the
// same pair. Nothing more elaborate is warranted for a lookahead proxy.
double MinLookaheadClearance(const adapters::dpcbf::ProbeRobotState& robot,
                             const adapters::dpcbf::ProbeCommand& command,
                             const std::vector<core::OracleObstacleState>& truth,
                             double horizon_s, double robot_radius_m) {
  const double cos_phi = std::cos(robot.phi);
  const double sin_phi = std::sin(robot.phi);
  const double vx = command.sagittal * cos_phi - command.lateral * sin_phi;
  const double vy = command.sagittal * sin_phi + command.lateral * cos_phi;
  const double x = robot.x + vx * horizon_s;
  const double y = robot.y + vy * horizon_s;

  double worst = std::numeric_limits<double>::infinity();
  for (const core::OracleObstacleState& obstacle : truth) {
    const double ox = obstacle.center.x() + obstacle.velocity.x() * horizon_s;
    const double oy = obstacle.center.y() + obstacle.velocity.y() * horizon_s;
    const double dx = x - ox;
    const double dy = y - oy;
    const double clearance = std::sqrt(dx * dx + dy * dy) - obstacle.radius_m - robot_radius_m;
    worst = std::min(worst, clearance);
  }
  // An empty truth set has no clearance to report. Zero would read as "touching", which is the
  // opposite of what an empty arena means, so it is reported as unbounded and the caller's
  // "worst" accumulators leave it alone.
  return worst;
}

bool IsIntervention(const adapters::dpcbf::ProbeCommand& desired,
                    const adapters::dpcbf::ProbeCommand& actual, double threshold) {
  const double ds = actual.sagittal - desired.sagittal;
  const double dl = actual.lateral - desired.lateral;
  return std::sqrt(ds * ds + dl * dl) > threshold;
}

}  // namespace

const char* EvaluatorParams::Validate() const {
  if (!(association_gate_m > 0.0)) return "evaluator.association_gate_m must be > 0";
  if (!(robot_radius_m >= 0.0)) return "evaluator.robot_radius_m must be >= 0";
  if (!(intervention_threshold_mps >= 0.0)) {
    return "evaluator.intervention_threshold_mps must be >= 0";
  }
  if (!(clearance_lookahead_s > 0.0)) return "evaluator.clearance_lookahead_s must be > 0";
  return nullptr;
}

Evaluator::Evaluator(const EvaluatorParams& params) : params_(params) {
  candidates_.reserve(256);
  oracle_claimed_.reserve(128);
  estimated_claimed_.reserve(128);
}

void Evaluator::Reset() {
  center_sq_sum_ = 0.0;
  velocity_sq_sum_ = 0.0;
  radius_true_sum_ = 0.0;
  radius_inflated_sum_ = 0.0;
  command_sq_sum_ = 0.0;
  yaw_sq_sum_ = 0.0;
  errors_ = ErrorSummary{};
  deltas_ = FilterDeltaSummary{};
}

const char* Evaluator::ObserveFrame(double stamp_s,
                                    const std::vector<core::OracleObstacleState>& oracle,
                                    const std::vector<core::SafetyObstacle>& estimated,
                                    std::vector<MatchedPair>* matched, AssociationStats* stats) {
  if (const char* reason = params_.Validate()) return reason;
  if (matched != nullptr) matched->clear();

  AssociationStats census;
  census.oracle_in = static_cast<std::int32_t>(oracle.size());
  census.estimated_in = static_cast<std::int32_t>(estimated.size());

  // ---- 1. Every pair inside the gate becomes a candidate. ---------------------------------
  candidates_.clear();
  for (std::size_t o = 0; o < oracle.size(); ++o) {
    for (std::size_t e = 0; e < estimated.size(); ++e) {
      const double distance = (estimated[e].center - oracle[o].center).norm();
      if (!(distance <= params_.association_gate_m)) continue;
      candidates_.push_back({distance, static_cast<std::int32_t>(o),
                             static_cast<std::int32_t>(e)});
    }
  }

  // ---- 2. Greedy over the gated candidates, nearest first. --------------------------------
  // The tie-break is on the two IDS and not on the two indices: indices are the tracker's and
  // the oracle provider's internal emission order, which is not a quantity any acceptance
  // claim should depend on, while ids are stable and carried in the contracts.
  std::sort(candidates_.begin(), candidates_.end(),
            [&oracle, &estimated](const Candidate& lhs, const Candidate& rhs) {
              if (lhs.distance != rhs.distance) return lhs.distance < rhs.distance;
              const std::int32_t lhs_oracle = oracle[static_cast<std::size_t>(lhs.oracle_index)].id;
              const std::int32_t rhs_oracle = oracle[static_cast<std::size_t>(rhs.oracle_index)].id;
              if (lhs_oracle != rhs_oracle) return lhs_oracle < rhs_oracle;
              return estimated[static_cast<std::size_t>(lhs.estimated_index)].id <
                     estimated[static_cast<std::size_t>(rhs.estimated_index)].id;
            });

  oracle_claimed_.assign(oracle.size(), 0);
  estimated_claimed_.assign(estimated.size(), 0);

  for (const Candidate& candidate : candidates_) {
    const auto o = static_cast<std::size_t>(candidate.oracle_index);
    const auto e = static_cast<std::size_t>(candidate.estimated_index);
    if (oracle_claimed_[o] != 0 || estimated_claimed_[e] != 0) continue;
    oracle_claimed_[o] = 1;
    estimated_claimed_[e] = 1;

    const core::OracleObstacleState& truth = oracle[o];
    const core::SafetyObstacle& estimate = estimated[e];

    MatchedPair pair;
    pair.stamp_s = stamp_s;
    pair.estimated_id = estimate.id;
    pair.oracle_id = truth.id;
    pair.center_error_m = candidate.distance;
    pair.velocity_error_mps = (estimate.velocity - truth.velocity).norm();
    pair.radius_true_error_m = estimate.radius_true_m - truth.radius_m;
    pair.radius_inflated_error_m = estimate.radius_inflated_m - truth.radius_m;

    ++census.matched;
    ++errors_.matched_pairs;
    center_sq_sum_ += pair.center_error_m * pair.center_error_m;
    velocity_sq_sum_ += pair.velocity_error_mps * pair.velocity_error_mps;
    radius_true_sum_ += pair.radius_true_error_m;
    radius_inflated_sum_ += pair.radius_inflated_error_m;
    errors_.center_worst_m = std::max(errors_.center_worst_m, pair.center_error_m);
    errors_.velocity_worst_mps = std::max(errors_.velocity_worst_mps, pair.velocity_error_mps);
    errors_.radius_inflated_worst_m =
        std::max(errors_.radius_inflated_worst_m, pair.radius_inflated_error_m);

    if (matched != nullptr) matched->push_back(pair);
  }

  census.oracle_unmatched = census.oracle_in - census.matched;
  census.estimated_unmatched = census.estimated_in - census.matched;

  ++errors_.frames;
  errors_.oracle_seen += census.oracle_in;
  errors_.estimated_seen += census.estimated_in;
  errors_.oracle_unmatched += census.oracle_unmatched;
  errors_.estimated_unmatched += census.estimated_unmatched;

  const auto pairs = static_cast<double>(errors_.matched_pairs);
  if (errors_.matched_pairs > 0) {
    errors_.center_rmse_m = std::sqrt(center_sq_sum_ / pairs);
    errors_.velocity_rmse_mps = std::sqrt(velocity_sq_sum_ / pairs);
    errors_.radius_true_bias_m = radius_true_sum_ / pairs;
    errors_.radius_inflated_bias_m = radius_inflated_sum_ / pairs;
  }

  if (stats != nullptr) *stats = census;
  return nullptr;
}

const char* Evaluator::ObserveFilterPair(const adapters::dpcbf::ProbeRobotState& robot,
                                         const adapters::dpcbf::ProbeCommand& desired,
                                         const std::vector<core::OracleObstacleState>& truth,
                                         const adapters::dpcbf::ProbeOutcome& oracle,
                                         const adapters::dpcbf::ProbeOutcome& estimated) {
  if (const char* reason = params_.Validate()) return reason;

  ++deltas_.samples;

  const double ds = estimated.command.sagittal - oracle.command.sagittal;
  const double dl = estimated.command.lateral - oracle.command.lateral;
  const double planar = std::sqrt(ds * ds + dl * dl);
  const double dyaw = std::abs(estimated.command.yaw_rate - oracle.command.yaw_rate);

  command_sq_sum_ += planar * planar;
  yaw_sq_sum_ += dyaw * dyaw;
  deltas_.command_worst_mps = std::max(deltas_.command_worst_mps, planar);
  deltas_.yaw_worst_rps = std::max(deltas_.yaw_worst_rps, dyaw);

  const auto samples = static_cast<double>(deltas_.samples);
  deltas_.command_rmse_mps = std::sqrt(command_sq_sum_ / samples);
  deltas_.yaw_rmse_rps = std::sqrt(yaw_sq_sum_ / samples);

  if (oracle.solved) ++deltas_.oracle_solved;
  if (estimated.solved) ++deltas_.estimated_solved;
  if (IsIntervention(desired, oracle.command, params_.intervention_threshold_mps)) {
    ++deltas_.oracle_interventions;
  }
  if (IsIntervention(desired, estimated.command, params_.intervention_threshold_mps)) {
    ++deltas_.estimated_interventions;
  }

  deltas_.oracle_active_constraints_sum += oracle.active_constraints;
  deltas_.estimated_active_constraints_sum += estimated.active_constraints;
  deltas_.oracle_obstacles_sum += oracle.obstacles_supplied;
  deltas_.estimated_obstacles_sum += estimated.obstacles_supplied;

  deltas_.filter_wall_us_sum += oracle.filter_wall_us + estimated.filter_wall_us;
  deltas_.filter_wall_us_worst =
      std::max({deltas_.filter_wall_us_worst, oracle.filter_wall_us, estimated.filter_wall_us});

  // The open-loop lookahead clearance proxy. See the header essay for what it is and is not.
  if (!truth.empty()) {
    const double oracle_clearance = MinLookaheadClearance(
        robot, oracle.command, truth, params_.clearance_lookahead_s, params_.robot_radius_m);
    const double estimated_clearance = MinLookaheadClearance(
        robot, estimated.command, truth, params_.clearance_lookahead_s, params_.robot_radius_m);
    const double difference = estimated_clearance - oracle_clearance;

    deltas_.clearance_delta_sum_m += difference;
    deltas_.clearance_delta_worst_m = std::min(deltas_.clearance_delta_worst_m, difference);
    // Initialised to 0.0, so the first sample has to be able to raise it as well as lower it;
    // these two track the SMALLEST clearance each arm ever had, which is what "min clearance"
    // means, and a clearance is allowed to be negative.
    if (deltas_.samples == 1) {
      deltas_.oracle_min_clearance_worst_m = oracle_clearance;
      deltas_.estimated_min_clearance_worst_m = estimated_clearance;
    } else {
      deltas_.oracle_min_clearance_worst_m =
          std::min(deltas_.oracle_min_clearance_worst_m, oracle_clearance);
      deltas_.estimated_min_clearance_worst_m =
          std::min(deltas_.estimated_min_clearance_worst_m, estimated_clearance);
    }
    if (oracle_clearance < 0.0) ++deltas_.oracle_lookahead_collisions;
    if (estimated_clearance < 0.0) ++deltas_.estimated_lookahead_collisions;
  }

  return nullptr;
}

std::string Evaluator::ToJsonManifest(const std::string& run_name) const {
  JsonTextWriter writer(true);
  writer.BeginObject();

  writer.Key("run");
  writer.String(run_name);
  writer.Key("producer");
  writer.String("perception/integration/Evaluator (P11)");

  // The qualifiers travel WITH the numbers. A manifest whose caveats live in a phase report
  // is a manifest whose caveats get lost the first time someone greps it for a metric.
  writer.Key("measurement_notes");
  writer.BeginObject();
  writer.Key("clearance_and_collisions");
  writer.String(
      "OPEN-LOOP LOOKAHEAD PROXY, not a campaign measurement. At ladder step 4/5 the estimated "
      "arm's commands are discarded, so both arms share one robot trajectory and a true "
      "closed-loop clearance difference is identically zero. These figures hold each arm's "
      "filtered command for clearance_lookahead_s from the shared state and score both against "
      "the same true geometry. Genuine min-clearance and collision counts require the closed "
      "loop: P12 wires it, P15 measures it.");
  writer.Key("statistical_power");
  writer.String(
      "First read on a limited corpus. Campaign-scale statistical power is P15 by doc section "
      "10; section 12 tags the DPCBF block (P11/P15) for exactly this reason.");
  writer.Key("jsonl_number_note");
  writer.String("NaN and infinities are written as the bare tokens NaN / Infinity / -Infinity.");
  writer.EndObject();

  writer.Key("params");
  writer.BeginObject();
  writer.Key("association_gate_m");
  writer.Double(params_.association_gate_m);
  writer.Key("robot_radius_m");
  writer.Double(params_.robot_radius_m);
  writer.Key("intervention_threshold_mps");
  writer.Double(params_.intervention_threshold_mps);
  writer.Key("clearance_lookahead_s");
  writer.Double(params_.clearance_lookahead_s);
  writer.EndObject();

  writer.Key("estimation");
  writer.BeginObject();
  writer.Key("frames");
  writer.Int64(errors_.frames);
  writer.Key("oracle_seen");
  writer.Int64(errors_.oracle_seen);
  writer.Key("estimated_seen");
  writer.Int64(errors_.estimated_seen);
  writer.Key("matched_pairs");
  writer.Int64(errors_.matched_pairs);
  writer.Key("oracle_unmatched");
  writer.Int64(errors_.oracle_unmatched);
  writer.Key("estimated_unmatched");
  writer.Int64(errors_.estimated_unmatched);
  writer.Key("detection_rate");
  writer.Double(errors_.DetectionRate());
  writer.Key("spurious_rate");
  writer.Double(errors_.SpuriousRate());
  writer.Key("center_rmse_m");
  writer.Double(errors_.center_rmse_m);
  writer.Key("center_worst_m");
  writer.Double(errors_.center_worst_m);
  writer.Key("velocity_rmse_mps");
  writer.Double(errors_.velocity_rmse_mps);
  writer.Key("velocity_worst_mps");
  writer.Double(errors_.velocity_worst_mps);
  writer.Key("radius_true_bias_m");
  writer.Double(errors_.radius_true_bias_m);
  writer.Key("radius_inflated_bias_m");
  writer.Double(errors_.radius_inflated_bias_m);
  writer.Key("radius_inflated_worst_m");
  writer.Double(errors_.radius_inflated_worst_m);
  writer.EndObject();

  writer.Key("dpcbf_paired");
  writer.BeginObject();
  writer.Key("samples");
  writer.Int64(deltas_.samples);
  writer.Key("command_rmse_mps");
  writer.Double(deltas_.command_rmse_mps);
  writer.Key("command_worst_mps");
  writer.Double(deltas_.command_worst_mps);
  writer.Key("yaw_rmse_rps");
  writer.Double(deltas_.yaw_rmse_rps);
  writer.Key("yaw_worst_rps");
  writer.Double(deltas_.yaw_worst_rps);
  writer.Key("oracle_qp_feasibility");
  writer.Double(deltas_.QpFeasibility(deltas_.oracle_solved));
  writer.Key("estimated_qp_feasibility");
  writer.Double(deltas_.QpFeasibility(deltas_.estimated_solved));
  writer.Key("oracle_intervention_rate");
  writer.Double(deltas_.InterventionRate(deltas_.oracle_interventions));
  writer.Key("estimated_intervention_rate");
  writer.Double(deltas_.InterventionRate(deltas_.estimated_interventions));
  writer.Key("oracle_mean_active_constraints");
  writer.Double(deltas_.samples > 0 ? deltas_.oracle_active_constraints_sum /
                                          static_cast<double>(deltas_.samples)
                                    : 0.0);
  writer.Key("estimated_mean_active_constraints");
  writer.Double(deltas_.samples > 0 ? deltas_.estimated_active_constraints_sum /
                                          static_cast<double>(deltas_.samples)
                                    : 0.0);
  writer.Key("oracle_mean_obstacles_supplied");
  writer.Double(deltas_.samples > 0 ? static_cast<double>(deltas_.oracle_obstacles_sum) /
                                          static_cast<double>(deltas_.samples)
                                    : 0.0);
  writer.Key("estimated_mean_obstacles_supplied");
  writer.Double(deltas_.samples > 0 ? static_cast<double>(deltas_.estimated_obstacles_sum) /
                                          static_cast<double>(deltas_.samples)
                                    : 0.0);
  writer.Key("lookahead_clearance_delta_worst_m");
  writer.Double(deltas_.clearance_delta_worst_m);
  writer.Key("lookahead_clearance_delta_mean_m");
  writer.Double(deltas_.samples > 0
                    ? deltas_.clearance_delta_sum_m / static_cast<double>(deltas_.samples)
                    : 0.0);
  writer.Key("oracle_lookahead_min_clearance_m");
  writer.Double(deltas_.oracle_min_clearance_worst_m);
  writer.Key("estimated_lookahead_min_clearance_m");
  writer.Double(deltas_.estimated_min_clearance_worst_m);
  writer.Key("oracle_lookahead_collisions");
  writer.Int64(deltas_.oracle_lookahead_collisions);
  writer.Key("estimated_lookahead_collisions");
  writer.Int64(deltas_.estimated_lookahead_collisions);
  writer.Key("filter_wall_us_mean");
  writer.Double(deltas_.samples > 0 ? deltas_.filter_wall_us_sum /
                                          (2.0 * static_cast<double>(deltas_.samples))
                                    : 0.0);
  writer.Key("filter_wall_us_worst");
  writer.Double(deltas_.filter_wall_us_worst);
  writer.EndObject();

  writer.EndObject();
  return writer.text();
}

}  // namespace perception::integration
