#include "perception/core/safety/safety_state_generator.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace perception::core {

const char* SafetyParams::Validate() const {
  if (!IsFinitePositive(max_age_s)) return "SafetyParams::max_age_s must be > 0";
  if (!IsFiniteNonNegative(min_track_age_s)) {
    return "SafetyParams::min_track_age_s must be finite and >= 0";
  }
  if (min_track_hits < 1) return "SafetyParams::min_track_hits must be >= 1";
  if (!IsFiniteNonNegative(radius_inflation_k_sigma)) {
    return "SafetyParams::radius_inflation_k_sigma must be finite and >= 0";
  }
  if (!IsFiniteNonNegative(radius_inflation_fixed_m)) {
    return "SafetyParams::radius_inflation_fixed_m must be finite and >= 0";
  }
  if (!IsFiniteNonNegative(latency_inflation_s)) {
    return "SafetyParams::latency_inflation_s must be finite and >= 0";
  }
  if (!IsFinitePositive(min_radius_m)) return "SafetyParams::min_radius_m must be > 0";
  if (!IsFinitePositive(max_radius_m)) return "SafetyParams::max_radius_m must be > 0";
  if (min_radius_m > max_radius_m) {
    return "SafetyParams::min_radius_m must be <= max_radius_m";
  }
  if (!IsFinitePositive(max_speed_mps)) return "SafetyParams::max_speed_mps must be > 0";
  if (max_obstacles < 1) return "SafetyParams::max_obstacles must be >= 1";
  return nullptr;
}

SafetyStateGenerator::SafetyStateGenerator(const SafetyParams& params) : params_(params) {
  order_.reserve(static_cast<size_t>(params_.max_obstacles) + 1);
}

int32_t SafetyStateGenerator::HitsOf(const std::vector<TrackState2D>& tracks, uint32_t id) {
  for (const TrackState2D& track : tracks) {
    if (track.id == id) return track.hits;
  }
  return -1;
}

const char* SafetyStateGenerator::Generate(const Tracking2DResult& tracking, double now_s,
                                           SafetyStateResult* out) {
  if (out == nullptr) return "SafetyStateGenerator::Generate needs an output";
  out->Clear();
  const auto wall_start = std::chrono::steady_clock::now();

  if (!IsFinite(now_s)) return "SafetyStateGenerator::Generate now_s must be finite";
  if (const char* reason = params_.Validate(); reason != nullptr) return reason;

  SafetyStats& stats = out->stats;
  stats.input_obstacles = static_cast<int32_t>(tracking.obstacles.size());
  stats.input_tracks = static_cast<int32_t>(tracking.tracks.size());
  for (const TrackState2D& track : tracking.tracks) {
    if (track.status == TrackStatus::kTentative) ++stats.tentative_suppressed;
  }

  out->obstacles.reserve(std::min<size_t>(tracking.obstacles.size(),
                                          static_cast<size_t>(params_.max_obstacles)));

  for (const PerceptionObstacle& obstacle : tracking.obstacles) {
    // ---- 1. Screen the input. -------------------------------------------------------------
    // The tracker already refuses to emit an obstacle that fails Validate(), so reaching this
    // branch means the two stages disagree about the contract. Counted rather than trusted.
    if (!obstacle.IsValid()) {
      ++stats.dropped_invalid_input;
      continue;
    }

    // ---- 2. Staleness, measured against the MEASUREMENT stamp. ------------------------------
    // PerceptionObstacle::last_update_stamp_s is TrackState2D::last_measurement_stamp_s, not the
    // advanced-to stamp (kf_circle_tracker.cpp ObstacleOf, and the essay there on why). Using
    // the advanced-to stamp instead would make every coasting track look brand new and disable
    // this gate silently, which is the failure mode this comment exists to keep visible.
    const double age_s = now_s - obstacle.last_update_stamp_s;
    if (age_s < 0.0) {
      // A state stamped in the future is a clock fault. It is dropped rather than clamped to
      // zero: clamping would publish it as maximally fresh, which is the least safe reading of
      // a symptom whose cause is unknown.
      ++stats.dropped_future_stamp;
      continue;
    }
    if (age_s > params_.max_age_s) {
      // SECTION 11'S SAFETY-DOMINANCE POLICY, APPLIED. A stale track is DROPPED, never shrunk
      // toward invalidity and never emitted with a reduced radius: a radius that decays with
      // age is a safety circle that stops containing the object exactly when the evidence for
      // where it is has gone. Absence hands the decision to the section-16 per-frame fallback,
      // which is the mechanism designed to make it.
      ++stats.dropped_stale;
      continue;
    }

    // ---- 3. Confirmation gating. ------------------------------------------------------------
    if (obstacle.track_age_s < params_.min_track_age_s) {
      ++stats.dropped_young_age;
      continue;
    }
    // `hits` is not on PerceptionObstacle and is not recoverable from it: `confidence` is the
    // hits/(hits+misses) RATIO and `track_age_s` is a duration, so neither yields a count. The
    // join against TrackState2D is what makes safety.min_track_hits enforceable at all; without
    // it the key would be silently inert whenever it exceeded tracking.confirm_hits.
    const int32_t hits = HitsOf(tracking.tracks, obstacle.id);
    if (hits < 0) {
      ++stats.dropped_no_track;
      continue;
    }
    if (hits < params_.min_track_hits) {
      ++stats.dropped_few_hits;
      continue;
    }

    // ---- 4. Velocity spike clamp - risk R10's mitigation. -----------------------------------
    const double speed_raw = obstacle.velocity.norm();
    Eigen::Vector2d velocity = obstacle.velocity;
    if (speed_raw > params_.max_speed_mps) {
      // Direction preserved, magnitude bounded. speed_raw > max_speed_mps > 0 so the division
      // is safe.
      velocity *= params_.max_speed_mps / speed_raw;
      ++stats.velocity_clamped;
    }

    // ---- 5. Conservative inflation. ---------------------------------------------------------
    // The horizon is elapsed-plus-consumption; see the header essay on why `age_s` belongs in
    // it and why using it is conservative rather than tuned.
    const double horizon_s = age_s + params_.latency_inflation_s;

    double base_radius = obstacle.radius_true_m;
    if (params_.use_enclosing_radius) {
      // max(fitted, enclosing). PerceptionObstacle carries no enclosing radius, so the term has
      // nothing to select and this is a no-op by construction rather than by accident. Written
      // out so the absent operand is visible at the point the doc's formula would use it.
      constexpr double kEnclosingRadiusUnavailable = 0.0;
      base_radius = std::max(base_radius, kEnclosingRadiusUnavailable);
    }
    if (base_radius < params_.min_radius_m) {
      // Q13's floor, applied to the BASE and not to the inflated result - it is a statement
      // about how small an object may be believed to be, and satisfying it with margin that
      // exists for a different reason would make it unfalsifiable.
      base_radius = params_.min_radius_m;
      ++stats.radius_floored;
    }

    const double sigma_radius = std::sqrt(obstacle.radius_variance);
    // An isotropic bound on a per-axis covariance: the larger axis, so the disc of radius
    // k*sigma contains the k-sigma ellipse rather than merely touching it.
    const double sigma_position =
        std::sqrt(std::max(obstacle.center_variance.x(), obstacle.center_variance.y()));

    // The unclamped speed drives the drift term - see the header essay. The ceiling below is
    // what keeps a spike from turning into an unbounded circle.
    const double position_inflation =
        params_.radius_inflation_k_sigma * sigma_position + horizon_s * speed_raw;

    double radius_inflated = base_radius +
                             params_.radius_inflation_k_sigma * sigma_radius +
                             params_.radius_inflation_fixed_m + position_inflation;

    if (radius_inflated > params_.max_radius_m) {
      // The ceiling stops a degenerate fit or a velocity spike inflating into a wall. It is
      // NEVER allowed below the radius the estimator actually reported: clipping a safety
      // radius under the estimate is under-estimation, which section 11 forbids outright, and
      // it would also break SafetyObstacle's own radius_inflated_m >= radius_true_m invariant.
      radius_inflated = std::max(params_.max_radius_m, obstacle.radius_true_m);
      ++stats.radius_ceiled;
    }

    SafetyObstacle safety;
    safety.id = obstacle.id;
    safety.center = obstacle.center;
    safety.velocity = velocity;
    // PASS-THROUGH, DELIBERATELY UNMODIFIED. Whatever radius_true_m holds - and on the shipped
    // pipeline it holds the enlargement-inclusive tracked radius, not a de-enlarged one - is
    // what the evaluator needs to see in order to attribute the inflation. De-enlarging here is
    // forbidden; see the header essay.
    safety.radius_true_m = obstacle.radius_true_m;
    safety.radius_inflated_m = radius_inflated;
    safety.position_inflation_m = position_inflation;
    safety.stamp_s = obstacle.last_update_stamp_s;
    safety.age_s = age_s;
    // Everything that reaches here passed every gate. This stage DROPS rather than marks: the
    // false state of this flag is reserved for a default-constructed object, and the DPCBF
    // adapter's `drop_invalid` is a belt-and-braces check on a set that already has none.
    safety.valid = true;
    safety.source = ObstacleSource::kEstimated;
    safety.frame = FrameId::kWorld;

    if (!safety.IsValid()) {
      // Unreachable by construction given the gates above; kept because "unreachable by
      // construction" is a claim about code that changes, and the alternative to counting it is
      // feeding DPCBF an object the contract rejects.
      ++stats.dropped_invalid_output;
      continue;
    }

    out->obstacles.push_back(safety);
    // Both "worst" figures are over EMITTED obstacles only, so they describe what DPCBF was
    // actually given rather than what the stage considered.
    stats.worst_age_s = std::max(stats.worst_age_s, safety.age_s);
    stats.worst_inflation_m =
        std::max(stats.worst_inflation_m, safety.radius_inflated_m - safety.radius_true_m);
  }

  // ---- 6. Capacity. -------------------------------------------------------------------------
  // safety.max_obstacles <= tracking.max_tracks is a validated cross-field constraint, so this
  // fires only when more tracks are simultaneously confirmed than the safety stage will carry.
  //
  // WHICH ONES SURVIVE. Not insertion order, which is the tracker's internal churn and would
  // make the cull non-reproducible in intent even where it is deterministic in fact. Freshest
  // first (smallest age), then largest inflated radius, then smallest id: the first key keeps
  // the best-evidenced states, the second prefers the bigger hazard among equally fresh ones,
  // and the third makes the order total so the result is replayable.
  if (static_cast<int>(out->obstacles.size()) > params_.max_obstacles) {
    order_.clear();
    for (int32_t i = 0; i < static_cast<int32_t>(out->obstacles.size()); ++i) order_.push_back(i);
    const std::vector<SafetyObstacle>& obstacles = out->obstacles;
    std::sort(order_.begin(), order_.end(), [&obstacles](int32_t a, int32_t b) {
      const SafetyObstacle& lhs = obstacles[static_cast<size_t>(a)];
      const SafetyObstacle& rhs = obstacles[static_cast<size_t>(b)];
      if (lhs.age_s != rhs.age_s) return lhs.age_s < rhs.age_s;
      if (lhs.radius_inflated_m != rhs.radius_inflated_m) {
        return lhs.radius_inflated_m > rhs.radius_inflated_m;
      }
      return lhs.id < rhs.id;
    });
    order_.resize(static_cast<size_t>(params_.max_obstacles));
    // Restore the tracker's emission order among the survivors, so the cull changes WHICH
    // obstacles are published without also reordering the ones that were going to be.
    std::sort(order_.begin(), order_.end());

    stats.dropped_capacity =
        static_cast<int32_t>(out->obstacles.size()) - params_.max_obstacles;
    for (size_t i = 0; i < order_.size(); ++i) {
      out->obstacles[i] = out->obstacles[static_cast<size_t>(order_[i])];
    }
    out->obstacles.resize(order_.size());
  }

  stats.emitted = static_cast<int32_t>(out->obstacles.size());

  const auto wall_end = std::chrono::steady_clock::now();
  stats.wall_time_us = std::chrono::duration<double, std::micro>(wall_end - wall_start).count();
  return nullptr;
}

}  // namespace perception::core
