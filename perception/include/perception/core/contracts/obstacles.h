// PerceptionObstacle, SafetyObstacle and OracleObstacleState - the three world-frame
// obstacle views the rest of the system consumes.
//
// SafetyObstacle is the direct precursor of dpcbf::ObstacleState {x, y, radius,
// velocity_x, velocity_y, id}. It deliberately does NOT include the dpcbf header: core
// must not depend on dpcbf (architecture doc section 8), and the conversion lives in
// adapters/dpcbf. What SafetyObstacle adds over the DPCBF POD is exactly what the DPCBF
// interface lacks and the caller is responsible for - a timestamp, a validity flag, and
// the distinction between the true radius and the inflated one that gets fed to the QP.
#ifndef PERCEPTION_CORE_CONTRACTS_OBSTACLES_H_
#define PERCEPTION_CORE_CONTRACTS_OBSTACLES_H_

#include <cstdint>

#include <Eigen/Core>

#include "perception/core/contracts/frames.h"
#include "perception/core/contracts/validation.h"

namespace perception::core {

enum class ObstacleSource : uint8_t {
  kUnknown = 0,
  kEstimated,  // Came out of the perception pipeline.
  kOracle,     // Came from DynamicObstacleManager ground truth.
};

inline const char* ToString(ObstacleSource source) {
  switch (source) {
    case ObstacleSource::kEstimated: return "estimated";
    case ObstacleSource::kOracle:    return "oracle";
    case ObstacleSource::kUnknown:   break;
  }
  return "unknown";
}

// The per-frame view of a CONFIRMED track.
//
// CORRECTION, P10 (this comment previously said the opposite, and the safety stage is where
// believing it would have done damage). `radius_true_m` is NOT upstream's `true_radius`. It is
// `TrackState2D::radius_m`, which is the Kalman filter driven by
// `CircleObservation::radius_fitted_m`, and that observation is
// `0.5773502 * chord + detection.radius_enlargement_m` (segment_circle_detector.cpp). The
// enlargement - 0.25 m as shipped - is INSIDE this number. In upstream's own vocabulary this
// field carries `CircleObstacle::radius`; `true_radius` is `radius - radius_enlargement` and no
// stage computes it after detection.
//
// The size of the discrepancy, measured on the P8 corpus: this field over-states the real
// obstacle radius by +0.089 m to +0.278 m. That over-statement is not a defect - it is the only
// term in the pipeline covering P8's short-arc UNDER-estimate of up to 0.161 m, which is why
// the safety stage is forbidden from subtracting it back off and why
// PerceptionConfig::Validate() enforces a floor on the two terms that provide it.
//
// The name is kept rather than corrected because renaming a frozen contract field would touch
// every consumer for no behavioural gain; what it means is written down here instead.
struct PerceptionObstacle {
  uint32_t id = 0;  // The originating TrackState2D::id; stable across frames.

  Eigen::Vector2d center{0.0, 0.0};
  Eigen::Vector2d velocity{0.0, 0.0};
  double radius_true_m = 0.0;  // See the CORRECTION above: enlargement-inclusive, not "true".

  // Variances, matching TrackState2D's units and per-axis structure (m^2, (m/s)^2, m^2).
  Eigen::Vector2d center_variance{0.0, 0.0};
  Eigen::Vector2d velocity_variance{0.0, 0.0};
  double radius_variance = 0.0;

  double last_update_stamp_s = 0.0;
  double track_age_s = 0.0;  // Same definition as TrackState2D::age_s(): a duration.

  // CONFIDENCE DEFINITION: the track's lifetime hit ratio, hits / (hits + misses), in
  // [0, 1]. It is NOT a probability of existence and NOT a fit quality; it is the single
  // number TrackState2D::HitRatio() returns, copied here so consumers do not have to
  // reach back into the tracker's private state to recompute it.
  double confidence = 0.0;

  FrameId frame = FrameId::kWorld;

  const char* Validate() const {
    if (id == 0) return "PerceptionObstacle::id must be non-zero";
    if (frame != FrameId::kWorld) return "PerceptionObstacle::frame must be kWorld";
    if (!IsFinite(center) || !IsFinite(velocity)) {
      return "PerceptionObstacle centre/velocity must be finite";
    }
    if (!IsFinitePositive(radius_true_m)) return "PerceptionObstacle::radius_true_m must be > 0";
    if (!IsFiniteNonNegative(center_variance.x()) || !IsFiniteNonNegative(center_variance.y()) ||
        !IsFiniteNonNegative(velocity_variance.x()) ||
        !IsFiniteNonNegative(velocity_variance.y()) || !IsFiniteNonNegative(radius_variance)) {
      return "PerceptionObstacle variances must be finite and >= 0";
    }
    if (!IsFinite(last_update_stamp_s)) {
      return "PerceptionObstacle::last_update_stamp_s must be finite";
    }
    if (!IsFiniteNonNegative(track_age_s)) {
      return "PerceptionObstacle::track_age_s must be finite and >= 0";
    }
    if (!IsFinite(confidence) || confidence < 0.0 || confidence > 1.0) {
      return "PerceptionObstacle::confidence must lie in [0, 1]";
    }
    return nullptr;
  }

  bool IsValid() const { return Validate() == nullptr; }
};

// What the DPCBF adapter converts. The invariant that makes this contract worth having is
// radius_inflated_m >= radius_true_m: under-estimating a radius is the one unrecoverable
// error in the whole subsystem (architecture doc section 17), so it is checked here rather
// than left to a review of the safety stage.
struct SafetyObstacle {
  uint32_t id = 0;

  Eigen::Vector2d center{0.0, 0.0};
  Eigen::Vector2d velocity{0.0, 0.0};

  // The safety stage's PASS-THROUGH of PerceptionObstacle::radius_true_m on the estimated path,
  // and the ground-truth radius on the oracle path. On the estimated path it therefore carries
  // the same enlargement-inclusive value - see the CORRECTION on PerceptionObstacle above. It
  // is reported so the evaluator can attribute the inflation; it is not a claim about the
  // object's real size.
  double radius_true_m = 0.0;
  double radius_inflated_m = 0.0;  // What DPCBF is fed. Always >= radius_true_m.

  // Isotropic positional margin already folded into radius_inflated_m, reported
  // separately so the evaluator can attribute inflation to its terms. Metres.
  double position_inflation_m = 0.0;

  // The measurement time this obstacle derives from, and how old it was when the safety
  // stage generated it. AGE DEFINITION: generation_time - stamp_s, seconds of sim time.
  // The selector's per-frame fallback compares its own `now - frame stamp` against
  // mode.fallback_max_age_s; this field is the SAFETY stage's view, taken earlier.
  double stamp_s = 0.0;
  double age_s = 0.0;

  bool valid = false;
  ObstacleSource source = ObstacleSource::kUnknown;
  FrameId frame = FrameId::kWorld;

  const char* Validate() const {
    if (id == 0) return "SafetyObstacle::id must be non-zero";
    if (frame != FrameId::kWorld) return "SafetyObstacle::frame must be kWorld";
    if (source == ObstacleSource::kUnknown) return "SafetyObstacle::source must be set";
    if (!IsFinite(center) || !IsFinite(velocity)) {
      return "SafetyObstacle centre/velocity must be finite";
    }
    if (!IsFinitePositive(radius_true_m)) return "SafetyObstacle::radius_true_m must be > 0";
    if (!IsFinitePositive(radius_inflated_m)) {
      return "SafetyObstacle::radius_inflated_m must be > 0";
    }
    if (radius_inflated_m < radius_true_m) {
      return "SafetyObstacle::radius_inflated_m must be >= radius_true_m (never shrink)";
    }
    if (!IsFiniteNonNegative(position_inflation_m)) {
      return "SafetyObstacle::position_inflation_m must be finite and >= 0";
    }
    if (!IsFinite(stamp_s)) return "SafetyObstacle::stamp_s must be finite";
    if (!IsFiniteNonNegative(age_s)) return "SafetyObstacle::age_s must be finite and >= 0";
    return nullptr;
  }

  bool IsValid() const { return Validate() == nullptr; }
};

// Ground truth from DynamicObstacleManager::Snapshot(). Contains NO estimator fields by
// construction - no covariance, no confidence, no track status. The oracle provider must
// never learn anything from the estimator (architecture doc section 8), and the cheapest
// way to enforce that is to give it a type with nowhere to put estimator output.
struct OracleObstacleState {
  int32_t id = -1;  // The DynamicObstacleManager index; distinct id space from tracks.

  Eigen::Vector2d center{0.0, 0.0};
  Eigen::Vector2d velocity{0.0, 0.0};
  double radius_m = 0.0;

  double stamp_s = 0.0;
  FrameId frame = FrameId::kWorld;

  const char* Validate() const {
    if (id < 0) return "OracleObstacleState::id must be >= 0";
    if (frame != FrameId::kWorld) return "OracleObstacleState::frame must be kWorld";
    if (!IsFinite(center) || !IsFinite(velocity)) {
      return "OracleObstacleState centre/velocity must be finite";
    }
    if (!IsFinitePositive(radius_m)) return "OracleObstacleState::radius_m must be > 0";
    if (!IsFinite(stamp_s)) return "OracleObstacleState::stamp_s must be finite";
    return nullptr;
  }

  bool IsValid() const { return Validate() == nullptr; }
};

}  // namespace perception::core

#endif  // PERCEPTION_CORE_CONTRACTS_OBSTACLES_H_
