// TrackState2D and TrackPrediction2D - the tracking-stage contracts.
//
// Layout follows the ported obstacle_detector tracker: three INDEPENDENT 2-state Kalman
// filters per obstacle (x, y, radius), each with state [value, rate] and a constant-value
// -plus-rate transition. That is why the variances below are per axis and there is no
// cross-covariance term - upstream has none, and inventing one here would misrepresent
// what the ported filter actually computes. Upgrading to a joint 2-D filter with
// cross-covariance is open question Q19 and would be a contract change.
//
// Frame: kWorld. Tracking happens in the world frame because that is how DPCBF consumes
// obstacle states and how upstream tracked in a fixed frame.
#ifndef PERCEPTION_CORE_CONTRACTS_TRACKING_2D_H_
#define PERCEPTION_CORE_CONTRACTS_TRACKING_2D_H_

#include <cstdint>

#include <Eigen/Core>

#include "perception/core/contracts/frames.h"
#include "perception/core/contracts/validation.h"

namespace perception::core {

enum class TrackStatus : uint8_t {
  kInvalid = 0,
  kTentative,  // Seen, not yet confirmed. Never leaves the tracker.
  kConfirmed,  // Met the confirmation gate; eligible for the safety stage.
  kCoasting,   // Was confirmed, is currently unmatched, still being predicted forward.
};

inline const char* ToString(TrackStatus status) {
  switch (status) {
    case TrackStatus::kTentative: return "tentative";
    case TrackStatus::kConfirmed: return "confirmed";
    case TrackStatus::kCoasting:  return "coasting";
    case TrackStatus::kInvalid:   break;
  }
  return "invalid";
}

struct TrackState2D {
  uint32_t id = 0;  // Stable for the track's whole life; 0 is reserved for "unassigned".

  Eigen::Vector2d center{0.0, 0.0};    // m, in kWorld.
  Eigen::Vector2d velocity{0.0, 0.0};  // m/s. The KF RATE state, never a finite difference.
  double radius_m = 0.0;
  double radius_rate_mps = 0.0;  // The radius filter's rate state; growth of a fitted radius.

  // VARIANCES, not standard deviations. Units: m^2 for centre, (m/s)^2 for velocity,
  // m^2 for radius, (m/s)^2 for radius rate. Per axis, no cross terms - see the header
  // comment on why.
  Eigen::Vector2d center_variance{0.0, 0.0};
  Eigen::Vector2d velocity_variance{0.0, 0.0};
  double radius_variance = 0.0;
  double radius_rate_variance = 0.0;

  double created_stamp_s = 0.0;           // Sim time of the first observation.
  double last_update_stamp_s = 0.0;       // Sim time this state was last advanced to.
  double last_measurement_stamp_s = 0.0;  // Sim time of the last MATCHED measurement.

  // COUNT DEFINITIONS, all since creation and all in scans, not seconds:
  //   hits    - scans in which this track was matched to a measurement.
  //   misses  - scans in which it ran and was not matched.
  //   consecutive_misses - misses since the last hit; reset to 0 on every hit.
  // hits + misses is therefore the number of scans the track has existed for.
  int32_t hits = 0;
  int32_t misses = 0;
  int32_t consecutive_misses = 0;

  TrackStatus status = TrackStatus::kInvalid;
  FrameId frame = FrameId::kWorld;

  // AGE DEFINITION: seconds of sim time between the first observation and the last
  // update - a duration, NOT a scan count and NOT "time since last measurement".
  // Staleness is `now - last_measurement_stamp_s` and is deliberately a different number.
  double age_s() const { return last_update_stamp_s - created_stamp_s; }

  double staleness_s(double now_s) const { return now_s - last_measurement_stamp_s; }

  int32_t scans_seen() const { return hits + misses; }

  // FRACTION DEFINITION: hits / (hits + misses), over the track's whole life. Denominator
  // is every scan the track existed for, not a sliding window.
  double HitRatio() const {
    const int32_t total = scans_seen();
    return total > 0 ? static_cast<double>(hits) / total : 0.0;
  }

  const char* Validate() const {
    if (status == TrackStatus::kInvalid) return "TrackState2D::status must be set";
    if (id == 0) return "TrackState2D::id must be non-zero";
    if (frame != FrameId::kWorld) return "TrackState2D::frame must be kWorld";
    if (!IsFinite(center) || !IsFinite(velocity)) {
      return "TrackState2D centre/velocity must be finite";
    }
    if (!IsFinitePositive(radius_m)) return "TrackState2D::radius_m must be > 0";
    if (!IsFinite(radius_rate_mps)) return "TrackState2D::radius_rate_mps must be finite";
    // A negative variance is not a large uncertainty, it is a broken filter, and it would
    // make every downstream sigma a NaN the moment someone takes a square root.
    if (!IsFiniteNonNegative(center_variance.x()) || !IsFiniteNonNegative(center_variance.y()) ||
        !IsFiniteNonNegative(velocity_variance.x()) ||
        !IsFiniteNonNegative(velocity_variance.y()) || !IsFiniteNonNegative(radius_variance) ||
        !IsFiniteNonNegative(radius_rate_variance)) {
      return "TrackState2D variances must be finite and >= 0";
    }
    if (!IsFinite(created_stamp_s) || !IsFinite(last_update_stamp_s) ||
        !IsFinite(last_measurement_stamp_s)) {
      return "TrackState2D stamps must be finite";
    }
    if (last_update_stamp_s < created_stamp_s) {
      return "TrackState2D::last_update_stamp_s must be >= created_stamp_s";
    }
    if (last_measurement_stamp_s < created_stamp_s) {
      return "TrackState2D::last_measurement_stamp_s must be >= created_stamp_s";
    }
    if (last_measurement_stamp_s > last_update_stamp_s) {
      return "TrackState2D::last_measurement_stamp_s must be <= last_update_stamp_s";
    }
    if (hits < 0 || misses < 0 || consecutive_misses < 0) {
      return "TrackState2D counters must be >= 0";
    }
    if (hits < 1) return "TrackState2D must have at least one hit (it was created by one)";
    if (consecutive_misses > misses) {
      return "TrackState2D::consecutive_misses must be <= misses";
    }
    // A coasting track is by definition currently unmatched; a confirmed one is matched.
    if (status == TrackStatus::kCoasting && consecutive_misses == 0) {
      return "TrackState2D::kCoasting requires consecutive_misses > 0";
    }
    if (status == TrackStatus::kConfirmed && consecutive_misses != 0) {
      return "TrackState2D::kConfirmed requires consecutive_misses == 0 (else it coasts)";
    }
    return nullptr;
  }

  bool IsValid() const { return Validate() == nullptr; }
};

// A track propagated forward to a time no measurement exists for. Separate from
// TrackState2D so that a predicted position can never be mistaken for an observed one
// when it reaches the safety stage or a dump.
struct TrackPrediction2D {
  uint32_t id = 0;

  double stamp_s = 0.0;         // The time predicted TO.
  double source_stamp_s = 0.0;  // The TrackState2D::last_update_stamp_s predicted FROM.

  Eigen::Vector2d center{0.0, 0.0};
  Eigen::Vector2d velocity{0.0, 0.0};
  double radius_m = 0.0;

  // Variances grown by the prediction step. Same units as TrackState2D.
  Eigen::Vector2d center_variance{0.0, 0.0};
  Eigen::Vector2d velocity_variance{0.0, 0.0};
  double radius_variance = 0.0;

  FrameId frame = FrameId::kWorld;

  // Extrapolation horizon. Always >= 0: predicting backwards is not a prediction.
  double dt_s() const { return stamp_s - source_stamp_s; }

  const char* Validate() const {
    if (id == 0) return "TrackPrediction2D::id must be non-zero";
    if (frame != FrameId::kWorld) return "TrackPrediction2D::frame must be kWorld";
    if (!IsFinite(stamp_s) || !IsFinite(source_stamp_s)) {
      return "TrackPrediction2D stamps must be finite";
    }
    if (dt_s() < 0.0) return "TrackPrediction2D must not predict backwards in time";
    if (!IsFinite(center) || !IsFinite(velocity)) {
      return "TrackPrediction2D centre/velocity must be finite";
    }
    if (!IsFinitePositive(radius_m)) return "TrackPrediction2D::radius_m must be > 0";
    if (!IsFiniteNonNegative(center_variance.x()) || !IsFiniteNonNegative(center_variance.y()) ||
        !IsFiniteNonNegative(velocity_variance.x()) ||
        !IsFiniteNonNegative(velocity_variance.y()) || !IsFiniteNonNegative(radius_variance)) {
      return "TrackPrediction2D variances must be finite and >= 0";
    }
    return nullptr;
  }

  bool IsValid() const { return Validate() == nullptr; }
};

}  // namespace perception::core

#endif  // PERCEPTION_CORE_CONTRACTS_TRACKING_2D_H_
