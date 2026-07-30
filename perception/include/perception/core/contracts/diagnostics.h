// StageTiming and EstimationDiagnostics - the per-frame accounting contracts.
//
// Everything here is serialized to JSONL by core/diagnostics (P2) and is the headless
// substitute for a visualization. The design rule applied throughout: a rate is stored as
// the two integers it is a ratio of, and the ratio is exposed as a method whose comment
// names the denominator. Storing a bare float would let the denominator drift the way
// "self-hit fraction" already did once (architecture doc section 7 amendment).
#ifndef PERCEPTION_CORE_CONTRACTS_DIAGNOSTICS_H_
#define PERCEPTION_CORE_CONTRACTS_DIAGNOSTICS_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "perception/core/contracts/labeled_point_cloud.h"
#include "perception/core/contracts/projected_scan.h"
#include "perception/core/contracts/timed_point_cloud.h"
#include "perception/core/contracts/validation.h"

namespace perception::core {

// Pipeline stages, in execution order. The enum is the array index, so inserting a stage
// in the middle renumbers the dumps - append new stages before kCount instead.
enum class PipelineStage : uint8_t {
  kRaycast = 0,
  kDeskew,
  kGravityAlign,
  kSelfFilter,
  kSegmentation,
  kProjection,
  kDetection,
  kTracking,
  kSafety,
  kCount,
};

inline constexpr std::size_t kPipelineStageCount = static_cast<std::size_t>(PipelineStage::kCount);

inline const char* ToString(PipelineStage stage) {
  switch (stage) {
    case PipelineStage::kRaycast:      return "raycast";
    case PipelineStage::kDeskew:       return "deskew";
    case PipelineStage::kGravityAlign: return "gravity_align";
    case PipelineStage::kSelfFilter:   return "self_filter";
    case PipelineStage::kSegmentation: return "segmentation";
    case PipelineStage::kProjection:   return "projection";
    case PipelineStage::kDetection:    return "detection";
    case PipelineStage::kTracking:     return "tracking";
    case PipelineStage::kSafety:       return "safety";
    case PipelineStage::kCount:        break;
  }
  return "unknown";
}

struct StageTiming {
  // WALL time in microseconds per stage, not CPU time: the budgets in section 12 are
  // latency budgets, and a stage that blocks on a lock has spent the time regardless of
  // whether it burned a core doing it.
  std::array<double, kPipelineStageCount> wall_time_us{};

  double& operator[](PipelineStage stage) {
    return wall_time_us[static_cast<std::size_t>(stage)];
  }
  double operator[](PipelineStage stage) const {
    return wall_time_us[static_cast<std::size_t>(stage)];
  }

  // Sum of the per-stage times. NOT the end-to-end frame latency, which additionally
  // includes queueing and is measured separately by the runner.
  double SumUs() const {
    double total = 0.0;
    for (const double value : wall_time_us) total += value;
    return total;
  }

  void Clear() { wall_time_us.fill(0.0); }

  const char* Validate() const {
    for (const double value : wall_time_us) {
      if (!IsFiniteNonNegative(value)) return "StageTiming entries must be finite and >= 0";
    }
    return nullptr;
  }

  bool IsValid() const { return Validate() == nullptr; }
};

// One oracle-to-estimate association, produced only in comparison mode. The evaluator is
// forbidden from feeding anything here back into the estimator (architecture doc
// section 8); this struct exists purely to be written to a metrics file.
struct MatchedPairError {
  uint32_t track_id = 0;
  int32_t oracle_id = -1;

  // Euclidean centre distance in metres, oracle minus estimate. A distance, so >= 0.
  double center_error_m = 0.0;
  // Euclidean velocity-vector difference magnitude in m/s. A magnitude, not a signed
  // speed error, so >= 0.
  double velocity_error_mps = 0.0;
  // SIGNED radius error: estimate minus oracle. Negative means the estimate is SMALLER
  // than the truth, which is the direction that matters for safety, so the sign is kept.
  double radius_error_m = 0.0;

  double stamp_s = 0.0;

  const char* Validate() const {
    if (track_id == 0) return "MatchedPairError::track_id must be non-zero";
    if (oracle_id < 0) return "MatchedPairError::oracle_id must be >= 0";
    if (!IsFiniteNonNegative(center_error_m)) {
      return "MatchedPairError::center_error_m must be finite and >= 0";
    }
    if (!IsFiniteNonNegative(velocity_error_mps)) {
      return "MatchedPairError::velocity_error_mps must be finite and >= 0";
    }
    if (!IsFinite(radius_error_m)) return "MatchedPairError::radius_error_m must be finite";
    if (!IsFinite(stamp_s)) return "MatchedPairError::stamp_s must be finite";
    return nullptr;
  }

  bool IsValid() const { return Validate() == nullptr; }
};

struct EstimationDiagnostics {
  uint64_t sequence = 0;
  double stamp_s = 0.0;

  // Per-stage censuses, reused rather than re-declared so there is exactly one definition
  // of each count in the codebase.
  ScanStats scan;
  GroundSegmentationResult segmentation;
  ProjectionStats projection;

  int32_t clusters_found = 0;
  int32_t primitives_fitted = 0;
  int32_t circles_detected = 0;
  int32_t circles_rejected_radius = 0;     // Exceeded detection.max_circle_radius_m.
  int32_t circles_rejected_visibility = 0; // Dropped by circles_from_visibles.

  int32_t tracks_tentative = 0;
  int32_t tracks_confirmed = 0;
  int32_t tracks_coasting = 0;
  int32_t tracks_created = 0;   // This frame.
  int32_t tracks_deleted = 0;   // This frame.
  int32_t id_switches = 0;      // Detected only in comparison mode against oracle ids.

  int32_t obstacles_published = 0;
  int32_t obstacles_dropped_stale = 0;
  int32_t obstacles_dropped_unconfirmed = 0;
  int32_t obstacles_dropped_radius_floor = 0;

  // Comparison mode only. `comparison_valid` false means the three fields below are
  // meaningless rather than zero - the distinction the evaluator needs so an unrun
  // comparison does not read as a perfect score.
  bool comparison_valid = false;
  std::vector<MatchedPairError> matched;
  int32_t oracle_count = 0;        // Oracle obstacles present this frame.
  int32_t unmatched_oracle = 0;    // Oracle obstacles with no estimate: the miss count.
  int32_t unmatched_estimated = 0; // Estimates with no oracle: the false-positive count.

  StageTiming timing;

  // Latency from the sim time the scan was stamped to the sim time this frame was
  // published, in seconds. The section-12 end-to-end budget is measured on this.
  double pipeline_latency_s = 0.0;

  // FRACTION DEFINITION: matched / oracle_count. Denominator is the ORACLE obstacles
  // present this frame, so this is a recall, and it is undefined (returns 0) when no
  // oracle obstacle exists rather than being reported as a perfect 1.
  double DetectionRecall() const {
    return oracle_count > 0 ? static_cast<double>(matched.size()) / oracle_count : 0.0;
  }

  // FRACTION DEFINITION: matched / (matched + unmatched_estimated). Denominator is the
  // ESTIMATED obstacles published this frame, so this is a precision. Deliberately a
  // different denominator from DetectionRecall above.
  double DetectionPrecision() const {
    const int32_t estimated_total = static_cast<int32_t>(matched.size()) + unmatched_estimated;
    return estimated_total > 0 ? static_cast<double>(matched.size()) / estimated_total : 0.0;
  }

  void Reserve(std::size_t max_matched_pairs) { matched.reserve(max_matched_pairs); }

  void Clear() {
    *this = EstimationDiagnostics{};
  }

  const char* Validate() const {
    if (!IsFinite(stamp_s)) return "EstimationDiagnostics::stamp_s must be finite";
    if (!IsFiniteNonNegative(pipeline_latency_s)) {
      return "EstimationDiagnostics::pipeline_latency_s must be finite and >= 0";
    }
    if (!scan.IsBalanced()) return "EstimationDiagnostics::scan census does not balance";
    if (const char* reason = segmentation.Validate()) return reason;
    if (const char* reason = projection.Validate()) return reason;
    if (const char* reason = timing.Validate()) return reason;
    if (clusters_found < 0 || primitives_fitted < 0 || circles_detected < 0 ||
        circles_rejected_radius < 0 || circles_rejected_visibility < 0 || tracks_tentative < 0 ||
        tracks_confirmed < 0 || tracks_coasting < 0 || tracks_created < 0 || tracks_deleted < 0 ||
        id_switches < 0 || obstacles_published < 0 || obstacles_dropped_stale < 0 ||
        obstacles_dropped_unconfirmed < 0 || obstacles_dropped_radius_floor < 0 ||
        oracle_count < 0 || unmatched_oracle < 0 || unmatched_estimated < 0) {
      return "EstimationDiagnostics counts must be >= 0";
    }
    if (!comparison_valid) {
      if (!matched.empty() || oracle_count != 0 || unmatched_oracle != 0 ||
          unmatched_estimated != 0) {
        return "EstimationDiagnostics comparison fields must be empty when comparison_valid "
               "is false";
      }
    } else {
      if (static_cast<int32_t>(matched.size()) + unmatched_oracle != oracle_count) {
        return "EstimationDiagnostics: matched + unmatched_oracle must equal oracle_count";
      }
      for (const auto& pair : matched) {
        if (const char* reason = pair.Validate()) return reason;
      }
    }
    return nullptr;
  }

  bool IsValid() const { return Validate() == nullptr; }
};

}  // namespace perception::core

#endif  // PERCEPTION_CORE_CONTRACTS_DIAGNOSTICS_H_
