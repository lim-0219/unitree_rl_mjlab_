// LabeledPointCloud - the self-filter and ground-segmentation output.
//
// DESIGN NOTE (the question the architecture doc left open). Section 7 says
// LabeledPointCloud "replaces separate GroundSegmentationResult container;
// GroundSegmentationResult = label stats + plane params". Both halves are honoured here:
// there is exactly ONE point container (this one), and `GroundSegmentationResult` remains
// a named struct that lives inside it as the `segmentation` member.
//
// Why not fold the stats straight into LabeledPointCloud as loose fields: the segmenter
// interface (ISegmenter, P6) has to return its verdict, the diagnostics writer has to
// serialize it, and the evaluator has to compare two of them - all without touching the
// point buffer. A named 60-byte struct does that; loose fields would force every one of
// those consumers to accept the whole cloud.
//
// Labels live in a vector parallel to `points` rather than inside RawTimedPoint, for two
// reasons: RawTimedPoint is a shipped type used by the raycaster and must not grow, and
// the projector's hot loop reads only the label, so keeping labels contiguous keeps that
// scan cache-dense.
#ifndef PERCEPTION_CORE_CONTRACTS_LABELED_POINT_CLOUD_H_
#define PERCEPTION_CORE_CONTRACTS_LABELED_POINT_CLOUD_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "perception/core/contracts/frames.h"
#include "perception/core/contracts/timed_point_cloud.h"
#include "perception/core/contracts/validation.h"

namespace perception::core {

enum class PointLabel : uint8_t {
  kInvalid = 0,  // Non-finite, out of range, or otherwise unusable.
  kGround,       // Assigned to the ground surface by the segmenter.
  kNonGround,    // The obstacle candidates; the only label the projector consumes.
  kSelf,         // Rejected by the defensive self filter.
};

inline const char* ToString(PointLabel label) {
  switch (label) {
    case PointLabel::kGround:    return "ground";
    case PointLabel::kNonGround: return "non_ground";
    case PointLabel::kSelf:      return "self";
    case PointLabel::kInvalid:   break;
  }
  return "invalid";
}

// The fitted ground plane, in the cloud's frame. In the gravity-aligned base frame a
// correct fit has a normal very close to +Z; `normal` is kept general so the optional
// RANSAC refinement (P6) can report what it actually found rather than what it should
// have found.
struct GroundPlane {
  // Unit normal, pointing UP (positive Z component), in the cloud's frame.
  Eigen::Vector3d normal{0.0, 0.0, 1.0};

  // Signed plane offset: the plane is { p : normal.dot(p) + offset_m == 0 }. For a floor
  // one metre below the frame origin this is +1.0, not -1.0.
  double offset_m = 0.0;

  // FRACTION DEFINITION: inliers / points_considered_by_the_plane_fit. The denominator is
  // the points the fit was actually run on (candidate ground points), NOT the whole cloud
  // and NOT the finally-labelled ground points. Stated here because a "plane inlier
  // fraction" over the whole cloud is a different, much smaller number.
  double inlier_fraction = 0.0;
  int32_t fit_input_points = 0;  // The denominator above, carried explicitly.

  // False when the segmenter used a pure z-band and never fitted a plane, or when the fit
  // failed. A z-band result is still a valid segmentation; it just has no plane.
  bool valid = false;

  const char* Validate() const {
    if (!IsFinite(normal)) return "GroundPlane::normal must be finite";
    if (!IsFinite(offset_m)) return "GroundPlane::offset_m must be finite";
    if (!IsFinite(inlier_fraction) || inlier_fraction < 0.0 || inlier_fraction > 1.0) {
      return "GroundPlane::inlier_fraction must lie in [0, 1]";
    }
    if (fit_input_points < 0) return "GroundPlane::fit_input_points must be >= 0";
    if (valid) {
      if (std::abs(normal.norm() - 1.0) > 1e-9) return "GroundPlane::normal must be a unit vector";
      if (normal.z() <= 0.0) return "GroundPlane::normal must point up (positive z)";
      if (fit_input_points <= 0) return "a valid GroundPlane must have fit_input_points > 0";
    }
    return nullptr;
  }

  bool IsValid() const { return Validate() == nullptr; }
};

// Label census plus plane parameters - the "GroundSegmentationResult" of section 7.
struct GroundSegmentationResult {
  int32_t total_points = 0;
  int32_t ground_points = 0;
  int32_t non_ground_points = 0;
  int32_t self_points = 0;
  int32_t invalid_points = 0;

  GroundPlane plane;

  double wall_time_us = 0.0;

  // Every point carries exactly one label, so the four buckets must sum to the total.
  // Same discipline as ScanStats::IsBalanced: a census that does not close is a census
  // whose derived fractions cannot be trusted.
  bool IsBalanced() const {
    return ground_points + non_ground_points + self_points + invalid_points == total_points;
  }

  // FRACTION DEFINITION: ground_points / total_points. Denominator is EVERY point handed
  // to the segmenter, including self and invalid ones - not just the ground/non-ground
  // pair. Reported this way so that a spike in self-hits shows up as a falling ground
  // fraction instead of hiding behind a renormalised denominator.
  double GroundFraction() const {
    return total_points > 0 ? static_cast<double>(ground_points) / total_points : 0.0;
  }

  // FRACTION DEFINITION: non_ground_points / total_points. Same denominator as above.
  double NonGroundFraction() const {
    return total_points > 0 ? static_cast<double>(non_ground_points) / total_points : 0.0;
  }

  const char* Validate() const {
    if (total_points < 0 || ground_points < 0 || non_ground_points < 0 || self_points < 0 ||
        invalid_points < 0) {
      return "GroundSegmentationResult counts must be >= 0";
    }
    if (!IsBalanced()) return "GroundSegmentationResult label counts do not sum to total_points";
    if (!IsFiniteNonNegative(wall_time_us)) {
      return "GroundSegmentationResult::wall_time_us must be finite and >= 0";
    }
    return plane.Validate();
  }

  bool IsValid() const { return Validate() == nullptr; }
};

struct LabeledPointCloud {
  std::vector<RawTimedPoint> points;
  std::vector<PointLabel> labels;  // Parallel to `points`; same size, always.

  double stamp_s = 0.0;
  uint64_t sequence = 0;
  FrameId frame = FrameId::kGravityAlignedBase;
  MotionCompensation motion_compensation = MotionCompensation::kDeskewedToStamp;

  GroundSegmentationResult segmentation;

  std::size_t size() const { return points.size(); }
  bool empty() const { return points.empty(); }

  void Reserve(std::size_t capacity) {
    points.reserve(capacity);
    labels.reserve(capacity);
  }

  void Clear() {
    points.clear();
    labels.clear();
    segmentation = GroundSegmentationResult{};
  }

  const char* Validate() const {
    if (frame != FrameId::kGravityAlignedBase) {
      return "LabeledPointCloud::frame must be kGravityAlignedBase";
    }
    if (labels.size() != points.size()) {
      return "LabeledPointCloud::labels must be parallel to points";
    }
    if (!IsFinite(stamp_s)) return "LabeledPointCloud::stamp_s must be finite";
    if (segmentation.total_points != static_cast<int32_t>(points.size())) {
      return "LabeledPointCloud::segmentation.total_points must equal the point count";
    }
    if (const char* reason = segmentation.Validate()) return reason;

    // The census must describe THIS cloud, not a previous one. Recount and compare;
    // a stale census is the failure mode that would silently corrupt every downstream
    // rate in EstimationDiagnostics.
    int32_t counted[4] = {0, 0, 0, 0};
    for (const PointLabel label : labels) {
      counted[static_cast<std::size_t>(label)]++;
    }
    if (counted[static_cast<std::size_t>(PointLabel::kInvalid)] != segmentation.invalid_points ||
        counted[static_cast<std::size_t>(PointLabel::kGround)] != segmentation.ground_points ||
        counted[static_cast<std::size_t>(PointLabel::kNonGround)] !=
            segmentation.non_ground_points ||
        counted[static_cast<std::size_t>(PointLabel::kSelf)] != segmentation.self_points) {
      return "LabeledPointCloud::segmentation does not match the actual labels";
    }
    return nullptr;
  }

  bool IsValid() const { return Validate() == nullptr; }
};

}  // namespace perception::core

#endif  // PERCEPTION_CORE_CONTRACTS_LABELED_POINT_CLOUD_H_
