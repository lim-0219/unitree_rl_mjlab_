// Cluster2D, FittedPrimitive2D and CircleObservation - the detection-stage contracts.
//
// These mirror the semantics of the audited obstacle_detector types (PointSet, Segment,
// Circle) without importing any of them: upstream is ROS1 + Armadillo and never enters
// production (architecture doc section 15). Field names follow upstream where the meaning
// is identical, so the two can be cross-tuned with the same numbers.
//
// Frame: the detector works in kGravityAlignedBase and the pipeline transforms its
// CircleObservations to kWorld afterwards, which is why CircleObservation carries a frame
// field and the other two do not - they never leave the detector.
#ifndef PERCEPTION_CORE_CONTRACTS_DETECTION_2D_H_
#define PERCEPTION_CORE_CONTRACTS_DETECTION_2D_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "perception/core/contracts/frames.h"
#include "perception/core/contracts/validation.h"

namespace perception::core {

// A contiguous run of scan points that survived the grouping stage. Upstream's PointSet
// holds iterators into a point list; an index span is the same thing without the aliasing
// hazard, and it survives being copied into a dump.
//
// The indices address the detector's ORDERED POINT LIST - the sequence of (angle, range)
// pairs built from the occupied bins of a ProjectedScan, in increasing bin order. They
// are NOT bin indices, because empty bins are not in the list; `first_bin`/`last_bin`
// carry the bin traceability separately.
struct Cluster2D {
  int32_t first_index = -1;  // Inclusive.
  int32_t last_index = -1;   // Inclusive.
  int32_t point_count = 0;

  int32_t first_bin = -1;  // ProjectedScan bin of the first point; diagnostics/replay.
  int32_t last_bin = -1;

  // Upstream carries a single `is_visible` per PointSet, set when the group's boundary
  // was a genuine range discontinuity rather than an occlusion edge. Splitting it per
  // endpoint costs one byte and lets the circle fitter know WHICH end is untrustworthy,
  // which is exactly what the short-arc bias (risk R8) needs.
  bool front_visible = false;
  bool back_visible = false;

  bool is_visible() const { return front_visible && back_visible; }

  const char* Validate() const {
    if (first_index < 0 || last_index < 0) return "Cluster2D indices must be >= 0";
    if (last_index < first_index) return "Cluster2D::last_index must be >= first_index";
    if (point_count != last_index - first_index + 1) {
      return "Cluster2D::point_count must equal the index span";
    }
    if (first_bin < 0 || last_bin < 0) return "Cluster2D bin indices must be >= 0";
    return nullptr;
  }

  bool IsValid() const { return Validate() == nullptr; }
};

enum class PrimitiveKind : uint8_t { kNone = 0, kSegment, kCircle };

inline const char* ToString(PrimitiveKind kind) {
  switch (kind) {
    case PrimitiveKind::kSegment: return "segment";
    case PrimitiveKind::kCircle:  return "circle";
    case PrimitiveKind::kNone:    break;
  }
  return "none";
}

// A segment or a circle from the split-and-merge stage. One struct with a kind tag rather
// than a variant: these are pooled by the thousand and must be trivially preallocatable
// and memcpy-able into a dump.
struct FittedPrimitive2D {
  PrimitiveKind kind = PrimitiveKind::kNone;

  // Segment fields. `normal` is the unit left-hand normal of (last - first).
  Eigen::Vector2d first_point{0.0, 0.0};
  Eigen::Vector2d last_point{0.0, 0.0};
  Eigen::Vector2d normal{0.0, 0.0};

  // Circle fields.
  Eigen::Vector2d center{0.0, 0.0};
  double radius_m = 0.0;

  // RESIDUAL DEFINITION: root-mean-square ORTHOGONAL distance, in metres, from each of
  // the `point_count` contributing points to the fitted primitive - the perpendicular
  // distance to the line for a segment, and | ||p - center|| - radius_m | for a circle.
  // The denominator is point_count. It is an RMS, not a maximum and not a sum, and it is
  // in metres, not metres squared.
  double fit_residual_m = 0.0;

  int32_t point_count = 0;
  int32_t cluster_index = -1;  // The Cluster2D this was fitted from.

  bool front_visible = false;
  bool back_visible = false;

  double Length() const { return (last_point - first_point).norm(); }

  const char* Validate() const {
    if (kind == PrimitiveKind::kNone) return "FittedPrimitive2D::kind must be set";
    if (point_count < 2) return "FittedPrimitive2D::point_count must be >= 2";
    if (cluster_index < 0) return "FittedPrimitive2D::cluster_index must be >= 0";
    if (!IsFiniteNonNegative(fit_residual_m)) {
      return "FittedPrimitive2D::fit_residual_m must be finite and >= 0";
    }
    if (kind == PrimitiveKind::kSegment) {
      if (!IsFinite(first_point) || !IsFinite(last_point) || !IsFinite(normal)) {
        return "FittedPrimitive2D segment fields must be finite";
      }
      if (Length() <= 0.0) return "FittedPrimitive2D segment must have positive length";
      if (std::abs(normal.norm() - 1.0) > 1e-9) {
        return "FittedPrimitive2D::normal must be a unit vector for a segment";
      }
      // The normal has to belong to THIS segment; an inherited normal from a previous
      // fit would send every downstream offset the wrong way with no other symptom.
      if (std::abs(normal.dot((last_point - first_point).normalized())) > 1e-9) {
        return "FittedPrimitive2D::normal must be perpendicular to the segment";
      }
    } else {
      if (!IsFinite(center)) return "FittedPrimitive2D::center must be finite";
      if (!IsFinitePositive(radius_m)) return "FittedPrimitive2D::radius_m must be > 0";
    }
    return nullptr;
  }

  bool IsValid() const { return Validate() == nullptr; }
};

// One circular obstacle hypothesis from a single scan. Produced in kGravityAlignedBase and
// transformed to kWorld by the pipeline before it reaches the tracker.
struct CircleObservation {
  Eigen::Vector2d center{0.0, 0.0};  // In `frame`.

  // The circle the fitter produced. For the ported obstacle_detector rule this is
  // (sqrt(3)/3) * segment_length plus the configured enlargement, and it is BIASED SMALL
  // on short arcs - the bias that risk R8 and the P8 experiment exist to quantify.
  double radius_fitted_m = 0.0;

  // Radius of the smallest circle CENTRED AT `center` that contains every contributing
  // point, i.e. max over points of ||p - center||. Not a re-fit and not a bounding circle
  // with a free centre - the centre is fixed so the two radii are directly comparable.
  // This is the conservative alternative the safety stage may feed instead (open Q11).
  double radius_enclosing_m = 0.0;

  // ARC EXTENT DEFINITION: the angular width of the contributing points AS SEEN FROM THE
  // SCAN ORIGIN, in radians, i.e. (last_bin - first_bin) * angle_increment. It is NOT the
  // arc subtended at the circle's own centre, which for a distant object is far larger.
  // The two differ by roughly range/radius, so confusing them is not a rounding error.
  double arc_extent_rad = 0.0;

  // Length along the fitted circle actually observed: radius_fitted_m times the angle
  // subtended AT THE CIRCLE CENTRE. This one IS centre-referenced; the pair is kept
  // because the short-arc bias correlates with the centre-referenced angle while the
  // occlusion logic keys off the sensor-referenced one.
  double arc_length_m = 0.0;

  // Same definition as FittedPrimitive2D::fit_residual_m: RMS of
  // | ||p - center|| - radius_fitted_m | over the point_count contributing points.
  double fit_residual_m = 0.0;

  // Range from the scan origin to `center`, in the frame the observation was MADE in
  // (kGravityAlignedBase). Preserved across the transform to world, because after the
  // transform the origin is no longer the sensor and the number becomes unrecoverable.
  double range_to_center_m = 0.0;

  // Measurement standard deviations handed to the tracker. Derived by the detector from
  // fit_residual_m and the arc extent; units are metres (1 sigma), not variances.
  double sigma_center_m = 0.0;
  double sigma_radius_m = 0.0;

  int32_t point_count = 0;
  int32_t primitive_index = -1;  // The FittedPrimitive2D this came from.

  bool front_visible = false;
  bool back_visible = false;

  double stamp_s = 0.0;
  FrameId frame = FrameId::kGravityAlignedBase;

  bool is_fully_visible() const { return front_visible && back_visible; }

  const char* Validate() const {
    if (frame != FrameId::kGravityAlignedBase && frame != FrameId::kWorld) {
      return "CircleObservation::frame must be kGravityAlignedBase or kWorld";
    }
    if (!IsFinite(center)) return "CircleObservation::center must be finite";
    if (!IsFinitePositive(radius_fitted_m)) {
      return "CircleObservation::radius_fitted_m must be > 0";
    }
    if (!IsFinitePositive(radius_enclosing_m)) {
      return "CircleObservation::radius_enclosing_m must be > 0";
    }
    if (!IsFiniteNonNegative(arc_extent_rad) || arc_extent_rad > 2.0 * kPi) {
      return "CircleObservation::arc_extent_rad must lie in [0, 2*pi]";
    }
    if (!IsFiniteNonNegative(arc_length_m)) {
      return "CircleObservation::arc_length_m must be finite and >= 0";
    }
    if (!IsFiniteNonNegative(fit_residual_m)) {
      return "CircleObservation::fit_residual_m must be finite and >= 0";
    }
    if (!IsFiniteNonNegative(range_to_center_m)) {
      return "CircleObservation::range_to_center_m must be finite and >= 0";
    }
    if (!IsFiniteNonNegative(sigma_center_m) || !IsFiniteNonNegative(sigma_radius_m)) {
      return "CircleObservation sigmas must be finite and >= 0";
    }
    if (point_count < 2) return "CircleObservation::point_count must be >= 2";
    if (primitive_index < 0) return "CircleObservation::primitive_index must be >= 0";
    if (!IsFinite(stamp_s)) return "CircleObservation::stamp_s must be finite";
    return nullptr;
  }

  bool IsValid() const { return Validate() == nullptr; }
};

}  // namespace perception::core

#endif  // PERCEPTION_CORE_CONTRACTS_DETECTION_2D_H_
