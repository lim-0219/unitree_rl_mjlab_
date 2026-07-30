// DeskewedPointCloud and GravityAlignedCloud - the two motion-compensation stage outputs.
//
// Both reuse RawTimedPoint as their element type, because deskew and gravity alignment
// only ever REWRITE `position`; `range_m`, `time_offset_s`, `ray_index`, `geom_id` and
// `body_id` are provenance and survive untouched all the way to the diagnostics dump.
//
// They are nevertheless distinct types rather than one cloud with a frame field, so that
// a stage expecting a gravity-aligned cloud cannot be handed a merely-deskewed one. The
// `frame` member then records WHICH base frame, and the type records HOW FAR along the
// pipeline the cloud is. See perception/docs/architecture.md, "Internal data contracts".
//
// Consumed by (forward reference, not implemented here): the deskew and gravity-align
// stages of P5, then the self filter and ground segmentation of P6.
#ifndef PERCEPTION_CORE_CONTRACTS_POINT_CLOUDS_H_
#define PERCEPTION_CORE_CONTRACTS_POINT_CLOUDS_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "perception/core/contracts/frames.h"
#include "perception/core/contracts/timed_point_cloud.h"
#include "perception/core/contracts/validation.h"

namespace perception::core {

// Every point re-projected to where it would have been observed had the whole scan been
// taken instantaneously at `stamp_s`. `time_offset_s` is retained rather than zeroed:
// it is the only record of when the measurement actually happened, and the deskew
// residual tests need it.
struct DeskewedPointCloud {
  std::vector<RawTimedPoint> points;

  double stamp_s = 0.0;   // The reference time every point was compensated TO (scan end).
  uint64_t sequence = 0;  // Copied from the source TimedPointCloud.
  FrameId frame = FrameId::kBase;
  MotionCompensation motion_compensation = MotionCompensation::kDeskewedToStamp;

  // Span of the source scan, i.e. min and max `time_offset_s` over its points. The
  // deskew correction is bounded by base motion over exactly this window, so it is
  // recorded here rather than recomputed by every consumer.
  double window_begin_s = 0.0;  // Relative to stamp_s; normally negative.
  double window_end_s = 0.0;    // Relative to stamp_s; normally 0.

  // Largest per-point correction applied, in metres. Diagnostic only - this is what the
  // P5 acceptance gate ("residual distortion <= 5 mm @ 2 rad/s yaw") is measured against.
  double max_correction_m = 0.0;

  std::size_t size() const { return points.size(); }
  bool empty() const { return points.empty(); }
  void Reserve(std::size_t capacity) { points.reserve(capacity); }
  void Clear() { points.clear(); }

  const char* Validate() const {
    if (frame != FrameId::kBase) return "DeskewedPointCloud::frame must be kBase";
    if (motion_compensation != MotionCompensation::kDeskewedToStamp) {
      return "DeskewedPointCloud::motion_compensation must be kDeskewedToStamp";
    }
    if (!IsFinite(stamp_s)) return "DeskewedPointCloud::stamp_s must be finite";
    if (!IsFinite(window_begin_s) || !IsFinite(window_end_s)) {
      return "DeskewedPointCloud window bounds must be finite";
    }
    if (window_begin_s > window_end_s) {
      return "DeskewedPointCloud::window_begin_s must be <= window_end_s";
    }
    if (!IsFiniteNonNegative(max_correction_m)) {
      return "DeskewedPointCloud::max_correction_m must be finite and >= 0";
    }
    for (const auto& point : points) {
      if (!IsFinite(point.position)) return "DeskewedPointCloud contains a non-finite position";
      if (!IsFiniteNonNegative(point.range_m)) {
        return "DeskewedPointCloud contains a non-finite or negative range_m";
      }
    }
    return nullptr;
  }

  bool IsValid() const { return Validate() == nullptr; }
};

// The deskewed cloud rotated so that gravity is -Z, with YAW PRESERVED. Only roll and
// pitch are removed; rotating yaw out as well would destroy the heading the detector and
// tracker rely on.
struct GravityAlignedCloud {
  std::vector<RawTimedPoint> points;

  double stamp_s = 0.0;
  uint64_t sequence = 0;
  FrameId frame = FrameId::kGravityAlignedBase;
  MotionCompensation motion_compensation = MotionCompensation::kDeskewedToStamp;

  // The rotation this stage applied: p_gravityAlignedBase = alignment_from_base * p_base.
  // Retained so the detector's world-frame transform can be composed exactly rather than
  // re-derived from an IMU reading that may have moved on.
  Eigen::Matrix3d alignment_from_base = Eigen::Matrix3d::Identity();

  // The roll and pitch that `alignment_from_base` removed, in radians, signed, in the
  // base frame. Diagnostics only; the rotation above is authoritative.
  double removed_roll_rad = 0.0;
  double removed_pitch_rad = 0.0;

  // Angle between the aligned frame's +Z and true gravity-up after alignment, radians.
  // The P5 gate is "residual ground tilt <= 0.1 deg", measured on exactly this number.
  double residual_tilt_rad = 0.0;

  std::size_t size() const { return points.size(); }
  bool empty() const { return points.empty(); }
  void Reserve(std::size_t capacity) { points.reserve(capacity); }
  void Clear() { points.clear(); }

  const char* Validate() const {
    if (frame != FrameId::kGravityAlignedBase) {
      return "GravityAlignedCloud::frame must be kGravityAlignedBase";
    }
    if (motion_compensation != MotionCompensation::kDeskewedToStamp) {
      return "GravityAlignedCloud::motion_compensation must be kDeskewedToStamp";
    }
    if (!IsFinite(stamp_s)) return "GravityAlignedCloud::stamp_s must be finite";
    if (!alignment_from_base.allFinite()) {
      return "GravityAlignedCloud::alignment_from_base must be finite";
    }
    // A rotation, not merely a matrix: orthonormal with determinant +1. A silently
    // non-orthonormal alignment would scale every range without changing any flag.
    const Eigen::Matrix3d should_be_identity =
        alignment_from_base.transpose() * alignment_from_base;
    if ((should_be_identity - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() > 1e-9) {
      return "GravityAlignedCloud::alignment_from_base is not orthonormal";
    }
    if (alignment_from_base.determinant() < 0.0) {
      return "GravityAlignedCloud::alignment_from_base is a reflection, not a rotation";
    }
    if (!IsFinite(removed_roll_rad) || !IsFinite(removed_pitch_rad)) {
      return "GravityAlignedCloud removed roll/pitch must be finite";
    }
    if (!IsFiniteNonNegative(residual_tilt_rad)) {
      return "GravityAlignedCloud::residual_tilt_rad must be finite and >= 0";
    }
    for (const auto& point : points) {
      if (!IsFinite(point.position)) return "GravityAlignedCloud contains a non-finite position";
    }
    return nullptr;
  }

  bool IsValid() const { return Validate() == nullptr; }
};

}  // namespace perception::core

#endif  // PERCEPTION_CORE_CONTRACTS_POINT_CLOUDS_H_
