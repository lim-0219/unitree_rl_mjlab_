// ProjectedScan - the detector-facing contract, and the module boundary that neutralises
// obstacle_detector's Livox-hostile assumptions.
//
// The ported detector needs only these numerical semantics; it never needs
// sensor_msgs/LaserScan. Upstream's ingestion reads exactly angle_min, angle_increment
// and the range bounds (obstacle_extractor.cpp:132-146), all of which are present here.
//
// MISSING-BIN POLICY. Internally a bin with no return is NaN. The ROS2 adapter converts
// NaN to +inf on the way out, matching upstream pointcloud_to_laserscan's `use_inf=true`
// default. NaN is the internal choice because +inf silently survives arithmetic while NaN
// poisons it visibly, and a bin that was never measured must never be mistaken for a bin
// measured at very long range.
#ifndef PERCEPTION_CORE_CONTRACTS_PROJECTED_SCAN_H_
#define PERCEPTION_CORE_CONTRACTS_PROJECTED_SCAN_H_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "perception/core/contracts/frames.h"
#include "perception/core/contracts/timed_point_cloud.h"
#include "perception/core/contracts/validation.h"

namespace perception::core {

// The smallest bin count the projector will accept. Below this the angular resolution is
// coarser than 45 degrees and the split-and-merge stage downstream has nothing to work
// with. Mirrored by the `projection.bins >= 8` config constraint.
inline constexpr int kMinScanBins = 8;

// Where every input point went. Reported per scan so the section-12 "information-loss
// stats (points/bin histogram)" gate has a source that cannot drift from the code.
struct ProjectionStats {
  // DENOMINATOR for every fraction below: the non-ground points the projector was
  // offered. Not the whole labelled cloud, and not the accepted subset.
  int32_t input_points = 0;

  int32_t rejected_non_finite = 0;
  int32_t rejected_height_band = 0;  // Outside [height_band_min_m, height_band_max_m].
  int32_t rejected_range = 0;        // Outside [range_min_m, range_max_m].
  int32_t rejected_angle = 0;        // Outside [angle_min_rad, angle_max_rad].

  // Points that landed in a bin, INCLUDING those later beaten by a nearer return in the
  // same bin. This is what makes the loss fraction below computable.
  int32_t binned_points = 0;

  int32_t occupied_bins = 0;
  int32_t total_bins = 0;

  double wall_time_us = 0.0;

  bool IsBalanced() const {
    return rejected_non_finite + rejected_height_band + rejected_range + rejected_angle +
               binned_points ==
           input_points;
  }

  // FRACTION DEFINITION: occupied_bins / total_bins. How much of the 360-degree scan
  // carries a measurement this frame.
  double OccupancyFraction() const {
    return total_bins > 0 ? static_cast<double>(occupied_bins) / total_bins : 0.0;
  }

  // FRACTION DEFINITION: (binned_points - occupied_bins) / binned_points. The share of
  // points that reached a bin and were then DISCARDED by the nearest-return rule. This is
  // the reduction's information loss; the denominator is binned points, NOT input points,
  // because a point rejected by the height band was never a candidate for a bin.
  double NearestReturnLossFraction() const {
    return binned_points > 0
               ? static_cast<double>(binned_points - occupied_bins) / binned_points
               : 0.0;
  }

  const char* Validate() const {
    if (input_points < 0 || rejected_non_finite < 0 || rejected_height_band < 0 ||
        rejected_range < 0 || rejected_angle < 0 || binned_points < 0 || occupied_bins < 0 ||
        total_bins < 0) {
      return "ProjectionStats counts must be >= 0";
    }
    if (!IsBalanced()) return "ProjectionStats point census does not sum to input_points";
    if (occupied_bins > total_bins) return "ProjectionStats::occupied_bins exceeds total_bins";
    if (occupied_bins > binned_points) {
      return "ProjectionStats::occupied_bins exceeds binned_points";
    }
    if (!IsFiniteNonNegative(wall_time_us)) {
      return "ProjectionStats::wall_time_us must be finite and >= 0";
    }
    return nullptr;
  }

  bool IsValid() const { return Validate() == nullptr; }
};

struct ProjectedScan {
  // NaN in a bin means "no return". Float, matching sensor_msgs/LaserScan's own width and
  // halving the cache footprint of the detector's inner loop.
  std::vector<float> ranges;

  // Points that fell into each bin before the nearest-return reduction. Parallel to
  // `ranges` when populated; may be left EMPTY when the histogram is not being collected,
  // which is the only optional buffer in this contract.
  std::vector<int32_t> bin_point_counts;

  double angle_min_rad = -kPi;
  double angle_max_rad = kPi;
  double angle_increment_rad = 0.0;  // (angle_max - angle_min) / bins.

  // Always exactly 0. The scan is synchronic AFTER deskew, and upstream
  // pointcloud_to_laserscan publishes 0.0 here too. Kept as a field, not a constant, so
  // the ROS2 adapter has something to copy and a future non-deskewed path is a value
  // change rather than a contract change.
  double time_increment_s = 0.0;

  // Metadata only: the length of the accumulation window this scan came from. Nothing in
  // the pipeline computes with it; the ROS2 adapter publishes it as LaserScan::scan_time.
  double scan_time_s = 0.0;

  double range_min_m = 0.0;
  double range_max_m = 0.0;

  // The slab of the gravity-aligned frame that was collapsed into this 2-D scan.
  double height_band_min_m = 0.0;
  double height_band_max_m = 0.0;

  double stamp_s = 0.0;
  uint64_t sequence = 0;
  FrameId frame = FrameId::kGravityAlignedBase;
  MotionCompensation motion_compensation = MotionCompensation::kDeskewedToStamp;

  ProjectionStats stats;

  int bins() const { return static_cast<int>(ranges.size()); }

  // Bin centre convention: bin i spans [angle_min + i*inc, angle_min + (i+1)*inc) and is
  // addressed by its LOWER edge, matching upstream's index = (angle - angle_min)/inc.
  double AngleAt(int index) const {
    return angle_min_rad + static_cast<double>(index) * angle_increment_rad;
  }

  bool HasReturn(int index) const { return !std::isnan(ranges[static_cast<std::size_t>(index)]); }

  void Resize(int bin_count, bool with_histogram) {
    ranges.assign(static_cast<std::size_t>(bin_count), std::numeric_limits<float>::quiet_NaN());
    if (with_histogram) {
      bin_point_counts.assign(static_cast<std::size_t>(bin_count), 0);
    } else {
      bin_point_counts.clear();
    }
    angle_increment_rad =
        bin_count > 0 ? (angle_max_rad - angle_min_rad) / static_cast<double>(bin_count) : 0.0;
    stats = ProjectionStats{};
    stats.total_bins = bin_count;
  }

  const char* Validate() const {
    if (frame != FrameId::kGravityAlignedBase) {
      return "ProjectedScan::frame must be kGravityAlignedBase";
    }
    if (bins() < kMinScanBins) return "ProjectedScan must have at least 8 bins";
    if (!bin_point_counts.empty() && bin_point_counts.size() != ranges.size()) {
      return "ProjectedScan::bin_point_counts must be empty or parallel to ranges";
    }
    if (!IsFinite(angle_min_rad) || !IsFinite(angle_max_rad)) {
      return "ProjectedScan angle bounds must be finite";
    }
    if (!(angle_min_rad < angle_max_rad)) {
      return "ProjectedScan::angle_min_rad must be < angle_max_rad";
    }
    const double expected_increment =
        (angle_max_rad - angle_min_rad) / static_cast<double>(bins());
    if (std::abs(angle_increment_rad - expected_increment) > 1e-12) {
      return "ProjectedScan::angle_increment_rad must equal (angle_max - angle_min) / bins";
    }
    if (time_increment_s != 0.0) {
      return "ProjectedScan::time_increment_s must be 0 (the scan is synchronic after deskew)";
    }
    if (!IsFiniteNonNegative(scan_time_s)) {
      return "ProjectedScan::scan_time_s must be finite and >= 0";
    }
    if (!IsFiniteNonNegative(range_min_m)) return "ProjectedScan::range_min_m must be >= 0";
    if (!(range_min_m < range_max_m)) {
      return "ProjectedScan::range_min_m must be < range_max_m";
    }
    if (!IsFinite(range_max_m)) return "ProjectedScan::range_max_m must be finite";
    if (!IsFinite(height_band_min_m) || !IsFinite(height_band_max_m)) {
      return "ProjectedScan height band must be finite";
    }
    if (!(height_band_min_m < height_band_max_m)) {
      return "ProjectedScan::height_band_min_m must be < height_band_max_m";
    }
    if (!IsFinite(stamp_s)) return "ProjectedScan::stamp_s must be finite";

    // A range outside the declared bounds is the one error the detector cannot detect:
    // it would happily group a point the scan claims cannot exist.
    for (const float range : ranges) {
      if (std::isnan(range)) continue;  // Legitimate empty bin.
      if (!std::isfinite(range)) return "ProjectedScan contains a non-NaN, non-finite range";
      if (range < range_min_m || range > range_max_m) {
        return "ProjectedScan contains a range outside [range_min_m, range_max_m]";
      }
    }
    if (stats.total_bins != bins()) {
      return "ProjectedScan::stats.total_bins must equal the bin count";
    }
    return stats.Validate();
  }

  bool IsValid() const { return Validate() == nullptr; }
};

}  // namespace perception::core

#endif  // PERCEPTION_CORE_CONTRACTS_PROJECTED_SCAN_H_
