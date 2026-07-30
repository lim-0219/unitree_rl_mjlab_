#include "perception/core/projection/scan_projector.h"

#include <cmath>
#include <cstddef>

namespace perception::core {

const char* ScanProjectorParams::Validate() const {
  if (bins < kMinScanBins) return "ScanProjectorParams::bins must be >= 8";
  // Mirrors the projection.bins ceiling in the config loader: a bin count large enough to
  // be a preallocation hazard there is one here too.
  if (bins > 36000) return "ScanProjectorParams::bins must be <= 36000";
  if (!IsFinite(angle_min_rad) || !IsFinite(angle_max_rad)) {
    return "ScanProjectorParams angle bounds must be finite";
  }
  if (!(angle_min_rad < angle_max_rad)) {
    return "ScanProjectorParams::angle_min_rad must be < angle_max_rad";
  }
  if (!IsFiniteNonNegative(range_min_m)) return "ScanProjectorParams::range_min_m must be >= 0";
  if (!IsFinite(range_max_m)) return "ScanProjectorParams::range_max_m must be finite";
  if (!(range_min_m < range_max_m)) {
    return "ScanProjectorParams::range_min_m must be < range_max_m";
  }
  if (!IsFinite(height_band_min_m) || !IsFinite(height_band_max_m)) {
    return "ScanProjectorParams height band must be finite";
  }
  if (!(height_band_min_m < height_band_max_m)) {
    return "ScanProjectorParams::height_band_min_m must be < height_band_max_m";
  }
  return nullptr;
}

const char* ScanProjector::Project(const LabeledPointCloud& cloud, ProjectedScan* scan) const {
  if (scan == nullptr) return "ScanProjector::Project requires a caller-owned scan buffer";
  if (const char* reason = params_.Validate()) return reason;
  if (cloud.frame != FrameId::kGravityAlignedBase) {
    return "ScanProjector::Project requires a gravity-aligned cloud";
  }
  if (cloud.labels.size() != cloud.points.size()) {
    return "ScanProjector::Project requires labels parallel to points";
  }

  // Metadata first: Resize derives angle_increment_rad from the angle bounds, so those have
  // to be in place before it runs.
  scan->angle_min_rad = params_.angle_min_rad;
  scan->angle_max_rad = params_.angle_max_rad;
  scan->range_min_m = params_.range_min_m;
  scan->range_max_m = params_.range_max_m;
  scan->height_band_min_m = params_.height_band_min_m;
  scan->height_band_max_m = params_.height_band_max_m;
  scan->stamp_s = cloud.stamp_s;
  scan->sequence = cloud.sequence;
  scan->frame = FrameId::kGravityAlignedBase;
  scan->motion_compensation = cloud.motion_compensation;

  // Always exactly 0: the scan is synchronic after deskew, matching upstream's
  // `scan_msg->time_increment = 0.0`. `scan_time_s` is deliberately NOT touched - it is the
  // length of the accumulation window, which the pipeline knows and the projector does not,
  // and inventing a value here would launder a guess as a measurement.
  scan->time_increment_s = 0.0;

  // Allocation-free in steady state: assign() over a buffer that is already the right size
  // reuses its storage, so only the first frame (or a bin-count change) allocates.
  scan->Resize(params_.bins, params_.collect_bin_histogram);

  const int bins = params_.bins;
  const double angle_min = params_.angle_min_rad;
  const double angle_max = params_.angle_max_rad;
  const double increment = scan->angle_increment_rad;
  const double range_min = params_.range_min_m;
  const double range_max = params_.range_max_m;
  const double band_min = params_.height_band_min_m;
  const double band_max = params_.height_band_max_m;

  float* const ranges = scan->ranges.data();
  int32_t* const counts =
      scan->bin_point_counts.empty() ? nullptr : scan->bin_point_counts.data();

  ProjectionStats stats;
  stats.total_bins = bins;

  const std::size_t point_count = cloud.points.size();
  for (std::size_t i = 0; i < point_count; ++i) {
    // Ground, self and invalid points are not offered to the projector at all, so they are
    // outside the census rather than a bucket inside it.
    if (cloud.labels[i] != PointLabel::kNonGround) continue;
    ++stats.input_points;

    // Widened once, here. Everything below is double arithmetic on these three values -
    // see divergence 3 in the header for why the width is pinned rather than inherited
    // from whichever overload happens to be visible.
    const Eigen::Vector3f& position = cloud.points[i].position;
    const double x = static_cast<double>(position.x());
    const double y = static_cast<double>(position.y());
    const double z = static_cast<double>(position.z());

    // Upstream tests isnan() only. Testing isfinite() additionally catches infinities here
    // rather than at the height or range gate. That cannot change `ranges`: both angle
    // bounds, both range bounds and both band edges are finite by Validate(), so an
    // infinite coordinate is rejected either way - only the counter it lands in differs.
    // The equivalence test carries a scene of infinities that proves exactly this.
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      ++stats.rejected_non_finite;
      continue;
    }

    if (z > band_max || z < band_min) {
      ++stats.rejected_height_band;
      continue;
    }

    const double range = std::hypot(x, y);
    if (range < range_min || range > range_max) {
      ++stats.rejected_range;
      continue;
    }

    const double angle = std::atan2(y, x);
    if (angle < angle_min || angle > angle_max) {
      ++stats.rejected_angle;
      continue;
    }

    int index = static_cast<int>((angle - angle_min) / increment);
    // Divergence 1: upstream indexes unguarded and walks off the end of its buffer here.
    if (index >= bins) index = bins - 1;
    if (index < 0) index = 0;  // Unreachable given the window gate; kept as a hard floor.

    ++stats.binned_points;
    if (counts != nullptr) ++counts[index];

    // Divergence 2: "empty OR nearer" is upstream's `range < ranges[index]` with an +inf
    // sentinel, rewritten for a NaN one. The stored value is a float and the comparison is
    // against that float widened back to double - deliberately NOT a double running
    // minimum, which would be a different algorithm that happens to agree most of the time.
    float& slot = ranges[index];
    if (std::isnan(slot)) {
      slot = static_cast<float>(range);
      ++stats.occupied_bins;
    } else if (range < static_cast<double>(slot)) {
      slot = static_cast<float>(range);
    }
  }

  scan->stats = stats;
  return nullptr;
}

}  // namespace perception::core
