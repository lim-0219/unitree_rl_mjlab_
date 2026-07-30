// TimedPointCloud and the per-scan census that accompanies it.
//
// See perception/docs/architecture.md, "Internal data contracts".
#ifndef PERCEPTION_CORE_CONTRACTS_TIMED_POINT_CLOUD_H_
#define PERCEPTION_CORE_CONTRACTS_TIMED_POINT_CLOUD_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "perception/core/contracts/frames.h"

namespace perception::core {

// Motion-compensation state, so a consumer can never mistake a raw cloud for a
// deskewed one. The P4 slice only ever produces kNone.
enum class MotionCompensation { kNone = 0, kDeskewedToStamp };

struct RawTimedPoint {
  // Float, because at ~20k points/frame this is the memory-heavy field and downstream
  // stages do not need more. The EXACT hit is always recoverable in double as
  // origin + range_m * direction(ray_index), which is what the validation suite compares
  // against analytic geometry - so the float here never limits a correctness claim.
  Eigen::Vector3f position{0.0f, 0.0f, 0.0f};  // In TimedPointCloud::frame.

  double range_m = 0.0;        // Along-ray distance, as measured. Kept in double.
  float time_offset_s = 0.0f;  // Relative to TimedPointCloud::stamp_s.
  int32_t ray_index = -1;      // Index into RayPattern; recovers direction and timing.
  int32_t geom_id = -1;        // MuJoCo geom that terminated the ray.
  int32_t body_id = -1;        // Owning body; retained for self-hit accounting.
};

struct TimedPointCloud {
  std::vector<RawTimedPoint> points;
  double stamp_s = 0.0;  // MuJoCo sim time at the END of the frame (mjData::time).
  uint64_t sequence = 0;
  FrameId frame = FrameId::kSensor;
  MotionCompensation motion_compensation = MotionCompensation::kNone;

  std::size_t size() const { return points.size(); }
  bool empty() const { return points.empty(); }
  void Clear() {
    points.clear();
    motion_compensation = MotionCompensation::kNone;
  }
};

// Per-scan census. Every ray is accounted for by exactly one of the terminal outcomes
// (no_hit + self_rejected + range_rejected + accepted == rays_cast), which is what makes
// the self-hit rate a trustworthy number rather than a ratio of two unrelated counters.
struct ScanStats {
  int rays_cast = 0;
  int raw_hits = 0;            // Rays that hit any geom at all.
  int no_hit = 0;              // Rays that hit nothing.
  int self_rejected = 0;       // Hit a geom belonging to the robot.
  int range_rejected = 0;      // Hit outside [min_range_m, max_range_m].
  int accepted = 0;            // Points handed to the consumer.

  // Diagnostic for the aperture: how many rays would have terminated on an
  // aperture-excluded (group-masked) geom had the mask not been applied. This is the
  // number that shows the aperture is doing work rather than being a no-op.
  int aperture_transmitted = 0;
  bool aperture_diagnostic_valid = false;

  double scan_wall_time_us = 0.0;

  double SelfHitFraction() const {
    return rays_cast > 0 ? static_cast<double>(self_rejected) / rays_cast : 0.0;
  }

  bool IsBalanced() const {
    return no_hit + self_rejected + range_rejected + accepted == rays_cast;
  }
};

}  // namespace perception::core

#endif  // PERCEPTION_CORE_CONTRACTS_TIMED_POINT_CLOUD_H_
