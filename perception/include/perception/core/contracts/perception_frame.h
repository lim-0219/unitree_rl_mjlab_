// PerceptionFrame - the per-scan bundle the runner double-buffers, and the ONLY thing the
// ROS2 adapter, the evaluator and the dump writer are allowed to read.
//
// It carries every stage output rather than just the final SafetyObstacles, because the
// headless dump channel is the substitute for a visualization and has to be able to show
// any stage. The intermediate clouds are the expensive part, so the runner may leave them
// empty when `dumps.retain_stage_clouds` is false; `retained_stages` records what was
// actually kept so a consumer can tell "not retained" from "the stage produced nothing".
//
// Frames: MIXED, each element tagged by its own contract - raw cloud in kSensor, deskewed
// in kBase, gravity-aligned/labelled/scan in kGravityAlignedBase, observations onward in
// kWorld. That is why this bundle has no `frame` field of its own.
#ifndef PERCEPTION_CORE_CONTRACTS_PERCEPTION_FRAME_H_
#define PERCEPTION_CORE_CONTRACTS_PERCEPTION_FRAME_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "perception/core/contracts/detection_2d.h"
#include "perception/core/contracts/diagnostics.h"
#include "perception/core/contracts/frame_transform_snapshot.h"
#include "perception/core/contracts/labeled_point_cloud.h"
#include "perception/core/contracts/obstacles.h"
#include "perception/core/contracts/point_clouds.h"
#include "perception/core/contracts/projected_scan.h"
#include "perception/core/contracts/timed_point_cloud.h"
#include "perception/core/contracts/tracking_2d.h"
#include "perception/core/contracts/validation.h"

namespace perception::core {

// Why a frame is unusable. `kNone` with `valid == true` is the only combination the
// selector will consume; every other value routes the frame to the oracle fallback.
enum class FrameInvalidReason : uint8_t {
  kNone = 0,
  kNoTransformSnapshot,  // Deskew had no pose covering the scan window.
  kEmptyCloud,           // The raycaster produced nothing.
  kSegmentationFailed,
  kProjectionFailed,
  kStageException,
  kStale,  // Older than the configured max age when it was published.
};

inline const char* ToString(FrameInvalidReason reason) {
  switch (reason) {
    case FrameInvalidReason::kNoTransformSnapshot: return "no_transform_snapshot";
    case FrameInvalidReason::kEmptyCloud:          return "empty_cloud";
    case FrameInvalidReason::kSegmentationFailed:  return "segmentation_failed";
    case FrameInvalidReason::kProjectionFailed:    return "projection_failed";
    case FrameInvalidReason::kStageException:      return "stage_exception";
    case FrameInvalidReason::kStale:               return "stale";
    case FrameInvalidReason::kNone:                break;
  }
  return "none";
}

// Which optional stage outputs this frame actually retains. A bitmask rather than a set of
// bools so the dump header stays one integer across schema versions.
struct RetainedStages {
  bool raw_cloud = false;
  bool deskewed_cloud = false;
  bool gravity_aligned_cloud = false;
  bool labeled_cloud = false;
  bool projected_scan = false;
  bool clusters = false;
  bool primitives = false;
  bool observations = false;
  bool tracks = false;
};

// Preallocation sizes, supplied by the runner from the config tree. Lives in contracts
// rather than in the config header so that core never has to include integration/.
struct PerceptionFrameCapacities {
  std::size_t max_points = 0;
  std::size_t max_bins = 0;
  std::size_t max_clusters = 0;
  std::size_t max_primitives = 0;
  std::size_t max_observations = 0;
  std::size_t max_tracks = 0;
  std::size_t max_obstacles = 0;
  std::size_t max_matched_pairs = 0;
};

struct PerceptionFrame {
  uint64_t sequence = 0;

  // The scan's own timestamp (end of the accumulation window) and the sim time the runner
  // finished the frame. `publish_stamp_s - stamp_s` is the pipeline latency the selector's
  // staleness test is really about.
  double stamp_s = 0.0;
  double publish_stamp_s = 0.0;

  // The pose used to transform detections into the world frame. Carried with the frame so
  // a consumer never has to look up a snapshot that may since have been overwritten.
  FrameTransformSnapshot transform;

  // Stage outputs. Any of these may be empty when the corresponding RetainedStages flag is
  // false; see the header comment.
  TimedPointCloud raw_cloud;
  DeskewedPointCloud deskewed_cloud;
  GravityAlignedCloud gravity_aligned_cloud;
  LabeledPointCloud labeled_cloud;
  ProjectedScan projected_scan;
  std::vector<Cluster2D> clusters;
  std::vector<FittedPrimitive2D> primitives;
  std::vector<CircleObservation> observations;
  std::vector<TrackState2D> tracks;

  // Always populated on a valid frame - these are the reason the frame exists.
  std::vector<PerceptionObstacle> obstacles;
  std::vector<SafetyObstacle> safety_obstacles;

  EstimationDiagnostics diagnostics;
  StageTiming timing;

  RetainedStages retained;

  bool valid = false;
  FrameInvalidReason invalid_reason = FrameInvalidReason::kNone;

  double latency_s() const { return publish_stamp_s - stamp_s; }

  void Reserve(const PerceptionFrameCapacities& capacities) {
    raw_cloud.points.reserve(capacities.max_points);
    deskewed_cloud.Reserve(capacities.max_points);
    gravity_aligned_cloud.Reserve(capacities.max_points);
    labeled_cloud.Reserve(capacities.max_points);
    projected_scan.ranges.reserve(capacities.max_bins);
    projected_scan.bin_point_counts.reserve(capacities.max_bins);
    clusters.reserve(capacities.max_clusters);
    primitives.reserve(capacities.max_primitives);
    observations.reserve(capacities.max_observations);
    tracks.reserve(capacities.max_tracks);
    obstacles.reserve(capacities.max_obstacles);
    safety_obstacles.reserve(capacities.max_obstacles);
    diagnostics.Reserve(capacities.max_matched_pairs);
  }

  const char* Validate() const {
    if (!IsFinite(stamp_s) || !IsFinite(publish_stamp_s)) {
      return "PerceptionFrame stamps must be finite";
    }
    if (publish_stamp_s < stamp_s) {
      return "PerceptionFrame::publish_stamp_s must be >= stamp_s";
    }
    if (valid && invalid_reason != FrameInvalidReason::kNone) {
      return "a valid PerceptionFrame must carry FrameInvalidReason::kNone";
    }
    if (!valid && invalid_reason == FrameInvalidReason::kNone) {
      return "an invalid PerceptionFrame must say why";
    }
    if (valid) {
      // A valid frame must have a usable pose: everything in world coordinates was
      // produced through it, so an invalid transform silently invalidates every obstacle.
      if (const char* reason = transform.Validate()) return reason;
      if (const char* reason = timing.Validate()) return reason;
      if (const char* reason = diagnostics.Validate()) return reason;
      for (const auto& obstacle : obstacles) {
        if (const char* reason = obstacle.Validate()) return reason;
      }
      for (const auto& obstacle : safety_obstacles) {
        if (const char* reason = obstacle.Validate()) return reason;
      }
      for (const auto& track : tracks) {
        if (const char* reason = track.Validate()) return reason;
      }
      for (const auto& observation : observations) {
        if (const char* reason = observation.Validate()) return reason;
      }
      // Every published SafetyObstacle must trace back to a PerceptionObstacle of the
      // same id. A safety obstacle with no parent is either a duplicate or a fabrication,
      // and the DPCBF adapter has no way to notice either.
      for (const auto& safety : safety_obstacles) {
        bool found = false;
        for (const auto& obstacle : obstacles) {
          if (obstacle.id == safety.id) {
            found = true;
            break;
          }
        }
        if (!found) return "PerceptionFrame: a SafetyObstacle has no matching obstacle id";
      }
      if (retained.projected_scan) {
        if (const char* reason = projected_scan.Validate()) return reason;
      }
      if (retained.labeled_cloud) {
        if (const char* reason = labeled_cloud.Validate()) return reason;
      }
      if (retained.deskewed_cloud) {
        if (const char* reason = deskewed_cloud.Validate()) return reason;
      }
      if (retained.gravity_aligned_cloud) {
        if (const char* reason = gravity_aligned_cloud.Validate()) return reason;
      }
    }
    return nullptr;
  }

  bool IsValid() const { return Validate() == nullptr; }
};

}  // namespace perception::core

#endif  // PERCEPTION_CORE_CONTRACTS_PERCEPTION_FRAME_H_
