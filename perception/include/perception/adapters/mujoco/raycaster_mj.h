// RaycasterMj - simulates the Mid-360 by casting the ray table against the compiled scene.
//
// Two things here are load-bearing correctness decisions rather than preferences; both
// come from findings recorded in g1_mid360_sensor_model.md and skipping either silently
// reproduces a bug that was already diagnosed once:
//
//  1. THE APERTURE. head_link.STL is a closed opaque envelope with no modelled window,
//     and at the official extrinsic the sensor origin is INSIDE it, so a naive cast
//     self-terminates on ~100% of rays and delivers zero usable points. The head geoms
//     are placed in a MuJoCo visibility group which is masked out of the ray cast via
//     mj_ray/mj_multiRay's documented `geomgroup` argument. Note that a post-hoc
//     self-filter cannot fix this: the failure is occlusion, and deleting a point does
//     not recover the measurement it destroyed.
//
//  2. EXACT, NOT BATCHED. mj_multiRay prunes candidate geoms using a per-geom angular
//     bound computed from the 8 AABB corners. That is not a valid outer bound for a geom
//     subtending a large solid angle - the extreme bearing over a large flat face occurs
//     on an EDGE, not a vertex - so centre-of-face rays fall outside the corner-derived
//     bound and the geom is skipped entirely. Measured on this robot: up to 23.7% of wall
//     rays fly straight through an opaque wall. The error is always a missed occlusion,
//     never a wrong range. mj_ray scans every geom and has no such approximation, so
//     `raycast.exact` defaults to true and batched mode exists only as a documented
//     performance fallback.
#ifndef PERCEPTION_ADAPTERS_MUJOCO_RAYCASTER_MJ_H_
#define PERCEPTION_ADAPTERS_MUJOCO_RAYCASTER_MJ_H_

#include <random>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "perception/adapters/mujoco/frame_provider_mj.h"
#include "perception/core/contracts/frame_transform_snapshot.h"
#include "perception/core/contracts/ray_pattern.h"
#include "perception/core/contracts/timed_point_cloud.h"
#include "perception/integration/perception_config.h"
#include "perception/interfaces/i_raycaster.h"

namespace perception::adapters::mujoco {

class RaycasterMj final : public IRaycaster {
 public:
  RaycasterMj() = default;

  // Resolves the site, builds the ray table and preallocates every scratch buffer.
  // Call once per (model, data) pair - i.e. at each of the simulator's model loads,
  // exactly like DynamicObstacleManager::BindModel.
  bool Bind(const mjModel* model, mjData* data, const integration::LidarConfig& config,
            std::string& error);

  // Casts one full frame. The caller must hold whatever lock keeps `data` consistent.
  // `cloud.points` is reused, so there is no steady-state allocation.
  bool Scan(core::TimedPointCloud& cloud, core::ScanStats& stats) override;

  const core::RayPattern& pattern() const override { return pattern_; }
  const core::FrameTransformSnapshot& last_snapshot() const override { return last_snapshot_; }

  bool bound() const { return bound_; }
  const FrameProviderMj& frame_provider() const { return frame_provider_; }

  // When enabled, each scan additionally casts the table WITHOUT the aperture mask and
  // counts how many rays would have terminated on an aperture-excluded geom first. That
  // count is the evidence the aperture is doing work rather than being a no-op. Costs a
  // second full pass, so it is off by default and used by the validation suite and bench.
  void set_collect_aperture_diagnostic(bool enabled) { collect_aperture_diagnostic_ = enabled; }

  // Escape hatch for the before/after aperture evidence: with the mask disabled the
  // caster reproduces the pre-aperture behaviour exactly.
  void set_aperture_enabled(bool enabled) { aperture_enabled_ = enabled; }
  void set_raycast_exact(bool exact) { raycast_exact_ = exact; }

  bool aperture_enabled() const { return aperture_enabled_; }
  bool raycast_exact() const { return raycast_exact_; }

 private:
  const mjtByte* GeomGroupMask() const;

  const mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
  integration::LidarConfig config_;
  FrameProviderMj frame_provider_;
  core::RayPattern pattern_;
  core::FrameTransformSnapshot last_snapshot_;

  bool bound_ = false;
  bool aperture_enabled_ = true;
  bool raycast_exact_ = true;
  bool collect_aperture_diagnostic_ = false;
  uint64_t sequence_ = 0;

  mjtByte geom_group_mask_[mjNGROUP] = {0};

  // Preallocated scratch. World-frame directions are recomputed per scan because the
  // sensor moves; the batched path additionally needs contiguous output arrays.
  std::vector<mjtNum> world_directions_;
  std::vector<mjtNum> batched_distances_;
  std::vector<int> batched_geom_ids_;

  std::mt19937 noise_generator_{42};
};

}  // namespace perception::adapters::mujoco

#endif  // PERCEPTION_ADAPTERS_MUJOCO_RAYCASTER_MJ_H_
