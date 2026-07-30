// LiveScanOverlay - draws the live scan into the MuJoCo viewer's 3D scene.
//
// This is the interactive view: the vendored `simulate` app exposes an optional
// user-owned mjvScene (`Simulate::user_scn`) whose geoms are appended to the render scene
// every frame. Filling it from the physics thread, inside the same `sim.mtx` scope the
// scan is taken in, means the overlay is always consistent with the mjData the rays were
// cast against - there is no second copy to go stale.
//
// The overlay only ever READS the cloud; it owns no perception state and mutates nothing.
#ifndef PERCEPTION_ADAPTERS_MUJOCO_LIVE_SCAN_OVERLAY_H_
#define PERCEPTION_ADAPTERS_MUJOCO_LIVE_SCAN_OVERLAY_H_

#include <string>

#include <mujoco/mujoco.h>

#include "perception/core/contracts/frame_transform_snapshot.h"
#include "perception/core/contracts/timed_point_cloud.h"
#include "perception/integration/perception_config.h"

namespace perception::adapters::mujoco {

class LiveScanOverlay {
 public:
  LiveScanOverlay() = default;
  ~LiveScanOverlay();

  LiveScanOverlay(const LiveScanOverlay&) = delete;
  LiveScanOverlay& operator=(const LiveScanOverlay&) = delete;

  // Allocates the user scene. `expected_rays` sizes the geom budget; the overlay never
  // exceeds it. Safe to call again after a model reload - the previous scene is freed.
  bool Allocate(const mjModel* model, const integration::LidarVisualizationConfig& config,
                std::size_t expected_rays, std::string& error);

  void Free();

  // Rebuilds the geom list from one scan. Cheap: no allocation, just a memcpy-scale fill.
  void Update(const core::TimedPointCloud& cloud, const core::FrameTransformSnapshot& snapshot,
              const core::ScanStats& stats);

  bool allocated() const { return allocated_; }
  mjvScene* scene() { return allocated_ ? &scene_ : nullptr; }

  // Number of geoms the last Update emitted, for the status line.
  int drawn_geoms() const { return allocated_ ? scene_.ngeom : 0; }

 private:
  bool PushGeom(int type, const mjtNum size[3], const mjtNum pos[3], const float rgba[4]);
  bool PushLine(const mjtNum from[3], const mjtNum to[3], const float rgba[4], double width);

  integration::LidarVisualizationConfig config_;
  mjvScene scene_{};
  bool allocated_ = false;
  int max_geoms_ = 0;
};

}  // namespace perception::adapters::mujoco

#endif  // PERCEPTION_ADAPTERS_MUJOCO_LIVE_SCAN_OVERLAY_H_
