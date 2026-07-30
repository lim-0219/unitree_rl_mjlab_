// IRaycaster - the seam between "where the points came from" and everything downstream.
//
// The MuJoCo implementation lives in adapters/mujoco/raycaster_mj. A real Livox driver
// would implement the same interface, which is the migration seam reserved in
// perception/docs/architecture.md.
#ifndef PERCEPTION_INTERFACES_I_RAYCASTER_H_
#define PERCEPTION_INTERFACES_I_RAYCASTER_H_

#include "perception/core/contracts/frame_transform_snapshot.h"
#include "perception/core/contracts/ray_pattern.h"
#include "perception/core/contracts/timed_point_cloud.h"

namespace perception {

class IRaycaster {
 public:
  virtual ~IRaycaster() = default;

  // Fills `cloud` with one frame's worth of returns and `stats` with the census.
  // Returns false if the sensor could not be located; `cloud` is then left empty.
  // Implementations must not allocate in steady state: `cloud.points` is reused.
  virtual bool Scan(core::TimedPointCloud& cloud, core::ScanStats& stats) = 0;

  virtual const core::RayPattern& pattern() const = 0;

  // The transform snapshot that `Scan` used, for the frame it just produced.
  virtual const core::FrameTransformSnapshot& last_snapshot() const = 0;
};

}  // namespace perception

#endif  // PERCEPTION_INTERFACES_I_RAYCASTER_H_
