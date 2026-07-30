// FrameProviderMj - the only module that turns MuJoCo state into perception geometry.
//
// Resolves the LiDAR site and its parent body once at bind time, then snapshots the
// world<-base, world<-sensor and base<-sensor transforms per call.
#ifndef PERCEPTION_ADAPTERS_MUJOCO_FRAME_PROVIDER_MJ_H_
#define PERCEPTION_ADAPTERS_MUJOCO_FRAME_PROVIDER_MJ_H_

#include <string>

#include <mujoco/mujoco.h>

#include "perception/core/contracts/frame_transform_snapshot.h"
#include "perception/integration/perception_config.h"

namespace perception::adapters::mujoco {

class FrameProviderMj {
 public:
  FrameProviderMj() = default;

  // Resolves the site named by the extrinsic config plus the robot root body used for
  // self-hit accounting. Returns false and sets `error` if either is missing.
  bool Bind(const mjModel* model, const integration::LidarConfig& config, std::string& error);

  // Reads the current transforms out of `data`. The caller is responsible for holding
  // whatever lock keeps `data` consistent (in the simulator, `sim.mtx`).
  core::FrameTransformSnapshot Snapshot(const mjModel* model, const mjData* data) const;

  bool bound() const { return site_id_ >= 0; }
  int site_id() const { return site_id_; }
  int parent_body_id() const { return parent_body_id_; }
  int robot_root_body_id() const { return robot_root_body_id_; }

  // True if `body_id` is the robot root or any descendant of it. Used to classify a hit
  // as "self" without assuming anything about geom naming.
  static bool IsSelfBody(const mjModel* model, int body_id, int robot_root_body_id);

 private:
  int site_id_ = -1;
  int parent_body_id_ = -1;
  int robot_root_body_id_ = -1;
};

}  // namespace perception::adapters::mujoco

#endif  // PERCEPTION_ADAPTERS_MUJOCO_FRAME_PROVIDER_MJ_H_
