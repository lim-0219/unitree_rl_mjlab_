#include "perception/adapters/mujoco/oracle_provider_mj.h"

#include <cmath>

namespace perception::adapters::mujoco {

bool OracleProviderMj::BindModel(const mjModel* model,
                                const integration::FramesConfig& frames,
                                std::string& error) {
  base_body_id_ = -1;
  if (model == nullptr) {
    error = "model is null";
    return false;
  }
  const int body_id = mj_name2id(model, mjOBJ_BODY, frames.base_body.c_str());
  if (body_id < 0) {
    error = "frames.base_body '" + frames.base_body + "' was not found in the model";
    return false;
  }
  base_body_id_ = body_id;
  return true;
}

void OracleProviderMj::SampleObstacles(double stamp_s,
                                       std::vector<core::OracleObstacleState>& out) const {
  out.clear();
  if (manager_ == nullptr) return;

  // Exactly the call the inline code made, with exactly its locking: Snapshot() takes the
  // manager's own mutex and hands back a copy.
  const std::vector<::dpcbf::DynamicObstacle> snapshot = manager_->Snapshot();
  out.reserve(snapshot.size());

  for (std::size_t index = 0; index < snapshot.size(); ++index) {
    const ::dpcbf::DynamicObstacle& obstacle = snapshot[index];
    core::OracleObstacleState state;
    // The id IS the snapshot index. That is not an arbitrary choice - it is what the inline
    // conversion used (`static_cast<int>(obstacle_id)` over the loop index), and DPCBF's
    // top-k constraint selection and the visualizer's per-constraint colouring both key off
    // it, so changing the id scheme would change behaviour even with identical geometry.
    state.id = static_cast<int32_t>(index);
    state.center = Eigen::Vector2d(obstacle.position[0], obstacle.position[1]);
    state.velocity = Eigen::Vector2d(obstacle.velocity[0], obstacle.velocity[1]);
    state.radius_m = obstacle.radius;
    state.stamp_s = stamp_s;
    state.frame = core::FrameId::kWorld;

    // NO Validate() GATE HERE, DELIBERATELY. OracleObstacleState::Validate() rejects a
    // non-positive radius, and dropping such an obstacle would be a behaviour change: the
    // inline code passed everything the snapshot returned straight through. The oracle path's
    // job in this phase is to reproduce that bit for bit, and a filter - however defensible -
    // is not reproduction. Validity gating belongs to the estimated path's safety stage.
    out.push_back(state);
  }
}

OracleRobotPose OracleProviderMj::SampleRobot(const mjModel* model,
                                              const mjData* data) const {
  OracleRobotPose pose;
  if (model == nullptr || data == nullptr || base_body_id_ < 0) {
    return pose;
  }

  // Same five numbers, same formulas, as the simulator's ReadRobotGroundTruth. Kept
  // numerically identical on purpose: this phase must not perturb the robot state either,
  // even though the simulator keeps calling its own copy for the live filter (see the hook
  // comment in main.cc for why that call was deliberately not moved).
  const mjtNum* position = data->xpos + 3 * base_body_id_;
  const mjtNum* rotation = data->xmat + 9 * base_body_id_;
  pose.stamp_s = data->time;
  pose.x_m = position[0];
  pose.y_m = position[1];
  pose.yaw_rad = std::atan2(rotation[3], rotation[0]);

  mjtNum body_velocity[6] = {};
  mj_objectVelocity(model, data, mjOBJ_BODY, base_body_id_, body_velocity, 1);
  pose.sagittal_velocity_mps = body_velocity[3];
  pose.lateral_velocity_mps = body_velocity[4];

  pose.valid = true;
  return pose;
}

void OracleProviderMj::Sample(const mjModel* model, const mjData* data,
                              OracleSample& out) const {
  out.stamp_s = data != nullptr ? data->time : 0.0;
  SampleObstacles(out.stamp_s, out.obstacles);
  out.robot = SampleRobot(model, data);
}

}  // namespace perception::adapters::mujoco
