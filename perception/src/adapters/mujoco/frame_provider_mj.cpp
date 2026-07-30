#include "perception/adapters/mujoco/frame_provider_mj.h"

#include <Eigen/Geometry>

namespace perception::adapters::mujoco {
namespace {

Eigen::Isometry3d MakeIsometry(const mjtNum* position, const mjtNum* rotation_row_major) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  // MuJoCo xmat/site_xmat are 3x3 row-major.
  Eigen::Matrix3d rotation;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      rotation(row, col) = rotation_row_major[3 * row + col];
    }
  }
  pose.linear() = rotation;
  pose.translation() = Eigen::Vector3d(position[0], position[1], position[2]);
  return pose;
}

}  // namespace

bool FrameProviderMj::Bind(const mjModel* model, const integration::LidarConfig& config,
                           std::string& error) {
  site_id_ = -1;
  parent_body_id_ = -1;
  robot_root_body_id_ = -1;

  if (model == nullptr) {
    error = "model is null";
    return false;
  }

  const int site_id = mj_name2id(model, mjOBJ_SITE, config.extrinsic.site_name.c_str());
  if (site_id < 0) {
    error = "MJCF site '" + config.extrinsic.site_name +
            "' was not found. The Mid-360 mount must be authored as a <site> on '" +
            config.extrinsic.parent_body + "'.";
    return false;
  }

  const int expected_parent =
      mj_name2id(model, mjOBJ_BODY, config.extrinsic.parent_body.c_str());
  if (expected_parent < 0) {
    error = "extrinsic parent body '" + config.extrinsic.parent_body + "' was not found";
    return false;
  }

  const int root_body =
      mj_name2id(model, mjOBJ_BODY, config.self_filter.robot_root_body.c_str());
  if (root_body < 0) {
    error = "self_filter.robot_root_body '" + config.self_filter.robot_root_body +
            "' was not found";
    return false;
  }

  site_id_ = site_id;
  parent_body_id_ = model->site_bodyid[site_id];
  robot_root_body_id_ = root_body;
  return true;
}

core::FrameTransformSnapshot FrameProviderMj::Snapshot(const mjModel* model,
                                                       const mjData* data) const {
  core::FrameTransformSnapshot snapshot;
  if (model == nullptr || data == nullptr || site_id_ < 0 || parent_body_id_ < 0) {
    return snapshot;
  }

  snapshot.stamp_s = data->time;
  snapshot.world_from_base =
      MakeIsometry(&data->xpos[3 * parent_body_id_], &data->xmat[9 * parent_body_id_]);
  snapshot.world_from_sensor =
      MakeIsometry(&data->site_xpos[3 * site_id_], &data->site_xmat[9 * site_id_]);
  snapshot.base_from_sensor = snapshot.world_from_base.inverse() * snapshot.world_from_sensor;

  // Gravity is authoritative from the model rather than assumed to be -z.
  const Eigen::Vector3d gravity_world(model->opt.gravity[0], model->opt.gravity[1],
                                      model->opt.gravity[2]);
  const double gravity_norm = gravity_world.norm();
  snapshot.gravity_in_base = gravity_norm > 0.0
                                 ? (snapshot.world_from_base.linear().transpose() *
                                    (gravity_world / gravity_norm))
                                 : Eigen::Vector3d(0.0, 0.0, -1.0);

  snapshot.valid = true;
  return snapshot;
}

bool FrameProviderMj::IsSelfBody(const mjModel* model, int body_id, int robot_root_body_id) {
  if (model == nullptr || body_id < 0 || robot_root_body_id < 0) {
    return false;
  }
  // Walk up to the world body (id 0). Guard the loop against a malformed tree.
  for (int current = body_id, guard = 0; current > 0 && guard <= model->nbody; ++guard) {
    if (current == robot_root_body_id) {
      return true;
    }
    current = model->body_parentid[current];
  }
  return robot_root_body_id == body_id;
}

}  // namespace perception::adapters::mujoco
