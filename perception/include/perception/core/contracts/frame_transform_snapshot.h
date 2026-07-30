// FrameTransformSnapshot - one consistent view of where the sensor and its parent body
// were at a given sim time.
//
// Produced by adapters/mujoco/frame_provider_mj; the only module that converts MuJoCo
// state into perception geometry. Consumed (later) by deskew and by the ROS2 TF adapter.
#ifndef PERCEPTION_CORE_CONTRACTS_FRAME_TRANSFORM_SNAPSHOT_H_
#define PERCEPTION_CORE_CONTRACTS_FRAME_TRANSFORM_SNAPSHOT_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "perception/core/contracts/validation.h"

namespace perception::core {

struct FrameTransformSnapshot {
  double stamp_s = 0.0;  // mjData::time.

  // world <- base (the extrinsic's parent body; `torso_link` on the G1).
  Eigen::Isometry3d world_from_base = Eigen::Isometry3d::Identity();

  // world <- sensor, read from the MJCF site, i.e. including the extrinsic.
  Eigen::Isometry3d world_from_sensor = Eigen::Isometry3d::Identity();

  // base <- sensor. Must equal the authored extrinsic for every robot pose, because the
  // mount is a fixed joint. This is what the rigid-attachment check (V5) verifies.
  Eigen::Isometry3d base_from_sensor = Eigen::Isometry3d::Identity();

  // Gravity direction expressed in the base frame, for later gravity alignment.
  Eigen::Vector3d gravity_in_base{0.0, 0.0, -1.0};

  bool valid = false;

  // Added in the contract-freeze phase; the snapshot previously carried only the `valid`
  // flag, which says a producer believed the snapshot but not that it is self-consistent.
  //
  // The load-bearing check is the composition one: world_from_base * base_from_sensor must
  // reproduce world_from_sensor. All three are read independently from MuJoCo, so a
  // mismatch means the body pose and the site pose came from different `mjData` states -
  // exactly the failure the V6 pose-tracking test exists to rule out, now cheap enough to
  // assert on every snapshot.
  const char* Validate() const {
    if (!world_from_base.matrix().allFinite() || !world_from_sensor.matrix().allFinite() ||
        !base_from_sensor.matrix().allFinite()) {
      return "FrameTransformSnapshot transforms must be finite";
    }
    if (!IsFinite(stamp_s)) return "FrameTransformSnapshot::stamp_s must be finite";
    if (!IsFinite(gravity_in_base)) {
      return "FrameTransformSnapshot::gravity_in_base must be finite";
    }
    if (!valid) return "FrameTransformSnapshot::valid is false";

    for (const Eigen::Isometry3d* pose : {&world_from_base, &world_from_sensor, &base_from_sensor}) {
      const Eigen::Matrix3d rotation = pose->linear();
      if ((rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() >
          1e-9) {
        return "FrameTransformSnapshot contains a non-orthonormal rotation";
      }
    }
    // Unit, because it is a direction. A zero or unnormalised gravity vector would tilt
    // the gravity-aligned frame by an amount nothing else reports.
    if (std::abs(gravity_in_base.norm() - 1.0) > 1e-9) {
      return "FrameTransformSnapshot::gravity_in_base must be a unit vector";
    }
    const Eigen::Isometry3d composed = world_from_base * base_from_sensor;
    if ((composed.matrix() - world_from_sensor.matrix()).cwiseAbs().maxCoeff() > 1e-6) {
      return "FrameTransformSnapshot: world_from_base * base_from_sensor != world_from_sensor";
    }
    return nullptr;
  }

  bool IsValid() const { return Validate() == nullptr; }
};

}  // namespace perception::core

#endif  // PERCEPTION_CORE_CONTRACTS_FRAME_TRANSFORM_SNAPSHOT_H_
