// Rotation conventions shared by the perception subsystem.
//
// The single load-bearing fact here is the RPY composition order. URDF `rpy` is
// fixed-axis (extrinsic) X-Y-Z, i.e.
//
//     R = Rz(yaw) * Ry(pitch) * Rx(roll)
//
// which is what `RpyToRotation` below implements. Because Unitree publishes the Mid-360
// mount as URDF `rpy`, the published numbers can be pasted in without conversion.
// See g1_mid360_extrinsic.md section 4.3, and the round-trip assertions in
// perception/tests/unit/geometry_rpy_test.cpp.
#ifndef PERCEPTION_CORE_GEOMETRY_RPY_H_
#define PERCEPTION_CORE_GEOMETRY_RPY_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace perception::core {

// Fixed-axis (extrinsic) X-Y-Z, matching URDF <origin rpy="...">.
inline Eigen::Matrix3d RpyToRotation(const Eigen::Vector3d& rpy) {
  return (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

// MuJoCo stores quaternions as (w, x, y, z); Eigen's constructor takes (w, x, y, z) but
// its coefficient order is (x, y, z, w). These two helpers keep the distinction explicit
// so MJCF `quat` attributes are never silently reordered.
inline Eigen::Matrix3d QuatWxyzToRotation(const double q[4]) {
  return Eigen::Quaterniond(q[0], q[1], q[2], q[3]).normalized().toRotationMatrix();
}

inline void RotationToQuatWxyz(const Eigen::Matrix3d& rotation, double q[4]) {
  const Eigen::Quaterniond quaternion(rotation);
  q[0] = quaternion.w();
  q[1] = quaternion.x();
  q[2] = quaternion.y();
  q[3] = quaternion.z();
}

// Rotations are compared through their matrices rather than their quaternions, because
// q and -q denote the same rotation and a naive coefficient comparison would report a
// spurious disagreement. Returns the largest absolute element-wise difference.
inline double RotationMaxAbsDiff(const Eigen::Matrix3d& lhs, const Eigen::Matrix3d& rhs) {
  return (lhs - rhs).cwiseAbs().maxCoeff();
}

// The published R_torso,lidar for the G1's Mid-360 (g1_mid360_extrinsic.md section 4.5),
// as an independent reference the RPY composition can be checked against.
Eigen::Matrix3d Mid360PublishedRotationTorsoFromLidar();

}  // namespace perception::core

#endif  // PERCEPTION_CORE_GEOMETRY_RPY_H_
