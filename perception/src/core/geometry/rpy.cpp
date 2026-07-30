#include "perception/core/geometry/rpy.h"

namespace perception::core {

// The published R_torso,lidar for the G1's Mid-360, transcribed from
// g1_mid360_extrinsic.md section 4.5. Kept here as an independent reference so the RPY
// composition order can be checked against a number nobody derived from the code.
//
// Columns are the sensor axes expressed in `torso_link`:
//   +X (forward) = (+0.998694, 0, -0.051098)   forward, 2.929 deg nose-down
//   +Y (left)    = ( 0, -1, 0)                 points to the robot's RIGHT (flipped)
//   +Z (up)      = (-0.051098, 0, -0.998694)   points DOWN (flipped)
Eigen::Matrix3d Mid360PublishedRotationTorsoFromLidar() {
  Eigen::Matrix3d rotation;
  rotation <<  0.998693622,  0.000000000, -0.051098431,
               0.000000000, -1.000000000,  0.000000000,
              -0.051098431,  0.000000000, -0.998693622;
  return rotation;
}

}  // namespace perception::core
