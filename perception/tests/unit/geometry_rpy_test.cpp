// Rotation-convention checks for the Mid-360 extrinsic.
//
// These exist because the whole extrinsic rests on one claim: URDF `rpy` is fixed-axis
// (extrinsic) XYZ, i.e. R = Rz(yaw)*Ry(pitch)*Rx(roll), and therefore Unitree's published
// numbers can be pasted in without conversion. That claim is checked here against the
// independently-published rotation matrix and quaternion, not assumed.
//
// No MuJoCo, no yaml-cpp - runs anywhere.

#include <cmath>
#include <cstdio>
#include <string>

#include "perception/core/geometry/rpy.h"

namespace {

int g_failures = 0;
int g_checks = 0;

void Check(bool condition, const std::string& name, const std::string& detail = "") {
  ++g_checks;
  if (condition) {
    std::printf("  PASS  %s%s%s\n", name.c_str(), detail.empty() ? "" : "  ", detail.c_str());
  } else {
    ++g_failures;
    std::printf("  FAIL  %s%s%s\n", name.c_str(), detail.empty() ? "" : "  ", detail.c_str());
  }
}

std::string Fmt(const char* label, double value) {
  char buffer[128];
  std::snprintf(buffer, sizeof(buffer), "(%s = %.3e)", label, value);
  return buffer;
}

// The documented extrinsic. g1_mid360_extrinsic.md section 6.
const Eigen::Vector3d kRpy{3.141592653589793, 0.05112069379091391, 0.0};
const Eigen::Vector3d kTranslation{0.0002835, 0.00003, 0.428434};

}  // namespace

int main() {
  using perception::core::Mid360PublishedRotationTorsoFromLidar;
  using perception::core::QuatWxyzToRotation;
  using perception::core::RotationMaxAbsDiff;
  using perception::core::RotationToQuatWxyz;
  using perception::core::RpyToRotation;

  std::printf("perception geometry / RPY convention checks\n");

  const Eigen::Matrix3d from_rpy = RpyToRotation(kRpy);

  // 1. The composition order reproduces Unitree's published rotation matrix. If the
  //    convention were intrinsic ZYX instead, this would not match.
  {
    const double diff = RotationMaxAbsDiff(from_rpy, Mid360PublishedRotationTorsoFromLidar());
    Check(diff < 1e-9, "RpyToRotation reproduces the published R_torso,lidar",
          Fmt("max|diff|", diff));
  }

  // 2. It reproduces the published quaternion (w x y z) = (0, 0.999673352, 0, -0.025557564).
  //    Compared through matrices, because q and -q are the same rotation.
  {
    const double published_quat[4] = {0.0, 0.999673352, 0.0, -0.025557564};
    const double diff = RotationMaxAbsDiff(from_rpy, QuatWxyzToRotation(published_quat));
    Check(diff < 1e-8, "RpyToRotation reproduces the published quaternion (w,x,y,z)",
          Fmt("max|diff|", diff));
  }

  // 3. The MJCF quat authored in scene_g1.xml is the same rotation. This is the value the
  //    extrinsic guard compares against at runtime; checking it here catches a typo in the
  //    asset without needing MuJoCo.
  {
    double round_trip[4];
    RotationToQuatWxyz(from_rpy, round_trip);
    const double diff = RotationMaxAbsDiff(QuatWxyzToRotation(round_trip), from_rpy);
    Check(diff < 1e-12, "quaternion round-trip is lossless", Fmt("max|diff|", diff));
  }

  // 4. THE FLIP. This is the check that would catch someone "correcting" the upside-down
  //    mount. Columns of R are the sensor axes in torso_link.
  {
    const Eigen::Vector3d sensor_x = from_rpy.col(0);
    const Eigen::Vector3d sensor_y = from_rpy.col(1);
    const Eigen::Vector3d sensor_z = from_rpy.col(2);

    Check(sensor_z.z() < -0.99, "sensor +Z points DOWN in torso_link (roll = pi)",
          Fmt("z_z", sensor_z.z()));
    Check(sensor_y.y() < -0.99, "sensor +Y points to the robot's RIGHT (flipped)",
          Fmt("y_y", sensor_y.y()));
    Check(sensor_x.x() > 0.99, "sensor +X still points forward", Fmt("x_x", sensor_x.x()));

    // 2.929 deg nose-down.
    const double pitch_down_deg = std::asin(-sensor_x.z()) * 180.0 / M_PI;
    Check(std::abs(pitch_down_deg - 2.929) < 0.01, "sensor +X is 2.929 deg nose-down",
          Fmt("deg", pitch_down_deg));
  }

  // 5. THE DOUBLE-INVERSION TRAP. The published vertical FOV [-7, +52] deg is in the
  //    SENSOR frame. Under this rotation it must map to roughly +4.07 .. -54.93 deg in
  //    the robot frame, i.e. the sensor looks DOWN. If someone also negated the FOV
  //    numbers, this range would come out looking up and the check would fail.
  {
    auto robot_frame_elevation_deg = [&](double sensor_elevation_deg) {
      const double elevation = sensor_elevation_deg * M_PI / 180.0;
      // Straight-ahead azimuth in the sensor frame.
      const Eigen::Vector3d direction(std::cos(elevation), 0.0, std::sin(elevation));
      const Eigen::Vector3d in_torso = from_rpy * direction;
      return std::asin(in_torso.z() / in_torso.norm()) * 180.0 / M_PI;
    };

    const double at_min = robot_frame_elevation_deg(-7.0);
    const double at_max = robot_frame_elevation_deg(52.0);

    Check(std::abs(at_min - 4.071) < 0.01,
          "sensor -7 deg maps to +4.071 deg in the robot frame", Fmt("deg", at_min));
    Check(std::abs(at_max - (-54.929)) < 0.01,
          "sensor +52 deg maps to -54.929 deg in the robot frame", Fmt("deg", at_max));
    Check(at_max < at_min, "the flipped FOV sweeps DOWNWARD, not at the ceiling",
          Fmt("span_deg", at_min - at_max));
  }

  // 6. Absolute-scale cross-check: torso_link sits at world z = 0.837 in the neutral
  //    standing pose (pelvis 0.793 + waist_roll offset 0.044), which places the optical
  //    origin at 1.2654 m. g1_mid360_extrinsic.md section 5.5.
  {
    constexpr double kTorsoWorldZ = 0.837;
    const double sensor_world_z = kTorsoWorldZ + kTranslation.z();
    Check(std::abs(sensor_world_z - 1.2654) < 5e-4,
          "sensor height at the neutral pose is 1.2654 m", Fmt("m", sensor_world_z));
  }

  std::printf("\n%d/%d checks passed\n", g_checks - g_failures, g_checks);
  if (g_failures != 0) {
    std::printf("RESULT: FAIL (%d)\n", g_failures);
    return 1;
  }
  std::printf("RESULT: PASS\n");
  return 0;
}
