#include "perception/adapters/mujoco/extrinsic_guard.h"

#include <iomanip>
#include <sstream>

#include <Eigen/Core>

#include "perception/core/geometry/rpy.h"

namespace perception::adapters::mujoco {

ExtrinsicCheck VerifyLidarExtrinsic(const mjModel* model, const integration::LidarConfig& config,
                                    double position_tolerance_m, double rotation_tolerance) {
  ExtrinsicCheck check;

  if (model == nullptr) {
    check.message = "VerifyLidarExtrinsic: model is null";
    return check;
  }

  const int site_id = mj_name2id(model, mjOBJ_SITE, config.extrinsic.site_name.c_str());
  if (site_id < 0) {
    check.message = "VerifyLidarExtrinsic: MJCF site '" + config.extrinsic.site_name +
                    "' was not found in the compiled model";
    return check;
  }

  const int site_body = model->site_bodyid[site_id];
  const char* body_name = mj_id2name(model, mjOBJ_BODY, site_body);
  check.parent_body_in_model = body_name != nullptr ? body_name : "<unnamed>";

  // Site pos/quat are stored in the parent body's frame, which is exactly the frame the
  // published extrinsic is expressed in.
  const Eigen::Vector3d site_position(model->site_pos[3 * site_id + 0],
                                      model->site_pos[3 * site_id + 1],
                                      model->site_pos[3 * site_id + 2]);
  const double site_quat[4] = {model->site_quat[4 * site_id + 0],
                               model->site_quat[4 * site_id + 1],
                               model->site_quat[4 * site_id + 2],
                               model->site_quat[4 * site_id + 3]};

  const Eigen::Matrix3d site_rotation = core::QuatWxyzToRotation(site_quat);
  const Eigen::Matrix3d yaml_rotation = core::RpyToRotation(config.extrinsic.rotation_rpy_rad);

  check.position_error_m = (site_position - config.extrinsic.translation_xyz_m).norm();
  check.rotation_max_abs_diff = core::RotationMaxAbsDiff(site_rotation, yaml_rotation);

  const bool parent_ok = check.parent_body_in_model == config.extrinsic.parent_body;
  const bool position_ok = check.position_error_m <= position_tolerance_m;
  const bool rotation_ok = check.rotation_max_abs_diff <= rotation_tolerance;
  check.ok = parent_ok && position_ok && rotation_ok;

  std::ostringstream out;
  out << std::fixed << std::setprecision(9);
  if (check.ok) {
    out << "Mid-360 extrinsic: site '" << config.extrinsic.site_name << "' on '"
        << check.parent_body_in_model << "' pos [" << site_position.x() << ' '
        << site_position.y() << ' ' << site_position.z() << "] agrees with perception.lidar";
  } else {
    out << "FATAL: the MJCF <site> and perception.lidar disagree about the Mid-360 extrinsic.\n";
    out << "  site name        " << config.extrinsic.site_name << '\n';
    out << "  site parent      " << check.parent_body_in_model
        << (parent_ok ? "" : "   <-- expected '" + config.extrinsic.parent_body + "'") << '\n';
    out << "  site pos         " << site_position.x() << ' ' << site_position.y() << ' '
        << site_position.z() << '\n';
    out << "  yaml pos         " << config.extrinsic.translation_xyz_m.x() << ' '
        << config.extrinsic.translation_xyz_m.y() << ' '
        << config.extrinsic.translation_xyz_m.z() << '\n';
    out << "  position error   " << check.position_error_m << " m"
        << (position_ok ? "" : "   <-- exceeds tolerance") << '\n';
    out << "  rotation |diff|  " << check.rotation_max_abs_diff
        << (rotation_ok ? "" : "   <-- exceeds tolerance") << '\n';
    out << "  Reconcile src/assets/robots/unitree_g1/xmls/scene_g1.xml against\n"
        << "  perception/configs/perception.yaml. Ground truth: g1_mid360_extrinsic.md section 6.";
  }
  check.message = out.str();
  return check;
}

}  // namespace perception::adapters::mujoco
