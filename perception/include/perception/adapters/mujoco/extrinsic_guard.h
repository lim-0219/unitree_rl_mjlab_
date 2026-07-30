// VerifyLidarExtrinsic - hard-fail cross-check between the MJCF <site> and the YAML.
//
// The extrinsic is encoded twice on purpose: the closed loop drives the sensor from the
// compiled MJCF site, while configuration, tests and any headless harness compose it from
// perception.lidar.extrinsic. Nothing structural keeps those in step, so without this
// check the repository can carry two different "official" extrinsics and the disagreement
// only shows up as quietly wrong perception.
//
// The caller is expected to terminate the process on failure. It runs at every model load.
#ifndef PERCEPTION_ADAPTERS_MUJOCO_EXTRINSIC_GUARD_H_
#define PERCEPTION_ADAPTERS_MUJOCO_EXTRINSIC_GUARD_H_

#include <string>

#include <mujoco/mujoco.h>

#include "perception/integration/perception_config.h"

namespace perception::adapters::mujoco {

struct ExtrinsicCheck {
  bool ok = false;
  double position_error_m = 0.0;
  double rotation_max_abs_diff = 0.0;  // Largest element-wise difference of the 3x3s.
  std::string parent_body_in_model;
  std::string message;  // Human-readable; multi-line on failure, with both poses printed.
};

// Compares site position, site orientation and site parent body against
// `config.extrinsic`. Tolerances are tight on purpose: both sides are meant to be the same
// transcribed constants, so any real difference is an authoring mistake, not drift.
//
// The rotation tolerance is 1e-8 rather than 1e-9 for a specific, measured reason: the two
// sides encode the same rotation DIFFERENTLY. The MJCF site carries the published
// quaternion (0, 0.999673352, 0, -0.025557564), rounded to 9 significant digits, while the
// YAML carries the rpy triple. Recomposing them differs by 4.9e-10 purely from that
// rounding. 1e-8 on a rotation-matrix element is ~6e-7 degrees - physically meaningless -
// and still catches every authoring error worth catching: the deprecated alternative
// extrinsic differs by roll = pi vs 0, i.e. by O(1).
ExtrinsicCheck VerifyLidarExtrinsic(const mjModel* model, const integration::LidarConfig& config,
                                    double position_tolerance_m = 1e-9,
                                    double rotation_tolerance = 1e-8);

}  // namespace perception::adapters::mujoco

#endif  // PERCEPTION_ADAPTERS_MUJOCO_EXTRINSIC_GUARD_H_
