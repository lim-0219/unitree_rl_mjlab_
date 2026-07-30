// V6 - continuous sensor-pose tracking under sustained robot motion.
//
// WHY THIS EXISTS. V5 checks rigid attachment at six STATIC joint poses. That cannot catch a
// staleness bug: a cached pose refreshed once per model load would pass every static check and
// still be frozen during motion. This test therefore runs CONTINUOUS trajectories with large
// height and orientation excursions and asserts pose consistency on every single scan.
//
// THREE INDEPENDENT READ PATHS, compared pairwise:
//
//   A  what the perception code CLAIMS it used:
//        RaycasterMj::last_snapshot().world_from_sensor, i.e. FrameProviderMj's read of
//        d->site_xpos / d->site_xmat.
//
//   B  independent ground truth, a completely separate query:
//        raw d->xpos / d->xmat for `torso_link`, indexed directly in this file, composed with
//        the extrinsic rebuilt from config via RpyToRotation. Shares no code with A beyond
//        MuJoCo itself.
//
//   C  what the RAYS ACTUALLY USED, recovered from the emitted cloud alone:
//        every accepted point satisfies |p_i - o| = r_i, so >= 4 points with diverse
//        directions determine the true ray origin `o` by trilateration - WITHOUT reference to
//        any pose the code reports. The rotation then follows from u_i = (p_i - o)/r_i and the
//        sensor-frame pattern directions by Kabsch. This is the path that would catch "the
//        snapshot is correct but the rays were built from something else".
//
// A == B proves the attachment maths; A == C proves the rays really used that pose; and an
// explicit anti-freeze check proves the pose is genuinely recomputed rather than cached.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <mujoco/mujoco.h>

#include "perception/adapters/mujoco/raycaster_mj.h"
#include "perception/core/geometry/rpy.h"
#include "perception/integration/perception_config.h"

namespace {

using perception::adapters::mujoco::RaycasterMj;
using perception::core::ScanStats;
using perception::core::TimedPointCloud;
using perception::integration::PerceptionConfig;

int g_failures = 0;
int g_checks = 0;

void Check(bool condition, const std::string& name, const std::string& detail = "") {
  ++g_checks;
  std::printf("    %s  %s%s%s\n", condition ? "PASS" : "FAIL", name.c_str(),
              detail.empty() ? "" : "  ", detail.c_str());
  if (!condition) {
    ++g_failures;
  }
}

std::string F(const char* label, double value, int precision = 9) {
  char buffer[192];
  std::snprintf(buffer, sizeof(buffer), "(%s = %.*g)", label, precision, value);
  return buffer;
}

Eigen::Matrix3d RowMajorToMatrix(const mjtNum* m) {
  Eigen::Matrix3d out;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out(r, c) = m[3 * r + c];
    }
  }
  return out;
}

// ---------------------------------------------------------------------------------------
// Path C: recover the pose the rays actually used, from the cloud alone.
// ---------------------------------------------------------------------------------------

struct RecoveredPose {
  bool valid = false;
  bool degenerate = false;  // Sampled returns were coplanar; `o` is not determined.
  Eigen::Vector3d origin{0.0, 0.0, 0.0};
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  int samples = 0;
  double residual = 0.0;      // Worst | |p_i - o| - r_i | over the samples used.
  double conditioning = 0.0;  // smallest/largest singular value of the trilateration matrix.
};

RecoveredPose RecoverPoseFromCloud(const TimedPointCloud& cloud, const RaycasterMj& raycaster,
                                   int max_samples = 400) {
  RecoveredPose out;
  const auto& pattern = raycaster.pattern();
  if (cloud.points.size() < 16) {
    return out;
  }

  // Spread the samples across the whole cloud so the ray directions are diverse; a
  // co-directional subset would make the trilateration ill-conditioned.
  const int stride =
      std::max<int>(1, static_cast<int>(cloud.points.size()) / std::max(1, max_samples));
  std::vector<const perception::core::RawTimedPoint*> picks;
  for (std::size_t i = 0; i < cloud.points.size(); i += stride) {
    const auto& point = cloud.points[i];
    if (point.ray_index >= 0 && point.range_m > 0.2) {
      picks.push_back(&point);
    }
  }
  if (picks.size() < 8) {
    return out;
  }

  // |p_i - o|^2 = r_i^2 ; subtracting the reference equation linearises in o:
  //   2 (p_i - p_0) . o = |p_i|^2 - |p_0|^2 - r_i^2 + r_0^2
  const Eigen::Vector3d p0 = picks[0]->position.cast<double>();
  const double r0 = picks[0]->range_m;
  Eigen::MatrixXd a(picks.size() - 1, 3);
  Eigen::VectorXd b(picks.size() - 1);
  for (std::size_t i = 1; i < picks.size(); ++i) {
    const Eigen::Vector3d pi = picks[i]->position.cast<double>();
    const double ri = picks[i]->range_m;
    a.row(i - 1) = 2.0 * (pi - p0).transpose();
    b(i - 1) = pi.squaredNorm() - p0.squaredNorm() - ri * ri + r0 * r0;
  }

  // RANK CHECK, not optional. If every sampled return lies on one plane - which is exactly
  // what happens in a bare scene where the only surface is the flat floor - then all
  // (p_i - p_0) are coplanar, A has rank 2, and the out-of-plane component of `o` is
  // mathematically unconstrained. A plain least-squares solve would silently return a
  // confident-looking wrong answer. Solve by SVD and refuse the recovery instead.
  const Eigen::JacobiSVD<Eigen::MatrixXd> solver(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd singular = solver.singularValues();
  out.conditioning = singular.size() >= 3 && singular(0) > 0.0 ? singular(2) / singular(0) : 0.0;
  if (out.conditioning < 1e-3) {
    out.degenerate = true;
    return out;  // Not recoverable from this scene's geometry; caller must not compare.
  }
  out.origin = solver.solve(b);

  // Rotation by Kabsch on (sensor-frame direction -> observed world direction).
  Eigen::Matrix3d h = Eigen::Matrix3d::Zero();
  for (const auto* point : picks) {
    const Eigen::Vector3d world = point->position.cast<double>() - out.origin;
    const double norm = world.norm();
    if (norm < 1e-9) {
      continue;
    }
    out.residual = std::max(out.residual, std::abs(norm - point->range_m));
    const Eigen::Vector3d sensor_dir = pattern.directions[point->ray_index];
    h += sensor_dir * (world / norm).transpose();
  }
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(h, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
  if ((svd.matrixV() * svd.matrixU().transpose()).determinant() < 0.0) {
    correction(2, 2) = -1.0;
  }
  out.rotation = svd.matrixV() * correction * svd.matrixU().transpose();

  out.samples = static_cast<int>(picks.size());
  out.valid = true;
  return out;
}

// ---------------------------------------------------------------------------------------

// The pose recovery in RecoverPoseFromCloud needs returns that are NOT all coplanar.
// A bare scene_g1.xml gives only the flat floor, which makes the trilateration rank-2 and
// leaves the sensor's height unconstrained. So the tracking scenes get deliberate 3-D
// structure: enclosing walls at four distances plus two cylinders and a raised block.
mjModel* CompileSceneWithStructure(const std::filesystem::path& xml, std::string& error) {
  char load_error[1024] = "";
  mjSpec* spec = mj_parseXML(xml.string().c_str(), nullptr, load_error, sizeof(load_error));
  if (spec == nullptr) {
    error = std::string("mj_parseXML failed: ") + load_error;
    return nullptr;
  }
  mjsBody* world = mjs_findBody(spec, "world");
  if (world == nullptr) {
    mj_deleteSpec(spec);
    error = "world body was not found";
    return nullptr;
  }

  auto add_box = [&](const std::string& name, double x, double y, double z, double sx, double sy,
                     double sz) {
    mjsGeom* geom = mjs_addGeom(world, nullptr);
    mjs_setName(geom->element, name.c_str());
    geom->type = mjGEOM_BOX;
    geom->pos[0] = x; geom->pos[1] = y; geom->pos[2] = z;
    geom->size[0] = sx; geom->size[1] = sy; geom->size[2] = sz;
    geom->contype = 0;
    geom->conaffinity = 0;
  };
  auto add_cylinder = [&](const std::string& name, double x, double y, double radius,
                          double half_height) {
    mjsGeom* geom = mjs_addGeom(world, nullptr);
    mjs_setName(geom->element, name.c_str());
    geom->type = mjGEOM_CYLINDER;
    geom->pos[0] = x; geom->pos[1] = y; geom->pos[2] = half_height;
    geom->size[0] = radius; geom->size[1] = half_height;
    geom->contype = 0;
    geom->conaffinity = 0;
  };

  // Four walls, deliberately at different distances so the geometry is not symmetric.
  add_box("track_wall_front", 4.0, 0.0, 1.25, 0.05, 6.0, 1.25);
  add_box("track_wall_back", -3.0, 0.0, 1.25, 0.05, 6.0, 1.25);
  add_box("track_wall_left", 0.0, 3.5, 1.25, 6.0, 0.05, 1.25);
  add_box("track_wall_right", 0.0, -2.5, 1.25, 6.0, 0.05, 1.25);
  add_cylinder("track_cylinder_a", 2.0, 1.1, 0.30, 0.75);
  add_cylinder("track_cylinder_b", -1.4, -1.0, 0.22, 0.55);
  // A raised block gives near-field returns well above the floor plane, which is what
  // conditions the vertical component of the trilateration when the sensor is low.
  add_box("track_block", 1.2, -1.6, 0.45, 0.35, 0.35, 0.45);

  mjModel* model = mj_compile(spec, nullptr);
  if (model == nullptr) {
    error = std::string("mj_compile failed: ") + mjs_getError(spec);
  }
  mj_deleteSpec(spec);
  return model;
}

struct Sample {
  double sim_time = 0.0;
  double torso_z = 0.0;
  double torso_tilt_deg = 0.0;  // Angle between the torso's +Z and world +Z.
  double sensor_z_a = 0.0;
  double delta_ab_position = 0.0;
  double delta_ab_rotation = 0.0;
  double delta_ac_position = 0.0;
  double delta_ac_rotation = 0.0;
  double cloud_residual = 0.0;
  int cloud_samples = 0;
  double cloud_conditioning = 0.0;
  bool cloud_degenerate = false;
  int accepted = 0;
  double self_fraction = 0.0;
};

struct Fixture {
  mjModel* model = nullptr;
  mjData* data = nullptr;
  RaycasterMj raycaster;
  TimedPointCloud cloud;
  ScanStats stats;
  int torso_body = -1;
  Eigen::Vector3d extrinsic_translation;
  Eigen::Matrix3d extrinsic_rotation;

  ~Fixture() {
    if (data != nullptr) mj_deleteData(data);
    if (model != nullptr) mj_deleteModel(model);
  }

  bool Build(const std::filesystem::path& xml, const PerceptionConfig& config,
             std::string& error) {
    model = CompileSceneWithStructure(xml, error);
    if (model == nullptr) {
      return false;
    }
    data = mj_makeData(model);
    mj_forward(model, data);

    torso_body = mj_name2id(model, mjOBJ_BODY, config.lidar.extrinsic.parent_body.c_str());
    if (torso_body < 0) {
      error = "parent body not found";
      return false;
    }
    extrinsic_translation = config.lidar.extrinsic.translation_xyz_m;
    extrinsic_rotation = perception::core::RpyToRotation(config.lidar.extrinsic.rotation_rpy_rad);
    return raycaster.Bind(model, data, config.lidar, error);
  }

  // Path B, deliberately hand-rolled here rather than reusing FrameProviderMj.
  void IndependentSensorPose(Eigen::Vector3d& position, Eigen::Matrix3d& rotation) const {
    const Eigen::Vector3d torso_position(data->xpos[3 * torso_body + 0],
                                         data->xpos[3 * torso_body + 1],
                                         data->xpos[3 * torso_body + 2]);
    const Eigen::Matrix3d torso_rotation = RowMajorToMatrix(&data->xmat[9 * torso_body]);
    position = torso_position + torso_rotation * extrinsic_translation;
    rotation = torso_rotation * extrinsic_rotation;
  }

  Sample ScanAndCompare() {
    Sample sample;
    raycaster.Scan(cloud, stats);

    // A - what the code says it used.
    const auto& snapshot = raycaster.last_snapshot();
    const Eigen::Vector3d position_a = snapshot.world_from_sensor.translation();
    const Eigen::Matrix3d rotation_a = snapshot.world_from_sensor.linear();

    // B - independent ground truth.
    Eigen::Vector3d position_b;
    Eigen::Matrix3d rotation_b;
    IndependentSensorPose(position_b, rotation_b);

    // C - what the rays actually used.
    const RecoveredPose recovered = RecoverPoseFromCloud(cloud, raycaster);

    const Eigen::Matrix3d torso_rotation = RowMajorToMatrix(&data->xmat[9 * torso_body]);

    sample.sim_time = data->time;
    sample.torso_z = data->xpos[3 * torso_body + 2];
    sample.torso_tilt_deg =
        std::acos(std::clamp(torso_rotation(2, 2), -1.0, 1.0)) * 180.0 / M_PI;
    sample.sensor_z_a = position_a.z();
    sample.delta_ab_position = (position_a - position_b).norm();
    sample.delta_ab_rotation = perception::core::RotationMaxAbsDiff(rotation_a, rotation_b);
    sample.cloud_conditioning = recovered.conditioning;
    sample.cloud_degenerate = recovered.degenerate;
    if (recovered.valid) {
      sample.delta_ac_position = (position_a - recovered.origin).norm();
      sample.delta_ac_rotation =
          perception::core::RotationMaxAbsDiff(rotation_a, recovered.rotation);
      sample.cloud_residual = recovered.residual;
      sample.cloud_samples = recovered.samples;
    } else {
      sample.delta_ac_position = std::numeric_limits<double>::quiet_NaN();
      sample.delta_ac_rotation = std::numeric_limits<double>::quiet_NaN();
    }
    sample.accepted = stats.accepted;
    sample.self_fraction = stats.SelfHitFraction();
    return sample;
  }
};

void SetFreeJointPose(const mjModel* model, mjData* data, const Eigen::Vector3d& position,
                      const Eigen::Quaterniond& orientation) {
  const int joint = mj_name2id(model, mjOBJ_JOINT, "floating_base_joint");
  const int adr = model->jnt_qposadr[joint];
  data->qpos[adr + 0] = position.x();
  data->qpos[adr + 1] = position.y();
  data->qpos[adr + 2] = position.z();
  data->qpos[adr + 3] = orientation.w();
  data->qpos[adr + 4] = orientation.x();
  data->qpos[adr + 5] = orientation.y();
  data->qpos[adr + 6] = orientation.z();
}

void Summarise(const std::vector<Sample>& samples, const std::string& regime) {
  if (samples.empty()) {
    Check(false, regime + ": produced samples");
    return;
  }

  double worst_ab_position = 0.0;
  double worst_ab_rotation = 0.0;
  double worst_ac_position = 0.0;
  double worst_ac_rotation = 0.0;
  double worst_cloud_residual = 0.0;
  double min_sensor_z = std::numeric_limits<double>::max();
  double max_sensor_z = -std::numeric_limits<double>::max();
  double max_tilt = 0.0;
  int cloud_valid = 0;
  int cloud_degenerate = 0;
  double worst_conditioning = std::numeric_limits<double>::max();
  int frozen_pairs = 0;
  int moving_pairs = 0;

  for (std::size_t i = 0; i < samples.size(); ++i) {
    const auto& s = samples[i];
    worst_ab_position = std::max(worst_ab_position, s.delta_ab_position);
    worst_ab_rotation = std::max(worst_ab_rotation, s.delta_ab_rotation);
    if (s.cloud_degenerate) {
      ++cloud_degenerate;
    }
    worst_conditioning = std::min(worst_conditioning, s.cloud_conditioning);
    if (!std::isnan(s.delta_ac_position)) {
      ++cloud_valid;
      worst_ac_position = std::max(worst_ac_position, s.delta_ac_position);
      worst_ac_rotation = std::max(worst_ac_rotation, s.delta_ac_rotation);
      worst_cloud_residual = std::max(worst_cloud_residual, s.cloud_residual);
    }
    min_sensor_z = std::min(min_sensor_z, s.sensor_z_a);
    max_sensor_z = std::max(max_sensor_z, s.sensor_z_a);
    max_tilt = std::max(max_tilt, s.torso_tilt_deg);

    // ANTI-FREEZE: whenever the torso actually moved between consecutive scans, the
    // reported sensor pose must have moved too. A cached pose fails here and nowhere else.
    if (i > 0) {
      const double torso_motion = std::abs(s.torso_z - samples[i - 1].torso_z) +
                                  std::abs(s.torso_tilt_deg - samples[i - 1].torso_tilt_deg) / 90.0;
      if (torso_motion > 1e-3) {
        ++moving_pairs;
        const double sensor_motion = std::abs(s.sensor_z_a - samples[i - 1].sensor_z_a);
        if (sensor_motion < 1e-9) {
          ++frozen_pairs;
        }
      }
    }
  }

  std::printf("      %zu scans | sensor world z range [%.4f, %.4f] (excursion %.4f m) | "
              "max torso tilt %.1f deg\n",
              samples.size(), min_sensor_z, max_sensor_z, max_sensor_z - min_sensor_z, max_tilt);
  std::printf("      worst |A-B| position %.3e m, rotation %.3e   (A = perception snapshot, "
              "B = independent torso read x extrinsic)\n",
              worst_ab_position, worst_ab_rotation);
  std::printf("      worst |A-C| position %.3e m, rotation %.3e   (C = pose recovered from the "
              "emitted cloud, %d/%zu scans)\n",
              worst_ac_position, worst_ac_rotation, cloud_valid, samples.size());
  std::printf("      worst cloud self-consistency residual | |p-o| - r | = %.3e m | "
              "worst trilateration conditioning %.3e | degenerate scans %d\n",
              worst_cloud_residual, worst_conditioning, cloud_degenerate);

  // A == B: both ultimately read the same mjData kinematics, so this must be near-exact.
  // The floor is the 9-significant-digit rounding of the published MJCF quaternion.
  Check(worst_ab_position < 1e-9, regime + ": sensor pose == independent torso pose x extrinsic",
        F("worst |dt|", worst_ab_position, 3));
  Check(worst_ab_rotation < 1e-8, regime + ": sensor rotation matches independently, every scan",
        F("worst |dR|", worst_ab_rotation, 3));

  // A == C: limited by the float storage of `position` (~1e-6 m at these ranges), so the
  // tolerance is 1e-3 m - still ~1000x tighter than any staleness would produce.
  Check(cloud_valid == static_cast<int>(samples.size()),
        regime + ": pose recoverable from the cloud on every scan",
        F("scans", cloud_valid, 3));
  Check(worst_ac_position < 1e-3,
        regime + ": the RAYS were built from that same pose (trilaterated origin)",
        F("worst |dt|", worst_ac_position, 3));
  Check(worst_ac_rotation < 1e-3,
        regime + ": the RAYS were built from that same rotation (Kabsch)",
        F("worst |dR|", worst_ac_rotation, 3));

  Check(frozen_pairs == 0,
        regime + ": sensor pose is never frozen while the torso is moving (anti-cache)",
        F("frozen of moving pairs", frozen_pairs, 3) + F(" moving pairs", moving_pairs, 3));
  Check(moving_pairs > 10, regime + ": the trajectory really did move the robot",
        F("moving pairs", moving_pairs, 3));
}

// ---------------------------------------------------------------------------------------
// Regime 1 - prescribed kinematic sweep. Large, continuous, ground-truth-controlled motion.
// This is stronger evidence for a pose-tracking test than a policy rollout would be: the
// excursion is guaranteed large and the trajectory is exactly known.
// ---------------------------------------------------------------------------------------
void RunPrescribedSweep(const std::filesystem::path& xml, const PerceptionConfig& config) {
  std::printf("\nV6a prescribed kinematic sweep (pelvis height 1.05 -> 0.12 m, roll/pitch to 85 deg)\n");
  Fixture fixture;
  std::string error;
  if (!fixture.Build(xml, config, error)) {
    Check(false, "build fixture", error);
    return;
  }

  std::vector<Sample> samples;
  constexpr int kSteps = 60;
  for (int i = 0; i < kSteps; ++i) {
    const double u = static_cast<double>(i) / (kSteps - 1);
    const double height = 1.05 - 0.93 * u;
    const double roll = 85.0 * u * M_PI / 180.0;
    const double pitch = 40.0 * std::sin(2.0 * M_PI * u) * M_PI / 180.0;
    const double yaw = 120.0 * u * M_PI / 180.0;

    const Eigen::Quaterniond orientation(
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
    SetFreeJointPose(fixture.model, fixture.data, {0.4 * u, -0.3 * u, height}, orientation);

    // Move the waist too, so torso_link differs from pelvis in a pose-dependent way.
    const int waist = mj_name2id(fixture.model, mjOBJ_JOINT, "waist_pitch_joint");
    if (waist >= 0) {
      fixture.data->qpos[fixture.model->jnt_qposadr[waist]] = 0.45 * std::sin(3.0 * M_PI * u);
    }

    fixture.data->time = 0.1 * i;
    mj_forward(fixture.model, fixture.data);
    samples.push_back(fixture.ScanAndCompare());
  }

  std::printf("      %-8s %-9s %-9s %-10s %-11s %-11s %-9s\n", "t[s]", "torso_z", "tilt[deg]",
              "sensor_z", "|A-B|pos", "|A-C|pos", "accepted");
  for (std::size_t i = 0; i < samples.size(); i += 6) {
    const auto& s = samples[i];
    std::printf("      %-8.2f %-9.4f %-9.1f %-10.4f %-11.2e %-11.2e %-9d\n", s.sim_time,
                s.torso_z, s.torso_tilt_deg, s.sensor_z_a, s.delta_ab_position,
                s.delta_ac_position, s.accepted);
  }

  Summarise(samples, "V6a prescribed sweep");
}

// ---------------------------------------------------------------------------------------
// Regime 2 - real dynamics. No controller, so the G1 collapses: exactly the scenario the
// person was watching when the live view "looked similar".
// ---------------------------------------------------------------------------------------
void RunDynamicFall(const std::filesystem::path& xml, const PerceptionConfig& config) {
  std::printf("\nV6b dynamic fall under gravity (real mj_step, no controller)\n");
  Fixture fixture;
  std::string error;
  if (!fixture.Build(xml, config, error)) {
    Check(false, "build fixture", error);
    return;
  }

  std::vector<Sample> samples;
  const double frame_period = 1.0 / config.lidar.sensor.scan_rate_hz;
  constexpr int kFrames = 60;  // 6 s of sim time at 10 Hz scans.
  for (int frame = 0; frame < kFrames; ++frame) {
    const double target = (frame + 1) * frame_period;
    while (fixture.data->time < target) {
      mj_step(fixture.model, fixture.data);
    }
    // Kinematics after mj_step correspond to the pre-integration state; refresh so the
    // scan geometry matches d->time. See the report's note on the 1-timestep offset.
    mj_forward(fixture.model, fixture.data);
    samples.push_back(fixture.ScanAndCompare());
  }

  std::printf("      %-8s %-9s %-9s %-10s %-11s %-11s %-9s %-8s\n", "t[s]", "torso_z",
              "tilt[deg]", "sensor_z", "|A-B|pos", "|A-C|pos", "accepted", "self");
  for (std::size_t i = 0; i < samples.size(); i += 6) {
    const auto& s = samples[i];
    std::printf("      %-8.2f %-9.4f %-9.1f %-10.4f %-11.2e %-11.2e %-9d %-8.3f\n", s.sim_time,
                s.torso_z, s.torso_tilt_deg, s.sensor_z_a, s.delta_ab_position,
                s.delta_ac_position, s.accepted, s.self_fraction);
  }

  Summarise(samples, "V6b dynamic fall");

  // The fall must be a real, large excursion - otherwise this test proves nothing.
  const double first = samples.front().sensor_z_a;
  const double last = samples.back().sensor_z_a;
  Check(first - last > 0.8,
        "V6b dynamic fall: the robot actually collapsed (sensor dropped > 0.8 m)",
        F("drop m", first - last, 4));
}

// ---------------------------------------------------------------------------------------
// The one-timestep question: after mj_step, d->xpos reflects the PRE-integration state
// while d->time has already advanced. Quantify it rather than assume it is negligible.
// ---------------------------------------------------------------------------------------
void RunStepPhaseCheck(const std::filesystem::path& xml, const PerceptionConfig& config) {
  std::printf("\nV6c geometry-vs-timestamp phase after mj_step (quantifying, not assuming)\n");
  Fixture fixture;
  std::string error;
  if (!fixture.Build(xml, config, error)) {
    Check(false, "build fixture", error);
    return;
  }

  // Let the robot get up to speed falling, then compare the site pose straight after
  // mj_step against the same pose after a refreshing mj_forward.
  for (int i = 0; i < 250; ++i) {
    mj_step(fixture.model, fixture.data);
  }

  double worst_offset = 0.0;
  double worst_speed = 0.0;
  for (int i = 0; i < 50; ++i) {
    mj_step(fixture.model, fixture.data);
    const int site = fixture.raycaster.frame_provider().site_id();
    const Eigen::Vector3d after_step(fixture.data->site_xpos[3 * site + 0],
                                     fixture.data->site_xpos[3 * site + 1],
                                     fixture.data->site_xpos[3 * site + 2]);
    mj_forward(fixture.model, fixture.data);
    const Eigen::Vector3d after_forward(fixture.data->site_xpos[3 * site + 0],
                                        fixture.data->site_xpos[3 * site + 1],
                                        fixture.data->site_xpos[3 * site + 2]);
    const double offset = (after_forward - after_step).norm();
    worst_offset = std::max(worst_offset, offset);
    worst_speed = std::max(worst_speed, offset / fixture.model->opt.timestep);
  }

  std::printf("      timestep %.4f s | worst site displacement between post-step and "
              "post-forward kinematics: %.6f m (implies %.3f m/s)\n",
              fixture.model->opt.timestep, worst_offset, worst_speed);
  std::printf("      => a real ONE-TIMESTEP phase offset, NOT a staleness bug: after mj_step,\n"
              "         d->xpos/site_xpos still describe the pre-integration state while d->time\n"
              "         has already advanced. In the simulator's physics loop mj_forward runs only\n"
              "         in the PAUSED branch, so while running the scan geometry lags its own\n"
              "         timestamp by one timestep. Bounded by sensor speed x timestep (%.1f mm at\n"
              "         the %.2f m/s measured here) and ~50x smaller than the 100 ms scan window\n"
              "         deskew will have to handle anyway. The correct fix is to record the true\n"
              "         acquisition time rather than to force a kinematics refresh from the\n"
              "         perception path, so this is deferred to the deskew phase. Recorded as an\n"
              "         open item; it is NOT silently assumed to be zero.\n",
              1000.0 * worst_offset, worst_speed);
  Check(worst_offset < 0.05, "the post-step phase offset is bounded and small",
        F("m", worst_offset, 4));
}

}  // namespace

int main(int argc, char** argv) {
  const std::filesystem::path repository_root = argc > 1 ? argv[1] : ".";
  const std::filesystem::path xml =
      repository_root / "src/assets/robots/unitree_g1/xmls/scene_g1.xml";

  std::printf("Mid-360 continuous pose-tracking suite (V6)\n");
  std::printf("scene: %s\n", xml.string().c_str());
  if (!std::filesystem::exists(xml)) {
    std::printf("FATAL: scene not found. Pass the repository root as argv[1].\n");
    return 2;
  }

  PerceptionConfig config;
  config.visualization.enabled = false;
  config.Validate();

  RunPrescribedSweep(xml, config);
  RunDynamicFall(xml, config);
  RunStepPhaseCheck(xml, config);

  std::printf("\n=====================================================\n");
  std::printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
  if (g_failures != 0) {
    std::printf("RESULT: FAIL (%d failing checks)\n", g_failures);
    return 1;
  }
  std::printf("RESULT: PASS\n");
  return 0;
}
