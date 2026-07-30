// Mid-360 ray bring-up validation suite.
//
// Usage:  perception_mid360_bringup_test <repository-root>
//
// Scenarios, each independently reported:
//   V0  the aperture geom-group change is provably dynamics-neutral
//   V1  empty world: no accepted point terminates on the head shell; self-hit rate is low;
//       before/after evidence that the aperture is what makes the sensor see anything
//   V2a front wall: range accuracy against an INDEPENDENT analytic plane intersection
//   V2b the same scenes on the batched path, showing why raycast.exact defaults to true
//   V3  a single known cylinder: radial residual, angular extent, independent circle fit
//   V4  self-occlusion: a raised arm must occlude, the sensor's own housing must not
//   V5  rigid attachment: base<-sensor equals the authored extrinsic in every robot pose
//   T1  whether a wall test could settle "is mid360_link the optical origin or the
//       mounting base" (it cannot - this reports the measurement that can)
//
// Hit points are reconstructed in double from (origin, direction, range) rather than read
// out of the float `position` field, so the accuracy numbers are not limited by storage.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "perception/adapters/mujoco/frame_provider_mj.h"
#include "perception/adapters/mujoco/raycaster_mj.h"
#include "perception/core/geometry/rpy.h"
#include "perception/integration/perception_config.h"

namespace {

using perception::adapters::mujoco::FrameProviderMj;
using perception::adapters::mujoco::RaycasterMj;
using perception::core::RawTimedPoint;
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
  std::snprintf(buffer, sizeof(buffer), "(%s = %.*f)", label, precision, value);
  return buffer;
}

std::string I(const char* label, long value) {
  char buffer[192];
  std::snprintf(buffer, sizeof(buffer), "(%s = %ld)", label, value);
  return buffer;
}

// ---------------------------------------------------------------------------------------
// Scene construction. Loads the production scene_g1.xml through mjSpec and optionally
// bolts extra static geometry onto the world, mirroring how DynamicObstacleManager adds
// its obstacles. Note that no dpcbf obstacles are added, so these scenes are clean.
// ---------------------------------------------------------------------------------------

struct SceneOptions {
  bool add_wall = false;
  double wall_x_center = 3.0;   // Box centre; the near face is wall_x_center - half_thickness.
  double wall_half_thickness = 0.02;
  double wall_width = 8.0;      // Full extent in y.
  double wall_height = 2.0;     // Full extent in z, resting on the floor.

  bool add_cylinder = false;
  double cylinder_x = 2.5;
  double cylinder_y = 0.0;
  double cylinder_radius = 0.30;
  double cylinder_height = 1.5;

  // V0 only: restore the pre-aperture geom groups so the two compiled models can be
  // compared. Pre-edit values were visual=1 and collision=<unset>, i.e. 0.
  bool restore_pre_aperture_head_groups = false;
};

mjModel* CompileScene(const std::filesystem::path& xml, const SceneOptions& options,
                      std::string& error) {
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

  if (options.restore_pre_aperture_head_groups) {
    const std::pair<const char*, int> pre_aperture[] = {{"head_link_visual", 1},
                                                        {"head_link_collision", 0}};
    for (const auto& [name, group] : pre_aperture) {
      mjsGeom* geom = mjs_asGeom(mjs_findElement(spec, mjOBJ_GEOM, name));
      if (geom == nullptr) {
        mj_deleteSpec(spec);
        error = std::string("geom '") + name + "' was not found";
        return nullptr;
      }
      geom->group = group;
    }
  }

  if (options.add_wall) {
    mjsGeom* wall = mjs_addGeom(world, nullptr);
    mjs_setName(wall->element, "validation_wall");
    wall->type = mjGEOM_BOX;
    wall->pos[0] = options.wall_x_center;
    wall->pos[1] = 0.0;
    wall->pos[2] = 0.5 * options.wall_height;
    wall->size[0] = options.wall_half_thickness;
    wall->size[1] = 0.5 * options.wall_width;
    wall->size[2] = 0.5 * options.wall_height;
    wall->contype = 0;
    wall->conaffinity = 0;
    wall->rgba[0] = 0.8f; wall->rgba[1] = 0.8f; wall->rgba[2] = 0.8f; wall->rgba[3] = 1.0f;
  }

  if (options.add_cylinder) {
    mjsGeom* cylinder = mjs_addGeom(world, nullptr);
    mjs_setName(cylinder->element, "validation_cylinder");
    cylinder->type = mjGEOM_CYLINDER;
    cylinder->pos[0] = options.cylinder_x;
    cylinder->pos[1] = options.cylinder_y;
    cylinder->pos[2] = 0.5 * options.cylinder_height;
    cylinder->size[0] = options.cylinder_radius;
    cylinder->size[1] = 0.5 * options.cylinder_height;
    cylinder->contype = 0;
    cylinder->conaffinity = 0;
    cylinder->rgba[0] = 0.2f; cylinder->rgba[1] = 0.6f; cylinder->rgba[2] = 1.0f;
    cylinder->rgba[3] = 1.0f;
  }

  mjModel* model = mj_compile(spec, nullptr);
  if (model == nullptr) {
    error = std::string("mj_compile failed: ") + mjs_getError(spec);
  }
  mj_deleteSpec(spec);
  return model;
}

// Joint helper: set a hinge/slide joint's position by name.
bool SetJoint(const mjModel* model, mjData* data, const char* name, double value) {
  const int joint = mj_name2id(model, mjOBJ_JOINT, name);
  if (joint < 0) {
    return false;
  }
  data->qpos[model->jnt_qposadr[joint]] = value;
  return true;
}

// The world-frame direction of ray `index`, in double.
Eigen::Vector3d WorldDirection(const RaycasterMj& raycaster, int index) {
  return raycaster.last_snapshot().world_from_sensor.linear() *
         raycaster.pattern().directions[static_cast<std::size_t>(index)];
}

// Analytic distance from `origin` along `direction` to the plane x = plane_x, valid only
// if the intersection lands inside the rectangular face. Returns -1 when there is no
// valid intersection. Independent of MuJoCo.
double AnalyticWallDistance(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction,
                            double plane_x, double half_width, double z_lo, double z_hi) {
  if (std::abs(direction.x()) < 1e-12) {
    return -1.0;
  }
  const double t = (plane_x - origin.x()) / direction.x();
  if (t <= 0.0) {
    return -1.0;
  }
  const double y = origin.y() + t * direction.y();
  const double z = origin.z() + t * direction.z();
  if (std::abs(y) > half_width || z < z_lo || z > z_hi) {
    return -1.0;
  }
  return t;
}

PerceptionConfig MakeConfig() {
  PerceptionConfig config;  // Defaults are the documented ground-truth values.
  config.visualization.enabled = false;  // Headless.
  config.Validate();
  return config;
}

struct Fixture {
  mjModel* model = nullptr;
  mjData* data = nullptr;
  RaycasterMj raycaster;
  TimedPointCloud cloud;
  ScanStats stats;

  ~Fixture() {
    if (data != nullptr) mj_deleteData(data);
    if (model != nullptr) mj_deleteModel(model);
  }

  bool Build(const std::filesystem::path& xml, const SceneOptions& options,
             const PerceptionConfig& config, std::string& error) {
    model = CompileScene(xml, options, error);
    if (model == nullptr) return false;
    data = mj_makeData(model);
    if (data == nullptr) {
      error = "mj_makeData failed";
      return false;
    }
    mj_forward(model, data);
    return raycaster.Bind(model, data, config.lidar, error);
  }

  void Refresh() { mj_forward(model, data); }

  bool Scan() { return raycaster.Scan(cloud, stats); }
};

// ---------------------------------------------------------------------------------------

void RunV0(const std::filesystem::path& xml) {
  std::printf("\nV0  aperture geom-group change is dynamics-neutral\n");
  std::string error;

  SceneOptions shipped;
  SceneOptions pre_aperture;
  pre_aperture.restore_pre_aperture_head_groups = true;

  mjModel* after = CompileScene(xml, shipped, error);
  if (after == nullptr) {
    Check(false, "compile shipped scene_g1.xml", error);
    return;
  }
  mjModel* before = CompileScene(xml, pre_aperture, error);
  if (before == nullptr) {
    Check(false, "compile pre-aperture variant", error);
    mj_deleteModel(after);
    return;
  }

  Check(after->nbody == before->nbody && after->ngeom == before->ngeom &&
            after->nq == before->nq && after->nv == before->nv,
        "model dimensions unchanged",
        I("nbody", after->nbody) + I(" ngeom", after->ngeom));

  double max_mass_diff = 0.0;
  for (int i = 0; i < after->nbody; ++i) {
    max_mass_diff = std::max(max_mass_diff, std::abs(after->body_mass[i] - before->body_mass[i]));
  }
  Check(max_mass_diff == 0.0, "every body mass is bit-identical", F("max|diff|", max_mass_diff));

  double max_inertia_diff = 0.0;
  for (int i = 0; i < 3 * after->nbody; ++i) {
    max_inertia_diff =
        std::max(max_inertia_diff, std::abs(after->body_inertia[i] - before->body_inertia[i]));
  }
  Check(max_inertia_diff == 0.0, "every body inertia is bit-identical",
        F("max|diff|", max_inertia_diff));

  int contact_diffs = 0;
  for (int i = 0; i < after->ngeom; ++i) {
    if (after->geom_contype[i] != before->geom_contype[i] ||
        after->geom_conaffinity[i] != before->geom_conaffinity[i]) {
      ++contact_diffs;
    }
  }
  Check(contact_diffs == 0, "contype/conaffinity unchanged for every geom",
        I("differing geoms", contact_diffs));

  // And confirm the change actually landed: the head geoms really are in group 2 now.
  const int visual = mj_name2id(after, mjOBJ_GEOM, "head_link_visual");
  const int collision = mj_name2id(after, mjOBJ_GEOM, "head_link_collision");
  Check(visual >= 0 && collision >= 0 && after->geom_group[visual] == 2 &&
            after->geom_group[collision] == 2,
        "both head_link geoms are in visibility group 2");

  mj_deleteModel(before);
  mj_deleteModel(after);
}

void RunV1(const std::filesystem::path& xml) {
  std::printf("\nV1  empty world: aperture works, head does not self-occlude\n");
  const PerceptionConfig config = MakeConfig();
  Fixture fixture;
  std::string error;
  if (!fixture.Build(xml, SceneOptions{}, config, error)) {
    Check(false, "build fixture", error);
    return;
  }

  fixture.raycaster.set_collect_aperture_diagnostic(true);
  if (!fixture.Scan()) {
    Check(false, "scan");
    return;
  }

  const auto origin = fixture.raycaster.last_snapshot().world_from_sensor.translation();
  std::printf("      origin_world = (%.4f, %.4f, %.4f)   rays=%d raw_hits=%d self=%d accepted=%d\n",
              origin.x(), origin.y(), origin.z(), fixture.stats.rays_cast,
              fixture.stats.raw_hits, fixture.stats.self_rejected, fixture.stats.accepted);

  Check(std::abs(origin.z() - 1.2654) < 2e-3, "sensor sits at world z = 1.2654 m",
        F("z", origin.z(), 4));
  Check(fixture.stats.IsBalanced(), "every ray is accounted for exactly once");

  const int head_visual = mj_name2id(fixture.model, mjOBJ_GEOM, "head_link_visual");
  const int head_collision = mj_name2id(fixture.model, mjOBJ_GEOM, "head_link_collision");
  int on_head = 0;
  for (const auto& point : fixture.cloud.points) {
    if (point.geom_id == head_visual || point.geom_id == head_collision) {
      ++on_head;
    }
  }
  Check(on_head == 0, "zero accepted points terminate on the head shell", I("points", on_head));

  Check(fixture.stats.SelfHitFraction() < 0.15, "self-hit fraction is low with the aperture on",
        F("fraction", fixture.stats.SelfHitFraction(), 4));
  Check(fixture.stats.accepted > 5000, "the sensor actually sees the world",
        I("accepted", fixture.stats.accepted));
  Check(fixture.stats.aperture_diagnostic_valid && fixture.stats.aperture_transmitted > 10000,
        "the shell would otherwise have swallowed nearly every ray",
        I("aperture_transmitted", fixture.stats.aperture_transmitted));

  // BEFORE/AFTER: the same scan with the mask disabled reproduces the pre-aperture state.
  const double self_fraction_after = fixture.stats.SelfHitFraction();
  const int accepted_after = fixture.stats.accepted;
  fixture.raycaster.set_collect_aperture_diagnostic(false);
  fixture.raycaster.set_aperture_enabled(false);
  if (fixture.Scan()) {
    std::printf("      aperture OFF: self=%d/%d = %.4f  accepted=%d\n",
                fixture.stats.self_rejected, fixture.stats.rays_cast,
                fixture.stats.SelfHitFraction(), fixture.stats.accepted);
    Check(fixture.stats.SelfHitFraction() > 0.99,
          "aperture OFF reproduces ~100% self hits (the bug this fixes)",
          F("fraction", fixture.stats.SelfHitFraction(), 4));
    Check(fixture.stats.accepted == 0, "aperture OFF delivers ZERO points to the consumer",
          I("accepted", fixture.stats.accepted));
    std::printf("      => aperture ON/OFF: self-hit %.4f -> %.4f, accepted %d -> %d\n",
                fixture.stats.SelfHitFraction(), self_fraction_after,
                fixture.stats.accepted, accepted_after);
  }
  fixture.raycaster.set_aperture_enabled(true);
}

// Shared wall analysis for V2a / V2b.
struct WallResult {
  int wall_points = 0;
  double worst_range_error = 0.0;
  int rays_through_wall = 0;
  double worst_through_gap = 0.0;
  double scan_us = 0.0;
};

WallResult AnalyseWall(Fixture& fixture, const SceneOptions& options) {
  WallResult result;
  const int wall_geom = mj_name2id(fixture.model, mjOBJ_GEOM, "validation_wall");
  const auto& snapshot = fixture.raycaster.last_snapshot();
  const Eigen::Vector3d origin = snapshot.world_from_sensor.translation();
  const double plane_x = options.wall_x_center - options.wall_half_thickness;
  const double half_width = 0.5 * options.wall_width;

  // Accuracy: for every accepted point that landed on the wall, compare the measured
  // range against the analytic plane intersection for that ray.
  std::vector<char> resolved(fixture.raycaster.pattern().size(), 0);
  for (const auto& point : fixture.cloud.points) {
    if (point.ray_index >= 0) {
      resolved[static_cast<std::size_t>(point.ray_index)] = 1;
    }
    if (point.geom_id != wall_geom) {
      continue;
    }
    ++result.wall_points;
    const Eigen::Vector3d direction = WorldDirection(fixture.raycaster, point.ray_index);
    const double analytic =
        AnalyticWallDistance(origin, direction, plane_x, half_width, 0.0, options.wall_height);
    if (analytic > 0.0) {
      result.worst_range_error =
          std::max(result.worst_range_error, std::abs(point.range_m - analytic));
    }
  }

  // Missed occlusion: a ray whose analytic wall intersection is valid, but which either
  // reported nothing or reported something FARTHER than the wall. Rays that resolved
  // nearer than the wall are legitimately blocked by an earlier surface.
  const auto& pattern = fixture.raycaster.pattern();
  std::vector<double> accepted_range(pattern.size(), -1.0);
  for (const auto& point : fixture.cloud.points) {
    if (point.ray_index >= 0) {
      accepted_range[static_cast<std::size_t>(point.ray_index)] = point.range_m;
    }
  }

  for (std::size_t i = 0; i < pattern.size(); ++i) {
    const Eigen::Vector3d direction = WorldDirection(fixture.raycaster, static_cast<int>(i));
    const double analytic =
        AnalyticWallDistance(origin, direction, plane_x, half_width, 0.0, options.wall_height);
    if (analytic <= 0.0) {
      continue;
    }
    // Re-cast this single ray exactly, with the same masks, to learn what the caster saw
    // including hits that the self filter later dropped.
    mjtByte geomgroup[mjNGROUP];
    for (int g = 0; g < mjNGROUP; ++g) geomgroup[g] = 1;
    geomgroup[2] = 0;
    const mjtNum pnt[3] = {origin.x(), origin.y(), origin.z()};
    const mjtNum vec[3] = {direction.x(), direction.y(), direction.z()};
    int geom_id = -1;
    const mjtNum distance =
        mj_ray(fixture.model, fixture.data, pnt, vec, geomgroup, 1, -1, &geom_id);
    const double reference = distance;  // Exact ground truth for "was anything nearer?".

    const double reported = accepted_range[i];
    const bool nearer_occluder = reference > 0.0 && reference < analytic - 1e-9;
    if (nearer_occluder) {
      continue;  // Legitimately blocked before the wall.
    }
    if (reported < 0.0 || reported > analytic + 1e-6) {
      ++result.rays_through_wall;
      if (reported > 0.0) {
        result.worst_through_gap = std::max(result.worst_through_gap, reported - analytic);
      }
    }
  }

  result.scan_us = fixture.stats.scan_wall_time_us;
  return result;
}

void RunV2(const std::filesystem::path& xml) {
  std::printf("\nV2a front wall: range accuracy vs an independent analytic plane\n");
  const PerceptionConfig config = MakeConfig();

  const double widths[] = {2.0, 8.0};
  WallResult exact_results[2];

  for (int w = 0; w < 2; ++w) {
    SceneOptions options;
    options.add_wall = true;
    options.wall_width = widths[w];

    Fixture fixture;
    std::string error;
    if (!fixture.Build(xml, options, config, error)) {
      Check(false, "build wall fixture", error);
      return;
    }
    if (!fixture.Scan()) {
      Check(false, "scan");
      return;
    }
    exact_results[w] = AnalyseWall(fixture, options);
    const auto& r = exact_results[w];
    char label[96];
    std::snprintf(label, sizeof(label), "%.0f m wide wall", widths[w]);
    std::printf("      %s: %d wall points, worst |measured - analytic| = %.9f m, "
                "rays through = %d, scan = %.0f us\n",
                label, r.wall_points, r.worst_range_error, r.rays_through_wall, r.scan_us);

    Check(r.wall_points > 100, std::string(label) + ": wall is seen",
          I("points", r.wall_points));
    Check(r.worst_range_error < 1e-6, std::string(label) + ": range matches analytic to 1e-6 m",
          F("worst", r.worst_range_error));
    Check(r.rays_through_wall == 0, std::string(label) + ": no ray passes through the wall",
          I("through", r.rays_through_wall));
  }

  std::printf("\nV2b the same scenes on the BATCHED path (why raycast.exact defaults true)\n");
  for (int w = 0; w < 2; ++w) {
    SceneOptions options;
    options.add_wall = true;
    options.wall_width = widths[w];

    PerceptionConfig batched = config;
    batched.lidar.raycast.exact = false;

    Fixture fixture;
    std::string error;
    if (!fixture.Build(xml, options, batched, error)) {
      Check(false, "build batched fixture", error);
      return;
    }
    if (!fixture.Scan()) {
      Check(false, "scan");
      return;
    }
    const WallResult r = AnalyseWall(fixture, options);
    const double through_fraction =
        exact_results[w].wall_points > 0
            ? static_cast<double>(r.rays_through_wall) /
                  (exact_results[w].wall_points + r.rays_through_wall)
            : 0.0;
    std::printf("      %.0f m wide wall, batched: %d wall points, worst range error = %.9f m, "
                "rays through = %d (%.1f%% of wall rays), worst gap = %.3f m, scan = %.0f us\n",
                widths[w], r.wall_points, r.worst_range_error, r.rays_through_wall,
                100.0 * through_fraction, r.worst_through_gap, r.scan_us);

    // The claim being asserted is not "batched is bad by exactly N%" but the shape of the
    // defect: ranges that DO resolve stay exact, and the loss is missed occlusion only.
    Check(r.worst_range_error < 1e-6,
          "batched: ranges that do resolve are still exact (the error is missed occlusion)",
          F("worst", r.worst_range_error));
  }
  std::printf("      => the exact path resolves every wall ray; the batched path is offered\n"
              "         only as a documented performance fallback.\n");
}

void RunV3(const std::filesystem::path& xml) {
  std::printf("\nV3  single known cylinder (r = 0.30 m at (2.5, 0))\n");
  const PerceptionConfig config = MakeConfig();

  SceneOptions options;
  options.add_cylinder = true;

  Fixture fixture;
  std::string error;
  if (!fixture.Build(xml, options, config, error)) {
    Check(false, "build cylinder fixture", error);
    return;
  }
  if (!fixture.Scan()) {
    Check(false, "scan");
    return;
  }

  const int cylinder_geom = mj_name2id(fixture.model, mjOBJ_GEOM, "validation_cylinder");
  const Eigen::Vector3d origin = fixture.raycaster.last_snapshot().world_from_sensor.translation();

  std::vector<Eigen::Vector2d> hits;
  double worst_radial_residual = 0.0;
  double min_bearing = std::numeric_limits<double>::max();
  double max_bearing = -std::numeric_limits<double>::max();

  for (const auto& point : fixture.cloud.points) {
    if (point.geom_id != cylinder_geom) {
      continue;
    }
    const Eigen::Vector3d direction = WorldDirection(fixture.raycaster, point.ray_index);
    const Eigen::Vector3d hit = origin + point.range_m * direction;

    // Points on the curved surface satisfy |(x,y) - centre| = r. The flat cap would not,
    // so exclude anything at the very top of the cylinder.
    if (hit.z() > options.cylinder_height - 1e-6) {
      continue;
    }
    const Eigen::Vector2d planar(hit.x(), hit.y());
    hits.push_back(planar);

    const double radial =
        (planar - Eigen::Vector2d(options.cylinder_x, options.cylinder_y)).norm();
    worst_radial_residual =
        std::max(worst_radial_residual, std::abs(radial - options.cylinder_radius));

    const double bearing = std::atan2(hit.y() - origin.y(), hit.x() - origin.x());
    min_bearing = std::min(min_bearing, bearing);
    max_bearing = std::max(max_bearing, bearing);
  }

  const double distance =
      std::hypot(options.cylinder_x - origin.x(), options.cylinder_y - origin.y());
  const double expected_half_extent = std::asin(options.cylinder_radius / distance);
  const double measured_half_extent = 0.5 * (max_bearing - min_bearing);
  const double azimuth_step = 2.0 * M_PI / config.lidar.sensor.azimuth_rays;

  std::printf("      %zu curved-surface points, worst radial residual = %.9f m\n", hits.size(),
              worst_radial_residual);
  std::printf("      angular half-extent measured %.6f rad vs asin(r/d) = %.6f rad "
              "(one ray step = %.6f rad)\n",
              measured_half_extent, expected_half_extent, azimuth_step);

  Check(hits.size() > 50, "cylinder is well sampled", I("points", static_cast<long>(hits.size())));
  Check(worst_radial_residual < 1e-6, "every point lies on the cylinder surface",
        F("worst residual", worst_radial_residual));
  // A discrete grid cannot sample the exact tangent points, so the measured extent is
  // narrower than the analytic one by up to one step.
  Check(measured_half_extent <= expected_half_extent + 1e-9 &&
            measured_half_extent >= expected_half_extent - azimuth_step,
        "angular extent matches asin(r/d) to within one ray step",
        F("deficit rad", expected_half_extent - measured_half_extent, 6));

  // Independent algebraic (Kasa) circle fit - deliberately NOT the production estimator.
  if (hits.size() >= 3) {
    Eigen::Matrix3d ata = Eigen::Matrix3d::Zero();
    Eigen::Vector3d atb = Eigen::Vector3d::Zero();
    for (const auto& p : hits) {
      const Eigen::Vector3d row(p.x(), p.y(), 1.0);
      ata += row * row.transpose();
      atb += row * (p.x() * p.x() + p.y() * p.y());
    }
    const Eigen::Vector3d solution = ata.ldlt().solve(atb);
    const double cx = 0.5 * solution.x();
    const double cy = 0.5 * solution.y();
    const double radius = std::sqrt(std::max(0.0, solution.z() + cx * cx + cy * cy));
    const double centre_error = std::hypot(cx - options.cylinder_x, cy - options.cylinder_y);
    const double radius_error = std::abs(radius - options.cylinder_radius);

    std::printf("      Kasa fit: centre (%.5f, %.5f) r = %.5f  -> centre error %.5f m, "
                "radius error %.5f m\n", cx, cy, radius, centre_error, radius_error);
    Check(centre_error < 5e-3, "independent circle fit recovers the centre within 5 mm",
          F("m", centre_error, 6));
    Check(radius_error < 5e-3, "independent circle fit recovers the radius within 5 mm",
          F("m", radius_error, 6));
  }
}

void RunV4(const std::filesystem::path& xml) {
  std::printf("\nV4  self-occlusion: the arm must block, the housing must not\n");
  const PerceptionConfig config = MakeConfig();

  SceneOptions options;
  options.add_wall = true;
  options.wall_width = 8.0;

  Fixture fixture;
  std::string error;
  if (!fixture.Build(xml, options, config, error)) {
    Check(false, "build fixture", error);
    return;
  }

  const int wall_geom = mj_name2id(fixture.model, mjOBJ_GEOM, "validation_wall");
  auto count_wall_points = [&]() {
    int count = 0;
    for (const auto& point : fixture.cloud.points) {
      if (point.geom_id == wall_geom) ++count;
    }
    return count;
  };

  if (!fixture.Scan()) {
    Check(false, "scan (arms down)");
    return;
  }
  const int wall_down = count_wall_points();
  const int self_down = fixture.stats.self_rejected;

  // Swing the right arm up and forward so it crosses the sensor's view of the wall.
  const bool posed = SetJoint(fixture.model, fixture.data, "right_shoulder_pitch_joint", -1.4) &&
                     SetJoint(fixture.model, fixture.data, "right_shoulder_roll_joint", -0.35) &&
                     SetJoint(fixture.model, fixture.data, "right_elbow_joint", 0.5);
  if (!posed) {
    Check(false, "resolve right-arm joints");
    return;
  }
  fixture.Refresh();
  if (!fixture.Scan()) {
    Check(false, "scan (arm raised)");
    return;
  }
  const int wall_raised = count_wall_points();
  const int self_raised = fixture.stats.self_rejected;

  const int occluded = wall_down - wall_raised;
  const double occluded_fraction =
      wall_down > 0 ? static_cast<double>(occluded) / wall_down : 0.0;

  std::printf("      arms down:   wall points %d, self hits %d\n", wall_down, self_down);
  std::printf("      arm raised:  wall points %d, self hits %d\n", wall_raised, self_raised);
  std::printf("      => the arm occludes %d wall points (%.1f%%) and adds %d self hits\n",
              occluded, 100.0 * occluded_fraction, self_raised - self_down);

  Check(occluded > 0, "the raised arm occludes wall points", I("occluded", occluded));
  Check(self_raised > self_down, "the raised arm produces additional self hits",
        I("delta", self_raised - self_down));
}

void RunV5(const std::filesystem::path& xml) {
  std::printf("\nV5  rigid attachment: base<-sensor is pose-invariant\n");
  const PerceptionConfig config = MakeConfig();

  Fixture fixture;
  std::string error;
  if (!fixture.Build(xml, SceneOptions{}, config, error)) {
    Check(false, "build fixture", error);
    return;
  }

  const Eigen::Vector3d expected_translation = config.lidar.extrinsic.translation_xyz_m;
  const Eigen::Matrix3d expected_rotation =
      perception::core::RpyToRotation(config.lidar.extrinsic.rotation_rpy_rad);

  struct Pose {
    const char* name;
    const char* joint;
    double value;
  };
  const Pose poses[] = {
      {"standing (neutral)", nullptr, 0.0},
      {"waist pitch +0.4 rad", "waist_pitch_joint", 0.4},
      {"waist yaw -0.5 rad", "waist_yaw_joint", -0.5},
      {"waist roll +0.3 rad", "waist_roll_joint", 0.3},
      {"right arm raised", "right_shoulder_pitch_joint", -1.4},
      {"knee bend (walking-like)", "left_knee_joint", 0.6},
  };

  double worst_translation = 0.0;
  double worst_rotation = 0.0;
  bool height_changed_with_waist = false;
  bool height_held_for_limbs = false;
  double neutral_height = 0.0;

  for (const auto& pose : poses) {
    mj_resetData(fixture.model, fixture.data);
    if (pose.joint != nullptr &&
        !SetJoint(fixture.model, fixture.data, pose.joint, pose.value)) {
      Check(false, std::string("resolve joint ") + pose.joint);
      continue;
    }
    fixture.Refresh();

    const auto snapshot = fixture.raycaster.frame_provider().Snapshot(fixture.model, fixture.data);
    const Eigen::Isometry3d base_from_sensor = snapshot.base_from_sensor;
    const double translation_error =
        (base_from_sensor.translation() - expected_translation).norm();
    const double rotation_error =
        perception::core::RotationMaxAbsDiff(base_from_sensor.linear(), expected_rotation);
    const double world_z = snapshot.world_from_sensor.translation().z();

    worst_translation = std::max(worst_translation, translation_error);
    worst_rotation = std::max(worst_rotation, rotation_error);

    std::printf("      %-26s sensor world z = %.4f   |dt| = %.3e   |dR| = %.3e\n", pose.name,
                world_z, translation_error, rotation_error);

    if (pose.joint == nullptr) {
      neutral_height = world_z;
    } else if (std::string(pose.joint) == "waist_pitch_joint" ||
               std::string(pose.joint) == "waist_roll_joint") {
      if (std::abs(world_z - neutral_height) > 1e-4) height_changed_with_waist = true;
    } else if (std::string(pose.joint) == "right_shoulder_pitch_joint") {
      height_held_for_limbs = std::abs(world_z - neutral_height) < 1e-9;
    }
  }

  Check(worst_translation < 1e-9, "translation is pose-invariant to machine precision",
        F("worst |dt|", worst_translation, 12));
  Check(worst_rotation < 1e-9, "rotation is pose-invariant to machine precision",
        F("worst |dR|", worst_rotation, 12));
  // These two together are what prove the mount is on torso_link and nothing else.
  Check(height_changed_with_waist, "sensor height DOES move with the waist joints");
  Check(height_held_for_limbs, "sensor height does NOT move with arm motion");
}

void RunT1(const std::filesystem::path& xml) {
  std::printf("\nT1  can a wall test settle 'optical origin vs mounting base'?\n");
  const PerceptionConfig config = MakeConfig();

  // The two candidates differ by the Livox drawing's 47.0 mm from the mounting base to
  // the point-cloud origin O. Under the flipped mount, the alternative sits LOWER.
  constexpr double kBaseToOriginM = 0.047;

  SceneOptions options;
  options.add_wall = true;
  options.wall_width = 8.0;

  Fixture fixture;
  std::string error;
  if (!fixture.Build(xml, options, config, error)) {
    Check(false, "build fixture", error);
    return;
  }
  if (!fixture.Scan()) {
    Check(false, "scan");
    return;
  }

  const auto& snapshot = fixture.raycaster.last_snapshot();
  const Eigen::Vector3d origin_shipped = snapshot.world_from_sensor.translation();
  // The alternative origin lies 47 mm along the sensor's +Z, which points DOWN.
  const Eigen::Vector3d origin_alternative =
      origin_shipped + kBaseToOriginM * snapshot.world_from_sensor.linear().col(2);

  const double three_d_separation = (origin_shipped - origin_alternative).norm();
  const double height_difference = std::abs(origin_shipped.z() - origin_alternative.z());

  // Horizontal range to a vertical wall, straight ahead, from each candidate.
  const double plane_x = options.wall_x_center - options.wall_half_thickness;
  const double range_shipped = plane_x - origin_shipped.x();
  const double range_alternative = plane_x - origin_alternative.x();
  const double range_difference = std::abs(range_shipped - range_alternative);

  std::printf("      shipped origin      z = %.4f m\n", origin_shipped.z());
  std::printf("      alternative origin  z = %.4f m\n", origin_alternative.z());
  std::printf("      3-D separation                 %.4f m\n", three_d_separation);
  std::printf("      difference in sensor HEIGHT    %.4f m   <-- discriminating measurement\n",
              height_difference);
  std::printf("      difference in horizontal RANGE %.4f m   <-- NOT discriminating\n",
              range_difference);

  Check(std::abs(three_d_separation - kBaseToOriginM) < 1e-9,
        "the two candidates are exactly 47.0 mm apart in 3-D", F("m", three_d_separation));
  Check(range_difference < 0.005,
        "a wall range test CANNOT discriminate (difference < 5 mm)", F("m", range_difference));
  Check(height_difference > 0.04,
        "sensor height above a fitted ground plane CAN discriminate (> 40 mm)",
        F("m", height_difference));
  std::printf("      => HARDWARE TEST: stand the G1 on flat ground, fit the ground plane in\n"
              "         the raw Livox cloud, read the sensor height. 1.2654 m => the site is\n"
              "         the optical origin O (what we ship). 1.2185 m => it is the mounting\n"
              "         base, and the fix is z -= 0.047. STILL OPEN.\n");
}

}  // namespace

int main(int argc, char** argv) {
  const std::filesystem::path repository_root = argc > 1 ? argv[1] : ".";
  const std::filesystem::path xml =
      repository_root / "src/assets/robots/unitree_g1/xmls/scene_g1.xml";

  std::printf("Mid-360 ray bring-up validation suite\n");
  std::printf("scene: %s\n", xml.string().c_str());
  if (!std::filesystem::exists(xml)) {
    std::printf("FATAL: scene not found. Pass the repository root as argv[1].\n");
    return 2;
  }

  RunV0(xml);
  RunV1(xml);
  RunV2(xml);
  RunV3(xml);
  RunV4(xml);
  RunV5(xml);
  RunT1(xml);

  std::printf("\n=====================================================\n");
  std::printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
  if (g_failures != 0) {
    std::printf("RESULT: FAIL (%d failing checks)\n", g_failures);
    return 1;
  }
  std::printf("RESULT: PASS\n");
  return 0;
}
