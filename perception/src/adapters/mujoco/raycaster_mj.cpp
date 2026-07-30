#include "perception/adapters/mujoco/raycaster_mj.h"

#include <chrono>
#include <cmath>

#include "perception/adapters/mujoco/ray_pattern_generator.h"

namespace perception::adapters::mujoco {
namespace {

// Static geoms (the floor plane, arena walls) are exactly what the sensor should see.
constexpr mjtByte kIncludeStaticGeoms = 1;

// Self rejection is done by body ancestry after the fact rather than by handing MuJoCo a
// bodyexclude, because bodyexclude takes a single body id and the robot is a whole tree.
constexpr int kNoBodyExclude = -1;

}  // namespace

bool RaycasterMj::Bind(const mjModel* model, mjData* data,
                       const integration::LidarConfig& config, std::string& error) {
  bound_ = false;
  model_ = nullptr;
  data_ = nullptr;

  if (model == nullptr || data == nullptr) {
    error = "RaycasterMj::Bind: model or data is null";
    return false;
  }
  if (!frame_provider_.Bind(model, config, error)) {
    return false;
  }

  model_ = model;
  data_ = data;
  config_ = config;
  raycast_exact_ = config.raycast.exact;
  aperture_enabled_ = config.aperture.transparent_geom_group >= 0;

  pattern_ = GenerateUniformGrid(config.sensor);
  if (!pattern_.IsConsistent()) {
    error = "RaycasterMj::Bind: generated ray pattern is inconsistent";
    return false;
  }

  // Enable every visibility group except the aperture group. `geomgroup` is documented as
  // "an array of length mjNGROUP, where 1 means the group should be included".
  for (int group = 0; group < mjNGROUP; ++group) {
    geom_group_mask_[group] = 1;
  }
  if (config.aperture.transparent_geom_group >= 0 &&
      config.aperture.transparent_geom_group < mjNGROUP) {
    geom_group_mask_[config.aperture.transparent_geom_group] = 0;
  }

  const std::size_t rays = pattern_.size();
  world_directions_.assign(3 * rays, 0.0);
  batched_distances_.assign(rays, -1.0);
  batched_geom_ids_.assign(rays, -1);

  noise_generator_.seed(config.sensor.seed);
  sequence_ = 0;
  bound_ = true;
  return true;
}

const mjtByte* RaycasterMj::GeomGroupMask() const {
  return aperture_enabled_ ? geom_group_mask_ : nullptr;
}

bool RaycasterMj::Scan(core::TimedPointCloud& cloud, core::ScanStats& stats) {
  cloud.Clear();
  stats = core::ScanStats{};

  if (!bound_) {
    return false;
  }

  const auto start = std::chrono::steady_clock::now();

  last_snapshot_ = frame_provider_.Snapshot(model_, data_);
  if (!last_snapshot_.valid) {
    return false;
  }

  const Eigen::Vector3d origin = last_snapshot_.world_from_sensor.translation();
  const Eigen::Matrix3d world_from_sensor_rotation = last_snapshot_.world_from_sensor.linear();

  const std::size_t rays = pattern_.size();
  cloud.stamp_s = last_snapshot_.stamp_s;
  cloud.sequence = sequence_++;
  cloud.frame = core::FrameId::kWorld;
  cloud.motion_compensation = core::MotionCompensation::kNone;
  cloud.points.reserve(rays);

  stats.rays_cast = static_cast<int>(rays);

  // Rotate the sensor-frame table into the world once per scan. This is where the
  // roll = pi flip actually takes effect - the table itself is never pre-flipped.
  for (std::size_t i = 0; i < rays; ++i) {
    const Eigen::Vector3d direction = world_from_sensor_rotation * pattern_.directions[i];
    world_directions_[3 * i + 0] = direction.x();
    world_directions_[3 * i + 1] = direction.y();
    world_directions_[3 * i + 2] = direction.z();
  }

  const mjtNum ray_origin[3] = {origin.x(), origin.y(), origin.z()};
  const mjtByte* group_mask = GeomGroupMask();
  const double min_range = config_.sensor.min_range_m;
  const double max_range = config_.sensor.max_range_m;

  if (raycast_exact_) {
    // Exact path: mj_ray scans every geom. Takes a const mjData*, so it cannot perturb
    // simulation state at all.
    for (std::size_t i = 0; i < rays; ++i) {
      int geom_id = -1;
      const mjtNum distance =
          mj_ray(model_, data_, ray_origin, &world_directions_[3 * i], group_mask,
                 kIncludeStaticGeoms, kNoBodyExclude, &geom_id);
      batched_distances_[i] = distance;
      batched_geom_ids_[i] = geom_id;
    }
  } else {
    // Batched fallback. Documented as lossy; see the header.
    mj_multiRay(model_, data_, ray_origin, world_directions_.data(), group_mask,
                kIncludeStaticGeoms, kNoBodyExclude, batched_geom_ids_.data(),
                batched_distances_.data(), static_cast<int>(rays), max_range);
  }

  std::normal_distribution<double> range_noise(0.0, config_.sensor.range_noise_std_m);
  const bool apply_noise =
      config_.sensor.apply_range_noise && config_.sensor.range_noise_std_m > 0.0;

  for (std::size_t i = 0; i < rays; ++i) {
    const mjtNum distance = batched_distances_[i];
    const int geom_id = batched_geom_ids_[i];

    if (distance < 0.0 || geom_id < 0) {
      ++stats.no_hit;
      continue;
    }
    ++stats.raw_hits;

    const int body_id = model_->geom_bodyid[geom_id];
    if (config_.self_filter.reject_self_hits &&
        FrameProviderMj::IsSelfBody(model_, body_id, frame_provider_.robot_root_body_id())) {
      ++stats.self_rejected;
      continue;
    }

    double range = distance;
    if (apply_noise) {
      range += range_noise(noise_generator_);
    }
    if (range < min_range || range > max_range) {
      ++stats.range_rejected;
      continue;
    }

    core::RawTimedPoint point;
    const Eigen::Vector3d hit(origin.x() + range * world_directions_[3 * i + 0],
                              origin.y() + range * world_directions_[3 * i + 1],
                              origin.z() + range * world_directions_[3 * i + 2]);
    point.position = hit.cast<float>();
    point.time_offset_s = static_cast<float>(pattern_.time_offsets_s[i]);
    point.range_m = range;
    point.ray_index = static_cast<int32_t>(i);
    point.geom_id = geom_id;
    point.body_id = body_id;
    cloud.points.push_back(point);
    ++stats.accepted;
  }

  // Stop the clock BEFORE the optional diagnostic pass: scan_wall_time_us must report the
  // cost of the production scan, not of the extra pass the validation harness asks for.
  stats.scan_wall_time_us =
      std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count();

  if (collect_aperture_diagnostic_ && config_.aperture.transparent_geom_group >= 0) {
    // Second pass with no mask: how many rays would the shell have swallowed?
    const int transparent_group = config_.aperture.transparent_geom_group;
    int transmitted = 0;
    for (std::size_t i = 0; i < rays; ++i) {
      int geom_id = -1;
      const mjtNum distance =
          mj_ray(model_, data_, ray_origin, &world_directions_[3 * i], nullptr,
                 kIncludeStaticGeoms, kNoBodyExclude, &geom_id);
      if (distance >= 0.0 && geom_id >= 0 && model_->geom_group[geom_id] == transparent_group) {
        ++transmitted;
      }
    }
    stats.aperture_transmitted = transmitted;
    stats.aperture_diagnostic_valid = true;
  }

  return true;
}

}  // namespace perception::adapters::mujoco
