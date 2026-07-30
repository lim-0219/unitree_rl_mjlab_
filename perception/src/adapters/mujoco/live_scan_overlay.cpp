#include "perception/adapters/mujoco/live_scan_overlay.h"

#include <algorithm>

namespace perception::adapters::mujoco {
namespace {

void ToFloatRgba(const Eigen::Vector4d& in, float out[4]) {
  for (int i = 0; i < 4; ++i) {
    out[i] = static_cast<float>(in[i]);
  }
}

}  // namespace

LiveScanOverlay::~LiveScanOverlay() { Free(); }

bool LiveScanOverlay::Allocate(const mjModel* model,
                               const integration::LidarVisualizationConfig& config,
                               std::size_t expected_rays, std::string& error) {
  Free();

  if (model == nullptr) {
    error = "LiveScanOverlay::Allocate: model is null";
    return false;
  }

  config_ = config;

  const int point_geoms =
      config.draw_points
          ? static_cast<int>(expected_rays / std::max(1, config.point_stride)) + 1
          : 0;
  const int ray_geoms =
      config.draw_rays ? static_cast<int>(expected_rays / std::max(1, config.ray_stride)) + 1 : 0;
  const int frame_geoms = config.draw_sensor_frame ? 4 : 0;  // origin marker + 3 axes.
  max_geoms_ = point_geoms + ray_geoms + frame_geoms + 8;    // slack

  mjv_defaultScene(&scene_);
  mjv_makeScene(model, &scene_, max_geoms_);
  scene_.ngeom = 0;
  // The overlay contributes geoms only; leaving the scene's own model-drawing flags off
  // avoids double-rendering the robot.
  allocated_ = true;
  return true;
}

void LiveScanOverlay::Free() {
  if (allocated_) {
    mjv_freeScene(&scene_);
    allocated_ = false;
    max_geoms_ = 0;
  }
}

bool LiveScanOverlay::PushGeom(int type, const mjtNum size[3], const mjtNum pos[3],
                               const float rgba[4]) {
  if (!allocated_ || scene_.ngeom >= max_geoms_) {
    return false;
  }
  mjv_initGeom(&scene_.geoms[scene_.ngeom], type, size, pos, nullptr, rgba);
  ++scene_.ngeom;
  return true;
}

bool LiveScanOverlay::PushLine(const mjtNum from[3], const mjtNum to[3], const float rgba[4],
                               double width) {
  if (!allocated_ || scene_.ngeom >= max_geoms_) {
    return false;
  }
  mjvGeom* geom = &scene_.geoms[scene_.ngeom];
  mjv_initGeom(geom, mjGEOM_LINE, nullptr, nullptr, nullptr, rgba);
  mjv_connector(geom, mjGEOM_LINE, width, from, to);
  ++scene_.ngeom;
  return true;
}

void LiveScanOverlay::Update(const core::TimedPointCloud& cloud,
                             const core::FrameTransformSnapshot& snapshot,
                             const core::ScanStats& stats) {
  (void)stats;
  if (!allocated_ || !config_.enabled) {
    return;
  }

  scene_.ngeom = 0;

  const Eigen::Vector3d origin = snapshot.world_from_sensor.translation();
  const mjtNum origin_mj[3] = {origin.x(), origin.y(), origin.z()};

  if (config_.draw_sensor_frame && snapshot.valid) {
    // A marker at the optical origin O, plus the sensor axes so the upside-down mount is
    // visible at a glance: +Z (blue) points DOWN and +Y (green) to the robot's right.
    const mjtNum marker_size[3] = {0.03, 0.03, 0.03};
    const float marker_rgba[4] = {1.0f, 0.9f, 0.1f, 1.0f};
    PushGeom(mjGEOM_SPHERE, marker_size, origin_mj, marker_rgba);

    constexpr double kAxisLength = 0.25;
    const float axis_rgba[3][4] = {{1.0f, 0.2f, 0.2f, 1.0f},
                                   {0.2f, 1.0f, 0.2f, 1.0f},
                                   {0.3f, 0.4f, 1.0f, 1.0f}};
    for (int axis = 0; axis < 3; ++axis) {
      const Eigen::Vector3d tip =
          origin + kAxisLength * snapshot.world_from_sensor.linear().col(axis);
      const mjtNum tip_mj[3] = {tip.x(), tip.y(), tip.z()};
      PushLine(origin_mj, tip_mj, axis_rgba[axis], 3.0);
    }
  }

  if (config_.draw_rays) {
    float ray_rgba[4];
    ToFloatRgba(config_.ray_rgba, ray_rgba);
    const int stride = std::max(1, config_.ray_stride);
    for (std::size_t i = 0; i < cloud.points.size(); i += stride) {
      const auto& point = cloud.points[i];
      const mjtNum hit[3] = {point.position.x(), point.position.y(), point.position.z()};
      if (!PushLine(origin_mj, hit, ray_rgba, config_.ray_width_px)) {
        break;
      }
    }
  }

  if (config_.draw_points) {
    float point_rgba[4];
    ToFloatRgba(config_.point_rgba, point_rgba);
    const mjtNum point_size[3] = {config_.point_size_m, config_.point_size_m,
                                  config_.point_size_m};
    const int stride = std::max(1, config_.point_stride);
    for (std::size_t i = 0; i < cloud.points.size(); i += stride) {
      const auto& point = cloud.points[i];
      const mjtNum position[3] = {point.position.x(), point.position.y(), point.position.z()};
      if (!PushGeom(mjGEOM_SPHERE, point_size, position, point_rgba)) {
        break;
      }
    }
  }
}

}  // namespace perception::adapters::mujoco
