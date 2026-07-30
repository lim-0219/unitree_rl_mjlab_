#include "perception/adapters/mujoco/ray_pattern_generator.h"

#include <cmath>

namespace perception::adapters::mujoco {

core::RayPattern GenerateUniformGrid(const integration::LidarSensorModelConfig& sensor) {
  constexpr double kDegToRad = M_PI / 180.0;

  core::RayPattern pattern;
  pattern.azimuth_rays = sensor.azimuth_rays;
  pattern.elevation_rays = sensor.elevation_rays;
  pattern.period_s = 1.0 / sensor.scan_rate_hz;
  pattern.frame = core::FrameId::kSensor;

  const std::size_t total = static_cast<std::size_t>(sensor.azimuth_rays) *
                            static_cast<std::size_t>(sensor.elevation_rays);
  pattern.directions.reserve(total);
  pattern.time_offsets_s.reserve(total);

  const double elevation_min = sensor.vertical_fov_deg_min * kDegToRad;
  const double elevation_max = sensor.vertical_fov_deg_max * kDegToRad;
  // Endpoint-inclusive: n rings across the closed interval need n-1 gaps.
  const double elevation_step = sensor.elevation_rays > 1
                                    ? (elevation_max - elevation_min) / (sensor.elevation_rays - 1)
                                    : 0.0;

  const double azimuth_span = sensor.horizontal_fov_deg * kDegToRad;
  // Endpoint-exclusive: at a full 360 deg the first and last column would coincide.
  const double azimuth_step = azimuth_span / sensor.azimuth_rays;
  const double azimuth_start = -0.5 * azimuth_span;

  for (int a = 0; a < sensor.azimuth_rays; ++a) {
    const double azimuth = azimuth_start + a * azimuth_step;
    const double cos_azimuth = std::cos(azimuth);
    const double sin_azimuth = std::sin(azimuth);
    // Whole column acquired at one instant, as a rotating multi-beam would.
    const double time_offset = pattern.period_s * a / sensor.azimuth_rays;

    for (int e = 0; e < sensor.elevation_rays; ++e) {
      const double elevation = elevation_min + e * elevation_step;
      const double cos_elevation = std::cos(elevation);
      pattern.directions.emplace_back(cos_elevation * cos_azimuth,
                                      cos_elevation * sin_azimuth,
                                      std::sin(elevation));
      pattern.time_offsets_s.push_back(time_offset);
    }
  }

  return pattern;
}

}  // namespace perception::adapters::mujoco
