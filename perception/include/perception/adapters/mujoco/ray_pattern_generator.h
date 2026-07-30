// RayPatternGenerator - builds the sensor-frame ray table for one frame period.
//
// This module is MuJoCo-free despite living under adapters/mujoco: it is grouped here
// because the pattern is a property of the simulated sensor. See the approximation notice
// on core::RayPattern.
#ifndef PERCEPTION_ADAPTERS_MUJOCO_RAY_PATTERN_GENERATOR_H_
#define PERCEPTION_ADAPTERS_MUJOCO_RAY_PATTERN_GENERATOR_H_

#include "perception/core/contracts/ray_pattern.h"
#include "perception/integration/perception_config.h"

namespace perception::adapters::mujoco {

// Uniform azimuth x elevation grid in the Livox point-cloud frame O-XYZ (x forward,
// y left, z up, right-handed).
//
// Elevation is measured from the sensor XY plane and spans [vertical_fov_deg_min,
// vertical_fov_deg_max] with BOTH endpoints inclusive, so the published -7 deg and
// +52 deg extremes are actually sampled. Azimuth spans horizontal_fov_deg with the
// last sample excluded, because at 360 deg the first and last would coincide.
//
// Ordering is azimuth-major with all elevations of one azimuth column adjacent, and
// every ray in a column shares that column's time offset. That mimics a rotating
// multi-beam sweep, which is the closest honest analogue of a real frame given that we
// are not reproducing the rosette.
core::RayPattern GenerateUniformGrid(const integration::LidarSensorModelConfig& sensor);

}  // namespace perception::adapters::mujoco

#endif  // PERCEPTION_ADAPTERS_MUJOCO_RAY_PATTERN_GENERATOR_H_
