// RayPattern - the precomputed ray table for one sensor frame period.
//
// APPROXIMATION, stated up front: this repository samples a uniform azimuth x elevation
// grid (default 360 x 32 = 11520 rays/frame), NOT the Mid-360's real non-repetitive
// rosette. The real sensor emits 200 000 pts/s, i.e. ~20 000 points/frame at 10 Hz, with a
// pattern that does not repeat between frames. Coverage statistics therefore differ.
// The rest of the pipeline is pattern-agnostic, so raising fidelity later is a change to
// the generator alone. See g1_mid360_extrinsic.md section 7.5 and
// g1_mid360_sensor_model.md section 7.4.
#ifndef PERCEPTION_CORE_CONTRACTS_RAY_PATTERN_H_
#define PERCEPTION_CORE_CONTRACTS_RAY_PATTERN_H_

#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "perception/core/contracts/frames.h"

namespace perception::core {

struct RayPattern {
  // Unit direction of each ray, in the SENSOR frame. The extrinsic (including the
  // roll = pi flip) is applied downstream by the ray caster; nothing here is
  // pre-flipped.
  std::vector<Eigen::Vector3d> directions;

  // Intra-frame acquisition time of each ray, relative to the start of the frame,
  // in seconds. Retained so a later phase can deskew; the P4 slice only records it.
  std::vector<double> time_offsets_s;

  double period_s = 0.1;      // 1 / scan_rate_hz
  int azimuth_rays = 0;
  int elevation_rays = 0;
  FrameId frame = FrameId::kSensor;

  std::size_t size() const { return directions.size(); }
  bool empty() const { return directions.empty(); }

  bool IsConsistent() const {
    return !directions.empty() && directions.size() == time_offsets_s.size() &&
           period_s > 0.0 &&
           static_cast<std::size_t>(azimuth_rays) * static_cast<std::size_t>(elevation_rays) ==
               directions.size();
  }
};

}  // namespace perception::core

#endif  // PERCEPTION_CORE_CONTRACTS_RAY_PATTERN_H_
