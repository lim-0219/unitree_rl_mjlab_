// SafetyConfig -> SafetyParams. The one place the two are tied together.
//
// Same reasoning as integration/detection_params.h, tracking_params.h and projection_params.h:
// `perception_core` links `perception_contracts` and nothing else, and that link set is what
// enforces the core dependency rules rather than an honour system. Integration is the side of the
// boundary allowed to see both types.
//
// The mapping is deliberately dumb and total - every SafetyParams field with a SafetyConfig
// counterpart is assigned here, and the safety test asserts field-completeness against the
// SHIPPED configs/perception.yaml, so a new `safety.*` key that never reaches the stage fails a
// test rather than being silently ignored.
#ifndef PERCEPTION_INTEGRATION_SAFETY_PARAMS_H_
#define PERCEPTION_INTEGRATION_SAFETY_PARAMS_H_

#include "perception/core/safety/safety_state_generator.h"
#include "perception/integration/perception_config.h"

namespace perception::integration {

inline core::SafetyParams MakeSafetyParams(const SafetyConfig& config) {
  core::SafetyParams params;
  params.max_age_s = config.max_age_s;
  params.min_track_age_s = config.min_track_age_s;
  params.min_track_hits = config.min_track_hits;
  params.radius_inflation_k_sigma = config.radius_inflation_k_sigma;
  params.radius_inflation_fixed_m = config.radius_inflation_fixed_m;
  params.latency_inflation_s = config.latency_inflation_s;
  params.use_enclosing_radius = config.use_enclosing_radius;
  params.min_radius_m = config.min_radius_m;
  params.max_radius_m = config.max_radius_m;
  params.max_speed_mps = config.max_speed_mps;
  params.max_obstacles = config.max_obstacles;
  return params;
}

}  // namespace perception::integration

#endif  // PERCEPTION_INTEGRATION_SAFETY_PARAMS_H_
