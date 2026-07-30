// TrackingConfig -> TrackingParams. The one place the two are tied together.
//
// Same reasoning as integration/detection_params.h and integration/projection_params.h:
// `perception_core` links `perception_contracts` and nothing else, and that link set is what
// enforces the core dependency rules rather than an honour system. Integration is the side of the
// boundary allowed to see both types.
//
// The mapping is deliberately dumb and total - every TrackingParams field with a TrackingConfig
// counterpart is assigned here, and the tracking test asserts field-completeness against the
// SHIPPED configs/perception.yaml, so a new `tracking.*` key that never reaches the tracker fails
// a test rather than being silently ignored.
#ifndef PERCEPTION_INTEGRATION_TRACKING_PARAMS_H_
#define PERCEPTION_INTEGRATION_TRACKING_PARAMS_H_

#include "perception/core/tracking/kf_circle_tracker.h"
#include "perception/integration/perception_config.h"

namespace perception::integration {

inline core::TrackingParams MakeTrackingParams(const TrackingConfig& config) {
  core::TrackingParams params;
  params.process_variance = config.process_variance;
  params.process_rate_variance = config.process_rate_variance;
  params.measurement_variance = config.measurement_variance;
  params.min_correspondence_cost_m = config.min_correspondence_cost_m;
  params.association_radius_weight = config.association_radius_weight;
  params.measurement_sigma_scale = config.measurement_sigma_scale;
  params.measurement_sigma_floor_m = config.measurement_sigma_floor_m;
  params.initial_rate_variance = config.initial_rate_variance;
  params.confirm_hits = config.confirm_hits;
  params.delete_misses = config.delete_misses;
  params.max_coast_s = config.max_coast_s;
  params.enable_fusion = config.enable_fusion;
  params.enable_fission = config.enable_fission;
  params.max_tracks = config.max_tracks;
  return params;
}

}  // namespace perception::integration

#endif  // PERCEPTION_INTEGRATION_TRACKING_PARAMS_H_
