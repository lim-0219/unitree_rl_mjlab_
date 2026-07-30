// DetectionConfig -> SegmentCircleDetectorParams. The one place the two are tied together.
//
// Same reasoning as integration/projection_params.h: `perception_core` links
// `perception_contracts` and nothing else, and that link set is what enforces the core
// dependency rules rather than an honour system. Integration is the side of the boundary
// allowed to see both types.
//
// The mapping is deliberately dumb and total - every SegmentCircleDetectorParams field with a
// DetectionConfig counterpart is assigned here, and the detection test asserts
// field-completeness against the SHIPPED configs/perception.yaml, so a new `detection.*` key
// that never reaches the detector fails a test rather than being silently ignored.
#ifndef PERCEPTION_INTEGRATION_DETECTION_PARAMS_H_
#define PERCEPTION_INTEGRATION_DETECTION_PARAMS_H_

#include "perception/core/detection/segment_circle_detector.h"
#include "perception/integration/perception_config.h"

namespace perception::integration {

inline core::SegmentCircleDetectorParams MakeSegmentCircleDetectorParams(
    const DetectionConfig& config) {
  core::SegmentCircleDetectorParams params;
  params.min_group_points = config.min_group_points;
  params.max_group_distance_m = config.max_group_distance_m;
  params.distance_proportion = config.distance_proportion;
  params.max_split_distance_m = config.max_split_distance_m;
  params.max_merge_separation_m = config.max_merge_separation_m;
  params.max_merge_spread_m = config.max_merge_spread_m;
  params.max_circle_radius_m = config.max_circle_radius_m;
  params.radius_enlargement_m = config.radius_enlargement_m;
  params.circles_from_visibles = config.circles_from_visibles;
  params.use_split_and_merge = config.use_split_and_merge;
  params.discard_converted_segments = config.discard_converted_segments;
  params.max_clusters = config.max_clusters;
  params.max_primitives = config.max_primitives;
  params.max_circles = config.max_circles;
  return params;
}

}  // namespace perception::integration

#endif  // PERCEPTION_INTEGRATION_DETECTION_PARAMS_H_
