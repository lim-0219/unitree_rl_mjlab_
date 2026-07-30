// ProjectionConfig -> ScanProjectorParams. The one place the two are tied together.
//
// This header sits in integration/ rather than core/ on purpose. `perception_core` links
// `perception_contracts` and nothing else, and that link set is what enforces the
// architecture doc's core-dependency rules; a core header that included
// integration/perception_config.h would still compile and would turn the enforcement back
// into an honour system. Integration is the side of the boundary allowed to see both types.
//
// The mapping is deliberately dumb and total: every ScanProjectorParams field that has a
// ProjectionConfig counterpart is assigned here, and the two that do not (angle_min_rad,
// angle_max_rad) are pinned to the section 7 full circle, which is not configurable. The
// projection test asserts field-completeness against the SHIPPED configs/perception.yaml,
// so a new `projection.*` key that never reaches the projector fails a test rather than
// being silently ignored.
#ifndef PERCEPTION_INTEGRATION_PROJECTION_PARAMS_H_
#define PERCEPTION_INTEGRATION_PROJECTION_PARAMS_H_

#include "perception/core/contracts/validation.h"
#include "perception/core/projection/scan_projector.h"
#include "perception/integration/perception_config.h"

namespace perception::integration {

inline core::ScanProjectorParams MakeScanProjectorParams(const ProjectionConfig& config) {
  core::ScanProjectorParams params;
  params.bins = config.bins;

  // NOT configurable. Section 7 fixes the scan at the full circle, and the detector's
  // grouping threshold is re-derived from `projection.bins` on the assumption that the
  // window spans exactly 2*pi (see the distance_proportion note in perception_config.h).
  // A configurable window would silently break that derivation.
  params.angle_min_rad = -core::kPi;
  params.angle_max_rad = core::kPi;

  params.range_min_m = config.range_min_m;
  params.range_max_m = config.range_max_m;
  params.height_band_min_m = config.height_band_min_m;
  params.height_band_max_m = config.height_band_max_m;
  params.collect_bin_histogram = config.collect_bin_histogram;
  return params;
}

}  // namespace perception::integration

#endif  // PERCEPTION_INTEGRATION_PROJECTION_PARAMS_H_
