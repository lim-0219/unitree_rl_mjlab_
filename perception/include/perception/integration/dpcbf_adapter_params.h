// DpcbfAdapterConfig -> DpcbfAdapterParams. The one place the two are tied together.
//
// Same reasoning as integration/detection_params.h, tracking_params.h, projection_params.h and
// safety_params.h: `perception_dpcbf_adapter` links `perception_contracts` plus a headers-only
// dpcbf include path and nothing else, and that link set is what enforces the dependency rules
// rather than an honour system. Integration is the side of the boundary allowed to see both.
//
// The mapping is deliberately dumb and total - every DpcbfAdapterParams field with a
// DpcbfAdapterConfig counterpart is assigned here, and the adapter test asserts
// field-completeness against the SHIPPED configs/perception.yaml, so a new `dpcbf_adapter.*`
// key that never reaches the adapter fails a test rather than being silently ignored.
#ifndef PERCEPTION_INTEGRATION_DPCBF_ADAPTER_PARAMS_H_
#define PERCEPTION_INTEGRATION_DPCBF_ADAPTER_PARAMS_H_

#include "perception/adapters/dpcbf/adapter_params.h"
#include "perception/integration/perception_config.h"

namespace perception::integration {

inline adapters::dpcbf::DpcbfAdapterParams MakeDpcbfAdapterParams(
    const DpcbfAdapterConfig& config) {
  adapters::dpcbf::DpcbfAdapterParams params;
  params.preserve_track_ids = config.preserve_track_ids;
  params.id_offset = config.id_offset;
  params.max_obstacles = config.max_obstacles;
  params.drop_invalid = config.drop_invalid;
  return params;
}

}  // namespace perception::integration

#endif  // PERCEPTION_INTEGRATION_DPCBF_ADAPTER_PARAMS_H_
