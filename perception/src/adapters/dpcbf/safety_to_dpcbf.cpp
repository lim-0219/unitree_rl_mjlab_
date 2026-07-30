#include "perception/adapters/dpcbf/safety_to_dpcbf.h"

namespace perception::adapters::dpcbf {

const char* DpcbfAdapterParams::Validate() const {
  if (max_obstacles <= 0) return "dpcbf_adapter.max_obstacles must be > 0";
  // The id space DPCBF sees is `int`. A negative offset that could drive a live id below zero
  // is refused here rather than producing an ObstacleState whose id collides with the -1 that
  // dpcbf::ObstacleState uses as its own default sentinel.
  if (id_offset < 0) return "dpcbf_adapter.id_offset must be >= 0";
  return nullptr;
}

::dpcbf::ObstacleState ToDpcbfObstacleState(const core::SafetyObstacle& safety,
                                            const DpcbfAdapterParams& params, int index) {
  ::dpcbf::ObstacleState state;
  state.x = safety.center.x();
  state.y = safety.center.y();
  // The INFLATED radius. See the header essay; this is the single field that makes this
  // conversion different from the oracle one, and de-inflating it here would undo the only
  // term covering P8's short-arc under-estimate.
  state.radius = safety.radius_inflated_m;
  // Already spike-clamped by the safety stage (risk R10). The adapter does not re-clamp: two
  // modules bounding the same quantity means the effective bound depends on which is smaller,
  // which is how a documented limit stops being the limit.
  state.velocity_x = safety.velocity.x();
  state.velocity_y = safety.velocity.y();
  state.id = params.preserve_track_ids ? static_cast<int>(safety.id) + params.id_offset
                                       : index + params.id_offset;
  return state;
}

const char* ToDpcbfObstacleStates(const std::vector<core::SafetyObstacle>& safety,
                                  const DpcbfAdapterParams& params,
                                  std::vector<::dpcbf::ObstacleState>& out,
                                  DpcbfAdapterStats* stats) {
  out.clear();
  if (const char* reason = params.Validate()) return reason;

  DpcbfAdapterStats census;
  census.input = static_cast<int32_t>(safety.size());
  out.reserve(safety.size());

  for (const core::SafetyObstacle& obstacle : safety) {
    if (params.drop_invalid && !obstacle.IsValid()) {
      ++census.dropped_invalid;
      continue;
    }
    if (static_cast<int>(out.size()) >= params.max_obstacles) {
      // Counted, not silently truncated - and counted for EVERY obstacle past the cap, so the
      // census answers "how many did the QP never see" rather than "did the cap fire".
      ++census.dropped_capacity;
      continue;
    }
    // The OUTPUT position, not the input one, so a positional id set is contiguous 0..n-1
    // even when something ahead of it was dropped.
    out.push_back(ToDpcbfObstacleState(obstacle, params, static_cast<int>(out.size())));
  }

  census.emitted = static_cast<int32_t>(out.size());
  if (stats != nullptr) *stats = census;
  return nullptr;
}

}  // namespace perception::adapters::dpcbf
