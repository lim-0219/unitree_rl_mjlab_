#include "perception/adapters/dpcbf/oracle_to_dpcbf.h"

namespace perception::adapters::dpcbf {

::dpcbf::ObstacleState ToDpcbfObstacleState(const core::OracleObstacleState& oracle) {
  ::dpcbf::ObstacleState state;
  state.x = oracle.center.x();
  state.y = oracle.center.y();
  state.radius = oracle.radius_m;
  state.velocity_x = oracle.velocity.x();
  state.velocity_y = oracle.velocity.y();
  state.id = static_cast<int>(oracle.id);
  return state;
}

void ToDpcbfObstacleStates(const std::vector<core::OracleObstacleState>& oracle,
                           std::vector<::dpcbf::ObstacleState>& out) {
  out.clear();
  out.reserve(oracle.size());
  for (const core::OracleObstacleState& state : oracle) {
    out.push_back(ToDpcbfObstacleState(state));
  }
}

}  // namespace perception::adapters::dpcbf
