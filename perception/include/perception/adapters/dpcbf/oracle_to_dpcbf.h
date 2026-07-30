// adapters/dpcbf - the ONLY module allowed to touch dpcbf::ObstacleState.
//
// This file is the oracle half of that adapter, landed now because the obstacle-source
// selector has to return dpcbf::ObstacleState in oracle mode and something has to produce it.
// The estimated half - SafetyObstacle -> dpcbf::ObstacleState, with safety inflation,
// staleness gating, validity dropping and the id-offset policy - is P11 and lives beside this
// in the same module when it arrives. Deliberately so: this phase's job is to move code
// without changing it, and P11's job is to change code without moving it. Doing both at once
// in one file is how a bit-exactness guarantee gets quietly lost.
//
// WHY A MODULE RATHER THAN A FEW LINES IN THE SELECTOR. The dependency rules name
// adapters/dpcbf as the only place dpcbf headers may be included, with `integration` allowed
// to name the adapter's output type but not to do the conversion. Putting the field copy in
// integration/ would work and would be two files lighter, at the cost of making P11 a
// migration of live, safety-critical conversion code out of a file that by then also holds
// mode-selection logic. The seam is cheaper now than later.
//
// This module must never include a MuJoCo header (forbidden edge: dpcbf adapter -> MuJoCo).
#ifndef PERCEPTION_ADAPTERS_DPCBF_ORACLE_TO_DPCBF_H_
#define PERCEPTION_ADAPTERS_DPCBF_ORACLE_TO_DPCBF_H_

#include <vector>

#include "dpcbf/dpcbf_safety_filter.h"
#include "perception/core/contracts/obstacles.h"

// The namespace intentionally mirrors the directory, which means an unqualified `dpcbf::`
// inside it would resolve to perception::adapters::dpcbf and not to the real thing. Every
// reference to the DPCBF library is therefore written `::dpcbf::`, here and in the .cpp.
namespace perception::adapters::dpcbf {

// A pure, total field copy. No inflation, no clamping, no unit conversion, no id remapping:
// core::OracleObstacleState already holds ground truth in exactly DPCBF's units and frame
// (world XY, metres, m/s), so anything beyond assignment would be a behaviour change.
//
// This IS the pre-change inline conversion from simulate/src/main.cc, relocated:
//     {position[0], position[1], radius, velocity[0], velocity[1], (int)index}
// Every field is double -> double or int32 -> int, so the copy is bit-exact by construction
// rather than by luck; perception_oracle_equivalence_test proves it against a fixture
// captured from the old code path anyway.
::dpcbf::ObstacleState ToDpcbfObstacleState(const core::OracleObstacleState& oracle);

// Vector form. TOTAL AND ORDER-PRESERVING, and both of those are load-bearing:
//
//   * total - every input becomes exactly one output. No cap and no drop-invalid pass, even
//     though perception.dpcbf_adapter.max_obstacles (20) and drop_invalid (true) exist in the
//     config today. Those knobs are P11's, for the ESTIMATED path. The oracle path passes the
//     complete snapshot - 90 obstacles under the shipped dpcbf_config.yaml - because that is
//     what the inline code did, and DpcbfSafetyFilter selects its own top-k
//     (default_num_constraints: 10) from whatever it is given. Applying a cap here would
//     silently change which constraints the QP sees. See obstacle_source_selector.h.
//
//   * order-preserving - DPCBF's constraint selection sorts by priority with distance as a
//     tie-break, so a reordered input list can change which of two equally-ranked obstacles
//     becomes an active constraint. Element-wise correctness is not sufficient; the sequence
//     has to match too.
//
// `out` is cleared and reserved, so a caller may reuse a buffer across calls.
void ToDpcbfObstacleStates(const std::vector<core::OracleObstacleState>& oracle,
                           std::vector<::dpcbf::ObstacleState>& out);

}  // namespace perception::adapters::dpcbf

#endif  // PERCEPTION_ADAPTERS_DPCBF_ORACLE_TO_DPCBF_H_
