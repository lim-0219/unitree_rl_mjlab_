// SafetyObstacle -> dpcbf::ObstacleState: the ESTIMATED half of the DPCBF adapter.
//
// Lands BESIDE oracle_to_dpcbf.{h,cpp} in the same module rather than inside it, which is the
// split P3 reserved: "this phase's job is to move code without changing it, and P11's job is to
// change code without moving it". The oracle conversion is untouched by this file.
//
// =========================================================================================
// WHICH RADIUS GOES TO THE QP, AND WHY THE QUESTION IS ALREADY CLOSED
// =========================================================================================
// `radius` is `SafetyObstacle::radius_inflated_m`. Not `radius_true_m`, which on the estimated
// path is the tracked, ENLARGEMENT-INCLUSIVE radius and not the object's real size (see the
// CORRECTION in core/contracts/obstacles.h), and not any recomputation of it here.
//
// This adapter deliberately performs NO radius policy of its own. The architecture doc §17
// still recommends "conservative enclosing over fitted", and following it here would be wrong:
// P10 measured `radius_enclosing_m` smaller than `radius_fitted_m` on 20 of 20 corpus circles,
// by 0.174-0.207 m at the shipped enlargement, so the doc's recommendation is an UNDER-estimate
// for this detector - the one unrecoverable error by the doc's own §17. `SafetyParams::
// use_enclosing_radius` is a documented no-op with a test asserting it inert. P10's output is
// final on this question and this module consumes it as given.
//
// The consequence a reader should carry away: what reaches the QP for a 0.25 m arena cylinder
// is about 0.69 m (tracked radius ~0.42 m plus mean total inflation 0.272 m), BEFORE the filter
// applies its own s=1.05 and r_rob. Whether a constraint set that wide is usable is not this
// file's claim to make; it is measured by the paired evaluator.
//
// That number was 0.94 m as P10 left it. Two later corrections brought it down and both are
// worth naming, because they were corrections to constants rather than to algorithms: the
// fit_residual_m fix cut the k_sigma inflation terms from 0.2926 m to 0.0739 m, and re-deriving
// `detection.radius_enlargement_m` from P8's measured short-arc bias (0.25 m -> 0.17 m, with
// `safety.radius_inflation_fixed_m` taking 0.03 m of that back) removed a further 0.05 m.
//
// =========================================================================================
// IDS ARE 1:1, AND WHAT THAT DOES AND DOES NOT GUARANTEE
// =========================================================================================
// `id = SafetyObstacle::id + id_offset`, unchanged otherwise (doc §17: "track ids map 1:1",
// because the filter's top-k selection uses them for hysteresis and the visualiser colours by
// constraint). uint32 -> int is a widening no-op for every id the tracker can mint inside
// `tracking.max_tracks`.
//
// That 1:1 mapping is a property of THIS FUNCTION, not of the pipeline feeding it. The tracker
// changes ids in two places and neither is visible from here:
//
//   FUSION (N tracks -> 1 measurement). The merged track inherits the id of its most confident
//   member; the other N-1 ids simply stop appearing. From the QP's side an obstacle silently
//   ceases to exist, with no event to observe. This is upstream's over-segmentation-recovery
//   behaviour with ids added, and it is the deliberate choice recorded in kf_circle_tracker.cpp
//   ("the merged track inherits the id ... of its most confident member").
//
//   FISSION (1 track -> N measurements). The lowest-cost observation continues the original id;
//   the other N-1 are minted FRESH ids AND reset to tentative with hits = 0. Because the safety
//   stage emits confirmed tracks only, those heirs are invisible here until they confirm on
//   their own evidence, `tracking.confirm_hits` scans later - so a fission costs at most one id
//   switch at the QP boundary, and the new constraint arrives late rather than untrustworthy.
//
// Both are asserted end-to-end in perception_dpcbf_adapter_test rather than assumed from the
// simple case.
//
// =========================================================================================
// THE CAP: THE ORACLE-PATH CONCLUSION DOES NOT TRANSFER, AND IS NOT ASSUMED TO
// =========================================================================================
// P3 established that applying `dpcbf_adapter.max_obstacles` (20) on the ORACLE path would drop
// 70 of 90 obstacles and change DpcbfSafetyFilter's output on 30/30 probed samples, which is
// why the oracle conversion is total and uncapped. That measurement says nothing about this
// path: the detector's `min_group_points` and its range-dependent misses mean the estimated
// obstacle count per frame is a different quantity entirely, and the P10 safety stage has
// already applied its own `safety.max_obstacles` (32) cull upstream of here. The cap's actual
// effect on the estimated path is measured in the adapter test, not inherited.
//
// WHICH ONES SURVIVE when it does bind: the leading `max_obstacles` in the safety stage's own
// emission order. The adapter deliberately does NOT re-rank. P10 already owns the cull policy
// (freshest, then largest inflated radius, then smallest id) and applied it at `safety.
// max_obstacles`; a second, differently-keyed ranking here would mean two modules deciding
// which hazards DPCBF may see, and the one that binds would depend on which limit was smaller.
//
// This module must never include a MuJoCo header (forbidden edge: dpcbf adapter -> MuJoCo).
#ifndef PERCEPTION_ADAPTERS_DPCBF_SAFETY_TO_DPCBF_H_
#define PERCEPTION_ADAPTERS_DPCBF_SAFETY_TO_DPCBF_H_

#include <vector>

#include "dpcbf/dpcbf_safety_filter.h"
#include "perception/adapters/dpcbf/adapter_params.h"
#include "perception/core/contracts/obstacles.h"

// As in oracle_to_dpcbf.h: the namespace mirrors the directory, so an unqualified `dpcbf::`
// would resolve to perception::adapters::dpcbf. Every reference to the library is `::dpcbf::`.
namespace perception::adapters::dpcbf {

// One obstacle. `index` is the obstacle's position in its frame's list, used only when
// `params.preserve_track_ids` is false; pass 0 for a standalone conversion.
//
// Total: no gating, no dropping, no clamping. Every gate this adapter has belongs to the
// vector form, because "drop" is a decision about a SET.
::dpcbf::ObstacleState ToDpcbfObstacleState(const core::SafetyObstacle& safety,
                                            const DpcbfAdapterParams& params, int index = 0);

// Vector form. ORDER-PRESERVING among survivors, for the same reason the oracle form is:
// DPCBF's constraint selection sorts by priority with distance as a tie-break, so a reordered
// list can change which of two equally-ranked obstacles becomes an active constraint. P3
// measured that selection to be order-invariant on its own corpus, which makes this
// defence-in-depth rather than presently load-bearing - preserved regardless.
//
// `out` is cleared and reserved, so a caller may reuse a buffer across calls. `stats` may be
// null. Returns nullptr on success or a STATIC reason string (invalid params).
const char* ToDpcbfObstacleStates(const std::vector<core::SafetyObstacle>& safety,
                                  const DpcbfAdapterParams& params,
                                  std::vector<::dpcbf::ObstacleState>& out,
                                  DpcbfAdapterStats* stats = nullptr);

}  // namespace perception::adapters::dpcbf

#endif  // PERCEPTION_ADAPTERS_DPCBF_SAFETY_TO_DPCBF_H_
