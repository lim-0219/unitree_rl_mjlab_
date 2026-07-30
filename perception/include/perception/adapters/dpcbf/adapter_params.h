// The DPCBF adapter's resolved parameters, and the census of what it did with them.
//
// WHY THIS IS NOT `DpcbfAdapterConfig`: the same rule SegmentCircleDetectorParams,
// TrackingParams and SafetyParams already follow. `perception_dpcbf_adapter` links
// `perception_contracts` plus a headers-only dpcbf include path and NOTHING else - no
// yaml-cpp, no MuJoCo - and that link set is what enforces the dependency rules. The one-way
// mapping from the config tree lives in perception/integration/dpcbf_adapter_params.h.
//
// This header names no dpcbf type at all, deliberately: it is the half of the adapter's
// interface that `integration` is allowed to see without reaching a dpcbf header (doc §8).
#ifndef PERCEPTION_ADAPTERS_DPCBF_ADAPTER_PARAMS_H_
#define PERCEPTION_ADAPTERS_DPCBF_ADAPTER_PARAMS_H_

#include <cstdint>

namespace perception::adapters::dpcbf {

struct DpcbfAdapterParams {
  // Track ids pass through unchanged. `false` is NOT a no-op and NOT an error: the emitted id
  // becomes the obstacle's zero-based position in the frame's list. That is the only other
  // thing an id could mean here, and it is implemented rather than rejected because - unlike
  // SafetyParams::use_enclosing_radius, whose second operand does not exist at its seam - this
  // one is fully expressible. It is also measurably worse for its stated purpose: a positional
  // id is not stable across frames, so the filter's top-k hysteresis and the visualiser's
  // per-constraint colouring both flicker. Shipped default is true; the test asserts the
  // difference rather than leaving the key's meaning to a reader's guess.
  bool preserve_track_ids = true;

  // Added to every emitted id. Exists so the oracle id space (DynamicObstacleManager indices,
  // 0-based) and the estimated one (TrackState2D ids, 1-based and monotonically increasing)
  // can be told apart in a log that carries both.
  int id_offset = 0;

  // Upper bound on the states handed to Filter(). See safety_to_dpcbf.h for what this cap
  // actually does on the ESTIMATED path, which is not what it does on the oracle path.
  int max_obstacles = 20;

  // Omit obstacles whose Validate() fails. Belt and braces on the estimated path: the P10
  // safety stage DROPS rather than marks, so `valid` is true on everything it emits and this
  // pass has nothing to find there. It is retained because the adapter is a boundary, and a
  // boundary that trusts its input is not a boundary.
  bool drop_invalid = true;

  // nullptr when self-consistent, or a STATIC reason string - the non-allocating convention
  // the contracts, the projector, the detector, the tracker and the safety stage all use.
  const char* Validate() const;
};

// Per-frame census. Same principle as DetectionStats, TrackingStats and SafetyStats: every
// number is a count of a decision this module made, so a claim about truncation has a source
// in the code rather than in a test's private bookkeeping.
struct DpcbfAdapterStats {
  int32_t input = 0;
  int32_t dropped_invalid = 0;
  int32_t dropped_capacity = 0;
  int32_t emitted = 0;

  // Every input is accounted for exactly once. A transport that silently loses an obstacle on
  // its way to the QP is precisely the failure this catches.
  bool IsBalanced() const { return input == dropped_invalid + dropped_capacity + emitted; }
};

}  // namespace perception::adapters::dpcbf

#endif  // PERCEPTION_ADAPTERS_DPCBF_ADAPTER_PARAMS_H_
