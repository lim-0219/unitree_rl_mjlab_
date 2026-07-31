// SafetyStateGenerator - validity gating and conservative inflation, the last core stage
// before the DPCBF adapter.
//
// =========================================================================================
// WHAT "fitted" AND "enclosing" ACTUALLY ARE AT THIS SEAM (architecture doc section 6)
// =========================================================================================
// The doc's formula is `radius <- max(fitted, enclosing) + k_sigma*sigma_r + latency*|v|*k_lat`.
// Both named quantities have to be located in the shipped code before that can be written down,
// and neither is where the formula implies.
//
//   `enclosing`  is `CircleObservation::radius_enclosing_m`, a DETECTION-stage field. It does
//                not survive tracking: neither TrackState2D nor PerceptionObstacle carries it,
//                so it is not available at this seam at all. Plumbing it through was rejected
//                on evidence rather than on cost - see the measurement below.
//
//   `fitted`     is `PerceptionObstacle::radius_true_m`, which is `track.r.value()`, which is
//                the Kalman filter driven by `CircleObservation::radius_fitted_m` - and that
//                observation is `0.5773502 * chord + detection.radius_enlargement_m`. The
//                enlargement (0.17 m as shipped) is INSIDE it. `PerceptionObstacle::radius_true_m`
//                is therefore upstream's `radius`, NOT upstream's `true_radius`, despite the
//                comment on the contract saying otherwise (see the correction in obstacles.h).
//
// THE MEASUREMENT THAT SETTLES OPEN QUESTION Q11. Over the whole P8 detection corpus (20 fitted
// circles, 6 scenes, full and half arc), `radius_enclosing_m` is smaller than `radius_fitted_m`
// in 20 cases out of 20, by 0.174 m to 0.207 m. It is never once the larger of the two. The
// reason is geometric and not incidental: the enclosing radius is measured from the SQRT(3)/3
// CENTRE, which sits behind the visible arc by construction (P8 finding 1 measured that offset at
// 0.09-0.19 m), and only the sensor-facing arc is ever hit - so the farthest contributing point
// is barely more than the true radius away from a centre that is already pulled toward the
// sensor, while the fitted radius carries a flat +0.17 m on top of a chord-derived estimate.
//
// (P10 measured this gap at 0.254-0.287 m when the enlargement was 0.25 m. Re-deriving the
// enlargement to 0.17 m moves the gap by exactly -0.08 m and nothing else: the enclosing radius
// is the farthest contributing point from `circle.center`, and neither that centre nor those
// points depend on the enlargement, so the whole spread translates rather than reshaping.)
//
// Two consequences, both load-bearing:
//   * `max(fitted, enclosing)` would be inert even if the enclosing radius were plumbed through
//     the tracker - it selects `fitted` on 20 samples out of 20, with 0.174 m of margin at the
//     narrowest. Adding a fourth filtered channel to TrackState2D and PerceptionObstacle to
//     carry a term that provably never binds was rejected on that evidence. `SafetyParams::
//     use_enclosing_radius` is retained because the config key is frozen, and is a documented
//     no-op with a test that asserts both settings produce byte-identical output.
//   * the architecture doc section 17's standing recommendation - "conservative enclosing over
//     fitted" - is WRONG for this detector, and following it would under-estimate every radius
//     by 0.17-0.21 m, which is the doc's own definition of the one unrecoverable error. It is
//     recorded here rather than only in the phase report because the recommendation is still
//     written down in the doc a future reader will consult.
//
// =========================================================================================
// WHY DE-ENLARGING IS FORBIDDEN HERE
// =========================================================================================
// It is tempting to subtract `detection.radius_enlargement_m` back off, on the grounds that
// `radius_true_m` is supposed to be the true radius. Do not. P8 measured the DE-ENLARGED radius
// to be biased SMALL by up to 0.161 m on short arcs, and the Kalman filter carries that bias
// through to 0.169 m at this seam (measured on P9's corpus). The enlargement is the only term in
// the whole pipeline that covers it, and this stage has no independent evidence with which to
// replace it. Subtracting a constant that happens to be covering a real bias is the exact shape
// of the error the safety-dominance policy exists to forbid.
//
// The consequence is a genuine cross-stage coupling: safety containment depends on a DETECTION
// parameter. It is enforced in PerceptionConfig::Validate() rather than left as a comment.
//
// =========================================================================================
// THE INFLATION TERMS, AND WHICH ONE COVERS WHAT
// =========================================================================================
// The doc's formula has one uncertainty term, `k_sigma * sigma_r`. That is not enough, and the
// gap is not small: on P9's corpus the worst CENTRE error is 0.204 m against a worst RADIUS
// error of 0.169 m, and a safety circle has to contain the true circle, which means covering
// both. The contract already anticipates this - SafetyObstacle::position_inflation_m exists and
// is documented as "isotropic positional margin already folded into radius_inflated_m" - and
// section 12 asks for "position/velocity uncertainty coverage calibrated", not radius coverage
// alone. So the implemented rule is:
//
//   base    = max(radius_true_m, min_radius_m)                     // Q13; see below
//   sigma_r = sqrt(radius_variance)                                // 1-D, the radius channel
//   sigma_p = sqrt(max(center_variance.x, center_variance.y))      // isotropic bound, 2-D
//   position_inflation = k_sigma * sigma_p + horizon_s * |v|
//   radius_inflated    = base + k_sigma * sigma_r + fixed + position_inflation
//
// and then the floor and the ceiling. `position_inflation_m` is reported separately, exactly as
// the contract asks, so the evaluator can attribute inflation to its terms.
//
// ONE k FOR TWO CHANNELS. `safety.radius_inflation_k_sigma` is a single number applied to a 1-D
// quantity (radius) and a 2-D one (centre). The correct quantiles differ - for 99.9% coverage a
// 1-D one-sided normal wants 3.09 and a 2-D chi-square wants sqrt(chi2_2(0.999)) = 3.72 - so the
// single k is calibrated against the STRICTER of the two and over-covers the radius channel.
// Over-covering is the safe direction and it avoids adding a config key whose only job would be
// to let someone set the position margin too low.
//
// =========================================================================================
// THE HORIZON: WHY IT IS `age_s + latency_inflation_s` AND NOT `latency_inflation_s`
// =========================================================================================
// The doc's `latency * |v|` covers the interval between this stage producing a state and DPCBF
// consuming it. It does NOT cover the interval that has already elapsed, and on a coasting track
// that interval is the larger of the two: `safety.max_age_s` is 0.30 s and obstacles reach
// 0.8 m/s, so a track admitted at the staleness limit has already drifted up to 0.24 m past
// where its centre was computed - more than the 0.12 m the fixed latency term covers.
//
// Using `age_s` as the elapsed part is deliberately an OVER-estimate. `age_s` is measured from
// `PerceptionObstacle::last_update_stamp_s`, which P9 defines as the last MATCHED MEASUREMENT
// time, while the published centre is the filter's estimate at the last SCAN time, which is at
// or after it. So the true elapsed extrapolation is <= age_s, and inflating on age_s is
// conservative by construction rather than by tuning. In the nominal case - a confirmed track,
// generated in the same frame as the scan - age_s is 0 and the term degenerates to the doc's.
//
// `k_lat` from the doc's formula has no config counterpart; `safety.latency_inflation_s` IS the
// coefficient (a time), and k_lat is folded into it. Stated because the doc names two symbols.
//
// =========================================================================================
// Q13 - THE SHORT-ARC RADIUS FLOOR
// =========================================================================================
// The floor is applied to the BASE radius, before inflation, not to the inflated result: it
// answers "how small may an object be believed to be", which is a statement about the estimate,
// and clamping after inflation would let the floor be satisfied by margin that is there for a
// different reason. `safety.min_radius_m` is 0.20 m, the minimum of dpcbf_config.yaml's
// `radius_range`, i.e. the smallest object that exists in this arena.
//
// It is a DEGENERATE-FIT GUARD, and it is NOT the answer to the short-arc bias. See the phase
// report for the numbers; the short version is that the floor never binds on any measured
// sample (the tracked radius carries the +0.17 m enlargement, so it measures 0.2886-0.4913 m
// where the floor is 0.20 m), while the short-arc bias is 0.161 m worst case and is covered
// entirely by that same enlargement. Keeping the floor costs nothing and catches a radius filter
// driven toward zero; claiming it answers Q13 would be false.
//
// The margin over the floor shrank when the enlargement was re-derived from 0.25 m to 0.17 m -
// P10 recorded the range as 0.36-0.55 m - and it is now 0.0886 m at the narrowest. Still 0 of
// 171 emissions, and the test prints the measured range and gates on it rather than trusting
// this comment's copy, because the next reduction is the one that could turn the floor into an
// active clamp.
//
// =========================================================================================
// VELOCITY SPIKE CLAMPING (risk R10's mitigation, architecture doc section 17)
// =========================================================================================
// Belongs here and not in the tracker: the tracker's job is to report what its filter believes,
// and a rate state that has been quietly clipped is an estimator whose NIS statistics no longer
// mean anything (P9 relies on those). The safety stage is where a belief becomes a number DPCBF
// acts on, so it is where an implausible belief gets bounded.
//
// The clamp preserves DIRECTION and scales the magnitude to `safety.max_speed_mps`. Dropping the
// obstacle instead would be less safe - a spike is evidence of a real object being tracked
// badly, not evidence of no object.
//
// THE INFLATION USES THE UNCLAMPED SPEED. The clamp bounds what DPCBF integrates; the inflation
// bounds where the object might actually be. Those are different questions and the conservative
// answer to the second one is the raw magnitude. The ceiling below keeps that bounded.
#ifndef PERCEPTION_CORE_SAFETY_SAFETY_STATE_GENERATOR_H_
#define PERCEPTION_CORE_SAFETY_SAFETY_STATE_GENERATOR_H_

#include <cstdint>
#include <vector>

#include "perception/core/contracts/obstacles.h"
#include "perception/core/contracts/tracking_2d.h"
#include "perception/interfaces/i_tracker_2d.h"

namespace perception::core {

// The safety stage's resolved parameters.
//
// WHY THIS IS NOT `SafetyConfig`: the same reason SegmentCircleDetectorParams is not
// DetectionConfig and TrackingParams is not TrackingConfig - `perception_core` links
// `perception_contracts` and nothing else, and that link set is what enforces the core
// dependency rules. The one-way mapping lives in perception/integration/safety_params.h.
struct SafetyParams {
  // ---- GATING ---------------------------------------------------------------------------
  // Oldest a state may be, measured as `now - PerceptionObstacle::last_update_stamp_s`, i.e.
  // against the last MATCHED MEASUREMENT and not against a predicted-to frame time.
  double max_age_s = 0.30;

  // A track must have existed this long AND accumulated this many matched scans.
  double min_track_age_s = 0.20;
  int min_track_hits = 3;

  // ---- INFLATION ------------------------------------------------------------------------
  double radius_inflation_k_sigma = 2.0;
  double radius_inflation_fixed_m = 0.08;
  double latency_inflation_s = 0.15;

  // max(fitted, enclosing) rather than fitted alone. A DOCUMENTED NO-OP on the shipped
  // pipeline: PerceptionObstacle carries no enclosing radius, and the measurement in the header
  // essay is why one was not added. Retained because the config key is frozen and because a
  // future detector could make the term bind; asserted inert by test.
  bool use_enclosing_radius = true;

  // ---- BOUNDS ---------------------------------------------------------------------------
  double min_radius_m = 0.20;
  double max_radius_m = 1.00;
  double max_speed_mps = 1.60;
  int max_obstacles = 32;

  // Returns nullptr when the parameter set is self-consistent, or a STATIC reason string.
  // Duplicated from the config validation deliberately, for the same reason TrackingParams
  // duplicates its own: a params struct built in a test has no other gate.
  const char* Validate() const;
};

// Per-frame census. Same principle as DetectionStats and TrackingStats: every number is a count
// of a decision this stage made, so the section-12 safety gates and the section-16 fallback
// trigger have a source in the code rather than in a test's private bookkeeping.
struct SafetyStats {
  int32_t input_obstacles = 0;
  int32_t input_tracks = 0;

  // Live tracks the tracker declined to publish as obstacles, i.e. still tentative. Counted
  // because "the safety stage emits nothing for a not-yet-confirmed object" is a deliberate
  // decision (see the class comment) and a decision with no counter is an assumption.
  int32_t tentative_suppressed = 0;

  // Drop reasons, kept apart because they answer different questions: a staleness drop is a
  // timing fault, an age/hits drop is the confirmation gate, and an invalid-output drop is a
  // contract breach. Collapsing them would make the fallback trigger undiagnosable.
  int32_t dropped_invalid_input = 0;
  int32_t dropped_no_track = 0;       // No TrackState2D with this id: hits are unknowable.
  int32_t dropped_future_stamp = 0;   // stamp > now: a clock fault, not a fresh obstacle.
  int32_t dropped_stale = 0;
  int32_t dropped_young_age = 0;
  int32_t dropped_few_hits = 0;
  int32_t dropped_capacity = 0;
  int32_t dropped_invalid_output = 0;

  int32_t velocity_clamped = 0;  // Spike clamp fired - risk R10.
  int32_t radius_floored = 0;    // min_radius_m bound the BASE radius.
  int32_t radius_ceiled = 0;     // max_radius_m bound the INFLATED radius.

  int32_t emitted = 0;

  double worst_age_s = 0.0;
  double worst_inflation_m = 0.0;  // Largest (radius_inflated_m - radius_true_m) emitted.
  double wall_time_us = 0.0;

  // Every input obstacle is accounted for exactly once, the same invariant ScanStats::IsBalanced
  // asserts over rays. A stage that silently loses an obstacle is the failure this catches.
  bool IsBalanced() const {
    return input_obstacles == dropped_invalid_input + dropped_no_track + dropped_future_stamp +
                                  dropped_stale + dropped_young_age + dropped_few_hits +
                                  dropped_capacity + dropped_invalid_output + emitted;
  }
};

struct SafetyStateResult {
  std::vector<SafetyObstacle> obstacles;
  SafetyStats stats;

  void Clear() {
    obstacles.clear();
    stats = SafetyStats{};
  }
};

// Stateless between frames, deliberately: everything with memory lives in the tracker, and a
// safety stage that remembered anything could disagree with the estimator about what is being
// tracked. It is a class rather than a free function only so the params and the scratch buffer
// have somewhere to live.
//
// TENTATIVE TRACKS ARE NOT EMITTED, and that is a decision, not an omission. The tracker
// confirms on `tracking.confirm_hits` CONSECUTIVE scans and a tentative track dies on its first
// miss, so an object that flickers on its first three scans never confirms and never appears
// here. Emitting something for it anyway was considered and rejected:
//
//   * A one-scan track has a velocity of exactly zero (the birth prior) and a radius from a
//     single unvalidated fit. Publishing it feeds DPCBF a stationary constraint at a location
//     that may be a detector artefact, and a spurious constraint is not a free safety win - it
//     can push the QP toward infeasibility or steer the robot into a real hazard to avoid an
//     imaginary one. The failure is in a different direction, not absent.
//   * The system's designed net for "something is out there and perception has not caught up"
//     is the per-frame fallback of architecture doc section 16, which switches the WHOLE frame
//     to the oracle rather than mixing a half-trusted estimate into a trusted set. Mixing is
//     exactly what that section forbids.
//   * The stress case - a genuinely new obstacle whose first appearance lands in the +/-pi seam
//     hole - is measured rather than reasoned about, in the P10 seam-birth scenario test.
//
// `tentative_suppressed` counts them, so the decision stays visible in the diagnostics.
class SafetyStateGenerator {
 public:
  explicit SafetyStateGenerator(const SafetyParams& params);

  // Gates and inflates `tracking.obstacles` into `out`. `now_s` is the sim time the safety
  // state is being generated for, and is what staleness is measured against; it must be finite
  // and is normally the scan stamp the tracker was just updated with.
  //
  // Returns nullptr on success or a STATIC reason string - the non-allocating convention the
  // contracts, the projector, the detector and the tracker already use.
  //
  // `out` is caller-owned and is cleared on entry; passing the same object back on every call
  // is the intended use and is what keeps the steady state allocation-free.
  const char* Generate(const Tracking2DResult& tracking, double now_s, SafetyStateResult* out);

  const SafetyParams& params() const { return params_; }

 private:
  // The hits count for `id`, or -1 when no live track carries it. Linear: max_tracks is 64 and
  // this runs at the scan rate, so an index would buy nothing and cost a second thing to keep
  // in step with the tracker's output.
  static int32_t HitsOf(const std::vector<TrackState2D>& tracks, uint32_t id);

  SafetyParams params_;

  // Scratch for the capacity cull, reserved in the constructor. A per-frame std::vector here
  // would be exactly the hidden allocation risk R14 exists to forbid.
  std::vector<int32_t> order_;
};

}  // namespace perception::core

#endif  // PERCEPTION_CORE_SAFETY_SAFETY_STATE_GENERATOR_H_
