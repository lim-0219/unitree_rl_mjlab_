// KfCircleTracker - the ported obstacle_detector tracker, rescheduled onto measurement time.
//
// Grouping of what is PORTED and what is deliberately REPLACED, because this module has more of
// the second than any other in the subsystem:
//
//   PORTED, faithfully:
//     * three independent 2-state KFs per obstacle (x, y, radius) - see axis_kalman.h.
//     * the association cost: a Euclidean distance over (dx, dy, dr) in metres, gated by
//       `min_correspondence_cost`  (obstacle_tracker.cpp:257, 302).
//     * row-minimum and column-minimum index selection, each independently gated
//       (obstacle_tracker.cpp:289-353).
//     * fusion (N tracks -> 1 measurement, covariance-weighted merge) and fission
//       (1 track -> N measurements)  (obstacle_tracker.cpp:156-198, 405-454).
//
//   REPLACED, deliberately, each recorded below with the reason:
//     R11  the timer-driven update loop            -> measurement-driven dt
//     (a)  Q as a per-tick constant                -> Q scaled by dt
//     (b)  the fade counter                        -> tentative/confirmed/coasting lifecycle
//     (c)  the untracked-obstacle side list        -> tentative tracks in the one track list
//     (d)  P reset to identity on fusion           -> the merged covariance is kept
//     (e)  R as one global constant                -> per-observation sigmas from the detector
//
// =========================================================================================
// R11 - WHY THE TIMER MODEL CANNOT BE PORTED
// =========================================================================================
// Upstream runs two decoupled loops. Measurements arrive on `obstaclesCallback` and correct the
// filters; a ROS timer at `loop_rate` (default 100 Hz) calls `updateObstacles()`
// (obstacle_tracker.cpp:118-121, 456-463), which advances every non-faded track. The KF sampling
// time baked into A is `1 / loop_rate` (obstacle_tracker.cpp:73, tracked_obstacle.h:129-131) -
// the TIMER period, not the sensor period - so between two 10 Hz measurements a track is
// advanced roughly ten times.
//
// The first reason this does not transfer is determinism. How many timer ticks land between two
// measurements is a fact about wall-clock scheduling, so the estimate depends on OS scheduling
// jitter. This subsystem's regression fixtures, its replayable dumps and its "two runs, same
// seed, identical dumps" gate (architecture doc P12) all require that the same inputs produce
// the same numbers. A tick count that varies with load makes every one of those untestable.
//
// The second reason is stronger and is NOT in the risk register, because it is only visible by
// reading `updateState()` rather than its name. `TrackedObstacle::updateState()`
// (tracked_obstacle.h:87-105) calls predictState() and then correctState() on each filter - and
// `KalmanFilter::correctState()` reads `y`, the measurement vector, which is written ONLY by
// `TrackedObstacle::correctState(const CircleObstacle&)` (tracked_obstacle.h:67-71). So the
// timer does not coast a track forward on its constant-velocity model at all: it re-applies the
// LAST RECEIVED MEASUREMENT, roughly ten times per scan, at full Kalman gain. An unmatched
// upstream track is therefore dragged back onto its last observed position and its velocity
// estimate is driven toward zero - the opposite of coasting, and precisely the wrong behaviour
// for the occlusion-gap and seam-gap cases this phase has to survive. Reproducing it would
// reproduce a bug, not a model.
//
// The replacement: time advances only inside Update(), by `stamp_s` minus the previous call's
// stamp; an unmatched track is predicted and NOT corrected; and A carries that real dt.
//
// =========================================================================================
// (a) Q SCALED BY dt
// =========================================================================================
// Upstream's Q is a stored constant, diag(process_variance, process_rate_variance), applied once
// per timer tick (tracked_obstacle.h:141-147). It is never scaled by dt because upstream's dt is
// a compile-of-config constant. With a measurement-driven dt, applying a fixed Q once per scan
// instead of ten times per scan would silently divide the injected process noise by ten, and the
// filter's tuning would then depend on the scan rate - the same non-determinism R11 exists to
// remove, re-entering through the covariance.
//
// Q(dt) = diag(process_variance * dt, process_rate_variance * dt). The config values are
// therefore PER-SECOND DENSITIES, not per-tick variances, and that is a reinterpretation of
// upstream's numbers rather than a copy of them. Q stays DIAGONAL: the textbook
// continuous-acceleration form has an off-diagonal dt^2/2 term coupling value and rate, and
// adding it here would be a third change to the estimator on top of the two above, for a
// smoothness benefit this phase has no evidence it needs. Recorded as the obvious follow-up if
// the velocity gate ever becomes marginal.
//
// =========================================================================================
// (e) R FROM THE DETECTOR'S PER-OBSERVATION SIGMAS
// =========================================================================================
// Upstream uses one global `measurement_variance` for all three axes of every track
// (tracked_obstacle.h:137-139). P8's detector emits per-observation `sigma_center_m` and
// `sigma_radius_m` (detection_2d.h:173-174), so a short-arc observation can be told apart from a
// full-arc one - which is the whole point of having measured the short-arc bias. Those sigmas
// are P8's own PROVISIONAL numbers and are rescaled here; see
// TrackingParams::measurement_sigma_scale for the calibration and its evidence.
#ifndef PERCEPTION_CORE_TRACKING_KF_CIRCLE_TRACKER_H_
#define PERCEPTION_CORE_TRACKING_KF_CIRCLE_TRACKER_H_

#include <cstdint>
#include <vector>

#include "perception/core/tracking/axis_kalman.h"
#include "perception/interfaces/i_tracker_2d.h"

namespace perception::core {

// The tracker's resolved parameters.
//
// WHY THIS IS NOT `TrackingConfig`: the same reason SegmentCircleDetectorParams is not
// DetectionConfig - `perception_core` links `perception_contracts` and nothing else, and that
// link set is what enforces the core-dependency rules. The one-way mapping lives in
// perception/integration/tracking_params.h.
struct TrackingParams {
  // ---- PORTED, upstream's names; the numbers are noted where they are NOT upstream's ------

  // Process noise, AS PER-SECOND DENSITIES - see change (a) above. Units: m^2/s for the value
  // channel, (m/s)^2/s for the rate channel.
  //
  // UPSTREAM'S VALUES ARE NOT KEPT (0.01 and 0.10 per 100 Hz tick), and this is the only place
  // in the port where a transcribed number was overruled by measurement. A Kalman gain depends
  // on the Q/R RATIO, and change (e) replaces upstream's flat R of 1.00 m^2 with the detector's
  // per-observation sigmas, which on the P8 corpus run 0.07-0.22 m - a variance 20 to 200 times
  // smaller. Holding Q at upstream's magnitude across that substitution does not preserve
  // upstream's filter; it produces a different one in which process noise dominates the
  // measurement. The NIS check makes the consequence visible rather than leaving it to be
  // inferred, and the retuned values are justified line by line in configs/perception.yaml.
  double process_variance = 0.0001;
  double process_rate_variance = 0.03;

  // Upstream's global measurement variance, m^2. Retained with a narrowed role: it is the
  // FALLBACK R for an observation that reports a zero sigma. CircleObservation::Validate()
  // admits sigma == 0 (it only requires >= 0), and a zero R tells a Kalman filter the
  // measurement is perfect, which drives the gain to 1 and the covariance to 0 - a track that
  // can never again be corrected. So the field survives as the thing that stops that, rather
  // than being deleted for having been superseded.
  double measurement_variance = 1.00;

  // The association gate, a distance in the weighted (x, y, r) space below, in metres.
  double min_correspondence_cost_m = 0.30;

  // ---- CHOSEN HERE. The four numbers this phase is answerable for. ----------------------

  // ASSOCIATION RADIUS WEIGHT - the response to P8's finding 2.
  //
  // Upstream's cost is sqrt(dx^2 + dy^2 + dr^2): a metre of radius disagreement counts exactly
  // as much as a metre of position disagreement. Upstream could afford that because it never
  // separated full-arc from half-arc error. P8 did, and measured the fitted radius to be
  // systematically SMALL by 0.006 m to 0.145 m as a function of how much of the arc is visible.
  //
  // That makes the radius channel actively hostile to correct association. When an obstacle's
  // visible arc shrinks - the robot walking past a cylinder, so the near side occludes the far
  // side - the fitted radius steps down by up to 0.145 m between consecutive scans with no
  // change in the object at all. Worse, the same short arc also drives the centre estimate off
  // by up to 0.15 m (section 12's own half-arc bound), so the two error terms PEAK TOGETHER:
  // exactly when the position residual is largest, the radius residual is largest too, and
  // upstream's unweighted cost adds them in quadrature. sqrt(0.23^2 + 0.145^2) = 0.272 m
  // against a 0.30 m gate - a correct association surviving on 9% of margin.
  //
  // Down-weighting the radius term rather than widening a separate radius gate: the radius
  // channel is BIASED, not merely noisy, so the right response is to let it inform identity
  // LESS, not to keep its full vote and forgive larger disagreements. A weight also keeps one
  // threshold - upstream's `min_correspondence_cost` - instead of introducing a second one, and
  // w = 1.0 recovers upstream's cost exactly, which is what lets the regression suite pin the
  // port.
  //
  // 0.25 is set against the measured spread: a full 0.145 m visibility-driven radius step
  // contributes 0.036 m to the cost, under 12% of the gate, so it can no longer break a correct
  // association; while two genuinely different objects, which in this arena differ in radius by
  // a good fraction of the 0.20-1.00 m configured range, still contribute 0.05-0.20 m and
  // retain their discriminating power.
  double association_radius_weight = 0.25;

  // MEASUREMENT SIGMA CALIBRATION - the response to P8's provisional sigmas.
  //
  // P8 derives sigma from the circle fit's RMS residual reduced by sqrt(point_count), times a
  // 1/(1 - cos alpha) centre-dilution term, and called both the residual floor and the dilution
  // cap placeholders for this phase to calibrate against measured innovation statistics.
  //
  // RE-DERIVED after the fit_residual_m correction, and the original reading is recorded because
  // it was wrong in an instructive way. P9 first measured sigma_center_m at 0.145 m against a
  // centre-error RMS of 0.130 m - an implied NIS of 0.8 - and concluded that P8's formula was
  // endorsed at scale 1.0, remarking that the 1/(1 - cos alpha) dilution term "supplies most of
  // the missing magnitude". It did not. The magnitude came from a defect: fit_residual_m was
  // computed against the ENLARGED radius, so every sigma carried radius_enlargement_m as an
  // additive constant. The calibration was reading a configuration value and calling it a fit
  // statistic, and it landed near the right answer by coincidence. Corrected, sigma_center_m is
  // 0.024 m against the same 0.130 m error: an implied NIS of 28.5.
  //
  // THE SCALE IS STILL 1.0, NOW ON EVIDENCE RATHER THAN ON THAT COINCIDENCE. A NIS of 28.5 is
  // not proof of a mis-scaled R unless the error it is measured against is noise, and here it is
  // not. Decomposed along the sensor line of sight over the P8 corpus, the centre error is
  // +0.1106 m of systematic RADIAL offset - the fitted centre pulled toward the sensor, which is
  // the sqrt(3)/3 construction doing exactly what P8 finding 1 measured - with 0.0416 m of
  // radial scatter and 0.0504 m of tangential scatter beside it. 74.7% of the error POWER is
  // bias. Scaling R up until the NIS reads 1 would be absorbing a systematic bias into a
  // stochastic term, which is the thing PerceptionConfig::Validate()'s short-arc bias budget
  // exists to say cannot be done; and it was measured to make matters worse rather than better.
  // At scale 2.0 and above, on the synthetic corpus whose sigmas are already honest, position
  // RMSE degrades from 0.062 m to 0.107 m and ID switches go from 0 to 5-9. The bias is covered
  // by the two FLAT terms that constraint enforces, which is where a bias can be covered.
  //
  // `measurement_sigma_floor_m` is NOT a scale correction. It is a guard against a degenerate
  // sigma: CircleObservation::Validate() admits sigma == 0, and a zero R drives the Kalman gain
  // to 1 and the covariance to 0, leaving a track that can never be corrected again. The
  // property that makes it a guard rather than a tuning parameter is that it sits below
  // everything the real detector emits - so it had to move with the residual fix. Post-fix the
  // detector's sigma_center_m spans 0.0144-0.0334 m over 190 circles, where the old 0.030 m
  // floor would have bound on nearly all of them; 0.005 m restores the guard.
  //
  // The calibration is applied HERE rather than by editing P8's two provisional constants on
  // purpose: P8's detector output, its committed golden fixtures and its published bias figures
  // all stay exactly as measured, and the tuned numbers live next to the innovation statistics
  // that justify them. (The residual fix itself is a different kind of change - a defect in what
  // the number MEANS, not a disagreement about its scale - which is why it belongs in P8's file
  // and this does not.)
  double measurement_sigma_scale = 1.0;
  double measurement_sigma_floor_m = 0.005;

  // Prior on the rate state of a newborn track, for both velocity and radius rate.
  // (m/s)^2. Obstacles in this arena top out at 0.8 m/s (dpcbf_config.yaml `speed_range`), so a
  // zero-mean prior with a 0.8 m/s standard deviation covers the whole reachable range and
  // nothing more - 0.8^2 = 0.64. Upstream's implicit 1.0 (kalman.h:53) is close by accident;
  // this one is close on purpose, which matters because it is also the number that decides how
  // fast a new track's velocity converges.
  double initial_rate_variance = 0.64;

  // ---- LIFECYCLE. New with the measurement-driven model - change (b). -------------------
  int confirm_hits = 3;
  int delete_misses = 10;
  double max_coast_s = 2.0;

  bool enable_fusion = true;
  bool enable_fission = true;

  int max_tracks = 64;

  // Returns nullptr when the parameter set is self-consistent, or a STATIC reason string.
  // Duplicated from the config validation deliberately: a params struct built in a test, or by
  // a future caller that does not go through PerceptionConfig, has no other gate.
  const char* Validate() const;
};

class KfCircleTracker : public ITracker2D {
 public:
  explicit KfCircleTracker(const TrackingParams& params);

  const char* Update(const std::vector<CircleObservation>& observations, double stamp_s,
                     Tracking2DResult* out) override;
  const char* Predict(double stamp_s, std::vector<TrackPrediction2D>* out) const override;
  void Reset() override;

  const TrackingParams& params() const { return params_; }
  int live_track_count() const { return static_cast<int>(tracks_.size()); }

 private:
  // One tracked obstacle. Upstream's TrackedObstacle plus the lifecycle counters, minus the
  // fade counter and minus the embedded CircleObstacle message - the filters ARE the state, and
  // keeping a second copy of the estimate next to them (as upstream does, tracked_obstacle.h:164)
  // is what lets the two disagree.
  struct Track {
    uint32_t id = 0;
    AxisKalman2 x;
    AxisKalman2 y;
    AxisKalman2 r;

    int32_t hits = 0;
    int32_t misses = 0;
    int32_t consecutive_misses = 0;
    TrackStatus status = TrackStatus::kTentative;

    double created_stamp_s = 0.0;
    double last_update_stamp_s = 0.0;
    double last_measurement_stamp_s = 0.0;
  };

  // The measurement variances one observation implies, after the sigma calibration.
  struct ObservationNoise {
    double center = 0.0;
    double radius = 0.0;
  };

  ObservationNoise NoiseOf(const CircleObservation& observation) const;

  // The ported cost, with the radius term weighted. `track` must already be predicted to the
  // measurement time - associating against a stale state is upstream's mistake, not its design.
  double Cost(const CircleObservation& observation, const Track& track) const;

  void Seed(const CircleObservation& observation, double stamp_s, TrackingStats* stats);
  void CorrectTrack(Track* track, const CircleObservation& observation, double stamp_s,
                    TrackingStats* stats);

  void FuseTracks(const std::vector<int>& indices, const CircleObservation& observation,
                  double stamp_s, std::vector<Track>* created, TrackingStats* stats);

  TrackState2D StateOf(const Track& track) const;
  PerceptionObstacle ObstacleOf(const Track& track) const;

  TrackingParams params_;
  std::vector<Track> tracks_;
  uint32_t next_id_ = 1;  // 0 is reserved by the contract for "unassigned".

  bool have_stamp_ = false;
  double last_stamp_s_ = 0.0;

  // Scratch, ALL of it reused across calls, because the architecture requires an allocation-free
  // steady state on the perception thread and a per-frame std::vector is exactly the hidden
  // allocation that rule exists to forbid (risk R14). Sized in the constructor from max_tracks;
  // the observation-dependent dimensions grow on demand and then keep their capacity, so
  // `assign`/`clear` reuse the buffer rather than reallocating.
  //
  // `disposition_` is a per-track enum for the current call and NOT part of the track state - a
  // track's disposition is meaningless between calls, which is why it lives here rather than in
  // Track where it would be one more field to keep in step.
  std::vector<double> cost_matrix_;  // Row-major, N x T.
  std::vector<int> row_min_;         // Per observation: best track index, or -1.
  std::vector<int> col_min_;         // Per track: best observation index, or -1.
  std::vector<char> used_observation_;
  std::vector<uint8_t> disposition_;
  std::vector<const CircleObservation*> valid_;
  std::vector<Track> created_;
  std::vector<Track> survivors_;
  std::vector<int> group_;
};

}  // namespace perception::core

#endif  // PERCEPTION_CORE_TRACKING_KF_CIRCLE_TRACKER_H_
