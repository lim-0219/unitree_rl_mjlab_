// ITracker2D - the tracking-stage seam.
//
// One scan's world-frame CircleObservations in, the multi-frame track set and the
// confirmed-obstacle view out. The interface exists for the same reason IDetector2D does: the
// association and filtering strategy is expected to be replaced (open question Q19 upgrades the
// three per-axis filters to a joint 2-D filter with cross-covariance, which is a different
// estimator behind the same seam), and the pipeline must not learn about it.
//
// STATE ACROSS CALLS, DELIBERATELY. This is the FIRST stage in the pipeline that remembers
// anything: IDetector2D's contract forbids cross-frame memory precisely so that all of it lands
// here. Everything upstream of this seam is a pure function of one scan; everything at or below
// it has history. That is why Reset() exists and why Update() takes an explicit stamp.
//
// MEASUREMENT-DRIVEN, NOT TIMER-DRIVEN. Upstream advances its filters from a ROS timer at
// `loop_rate` (100 Hz) decoupled from measurement arrival (obstacle_tracker.cpp:118-121,
// 456-463). This interface has no tick: time advances only when Update() is called, and the
// step size is `stamp_s` minus the previous call's stamp. See kf_circle_tracker.h for why that
// replacement is required rather than preferred (architecture doc risk R11).
#ifndef PERCEPTION_INTERFACES_I_TRACKER_2D_H_
#define PERCEPTION_INTERFACES_I_TRACKER_2D_H_

#include <cstdint>
#include <vector>

#include "perception/core/contracts/detection_2d.h"
#include "perception/core/contracts/obstacles.h"
#include "perception/core/contracts/tracking_2d.h"

namespace perception::core {

// Per-scan census. Same principle as DetectionStats: every number is a count of a decision the
// tracker made, so the section-12 tracking gates ("ID switches", "stale-output rate",
// "deletion delay") have a source in the code rather than in a test's private bookkeeping.
struct TrackingStats {
  int32_t input_observations = 0;
  int32_t observations_rejected_invalid = 0;  // Failed Validate() or arrived in the wrong frame.

  int32_t matched = 0;                 // Observations that corrected an existing track.
  int32_t unmatched_observations = 0;  // Observations that seeded a new tentative track.
  int32_t unmatched_tracks = 0;        // Live tracks that ran this scan without a measurement.

  // Observations whose nearest track had already been taken by a fusion, a fission or an
  // earlier plain match. Upstream discards these silently (obstacle_tracker.cpp:208); they are
  // counted here because a non-zero value means the association is contested, which is the
  // leading indicator of the id switches the section-12 gate bounds.
  int32_t observations_dropped_taken_track = 0;

  // Fusion (N tracks -> 1 measurement) and fission (1 track -> N measurements), ported from
  // obstacle_tracker.cpp:156-198. `consumed`/`created` are counted separately from the events
  // because one fusion event retires a variable number of tracks.
  int32_t fusions = 0;
  int32_t fusion_tracks_consumed = 0;
  int32_t fissions = 0;
  int32_t fission_tracks_created = 0;

  int32_t tracks_created = 0;
  int32_t tracks_confirmed = 0;  // Tentative -> confirmed transitions THIS scan.

  // Deletion reasons are counted apart because they answer different questions: a miss-count
  // deletion is an association failure, a coast-timeout deletion is a horizon expiry, and a
  // tentative-miss deletion is a rejected false alarm. Collapsing them into one counter would
  // make "deletion delay <= tracking_duration" unmeasurable.
  int32_t tracks_deleted_misses = 0;
  int32_t tracks_deleted_coast_timeout = 0;
  int32_t tracks_deleted_tentative_miss = 0;
  int32_t tracks_deleted_invalid_state = 0;
  int32_t tracks_dropped_capacity = 0;  // New tracks refused because max_tracks was reached.

  int32_t tentative = 0;  // Live counts AFTER this scan.
  int32_t confirmed = 0;
  int32_t coasting = 0;

  int32_t obstacles_emitted = 0;

  // NORMALIZED INNOVATION SQUARED, accumulated over every scalar measurement update this scan.
  // NIS = innovation^2 / (P(0,0) + R) per axis, which for a correctly tuned filter has
  // expectation 1.0 per scalar channel. This is in the production stats rather than only in a
  // test because the measurement sigmas it validates are a CALIBRATED input (see
  // TrackingParams::measurement_sigma_scale): a number that was tuned against innovation
  // statistics needs those statistics to stay observable, or the next config change silently
  // invalidates the calibration.
  //
  //   NIS >> 1  the filter is OVER-confident: R and/or Q are too small for the real errors.
  //   NIS << 1  the filter is UNDER-confident: it is ignoring good measurements.
  double nis_sum = 0.0;
  int32_t nis_samples = 0;

  double MeanNis() const { return nis_samples > 0 ? nis_sum / nis_samples : 0.0; }

  bool capacity_exceeded = false;
  double wall_time_us = 0.0;
};

struct Tracking2DResult {
  // Every LIVE track, whatever its status - the estimator's full view, for dumps and
  // visualization.
  std::vector<TrackState2D> tracks;

  // The subset the safety stage consumes: confirmed and coasting tracks only. Tentative
  // tracks never appear here, which is the whole purpose of the confirmation gate.
  std::vector<PerceptionObstacle> obstacles;

  TrackingStats stats;

  void Clear() {
    tracks.clear();
    obstacles.clear();
    stats = TrackingStats{};
  }
};

class ITracker2D {
 public:
  virtual ~ITracker2D() = default;

  // Advances every track to `stamp_s`, associates `observations` against them, and fills
  // `out`. Returns nullptr on success or a STATIC reason string - the non-allocating
  // convention the contracts, the projector and the detector already use.
  //
  // `stamp_s` is passed SEPARATELY from the observations rather than derived from them,
  // because it is the scan's time and must exist even when the scan produced no observation
  // at all: a scan with zero detections still ages every track, and a tracker that could only
  // advance time when it had something to associate would never delete anything. It must be
  // non-decreasing across calls.
  //
  // `out` is caller-owned and is cleared on entry; passing the same object back on every call
  // is the intended use.
  virtual const char* Update(const std::vector<CircleObservation>& observations, double stamp_s,
                             Tracking2DResult* out) = 0;

  // Propagates every live track to `stamp_s` WITHOUT consuming a measurement and without
  // mutating the tracker. This is how a consumer running faster than the scan rate gets a
  // current position; it is const because a prediction must never become history.
  virtual const char* Predict(double stamp_s, std::vector<TrackPrediction2D>* out) const = 0;

  // Drops all tracking state. The id counter is reset too, so a Reset() tracker is
  // indistinguishable from a fresh one - which is what makes replayed dumps reproducible.
  virtual void Reset() = 0;
};

}  // namespace perception::core

#endif  // PERCEPTION_INTERFACES_I_TRACKER_2D_H_
