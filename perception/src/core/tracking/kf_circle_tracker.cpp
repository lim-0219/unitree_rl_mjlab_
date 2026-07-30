#include "perception/core/tracking/kf_circle_tracker.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace perception::core {
namespace {

// Track disposition within one Update() call. Kept as an explicit enum rather than the two
// parallel `used_old`/`used_new` index vectors upstream searches linearly
// (obstacle_tracker.cpp:150-151, 355-403), because upstream's two vectors cannot distinguish
// "this track was consumed by a fusion" from "this track was corrected normally" - and that
// distinction is what decides whether the track is erased or aged, which is the one thing the
// lifecycle turns on.
enum class Disposition : uint8_t {
  kUntouched = 0,  // No measurement reached it: a miss.
  kCorrected,      // Matched an observation and was corrected in place.
  kConsumed,       // Retired by a fusion or replaced by a fission; `created_` holds its heirs.
};

}  // namespace

const char* TrackingParams::Validate() const {
  if (!IsFinitePositive(process_variance)) return "TrackingParams::process_variance must be > 0";
  if (!IsFinitePositive(process_rate_variance)) {
    return "TrackingParams::process_rate_variance must be > 0";
  }
  if (!IsFinitePositive(measurement_variance)) {
    return "TrackingParams::measurement_variance must be > 0";
  }
  if (!IsFinitePositive(min_correspondence_cost_m)) {
    return "TrackingParams::min_correspondence_cost_m must be > 0";
  }
  // A zero weight would delete the radius channel from the cost entirely, which is a different
  // association rule rather than a down-weighted one; a weight above 1 would give the biased
  // channel MORE say than upstream did, which is the opposite of the finding-2 response.
  if (!IsFinite(association_radius_weight) || association_radius_weight <= 0.0 ||
      association_radius_weight > 1.0) {
    return "TrackingParams::association_radius_weight must lie in (0, 1]";
  }
  if (!IsFinitePositive(measurement_sigma_scale)) {
    return "TrackingParams::measurement_sigma_scale must be > 0";
  }
  if (!IsFinitePositive(measurement_sigma_floor_m)) {
    return "TrackingParams::measurement_sigma_floor_m must be > 0";
  }
  if (!IsFinitePositive(initial_rate_variance)) {
    return "TrackingParams::initial_rate_variance must be > 0";
  }
  if (confirm_hits < 1) return "TrackingParams::confirm_hits must be >= 1";
  if (delete_misses < 1) return "TrackingParams::delete_misses must be >= 1";
  if (!IsFinitePositive(max_coast_s)) return "TrackingParams::max_coast_s must be > 0";
  if (max_tracks < 1) return "TrackingParams::max_tracks must be >= 1";
  return nullptr;
}

KfCircleTracker::KfCircleTracker(const TrackingParams& params) : params_(params) {
  const auto capacity = static_cast<size_t>(params_.max_tracks);
  tracks_.reserve(capacity);
  created_.reserve(capacity);
  survivors_.reserve(2 * capacity);  // Survivors plus the newly created, before the cap applies.
  col_min_.reserve(capacity);
  disposition_.reserve(capacity);
  group_.reserve(capacity);
  valid_.reserve(capacity);
}

void KfCircleTracker::Reset() {
  tracks_.clear();
  created_.clear();
  next_id_ = 1;
  have_stamp_ = false;
  last_stamp_s_ = 0.0;
}

KfCircleTracker::ObservationNoise KfCircleTracker::NoiseOf(
    const CircleObservation& observation) const {
  // The detector's sigma carries the right ARC DEPENDENCE and the wrong SCALE - see
  // TrackingParams::measurement_sigma_scale. A sigma of exactly zero means the detector declined
  // to produce one, so upstream's global variance stands in; anything positive is scaled and
  // floored.
  const double fallback = std::sqrt(params_.measurement_variance);

  const double raw_center = observation.sigma_center_m > 0.0 ? observation.sigma_center_m : fallback;
  const double raw_radius = observation.sigma_radius_m > 0.0 ? observation.sigma_radius_m : fallback;

  ObservationNoise noise;
  const double center =
      std::max(raw_center * params_.measurement_sigma_scale, params_.measurement_sigma_floor_m);
  const double radius =
      std::max(raw_radius * params_.measurement_sigma_scale, params_.measurement_sigma_floor_m);
  noise.center = center * center;
  noise.radius = radius * radius;
  return noise;
}

double KfCircleTracker::Cost(const CircleObservation& observation, const Track& track) const {
  const double dx = observation.center.x() - track.x.value();
  const double dy = observation.center.y() - track.y.value();
  // The one departure from obstacle_tracker.cpp:257, and the whole of P8's finding-2 response.
  // With association_radius_weight == 1.0 this expression IS upstream's.
  const double dr = params_.association_radius_weight *
                    (observation.radius_fitted_m - track.r.value());
  return std::sqrt(dx * dx + dy * dy + dr * dr);
}

void KfCircleTracker::Seed(const CircleObservation& observation, double stamp_s,
                           TrackingStats* stats) {
  if (static_cast<int>(tracks_.size() + created_.size()) >= params_.max_tracks) {
    ++stats->tracks_dropped_capacity;
    stats->capacity_exceeded = true;
    return;
  }

  const ObservationNoise noise = NoiseOf(observation);

  Track track;
  track.id = next_id_++;
  // Birth state: the measurement itself, a zero rate, and the measurement's own variance on the
  // value channel. See AxisKalman2::Initialize on why this diverges from upstream's P = I.
  track.x.Initialize(observation.center.x(), 0.0, noise.center, params_.initial_rate_variance);
  track.y.Initialize(observation.center.y(), 0.0, noise.center, params_.initial_rate_variance);
  track.r.Initialize(observation.radius_fitted_m, 0.0, noise.radius, params_.initial_rate_variance);

  track.hits = 1;
  track.misses = 0;
  track.consecutive_misses = 0;
  track.created_stamp_s = stamp_s;
  track.last_update_stamp_s = stamp_s;
  track.last_measurement_stamp_s = stamp_s;

  // A track can be born already confirmed only if confirm_hits == 1, which is a legal config.
  track.status =
      track.hits >= params_.confirm_hits ? TrackStatus::kConfirmed : TrackStatus::kTentative;
  if (track.status == TrackStatus::kConfirmed) ++stats->tracks_confirmed;

  created_.push_back(track);
  ++stats->tracks_created;
}

void KfCircleTracker::CorrectTrack(Track* track, const CircleObservation& observation,
                                   double stamp_s, TrackingStats* stats) {
  const ObservationNoise noise = NoiseOf(observation);

  const KalmanCorrection cx = track->x.Correct(observation.center.x(), noise.center);
  const KalmanCorrection cy = track->y.Correct(observation.center.y(), noise.center);
  const KalmanCorrection cr = track->r.Correct(observation.radius_fitted_m, noise.radius);

  stats->nis_sum += cx.NormalizedInnovationSquared() + cy.NormalizedInnovationSquared() +
                    cr.NormalizedInnovationSquared();
  stats->nis_samples += 3;

  ++track->hits;
  track->consecutive_misses = 0;
  track->last_measurement_stamp_s = stamp_s;
  track->last_update_stamp_s = stamp_s;

  const bool was_confirmed = track->status == TrackStatus::kConfirmed;
  if (track->hits >= params_.confirm_hits) {
    track->status = TrackStatus::kConfirmed;
    if (!was_confirmed) ++stats->tracks_confirmed;
  } else {
    track->status = TrackStatus::kTentative;
  }
}

void KfCircleTracker::FuseTracks(const std::vector<int>& indices,
                                 const CircleObservation& observation, double stamp_s,
                                 std::vector<Track>* created, TrackingStats* stats) {
  // The covariance-weighted merge of obstacle_tracker.cpp:405-441: each contributing estimate is
  // weighted by the reciprocal of its own variance, which is the maximum-likelihood combination
  // of independent Gaussian estimates.
  //
  // CHANGE (d). Upstream computes exactly this for the MEANS and then throws the covariance away:
  // `TrackedObstacle to(c)` (obstacle_tracker.cpp:435) runs the KalmanFilter constructor, which
  // sets P = eye (kalman.h:53). So a track fused from four confident estimates comes out claiming
  // 1.0 m^2 of variance - less certain than any of its parents, and no longer comparable to the
  // unfused tracks it now sits beside in the same cost matrix and the same fusion weighting next
  // scan. The merged information is the sum of the contributions, so the merged variance is its
  // reciprocal; that is kept here.
  struct AxisMerge {
    double weighted_value = 0.0;
    double weighted_rate = 0.0;
    double info_value = 0.0;
    double info_rate = 0.0;

    void Add(const AxisKalman2& kf) {
      const double vv = kf.value_variance();
      const double rv = kf.rate_variance();
      if (vv > 0.0) {
        weighted_value += kf.value() / vv;
        info_value += 1.0 / vv;
      }
      if (rv > 0.0) {
        weighted_rate += kf.rate() / rv;
        info_rate += 1.0 / rv;
      }
    }

    void Apply(AxisKalman2* kf) const {
      const double value = info_value > 0.0 ? weighted_value / info_value : kf->value();
      const double rate = info_rate > 0.0 ? weighted_rate / info_rate : kf->rate();
      kf->SetState(value, rate);
      // The off-diagonal is set to zero rather than merged: a value/rate cross-covariance is a
      // property of one estimate's update history, and the merged mean is a weighted average
      // ACROSS histories, so there is no cross term to carry over. Zero is the correct prior for
      // a quantity with no evidence, and the next Correct() regenerates it immediately.
      kf->SetCovariance(info_value > 0.0 ? 1.0 / info_value : 0.0, 0.0, 0.0,
                        info_rate > 0.0 ? 1.0 / info_rate : 0.0);
    }
  };

  AxisMerge merge_x;
  AxisMerge merge_y;
  AxisMerge merge_r;

  // IDENTITY ON FUSION. Upstream has no ids to preserve - `CircleObstacle` carries none, and the
  // fused obstacle is a brand-new object. This subsystem does have ids, and they are load-bearing
  // downstream: the DPCBF filter's top-k constraint selection uses them for hysteresis
  // (architecture doc P11), so a gratuitous id change flickers the constraint set. The merged
  // track therefore inherits the id and the accumulated evidence of its most confident member -
  // smallest positional variance - which is also the member contributing most of the merged mean.
  int donor = indices.front();
  double best_variance = -1.0;
  double earliest_created = 0.0;
  bool have_earliest = false;

  for (const int index : indices) {
    const Track& track = tracks_[static_cast<size_t>(index)];
    merge_x.Add(track.x);
    merge_y.Add(track.y);
    merge_r.Add(track.r);

    const double variance = track.x.value_variance() + track.y.value_variance();
    if (best_variance < 0.0 || variance < best_variance) {
      best_variance = variance;
      donor = index;
    }
    if (!have_earliest || track.created_stamp_s < earliest_created) {
      earliest_created = track.created_stamp_s;
      have_earliest = true;
    }
  }

  Track fused = tracks_[static_cast<size_t>(donor)];
  merge_x.Apply(&fused.x);
  merge_y.Apply(&fused.y);
  merge_r.Apply(&fused.r);
  // The fused track has existed since its earliest member did: it is the same physical obstacle
  // that was being over-segmented, not a new one.
  fused.created_stamp_s = earliest_created;

  CorrectTrack(&fused, observation, stamp_s, stats);
  created->push_back(fused);

  ++stats->fusions;
  stats->fusion_tracks_consumed += static_cast<int32_t>(indices.size());
}

TrackState2D KfCircleTracker::StateOf(const Track& track) const {
  TrackState2D state;
  state.id = track.id;
  state.center = Eigen::Vector2d(track.x.value(), track.y.value());
  // The KF RATE state, never a finite difference - risk R10's mitigation, and deliberately
  // unsmoothed: open question Q14 asks whether this velocity is adequate for DPCBF at 10 Hz, and
  // any post-filter here would answer that question by hiding it.
  state.velocity = Eigen::Vector2d(track.x.rate(), track.y.rate());
  state.radius_m = track.r.value();
  state.radius_rate_mps = track.r.rate();

  state.center_variance = Eigen::Vector2d(track.x.value_variance(), track.y.value_variance());
  state.velocity_variance = Eigen::Vector2d(track.x.rate_variance(), track.y.rate_variance());
  state.radius_variance = track.r.value_variance();
  state.radius_rate_variance = track.r.rate_variance();

  state.created_stamp_s = track.created_stamp_s;
  state.last_update_stamp_s = track.last_update_stamp_s;
  state.last_measurement_stamp_s = track.last_measurement_stamp_s;

  state.hits = track.hits;
  state.misses = track.misses;
  state.consecutive_misses = track.consecutive_misses;
  state.status = track.status;
  state.frame = FrameId::kWorld;
  return state;
}

PerceptionObstacle KfCircleTracker::ObstacleOf(const Track& track) const {
  PerceptionObstacle obstacle;
  obstacle.id = track.id;
  obstacle.center = Eigen::Vector2d(track.x.value(), track.y.value());
  obstacle.velocity = Eigen::Vector2d(track.x.rate(), track.y.rate());
  obstacle.radius_true_m = track.r.value();

  obstacle.center_variance = Eigen::Vector2d(track.x.value_variance(), track.y.value_variance());
  obstacle.velocity_variance = Eigen::Vector2d(track.x.rate_variance(), track.y.rate_variance());
  obstacle.radius_variance = track.r.value_variance();

  // THE FIELD MAPPING THAT DECIDES WHETHER P10'S STALENESS GATE WORKS, so it is spelled out
  // rather than left to look obvious.
  //
  // PerceptionObstacle has one stamp; TrackState2D has two, and they differ for exactly the
  // tracks that matter. `last_update_stamp_s` on a track is "advanced to", which for a coasting
  // track is always NOW; `last_measurement_stamp_s` is when real information last arrived. The
  // MEASUREMENT stamp is what goes here, because SafetyObstacle::stamp_s is defined as "the
  // measurement time this obstacle derives from" and safety.max_age_s (0.30 s) is the gate that
  // consumes it. Publishing the advanced-to stamp instead would make every coasting track look
  // brand new, silently disable that gate, and feed DPCBF two full seconds of extrapolation as
  // though it were fresh observation.
  //
  // The centre and velocity, by contrast, ARE the extrapolated present-time estimate. The pair
  // is intentional: here is where the obstacle is believed to be now, and here is how old the
  // last real evidence for that belief is.
  obstacle.last_update_stamp_s = track.last_measurement_stamp_s;
  obstacle.track_age_s = track.last_update_stamp_s - track.created_stamp_s;
  obstacle.confidence = track.hits + track.misses > 0
                            ? static_cast<double>(track.hits) /
                                  static_cast<double>(track.hits + track.misses)
                            : 0.0;
  obstacle.frame = FrameId::kWorld;
  return obstacle;
}

const char* KfCircleTracker::Update(const std::vector<CircleObservation>& observations,
                                    double stamp_s, Tracking2DResult* out) {
  if (out == nullptr) return "KfCircleTracker::Update needs an output";
  out->Clear();
  const auto wall_start = std::chrono::steady_clock::now();

  if (!IsFinite(stamp_s)) return "KfCircleTracker::Update stamp must be finite";
  if (have_stamp_ && stamp_s < last_stamp_s_) {
    // Non-decreasing is required rather than merely expected: a backwards step would make dt
    // negative, and a negative dt in a constant-rate transition SHRINKS the covariance, which
    // reads downstream as the tracker having become more certain by being handed older data.
    return "KfCircleTracker::Update stamp must be non-decreasing";
  }
  const double dt = have_stamp_ ? stamp_s - last_stamp_s_ : 0.0;
  have_stamp_ = true;
  last_stamp_s_ = stamp_s;

  TrackingStats& stats = out->stats;
  stats.input_observations = static_cast<int32_t>(observations.size());

  // ---- 1. Screen the observations. -------------------------------------------------------
  // Tracking is a world-frame activity (tracking_2d.h): a detector-frame observation would be
  // associated against world-frame tracks and silently pull every track toward the robot.
  // CircleObservation::Validate() admits both frames, so the frame check has to be made here.
  valid_.clear();
  for (const CircleObservation& observation : observations) {
    if (!observation.IsValid() || observation.frame != FrameId::kWorld) {
      ++stats.observations_rejected_invalid;
      continue;
    }
    valid_.push_back(&observation);
  }

  const int n = static_cast<int>(valid_.size());
  const int t = static_cast<int>(tracks_.size());

  // ---- 2. Advance every track to the scan time. -------------------------------------------
  // Before association, not after: the cost matrix must compare each observation against where
  // its track is predicted to BE at the measurement's own time. Upstream associates against
  // whatever its last timer tick left behind, which - per the R11 note in the header - is a state
  // that has been repeatedly re-corrected toward the previous measurement.
  //
  // Q is scaled by dt here; see change (a).
  const double q_value = params_.process_variance * dt;
  const double q_rate = params_.process_rate_variance * dt;
  if (dt > 0.0) {
    for (Track& track : tracks_) {
      track.x.Predict(dt, q_value, q_rate);
      track.y.Predict(dt, q_value, q_rate);
      track.r.Predict(dt, q_value, q_rate);
    }
  }
  for (Track& track : tracks_) track.last_update_stamp_s = stamp_s;

  created_.clear();
  disposition_.assign(static_cast<size_t>(t), static_cast<uint8_t>(Disposition::kUntouched));
  used_observation_.assign(static_cast<size_t>(n), 0);

  // ---- 3. Cost matrix, row minima and column minima. --------------------------------------
  // Ported from obstacle_tracker.cpp:267-353, including the fact that the two are computed
  // INDEPENDENTLY and each is gated by min_correspondence_cost. This is not a greedy
  // ascending-cost assignment and it is not Hungarian: `row_min_[i]` is observation i's nearest
  // track and `col_min_[j]` is track j's nearest observation, and the two need not agree. The
  // disagreements are exactly what the fusion and fission passes below detect.
  if (n > 0 && t > 0) {
    cost_matrix_.assign(static_cast<size_t>(n) * static_cast<size_t>(t), 0.0);
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < t; ++j) {
        cost_matrix_[static_cast<size_t>(i) * static_cast<size_t>(t) + static_cast<size_t>(j)] =
            Cost(*valid_[static_cast<size_t>(i)], tracks_[static_cast<size_t>(j)]);
      }
    }
  }

  row_min_.assign(static_cast<size_t>(n), -1);
  for (int i = 0; i < n; ++i) {
    double best = params_.min_correspondence_cost_m;  // The gate IS the initial minimum.
    for (int j = 0; j < t; ++j) {
      const double cost =
          cost_matrix_[static_cast<size_t>(i) * static_cast<size_t>(t) + static_cast<size_t>(j)];
      if (cost < best) {
        best = cost;
        row_min_[static_cast<size_t>(i)] = j;
      }
    }
  }

  col_min_.assign(static_cast<size_t>(t), -1);
  for (int j = 0; j < t; ++j) {
    double best = params_.min_correspondence_cost_m;
    for (int i = 0; i < n; ++i) {
      const double cost =
          cost_matrix_[static_cast<size_t>(i) * static_cast<size_t>(t) + static_cast<size_t>(j)];
      if (cost < best) {
        best = cost;
        col_min_[static_cast<size_t>(j)] = i;
      }
    }
  }

  // ---- 4. Fusion: several tracks whose nearest observation is the same one. ----------------
  // obstacle_tracker.cpp:156-176. The `j < t` inner scan and the `t - 1` outer bound are
  // upstream's; the outer bound is harmless because a group needs two members, so its lowest
  // index is at most t - 2 regardless.
  if (params_.enable_fusion) {
    for (int i = 0; i < t - 1; ++i) {
      if (disposition_[static_cast<size_t>(i)] != static_cast<uint8_t>(Disposition::kUntouched)) continue;
      const int observation_index = col_min_[static_cast<size_t>(i)];
      if (observation_index < 0) continue;
      if (used_observation_[static_cast<size_t>(observation_index)] != 0) continue;

      group_.clear();
      group_.push_back(i);
      for (int j = i + 1; j < t; ++j) {
        if (disposition_[static_cast<size_t>(j)] != static_cast<uint8_t>(Disposition::kUntouched)) continue;
        if (col_min_[static_cast<size_t>(j)] == observation_index) group_.push_back(j);
      }
      if (group_.size() < 2) continue;

      FuseTracks(group_, *valid_[static_cast<size_t>(observation_index)], stamp_s, &created_,
                 &stats);
      for (const int index : group_) disposition_[static_cast<size_t>(index)] = static_cast<uint8_t>(Disposition::kConsumed);
      used_observation_[static_cast<size_t>(observation_index)] = 1;
    }
  }

  // ---- 5. Fission: several observations whose nearest track is the same one. ---------------
  // obstacle_tracker.cpp:178-198 and 443-454. Upstream copies the source track once per
  // observation and corrects each copy separately, so the split halves inherit a common history
  // instead of starting from nothing - that is what is ported.
  //
  // CHANGE (c), AND A DEFECT THAT DISAPPEARS WITH IT. Upstream keeps unconfirmed detections in a
  // separate `untracked_obstacles_` list and indexes the cost matrix over the concatenation
  // [tracked | untracked]; `fissionObstacleUsed` then skips any match whose index lands in the
  // untracked half (obstacle_tracker.cpp:391 `row_min_indices[idx] >= T`). The consequence is
  // reachable and wrong: when two new observations both fall nearest the same untracked seed,
  // fission declines to handle them, and the plain-match loop at obstacle_tracker.cpp:201-223
  // then constructs a TrackedObstacle from that seed for EACH of them - because that loop
  // records the consumed observation in `used_new_obstacles` but never records the consumed old
  // obstacle in `used_old_obstacles`. Two tracks are born on top of each other from one seed.
  //
  // Here a tentative track is an ordinary member of the single track list, so the exclusion has
  // no counterpart to skip, fission applies uniformly, and the double-spawn path does not exist
  // to be reproduced. The plain-match loop below also marks the track it consumed, which closes
  // the same hole from the other side.
  if (params_.enable_fission) {
    for (int i = 0; i < n - 1; ++i) {
      if (used_observation_[static_cast<size_t>(i)] != 0) continue;
      const int track_index = row_min_[static_cast<size_t>(i)];
      if (track_index < 0) continue;
      if (disposition_[static_cast<size_t>(track_index)] != static_cast<uint8_t>(Disposition::kUntouched)) continue;

      group_.clear();
      group_.push_back(i);
      for (int j = i + 1; j < n; ++j) {
        if (used_observation_[static_cast<size_t>(j)] != 0) continue;
        if (row_min_[static_cast<size_t>(j)] == track_index) group_.push_back(j);
      }
      if (group_.size() < 2) continue;

      // IDENTITY ON FISSION. Upstream produces N indistinguishable copies; ids make that
      // illegal, since two live tracks may not share one. The lowest-cost observation is the
      // continuation of the original track and keeps its id and its history; the others are new
      // objects that happen to inherit a good initial state, so they are minted fresh ids. This
      // is what keeps a genuine over-segmentation event to ONE id switch at most.
      int best_observation = group_.front();
      double best_cost = -1.0;
      for (const int index : group_) {
        const double cost = cost_matrix_[static_cast<size_t>(index) * static_cast<size_t>(t) +
                                        static_cast<size_t>(track_index)];
        if (best_cost < 0.0 || cost < best_cost) {
          best_cost = cost;
          best_observation = index;
        }
      }

      for (const int index : group_) {
        if (static_cast<int>(tracks_.size() + created_.size()) >= params_.max_tracks) {
          ++stats.tracks_dropped_capacity;
          stats.capacity_exceeded = true;
          continue;
        }
        Track child = tracks_[static_cast<size_t>(track_index)];
        if (index != best_observation) {
          child.id = next_id_++;
          // The heir keeps the filter state but not the credit: an over-segmented half has not
          // itself been seen `hits` times, and letting it inherit a confirmed hit count would
          // let a split artefact reach the safety stage without ever passing the confirmation
          // gate on its own evidence.
          child.hits = 0;
          child.misses = 0;
          child.consecutive_misses = 0;
          child.status = TrackStatus::kTentative;
          child.created_stamp_s = stamp_s;
        }
        CorrectTrack(&child, *valid_[static_cast<size_t>(index)], stamp_s, &stats);
        created_.push_back(child);
        ++stats.fission_tracks_created;
        used_observation_[static_cast<size_t>(index)] = 1;
      }

      disposition_[static_cast<size_t>(track_index)] = static_cast<uint8_t>(Disposition::kConsumed);
      ++stats.fissions;
    }
  }

  // ---- 6. Plain matches, and new tracks for what is left over. -----------------------------
  for (int i = 0; i < n; ++i) {
    if (used_observation_[static_cast<size_t>(i)] != 0) continue;
    const int track_index = row_min_[static_cast<size_t>(i)];

    if (track_index < 0) {
      Seed(*valid_[static_cast<size_t>(i)], stamp_s, &stats);
      ++stats.unmatched_observations;
      used_observation_[static_cast<size_t>(i)] = 1;
      continue;
    }
    if (disposition_[static_cast<size_t>(track_index)] != static_cast<uint8_t>(Disposition::kUntouched)) {
      // Its track has already been consumed or corrected. Upstream drops such an observation
      // entirely (obstacle_tracker.cpp:208); seeding a new track from it instead would
      // manufacture a duplicate on top of a track that just matched. Dropped, and counted.
      ++stats.observations_dropped_taken_track;
      continue;
    }

    CorrectTrack(&tracks_[static_cast<size_t>(track_index)], *valid_[static_cast<size_t>(i)],
                 stamp_s, &stats);
    disposition_[static_cast<size_t>(track_index)] = static_cast<uint8_t>(Disposition::kCorrected);
    used_observation_[static_cast<size_t>(i)] = 1;
    ++stats.matched;
  }

  // ---- 7. Lifecycle for the tracks no measurement reached. --------------------------------
  // CHANGE (b). Upstream's whole lifecycle is one fade counter of `loop_rate * tracking_duration`
  // ticks, decremented by the timer and reset by any correction (tracked_obstacle.h:64, 84, 104,
  // 121); there is no confirmation gate at all, so a single spurious detection becomes a
  // published obstacle on its second scan. Counting in SCANS rather than ticks is what the
  // measurement-driven model makes possible, and a confirmation gate is what keeps a one-scan
  // artefact away from the safety filter.
  survivors_.clear();

  for (int j = 0; j < t; ++j) {
    Track& track = tracks_[static_cast<size_t>(j)];
    const auto disposition = static_cast<Disposition>(disposition_[static_cast<size_t>(j)]);
    if (disposition == Disposition::kConsumed) continue;  // Its heirs are in created_.
    if (disposition == Disposition::kCorrected) {
      survivors_.push_back(track);
      continue;
    }

    ++track.misses;
    ++track.consecutive_misses;
    ++stats.unmatched_tracks;

    if (track.status == TrackStatus::kTentative) {
      // A tentative track dies on its first miss, which is upstream's semantics for the same
      // object: `untracked_obstacles_` is cleared and reassigned on every callback
      // (obstacle_tracker.cpp:234-235), so an unconfirmed detection that is not immediately
      // re-seen is simply gone. Keeping it alive would let noise accumulate hits across gaps and
      // eventually confirm. The cost is that confirmation needs `confirm_hits` CONSECUTIVE
      // scans; that is a real limitation, it is what makes the confirmation-delay gate a hard
      // 3 scans rather than a distribution, and it is flagged for P10 rather than papered over.
      ++stats.tracks_deleted_tentative_miss;
      continue;
    }

    track.status = TrackStatus::kCoasting;

    if (track.consecutive_misses >= params_.delete_misses) {
      ++stats.tracks_deleted_misses;
      continue;
    }
    if (stamp_s - track.last_measurement_stamp_s > params_.max_coast_s) {
      ++stats.tracks_deleted_coast_timeout;
      continue;
    }
    survivors_.push_back(track);
  }

  for (Track& track : created_) survivors_.push_back(track);
  // A swap rather than an assignment, so the two buffers trade capacity instead of one of them
  // reallocating: `tracks_` becomes this scan's survivors and `survivors_` inherits last scan's
  // allocation to be cleared and refilled next time.
  tracks_.swap(survivors_);

  // ---- 8. Emit. ---------------------------------------------------------------------------
  out->tracks.reserve(tracks_.size());
  out->obstacles.reserve(tracks_.size());

  size_t write = 0;
  for (size_t read = 0; read < tracks_.size(); ++read) {
    const Track& track = tracks_[read];
    const TrackState2D state = StateOf(track);
    if (!state.IsValid()) {
      // The reachable cause is a radius filter driven to zero or below, which the contract
      // forbids and which cannot be repaired without inventing a number. Upstream would publish
      // it. Dropping the TRACK rather than just the output is deliberate: a state the contract
      // cannot represent must not stay in the pool to be associated against next scan, where it
      // would keep matching and keep being dropped, invisibly, forever.
      ++stats.tracks_deleted_invalid_state;
      continue;
    }
    out->tracks.push_back(state);
    if (track.status == TrackStatus::kConfirmed || track.status == TrackStatus::kCoasting) {
      const PerceptionObstacle obstacle = ObstacleOf(track);
      if (obstacle.IsValid()) out->obstacles.push_back(obstacle);
    }
    if (write != read) tracks_[write] = tracks_[read];
    ++write;
  }
  tracks_.resize(write);

  for (const Track& track : tracks_) {
    switch (track.status) {
      case TrackStatus::kTentative: ++stats.tentative; break;
      case TrackStatus::kConfirmed: ++stats.confirmed; break;
      case TrackStatus::kCoasting:  ++stats.coasting;  break;
      case TrackStatus::kInvalid:   break;
    }
  }
  stats.obstacles_emitted = static_cast<int32_t>(out->obstacles.size());

  const auto wall_end = std::chrono::steady_clock::now();
  stats.wall_time_us =
      std::chrono::duration<double, std::micro>(wall_end - wall_start).count();
  return nullptr;
}

const char* KfCircleTracker::Predict(double stamp_s, std::vector<TrackPrediction2D>* out) const {
  if (out == nullptr) return "KfCircleTracker::Predict needs an output";
  out->clear();
  if (!IsFinite(stamp_s)) return "KfCircleTracker::Predict stamp must be finite";
  if (have_stamp_ && stamp_s < last_stamp_s_) {
    return "KfCircleTracker::Predict must not predict backwards in time";
  }

  const double dt = have_stamp_ ? stamp_s - last_stamp_s_ : 0.0;
  const double q_value = params_.process_variance * dt;
  const double q_rate = params_.process_rate_variance * dt;

  out->reserve(tracks_.size());
  for (const Track& track : tracks_) {
    // Copies, so that a prediction can never become history - the reason this method is const.
    AxisKalman2 x = track.x;
    AxisKalman2 y = track.y;
    AxisKalman2 r = track.r;
    if (dt > 0.0) {
      x.Predict(dt, q_value, q_rate);
      y.Predict(dt, q_value, q_rate);
      r.Predict(dt, q_value, q_rate);
    }

    TrackPrediction2D prediction;
    prediction.id = track.id;
    prediction.stamp_s = stamp_s;
    prediction.source_stamp_s = track.last_update_stamp_s;
    prediction.center = Eigen::Vector2d(x.value(), y.value());
    prediction.velocity = Eigen::Vector2d(x.rate(), y.rate());
    prediction.radius_m = r.value();
    prediction.center_variance = Eigen::Vector2d(x.value_variance(), y.value_variance());
    prediction.velocity_variance = Eigen::Vector2d(x.rate_variance(), y.rate_variance());
    prediction.radius_variance = r.value_variance();
    prediction.frame = FrameId::kWorld;

    if (prediction.IsValid()) out->push_back(prediction);
  }
  return nullptr;
}

}  // namespace perception::core
