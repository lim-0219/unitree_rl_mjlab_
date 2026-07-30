// P10 acceptance: validity gating, conservative inflation, the velocity spike clamp, and the
// section-12 safety-generation gates.
//
// WHAT THIS SUITE MEASURES AND WHY ON WHICH CORPUS. Containment is a claim about an ABSOLUTE
// radius, which makes it far more sensitive to which stages a fixture runs than any P8 or P9
// gate was. So the gates below are measured on `safety_scenes::MovingScenes()` - ray-cast scans
// of moving cylinders through the REAL detector and the REAL tracker - and the calibration
// evidence is taken from P9's own seven streams, rebased onto the detector's radius convention.
// See tests/fixtures/safety_scenes.h for why the raw P9 streams cannot back an absolute-radius
// claim, and section E below for what both corpora say.
//
// argv[1] is perception/, for the shipped configs/perception.yaml.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "check.h"
#include "detection_scenes.h"
#include "safety_scenes.h"
#include "tracking_scenes.h"
#include "perception/core/detection/segment_circle_detector.h"
#include "perception/core/safety/safety_state_generator.h"
#include "perception/core/tracking/kf_circle_tracker.h"
#include "perception/integration/detection_params.h"
#include "perception/integration/perception_config.h"
#include "perception/integration/safety_params.h"
#include "perception/integration/tracking_params.h"

using perception::core::CircleObservation;
using perception::core::Detection2DResult;
using perception::core::FrameId;
using perception::core::KfCircleTracker;
using perception::core::ObstacleSource;
using perception::core::PerceptionObstacle;
using perception::core::SafetyObstacle;
using perception::core::SafetyParams;
using perception::core::SafetyStateGenerator;
using perception::core::SafetyStateResult;
using perception::core::SegmentCircleDetector;
using perception::core::SegmentCircleDetectorParams;
using perception::core::Tracking2DResult;
using perception::core::TrackingParams;
using perception::core::TrackState2D;
using perception::core::TrackStatus;
using perception_test::Check;
using perception_test::CheckValid;
using perception_test::Report;
using perception_test::Section;

namespace {

bool Near(double a, double b, double tolerance) { return std::abs(a - b) <= tolerance; }

std::string F(double value, int digits = 4) {
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.*f", digits, value);
  return buffer;
}

// ---------------------------------------------------------------------------------------
// Hand-built inputs for the gating tests.
// ---------------------------------------------------------------------------------------

// A confirmed obstacle plus the TrackState2D the safety stage joins against for its hit count.
// Both are built here rather than driven out of the tracker because the gating tests are about
// the GATES, and a fixture that had to steer a Kalman filter into each corner would be testing
// the tracker's reachability instead.
struct TrackedPair {
  PerceptionObstacle obstacle;
  TrackState2D track;
};

TrackedPair MakePair(std::uint32_t id, double stamp_s, double track_age_s, int hits) {
  TrackedPair pair;

  pair.obstacle.id = id;
  pair.obstacle.center = Eigen::Vector2d(2.0, 1.0);
  pair.obstacle.velocity = Eigen::Vector2d(0.4, 0.3);  // 0.5 m/s.
  pair.obstacle.radius_true_m = 0.50;
  pair.obstacle.center_variance = Eigen::Vector2d(0.0004, 0.0004);   // sigma 0.02 m.
  pair.obstacle.velocity_variance = Eigen::Vector2d(0.01, 0.01);
  pair.obstacle.radius_variance = 0.0004;                            // sigma 0.02 m.
  pair.obstacle.last_update_stamp_s = stamp_s;
  pair.obstacle.track_age_s = track_age_s;
  pair.obstacle.confidence = 1.0;
  pair.obstacle.frame = FrameId::kWorld;

  pair.track.id = id;
  pair.track.center = pair.obstacle.center;
  pair.track.velocity = pair.obstacle.velocity;
  pair.track.radius_m = pair.obstacle.radius_true_m;
  pair.track.center_variance = pair.obstacle.center_variance;
  pair.track.velocity_variance = pair.obstacle.velocity_variance;
  pair.track.radius_variance = pair.obstacle.radius_variance;
  pair.track.created_stamp_s = stamp_s - track_age_s;
  pair.track.last_update_stamp_s = stamp_s;
  pair.track.last_measurement_stamp_s = stamp_s;
  pair.track.hits = hits;
  pair.track.misses = 0;
  pair.track.consecutive_misses = 0;
  pair.track.status = TrackStatus::kConfirmed;
  pair.track.frame = FrameId::kWorld;
  return pair;
}

Tracking2DResult MakeTracking(const std::vector<TrackedPair>& pairs) {
  Tracking2DResult tracking;
  for (const TrackedPair& pair : pairs) {
    tracking.tracks.push_back(pair.track);
    tracking.obstacles.push_back(pair.obstacle);
  }
  return tracking;
}

// ---------------------------------------------------------------------------------------
// Containment measurement.
//
// CONTAINMENT DEFINITION, stated once. The true disc is contained in the safety disc iff
//   ||c_truth - c_safety|| + r_truth <= r_inflated.
// That is the whole claim: not "the centre is close" and not "the radius is big enough", but
// the conjunction, which is the only form that means what a CBF constraint needs it to mean.
// ---------------------------------------------------------------------------------------

struct ContainmentSample {
  std::string corpus;
  double truth_radius = 0.0;
  double center_error = 0.0;
  double radius_true = 0.0;      // SafetyObstacle::radius_true_m, i.e. the tracked radius.
  double radius_inflated = 0.0;
  double position_inflation = 0.0;
  double age_s = 0.0;

  // The margin by which the safety disc contains (positive) or fails to contain (negative) the
  // true disc.
  double Margin() const { return radius_inflated - (center_error + truth_radius); }
  bool Contained() const { return Margin() >= 0.0; }
  // The radius fed to DPCBF is smaller than the object itself - section 12's "radius
  // under-estimation", the one unrecoverable error.
  bool RadiusUnderEstimated() const { return radius_inflated < truth_radius; }

  // Normalized error statistics for the chi-square calibration check.
  double position_nees = 0.0;  // 2 dof.
  double radius_nis = 0.0;     // 1 dof.
  bool have_nees = false;
};

struct ContainmentSummary {
  int samples = 0;
  int contained = 0;
  int radius_under = 0;
  double worst_margin = std::numeric_limits<double>::infinity();
  double worst_center_error = 0.0;
  double worst_radius_under_estimate = 0.0;  // truth_radius - radius_true, positive = under.
  double inflation_sum = 0.0;
  double worst_inflation = 0.0;
  std::string worst_margin_case;

  double ContainmentRate() const {
    return samples > 0 ? static_cast<double>(contained) / samples : 0.0;
  }
  double UnderEstimationRate() const {
    return samples > 0 ? static_cast<double>(radius_under) / samples : 0.0;
  }
  double MeanInflation() const { return samples > 0 ? inflation_sum / samples : 0.0; }
};

ContainmentSummary Summarise(const std::vector<ContainmentSample>& samples) {
  ContainmentSummary summary;
  for (const ContainmentSample& sample : samples) {
    ++summary.samples;
    if (sample.Contained()) ++summary.contained;
    if (sample.RadiusUnderEstimated()) ++summary.radius_under;
    if (sample.Margin() < summary.worst_margin) {
      summary.worst_margin = sample.Margin();
      summary.worst_margin_case = sample.corpus;
    }
    summary.worst_center_error = std::max(summary.worst_center_error, sample.center_error);
    summary.worst_radius_under_estimate =
        std::max(summary.worst_radius_under_estimate, sample.truth_radius - sample.radius_true);
    const double inflation = sample.radius_inflated - sample.radius_true;
    summary.inflation_sum += inflation;
    summary.worst_inflation = std::max(summary.worst_inflation, inflation);
  }
  if (summary.samples == 0) summary.worst_margin = 0.0;
  return summary;
}

// Associate an emitted SafetyObstacle with the truth it derives from: nearest centre inside a
// generous gate. A tight gate would make the association itself part of the measurement, which
// is the same rule P8's and P9's harnesses follow.
constexpr double kAssociationGateM = 1.0;

// ---------------------------------------------------------------------------------------
// Corpus runners. Each drives a corpus all the way to SafetyObstacles and appends samples.
// ---------------------------------------------------------------------------------------

// A: the shipped pipeline. Ray-cast scan -> detector -> tracker -> safety.
void RunMovingScene(const safety_scenes::MovingScene& scene,
                    const SegmentCircleDetectorParams& detector_params,
                    const TrackingParams& tracking_params, const SafetyParams& safety_params,
                    std::vector<ContainmentSample>* samples, int* emitted_frames = nullptr) {
  SegmentCircleDetector detector(detector_params);
  KfCircleTracker tracker(tracking_params);
  SafetyStateGenerator safety(safety_params);
  Detection2DResult detection;
  Tracking2DResult tracking;
  SafetyStateResult result;

  for (const safety_scenes::MovingFrame& frame : scene.frames) {
    if (detector.Detect(frame.scan, &detection) != nullptr) return;

    // The sensor sits at the origin of both frames in these scenes, so the transform is the
    // identity and only the tag changes - see the fixture header.
    std::vector<CircleObservation> observations = detection.circles;
    for (CircleObservation& observation : observations) observation.frame = FrameId::kWorld;

    if (tracker.Update(observations, frame.stamp_s, &tracking) != nullptr) return;
    if (safety.Generate(tracking, frame.stamp_s, &result) != nullptr) return;
    if (emitted_frames != nullptr && !result.obstacles.empty()) ++(*emitted_frames);

    for (const SafetyObstacle& obstacle : result.obstacles) {
      const safety_scenes::MovingTruth* best = nullptr;
      double best_distance = kAssociationGateM;
      for (const safety_scenes::MovingTruth& truth : frame.truth) {
        const double distance = (obstacle.center - truth.center).norm();
        if (distance < best_distance) {
          best_distance = distance;
          best = &truth;
        }
      }
      if (best == nullptr) continue;

      ContainmentSample sample;
      sample.corpus = scene.name;
      sample.truth_radius = best->radius;
      sample.center_error = best_distance;
      sample.radius_true = obstacle.radius_true_m;
      sample.radius_inflated = obstacle.radius_inflated_m;
      sample.position_inflation = obstacle.position_inflation_m;
      sample.age_s = obstacle.age_s;
      samples->push_back(sample);
    }
  }
}

// B/C: P9's streams. Observation stream -> tracker -> safety. Carries the NEES/NIS statistics,
// because the per-axis variances that normalize them are the tracker's and this is the corpus
// whose error realisations P9 characterised.
void RunStream(const tracking_scenes::Stream& stream, const TrackingParams& tracking_params,
               const SafetyParams& safety_params, std::vector<ContainmentSample>* samples) {
  KfCircleTracker tracker(tracking_params);
  SafetyStateGenerator safety(safety_params);
  Tracking2DResult tracking;
  SafetyStateResult result;

  for (const tracking_scenes::Frame& frame : stream.frames) {
    if (tracker.Update(frame.observations, frame.stamp_s, &tracking) != nullptr) return;
    if (safety.Generate(tracking, frame.stamp_s, &result) != nullptr) return;

    for (const SafetyObstacle& obstacle : result.obstacles) {
      const tracking_scenes::TruthSample* best = nullptr;
      double best_distance = kAssociationGateM;
      for (const tracking_scenes::TruthSample& truth : frame.truth) {
        const double distance = (obstacle.center - truth.center).norm();
        if (distance < best_distance) {
          best_distance = distance;
          best = &truth;
        }
      }
      if (best == nullptr) continue;

      ContainmentSample sample;
      sample.corpus = stream.name;
      sample.truth_radius = best->radius;
      sample.center_error = best_distance;
      sample.radius_true = obstacle.radius_true_m;
      sample.radius_inflated = obstacle.radius_inflated_m;
      sample.position_inflation = obstacle.position_inflation_m;
      sample.age_s = obstacle.age_s;

      // The normalized statistics, taken from the PerceptionObstacle the SafetyObstacle came
      // from - the safety contract deliberately carries no covariance, so the variances have to
      // be read on the tracker's side of the seam.
      for (const PerceptionObstacle& source : tracking.obstacles) {
        if (source.id != obstacle.id) continue;
        const Eigen::Vector2d error = source.center - best->center;
        if (source.center_variance.x() > 0.0 && source.center_variance.y() > 0.0) {
          sample.position_nees = error.x() * error.x() / source.center_variance.x() +
                                 error.y() * error.y() / source.center_variance.y();
          if (source.radius_variance > 0.0) {
            const double radius_error = source.radius_true_m - best->radius;
            sample.radius_nis = radius_error * radius_error / source.radius_variance;
          }
          sample.have_nees = true;
        }
        break;
      }
      samples->push_back(sample);
    }
  }
}

double Quantile(std::vector<double> values, double p) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const auto index = static_cast<std::size_t>(
      std::min<double>(static_cast<double>(values.size()) - 1.0,
                       std::floor(p * static_cast<double>(values.size()))));
  return values[index];
}

}  // namespace

int main(int argc, char** argv) {
  const std::string root = argc > 1 ? argv[1] : ".";

  // -------------------------------------------------------------------------------------
  Section("A. Shipped config reaches the safety stage (integration/safety_params.h)");
  // -------------------------------------------------------------------------------------
  perception::integration::PerceptionConfig config;
  bool config_loaded = false;
  try {
    config = perception::integration::PerceptionConfig::LoadFromYaml(root +
                                                                    "/configs/perception.yaml");
    config_loaded = true;
    Check(true, "the shipped perception.yaml loads");
  } catch (const std::exception& error) {
    Check(false, "the shipped perception.yaml loads", error.what());
    return Report("perception_safety_test");
  }

  SafetyParams shipped = perception::integration::MakeSafetyParams(config.safety);
  const SegmentCircleDetectorParams detector_params =
      perception::integration::MakeSegmentCircleDetectorParams(config.detection);
  const TrackingParams tracking_params =
      perception::integration::MakeTrackingParams(config.tracking);
  (void)config_loaded;

  // FIELD COMPLETENESS, the same assertion the detection, projection and tracking tests make:
  // every safety parameter is checked against its shipped source, so a `safety.*` key that never
  // reaches the stage fails here rather than being silently ignored.
  Check(shipped.max_age_s == config.safety.max_age_s, "max_age_s mapped");
  Check(shipped.min_track_age_s == config.safety.min_track_age_s, "min_track_age_s mapped");
  Check(shipped.min_track_hits == config.safety.min_track_hits, "min_track_hits mapped");
  Check(shipped.radius_inflation_k_sigma == config.safety.radius_inflation_k_sigma,
        "radius_inflation_k_sigma mapped");
  Check(shipped.radius_inflation_fixed_m == config.safety.radius_inflation_fixed_m,
        "radius_inflation_fixed_m mapped");
  Check(shipped.latency_inflation_s == config.safety.latency_inflation_s,
        "latency_inflation_s mapped");
  Check(shipped.use_enclosing_radius == config.safety.use_enclosing_radius,
        "use_enclosing_radius mapped");
  Check(shipped.min_radius_m == config.safety.min_radius_m, "min_radius_m mapped");
  Check(shipped.max_radius_m == config.safety.max_radius_m, "max_radius_m mapped");
  Check(shipped.max_speed_mps == config.safety.max_speed_mps, "max_speed_mps mapped");
  Check(shipped.max_obstacles == config.safety.max_obstacles, "max_obstacles mapped");
  Check(shipped.Validate() == nullptr, "the shipped safety config is a valid param set",
        shipped.Validate() == nullptr ? "" : shipped.Validate());

  // -------------------------------------------------------------------------------------
  Section("B. SafetyParams validation");
  // -------------------------------------------------------------------------------------
  {
    SafetyParams bad = shipped;
    bad.max_age_s = 0.0;
    Check(bad.Validate() != nullptr, "a zero max_age_s is rejected");

    bad = shipped;
    bad.min_track_hits = 0;
    Check(bad.Validate() != nullptr, "a zero min_track_hits is rejected");

    bad = shipped;
    bad.radius_inflation_k_sigma = -1.0;
    Check(bad.Validate() != nullptr, "a negative k_sigma is rejected");

    bad = shipped;
    bad.min_radius_m = bad.max_radius_m + 0.1;
    Check(bad.Validate() != nullptr, "a floor above the ceiling is rejected");

    bad = shipped;
    bad.max_speed_mps = 0.0;
    Check(bad.Validate() != nullptr, "a zero speed clamp is rejected");

    bad = shipped;
    bad.max_obstacles = 0;
    Check(bad.Validate() != nullptr, "a zero obstacle cap is rejected");
  }

  // -------------------------------------------------------------------------------------
  Section("C. Validity gating");
  // -------------------------------------------------------------------------------------
  {
    SafetyStateGenerator generator(shipped);
    SafetyStateResult result;
    const double now = 10.0;

    // The nominal case: fresh, old enough, hit enough.
    {
      const Tracking2DResult tracking = MakeTracking({MakePair(1, now, 1.0, 8)});
      Check(generator.Generate(tracking, now, &result) == nullptr, "a nominal frame generates");
      Check(result.obstacles.size() == 1, "and emits the obstacle",
            std::to_string(result.obstacles.size()));
      Check(result.stats.IsBalanced(), "the per-frame census balances");
      if (result.obstacles.size() == 1) {
        const SafetyObstacle& obstacle = result.obstacles.front();
        CheckValid(obstacle, "the emitted SafetyObstacle is valid");
        Check(obstacle.source == ObstacleSource::kEstimated, "source is estimated");
        Check(obstacle.frame == FrameId::kWorld, "frame is world");
        Check(obstacle.valid, "valid is set on everything that survives the gates");
        Check(obstacle.id == 1, "the track id passes through unchanged");
        Check(Near(obstacle.age_s, 0.0, 1e-12), "a same-instant state has zero age");
        Check(obstacle.radius_inflated_m > obstacle.radius_true_m,
              "and the radius is inflated, not copied",
              F(obstacle.radius_inflated_m) + " > " + F(obstacle.radius_true_m));
      }
    }

    // STALENESS: the safety-dominance policy says DROP, never shrink. Both halves are asserted,
    // because "the stale obstacle is gone" and "no stale obstacle was published with a smaller
    // radius" are different claims and only the second one is the policy.
    {
      const double stale_stamp = now - shipped.max_age_s - 1e-6;
      const Tracking2DResult tracking = MakeTracking({MakePair(1, stale_stamp, 1.0, 8)});
      Check(generator.Generate(tracking, now, &result) == nullptr, "a stale frame generates");
      Check(result.obstacles.empty(), "a stale track is DROPPED",
            std::to_string(result.obstacles.size()) + " emitted");
      Check(result.stats.dropped_stale == 1, "and the drop is attributed to staleness");
      Check(result.stats.IsBalanced(), "the census balances on the stale path");
    }

    // Right at the limit it survives, and its radius is LARGER than the fresh one's - the age
    // is in the drift horizon, so approaching the staleness limit inflates rather than shrinks.
    {
      const Tracking2DResult fresh = MakeTracking({MakePair(1, now, 1.0, 8)});
      SafetyStateResult fresh_result;
      generator.Generate(fresh, now, &fresh_result);

      // A hair INSIDE the limit rather than exactly on it: `age_s` is recovered as
      // `now - (now - max_age_s)`, and that round trip lands a bit-fiddle either side of the
      // limit depending on the values. Testing the boundary to the last bit would be testing
      // IEEE-754, not the gate; the gate's two real behaviours - just inside survives, just
      // outside is dropped - are both covered, and the outside case is the one above.
      const double just_inside = shipped.max_age_s - 1e-9;
      const Tracking2DResult old = MakeTracking({MakePair(1, now - just_inside, 1.0, 8)});
      Check(generator.Generate(old, now, &result) == nullptr, "a just-in-time frame generates");
      Check(result.obstacles.size() == 1, "a track just inside max_age_s survives",
            std::to_string(result.obstacles.size()) + " emitted");
      if (result.obstacles.size() == 1 && fresh_result.obstacles.size() == 1) {
        Check(result.obstacles.front().radius_inflated_m >
                  fresh_result.obstacles.front().radius_inflated_m,
              "an older state is inflated MORE, never less - the policy in one inequality",
              F(result.obstacles.front().radius_inflated_m) + " > " +
                  F(fresh_result.obstacles.front().radius_inflated_m));
        Check(Near(result.obstacles.front().age_s, just_inside, 1e-9),
              "and its reported age is the elapsed time since the MEASUREMENT stamp");
      }
    }

    // A stamp in the future is a clock fault, not a fresh obstacle.
    {
      const Tracking2DResult tracking = MakeTracking({MakePair(1, now + 0.05, 1.0, 8)});
      generator.Generate(tracking, now, &result);
      Check(result.obstacles.empty() && result.stats.dropped_future_stamp == 1,
            "a state stamped in the future is dropped, not clamped to fresh");
    }

    // min_track_age and min_track_hits.
    {
      const Tracking2DResult young =
          MakeTracking({MakePair(1, now, shipped.min_track_age_s - 1e-6, 8)});
      generator.Generate(young, now, &result);
      Check(result.obstacles.empty() && result.stats.dropped_young_age == 1,
            "a track younger than min_track_age_s is dropped");

      const Tracking2DResult few =
          MakeTracking({MakePair(1, now, 1.0, shipped.min_track_hits - 1)});
      generator.Generate(few, now, &result);
      Check(result.obstacles.empty() && result.stats.dropped_few_hits == 1,
            "a track with fewer than min_track_hits matched scans is dropped");
    }

    // The hits join is what makes min_track_hits enforceable at all: the count is not on
    // PerceptionObstacle and cannot be recovered from `confidence` (a ratio) or `track_age_s`
    // (a duration). An obstacle with no matching track is dropped rather than assumed good.
    {
      Tracking2DResult orphan = MakeTracking({MakePair(1, now, 1.0, 8)});
      orphan.tracks.clear();
      generator.Generate(orphan, now, &result);
      Check(result.obstacles.empty() && result.stats.dropped_no_track == 1,
            "an obstacle with no matching TrackState2D is dropped, not assumed confirmed");
    }

    // A min_track_hits STRICTER than the tracker's confirm_hits must actually bite. If the join
    // were absent this would silently pass everything through, which is the failure the join
    // exists to prevent.
    {
      SafetyParams strict = shipped;
      strict.min_track_hits = 6;
      SafetyStateGenerator picky(strict);
      const Tracking2DResult tracking =
          MakeTracking({MakePair(1, now, 1.0, 5), MakePair(2, now, 1.0, 6)});
      picky.Generate(tracking, now, &result);
      Check(result.obstacles.size() == 1 && result.obstacles.front().id == 2,
            "a min_track_hits above tracking.confirm_hits gates on the real hit count",
            std::to_string(result.obstacles.size()) + " emitted");
    }

    // Tentative tracks are counted and never emitted.
    {
      Tracking2DResult tracking = MakeTracking({MakePair(1, now, 1.0, 8)});
      TrackState2D tentative = MakePair(2, now, 0.0, 1).track;
      tentative.status = TrackStatus::kTentative;
      tentative.hits = 1;
      tracking.tracks.push_back(tentative);  // In `tracks` only - the tracker never publishes it.
      generator.Generate(tracking, now, &result);
      Check(result.obstacles.size() == 1 && result.stats.tentative_suppressed == 1,
            "a tentative track is counted and not emitted");
    }

    // Capacity: freshest-first, and the census still balances.
    {
      SafetyParams capped = shipped;
      capped.max_obstacles = 2;
      SafetyStateGenerator small(capped);
      std::vector<TrackedPair> pairs;
      for (std::uint32_t id = 1; id <= 4; ++id) {
        // Increasing age with id, so the expected survivors are ids 1 and 2.
        pairs.push_back(MakePair(id, now - 0.01 * id, 1.0, 8));
      }
      small.Generate(MakeTracking(pairs), now, &result);
      Check(result.obstacles.size() == 2 && result.stats.dropped_capacity == 2,
            "the capacity cull keeps max_obstacles and counts the rest",
            std::to_string(result.obstacles.size()) + " kept");
      Check(result.obstacles.size() == 2 && result.obstacles[0].id == 1 &&
                result.obstacles[1].id == 2,
            "and it keeps the FRESHEST states, in the tracker's emission order");
      Check(result.stats.IsBalanced(), "the census balances after a capacity cull");
    }
  }

  // -------------------------------------------------------------------------------------
  Section("D. Velocity spike clamping - fault injection (risk R10, doc section 17)");
  // -------------------------------------------------------------------------------------
  {
    SafetyStateGenerator generator(shipped);
    SafetyStateResult result;

    // A NORMAL velocity passes through untouched. Asserted first, because a clamp that fires on
    // everything would satisfy the spike test and destroy the estimator.
    {
      TrackedPair pair = MakePair(1, 5.0, 1.0, 8);
      pair.obstacle.velocity = Eigen::Vector2d(0.48, 0.64);  // 0.80 m/s, the arena's maximum.
      pair.track.velocity = pair.obstacle.velocity;
      generator.Generate(MakeTracking({pair}), 5.0, &result);
      Check(result.obstacles.size() == 1 &&
                (result.obstacles.front().velocity - pair.obstacle.velocity).norm() < 1e-15,
            "a 0.80 m/s velocity - the arena maximum - passes through unclamped");
      Check(result.stats.velocity_clamped == 0, "and the clamp counter does not move");
    }

    // THE SPIKE, injected into a real KF rate state rather than assigned. Getting it there took
    // one correction worth recording: a single 3 m displacement does NOT produce a spike,
    // because the association cost exceeds `min_correspondence_cost_m` (0.30 m) and the tracker
    // simply drops the observation and coasts. The rate channel can only be driven by
    // displacements that stay INSIDE the gate, which is what an association DRAG looks like -
    // several consecutive scans each pulled a little further in the same direction, the exact
    // failure risk R10 names ("velocity from differentiated noisy centers").
    //
    // The drag is 0.25 m per scan - just inside the 0.30 m gate, since the residual against the
    // prediction is the amount by which the drag OUTRUNS the filter's current rate estimate -
    // sustained for 14 scans, which is what it takes for the rate channel to converge. The
    // apparent speed is then about 3.2 m/s against an arena whose obstacles top out at 0.8 m/s
    // (dpcbf_config.yaml `speed_range`), so the resulting rate state is a fault by construction
    // and not a fast obstacle. The number of scans is itself a finding: this filter damps a
    // velocity fault hard, and a short spike does not survive it.
    {
      tracking_scenes::Stream stream = tracking_scenes::ConstantVelocity();
      const std::size_t spike_frame = 20;
      const std::size_t spike_scans = 14;
      Check(spike_frame + spike_scans <= stream.frames.size() &&
                !stream.frames[spike_frame].observations.empty(),
            "the spike frames exist in the fixture");
      for (std::size_t f = spike_frame; f < spike_frame + spike_scans; ++f) {
        const double drag = 0.25 * static_cast<double>(f - spike_frame + 1);
        stream.frames[f].observations.front().center += Eigen::Vector2d(drag, 0.0);
      }

      KfCircleTracker tracker(tracking_params);
      SafetyStateGenerator spiky(shipped);
      Tracking2DResult tracking;
      SafetyStateResult spike_result;

      double worst_track_speed = 0.0;
      double worst_emitted_speed = 0.0;
      int clamped_frames = 0;
      int emitted = 0;
      for (const tracking_scenes::Frame& frame : stream.frames) {
        tracker.Update(frame.observations, frame.stamp_s, &tracking);
        spiky.Generate(tracking, frame.stamp_s, &spike_result);
        for (const PerceptionObstacle& obstacle : tracking.obstacles) {
          worst_track_speed = std::max(worst_track_speed, obstacle.velocity.norm());
        }
        for (const SafetyObstacle& obstacle : spike_result.obstacles) {
          worst_emitted_speed = std::max(worst_emitted_speed, obstacle.velocity.norm());
          ++emitted;
        }
        clamped_frames += spike_result.stats.velocity_clamped;
      }

      std::printf("      injected spike: worst KF rate state %.3f m/s -> worst emitted %.3f m/s"
                  " (clamp %.2f m/s, fired on %d of %d emissions)\n",
                  worst_track_speed, worst_emitted_speed, shipped.max_speed_mps, clamped_frames,
                  emitted);
      Check(worst_track_speed > shipped.max_speed_mps,
            "the injected spike really does drive the KF rate state past the clamp",
            F(worst_track_speed, 3) + " m/s");
      Check(worst_emitted_speed <= shipped.max_speed_mps + 1e-12,
            "and no emitted SafetyObstacle exceeds max_speed_mps",
            F(worst_emitted_speed, 3) + " m/s");
      Check(clamped_frames > 0, "the clamp counter records that it fired",
            std::to_string(clamped_frames));
    }

    // Direction is preserved, and the obstacle is NOT dropped: a spike is evidence of a real
    // object tracked badly, not evidence of no object.
    {
      TrackedPair pair = MakePair(1, 5.0, 1.0, 8);
      pair.obstacle.velocity = Eigen::Vector2d(6.0, 8.0);  // 10 m/s.
      pair.track.velocity = pair.obstacle.velocity;
      generator.Generate(MakeTracking({pair}), 5.0, &result);
      Check(result.obstacles.size() == 1, "a spiking obstacle is still published");
      if (result.obstacles.size() == 1) {
        const Eigen::Vector2d velocity = result.obstacles.front().velocity;
        Check(Near(velocity.norm(), shipped.max_speed_mps, 1e-12),
              "its speed is exactly the clamp", F(velocity.norm(), 6));
        Check(Near(velocity.normalized().dot(pair.obstacle.velocity.normalized()), 1.0, 1e-12),
              "and its direction is unchanged");
      }
    }
  }

  // -------------------------------------------------------------------------------------
  Section("E. Latency inflation - the analytic check");
  // -------------------------------------------------------------------------------------
  {
    // THE CLOSED-FORM MODEL. An obstacle whose state is stamped at t and consumed at t + L has,
    // under the constant-velocity model the tracker actually assumes, moved exactly |v| * L. The
    // published centre is the estimate at the scan time; the elapsed part of that interval is
    // `age_s` and the consumption part is `latency_inflation_s`, so the closed form for the
    // drift term is (age_s + latency_inflation_s) * |v| and nothing else.
    //
    // Isolated by zeroing every other term, so the assertion is about the drift model rather
    // than about the sum.
    SafetyParams isolated = shipped;
    isolated.radius_inflation_k_sigma = 0.0;
    isolated.radius_inflation_fixed_m = 0.0;
    isolated.min_radius_m = 1e-6;
    isolated.max_radius_m = 1e6;
    SafetyStateGenerator generator(isolated);
    SafetyStateResult result;

    bool all_exact = true;
    double worst_error = 0.0;
    for (const double speed : {0.0, 0.2, 0.5, 0.8, 1.6}) {
      // Ages strictly inside max_age_s: 0.30 lands exactly on the staleness limit and the
      // round-trip through `now - (now - age)` puts it a bit-fiddle over, which is the gate
      // working correctly rather than an interesting case for this check.
      for (const double age : {0.0, 0.05, 0.15, 0.25}) {
        const double now = 10.0;
        TrackedPair pair = MakePair(1, now - age, 1.0, 8);
        pair.obstacle.velocity = Eigen::Vector2d(speed, 0.0);
        pair.track.velocity = pair.obstacle.velocity;
        pair.obstacle.center_variance = Eigen::Vector2d::Zero();
        pair.obstacle.radius_variance = 0.0;
        pair.track.center_variance = Eigen::Vector2d::Zero();
        pair.track.radius_variance = 0.0;
        generator.Generate(MakeTracking({pair}), now, &result);
        if (result.obstacles.size() != 1) {
          all_exact = false;
          continue;
        }
        const double expected = (age + isolated.latency_inflation_s) * speed;
        const double error = std::abs(result.obstacles.front().position_inflation_m - expected);
        worst_error = std::max(worst_error, error);
        if (error > 1e-12) all_exact = false;
        // The whole inflation is the drift term once the others are zeroed, which is the second
        // half of the claim: the term is not merely present, it is the only thing there.
        const double total =
            result.obstacles.front().radius_inflated_m - result.obstacles.front().radius_true_m;
        if (std::abs(total - expected) > 1e-12) all_exact = false;
      }
    }
    Check(all_exact,
          "position_inflation_m equals (age_s + latency_inflation_s) * |v| exactly, over a "
          "speed x age grid",
          "worst deviation " + F(worst_error, 15) + " m");

    // And the term does what it is for: a state generated at t still contains the obstacle at
    // t + latency, for a mover on the constant-velocity model. This is the closed form's
    // PURPOSE rather than its algebra, and it is a separate assertion.
    {
      const double now = 10.0;
      const double latency = isolated.latency_inflation_s;
      TrackedPair pair = MakePair(1, now, 1.0, 8);
      pair.obstacle.velocity = Eigen::Vector2d(0.8, 0.0);
      pair.track.velocity = pair.obstacle.velocity;
      pair.obstacle.center_variance = Eigen::Vector2d::Zero();
      pair.obstacle.radius_variance = 0.0;
      pair.track.center_variance = Eigen::Vector2d::Zero();
      pair.track.radius_variance = 0.0;
      generator.Generate(MakeTracking({pair}), now, &result);
      if (result.obstacles.size() == 1) {
        const SafetyObstacle& obstacle = result.obstacles.front();
        const Eigen::Vector2d future = pair.obstacle.center + pair.obstacle.velocity * latency;
        const double needed = (future - obstacle.center).norm() + pair.obstacle.radius_true_m;
        Check(obstacle.radius_inflated_m >= needed - 1e-12,
              "a state generated now still contains the obstacle one latency later",
              F(obstacle.radius_inflated_m) + " >= " + F(needed));
      } else {
        Check(false, "a state generated now still contains the obstacle one latency later");
      }
    }
  }

  // -------------------------------------------------------------------------------------
  Section("F. Q12 - the k_sigma calibration sweep");
  // -------------------------------------------------------------------------------------
  //
  // The doc gives no number for k_sigma; the phase brief makes calibrating it P10's job, against
  // measured error distributions rather than assumed Gaussians. Three corpora are swept, and the
  // reason there are three is the finding: containment turns out to depend on a DETECTION
  // parameter, so a sweep run on only one of them would report a number that is true of a
  // pipeline nobody ships.
  {
    const std::vector<double> sweep = {0.0, 1.0, 2.0, 3.0, 3.72, 5.0, 8.0, 12.0};

    auto run_moving = [&](const SafetyParams& params) {
      std::vector<ContainmentSample> samples;
      for (const auto& scene : safety_scenes::MovingScenes()) {
        RunMovingScene(scene, detector_params, tracking_params, params, &samples);
      }
      return samples;
    };
    auto run_rebased = [&](const SafetyParams& params) {
      std::vector<ContainmentSample> samples;
      for (const auto& stream :
           safety_scenes::RebasedTrackingStreams(detector_params.radius_enlargement_m)) {
        RunStream(stream, tracking_params, params, &samples);
      }
      return samples;
    };
    auto run_raw = [&](const SafetyParams& params) {
      std::vector<ContainmentSample> samples;
      for (const auto& stream : safety_scenes::RawTrackingStreams()) {
        RunStream(stream, tracking_params, params, &samples);
      }
      return samples;
    };

    std::printf("      %-34s %6s %8s %10s %12s %12s\n", "corpus", "k", "samples", "contain%",
                "worstMargin", "meanInflate");
    double smallest_passing_k_moving = -1.0;
    double smallest_passing_k_rebased = -1.0;
    for (const double k : sweep) {
      SafetyParams params = shipped;
      params.radius_inflation_k_sigma = k;

      const ContainmentSummary moving = Summarise(run_moving(params));
      const ContainmentSummary rebased = Summarise(run_rebased(params));
      const ContainmentSummary raw = Summarise(run_raw(params));

      std::printf("      %-34s %6.2f %8d %9.4f%% %12.4f %12.4f\n", "A shipped pipeline (ray-cast)",
                  k, moving.samples, 100.0 * moving.ContainmentRate(), moving.worst_margin,
                  moving.MeanInflation());
      std::printf("      %-34s %6.2f %8d %9.4f%% %12.4f %12.4f\n", "B P9 streams, radius rebased",
                  k, rebased.samples, 100.0 * rebased.ContainmentRate(), rebased.worst_margin,
                  rebased.MeanInflation());
      std::printf("      %-34s %6.2f %8d %9.4f%% %12.4f %12.4f\n", "C P9 streams, raw (no enlarge)",
                  k, raw.samples, 100.0 * raw.ContainmentRate(), raw.worst_margin,
                  raw.MeanInflation());

      if (smallest_passing_k_moving < 0.0 && moving.ContainmentRate() >= 0.999) {
        smallest_passing_k_moving = k;
      }
      if (smallest_passing_k_rebased < 0.0 && rebased.ContainmentRate() >= 0.999) {
        smallest_passing_k_rebased = k;
      }
    }

    std::printf("      smallest swept k meeting 99.9%% containment: corpus A %.2f, corpus B %.2f\n",
                smallest_passing_k_moving, smallest_passing_k_rebased);
    Check(smallest_passing_k_moving >= 0.0 &&
              smallest_passing_k_moving <= shipped.radius_inflation_k_sigma,
          "the SHIPPED k_sigma meets 99.9% containment on the shipped pipeline with margin to "
          "spare",
          "needs " + F(smallest_passing_k_moving, 2) + ", shipped is " +
              F(shipped.radius_inflation_k_sigma, 2));
    Check(smallest_passing_k_rebased >= 0.0 &&
              smallest_passing_k_rebased <= shipped.radius_inflation_k_sigma,
          "and on P9's own error realisations at the detector's radius convention",
          "needs " + F(smallest_passing_k_rebased, 2));

    // THE FINDING THIS SWEEP EXISTS TO EXPOSE. Corpus C is P9's streams with the detector's
    // 0.25 m enlargement absent. If containment there needs a k the shipped config does not
    // have, then containment on the shipped pipeline is being delivered by a DETECTION
    // parameter, not by a safety one - which is a cross-stage coupling that has to be written
    // down and enforced rather than discovered later.
    {
      SafetyParams params = shipped;
      params.radius_inflation_k_sigma = shipped.radius_inflation_k_sigma;
      const ContainmentSummary raw = Summarise(run_raw(params));
      std::printf("      corpus C at the shipped k: containment %.4f%%, worst margin %.4f m,"
                  " worst radius under-estimate %.4f m\n",
                  100.0 * raw.ContainmentRate(), raw.worst_margin,
                  raw.worst_radius_under_estimate);
      Check(raw.ContainmentRate() < 0.999,
            "recorded: without the detector's radius enlargement the shipped k_sigma does NOT "
            "reach 99.9% - the coupling is real, not theoretical",
            F(100.0 * raw.ContainmentRate(), 2) + "%");

      // AND THE FIX, MEASURED. The shortfall is a systematic bias, so the term that covers it is
      // the FIXED one, not k_sigma - and P10 added a cross-field constraint requiring
      // detection.radius_enlargement_m + safety.radius_inflation_fixed_m >= 0.20 m. Here is the
      // proof that the threshold that constraint enforces is the right one: raise the safety-side
      // term to what the constraint would demand of an enlargement-free config, and corpus C
      // reaches the target with the SHIPPED k_sigma unchanged.
      SafetyParams compensated = shipped;
      compensated.radius_inflation_fixed_m = 0.20;  // What the constraint forces at enlargement 0.
      const ContainmentSummary fixed_up = Summarise(run_raw(compensated));
      std::printf("      corpus C with radius_inflation_fixed_m raised to 0.20 m (what the new "
                  "cross-field constraint forces): containment %.4f%%, worst margin %.4f m\n",
                  100.0 * fixed_up.ContainmentRate(), fixed_up.worst_margin);
      Check(fixed_up.ContainmentRate() >= 0.999,
            "the 0.20 m short-arc bias budget the config constraint enforces is sufficient, at "
            "the shipped k_sigma - a bias is covered by a fixed term, not by a variance",
            F(100.0 * fixed_up.ContainmentRate(), 4) + "%");
    }
  }

  // -------------------------------------------------------------------------------------
  Section("F2. The chi-square coverage check section 12 asks for");
  // -------------------------------------------------------------------------------------
  {
    // NEES over the 2-D centre and NIS over the 1-D radius, both normalized by the tracker's own
    // per-axis variances. Under a correctly-tuned Gaussian filter these are chi-square with 2
    // and 1 degrees of freedom, mean 2.0 and 1.0, with 99.9% quantiles 13.816 and 10.828.
    //
    // Measured on P9's streams because those are the ones whose truth and whose error
    // realisations are both known, and rebased so the radius channel is the shipped one.
    std::vector<ContainmentSample> samples;
    for (const auto& stream :
         safety_scenes::RebasedTrackingStreams(detector_params.radius_enlargement_m)) {
      RunStream(stream, tracking_params, shipped, &samples);
    }

    std::vector<double> nees;
    std::vector<double> nis;
    for (const ContainmentSample& sample : samples) {
      if (!sample.have_nees) continue;
      nees.push_back(sample.position_nees);
      nis.push_back(sample.radius_nis);
    }

    double nees_sum = 0.0;
    for (const double value : nees) nees_sum += value;
    const double nees_mean = nees.empty() ? 0.0 : nees_sum / static_cast<double>(nees.size());
    const double nees_q999 = Quantile(nees, 0.999);
    const double nis_q999 = Quantile(nis, 0.999);

    constexpr double kChi2Dof2At999 = 13.8155;  // -2 ln(0.001).
    constexpr double kChi2Dof1At999 = 10.8276;

    std::printf("      centre NEES: n=%zu mean %.3f (chi2_2 expects 2.000), 99.9%% quantile %.3f"
                " (chi2_2 expects %.3f)\n",
                nees.size(), nees_mean, nees_q999, kChi2Dof2At999);
    std::printf("      radius NIS : n=%zu 99.9%% quantile %.3f (chi2_1 expects %.3f)\n",
                nis.size(), nis_q999, kChi2Dof1At999);

    Check(!nees.empty(), "the coverage corpus produced normalized samples");
    // THE RESULT, AND IT IS A REJECTION. The errors this pipeline makes are bias-dominated -
    // P8's short-arc centre offset and radius bias are systematic, not noise - so the filter's
    // own covariance does not describe them and the chi-square model is rejected at any usable
    // level. That is exactly why k_sigma was calibrated EMPIRICALLY above rather than read off
    // a chi-square quantile: sqrt(chi2_2(0.999)) = 3.72 would be the Gaussian answer, and on
    // this corpus it is neither necessary nor sufficient on its own.
    Check(nees_q999 > kChi2Dof2At999,
          "recorded: the centre error is NOT chi-square distributed under the filter's own "
          "covariance - it is bias-dominated, so k_sigma cannot be read off a quantile",
          "measured 99.9% NEES " + F(nees_q999, 2) + " vs chi2_2 " + F(kChi2Dof2At999, 2));
  }

  // -------------------------------------------------------------------------------------
  Section("G. Section-12 safety-generation gates, on the shipped pipeline");
  // -------------------------------------------------------------------------------------
  {
    std::vector<ContainmentSample> samples;
    std::printf("      %-26s %8s %10s %12s %12s %12s\n", "scene", "samples", "contain%",
                "worstMargin", "worstCtrErr", "worstUnder");
    for (const auto& scene : safety_scenes::MovingScenes()) {
      std::vector<ContainmentSample> scene_samples;
      RunMovingScene(scene, detector_params, tracking_params, shipped, &scene_samples);
      const ContainmentSummary summary = Summarise(scene_samples);
      std::printf("      %-26s %8d %9.4f%% %12.4f %12.4f %12.4f\n", scene.name.c_str(),
                  summary.samples, 100.0 * summary.ContainmentRate(), summary.worst_margin,
                  summary.worst_center_error, summary.worst_radius_under_estimate);
      samples.insert(samples.end(), scene_samples.begin(), scene_samples.end());
    }

    const ContainmentSummary all = Summarise(samples);
    std::printf("      corpus total: %d samples, containment %.4f%%, radius under-estimation "
                "%.4f%%, mean inflation %.4f m, worst inflation %.4f m\n",
                all.samples, 100.0 * all.ContainmentRate(), 100.0 * all.UnderEstimationRate(),
                all.MeanInflation(), all.worst_inflation);

    Check(all.samples >= 100, "the corpus is large enough for a 99.9% claim to mean anything",
          std::to_string(all.samples) + " samples");
    Check(all.ContainmentRate() >= 0.999, "section 12: conservative containment >= 99.9%",
          F(100.0 * all.ContainmentRate(), 4) + "%");
    Check(all.UnderEstimationRate() <= 0.001, "section 12: radius under-estimation rate <= 0.1%",
          F(100.0 * all.UnderEstimationRate(), 4) + "%");

    // THE STRICT FORM. The drift term is there to cover the consumption latency, so crediting it
    // in a SAME-INSTANT containment measurement lets one term's margin pay for another term's
    // shortfall. Re-measured with `latency_inflation_s` zeroed - the elapsed half of the horizon
    // stays, because that drift has genuinely already happened.
    {
      SafetyParams strict = shipped;
      strict.latency_inflation_s = 0.0;
      std::vector<ContainmentSample> strict_samples;
      for (const auto& scene : safety_scenes::MovingScenes()) {
        RunMovingScene(scene, detector_params, tracking_params, strict, &strict_samples);
      }
      const ContainmentSummary summary = Summarise(strict_samples);
      std::printf("      same-instant containment with NO latency credit: %.4f%%, worst margin "
                  "%.4f m\n",
                  100.0 * summary.ContainmentRate(), summary.worst_margin);
      Check(summary.ContainmentRate() >= 0.999,
            "containment holds without crediting the latency term - the uncertainty terms carry "
            "it on their own",
            F(100.0 * summary.ContainmentRate(), 4) + "%");
    }

    // TERM ATTRIBUTION - what the inflation is actually made of. This is the number P11 and P15
    // need: the safety radius is what DPCBF's QP sees, and "containment holds" says nothing
    // about whether the constraint set is still usable. Each term is isolated by regenerating
    // with the others zeroed, which is exact rather than inferred.
    {
      auto mean_inflation = [&](const SafetyParams& params) {
        std::vector<ContainmentSample> local;
        for (const auto& scene : safety_scenes::MovingScenes()) {
          RunMovingScene(scene, detector_params, tracking_params, params, &local);
        }
        return Summarise(local).MeanInflation();
      };
      SafetyParams none = shipped;
      none.radius_inflation_k_sigma = 0.0;
      none.radius_inflation_fixed_m = 0.0;
      none.latency_inflation_s = 0.0;
      none.min_radius_m = 1e-6;

      SafetyParams sigma_only = none;
      sigma_only.radius_inflation_k_sigma = shipped.radius_inflation_k_sigma;
      SafetyParams fixed_only = none;
      fixed_only.radius_inflation_fixed_m = shipped.radius_inflation_fixed_m;
      SafetyParams latency_only = none;
      latency_only.latency_inflation_s = shipped.latency_inflation_s;

      const double elapsed = mean_inflation(none);  // The age-driven drift, always present.
      std::printf("      mean inflation attribution over %d samples: elapsed drift %.4f m, "
                  "k_sigma terms %.4f m, fixed %.4f m, latency drift %.4f m, total %.4f m\n",
                  all.samples, elapsed, mean_inflation(sigma_only) - elapsed,
                  mean_inflation(fixed_only) - elapsed, mean_inflation(latency_only) - elapsed,
                  all.MeanInflation());
      std::printf("      FORWARD REFERENCE for P11/P15: a 0.25 m arena cylinder is presented to "
                  "DPCBF at roughly %.2f m before the filter's own s=1.05 and r_rob\n",
                  0.25 + detector_params.radius_enlargement_m + all.MeanInflation());
    }

    // Inflation is bounded as well as sufficient. An unbounded safety radius passes every
    // containment gate and destroys the QP downstream, so the cost is asserted, not just the
    // benefit. The bound is stated against the arena's largest object (0.30 m).
    Check(all.worst_inflation < 1.0,
          "and the inflation stays bounded - a circle that contains everything is not a result",
          "worst " + F(all.worst_inflation) + " m over the tracked radius");
  }

  // -------------------------------------------------------------------------------------
  Section("H. Q13 - the short-arc radius floor, decided on the numbers");
  // -------------------------------------------------------------------------------------
  {
    // The doc's candidate answer to Q13 is a hard floor at `safety.min_radius_m` (0.20 m, the
    // minimum of dpcbf_config.yaml's radius_range). The competing candidate is that the existing
    // k_sigma * sigma_r term already scales up on short arcs via the detector's 1/(1 - cos alpha)
    // dilution. Both are measured here rather than chosen.
    std::vector<ContainmentSample> samples;
    for (const auto& scene : safety_scenes::MovingScenes()) {
      RunMovingScene(scene, detector_params, tracking_params, shipped, &samples);
    }

    // (i) Does the floor ever bind? Count the emissions where it did.
    int floored = 0;
    int emissions = 0;
    {
      Detection2DResult detection;
      Tracking2DResult tracking;
      SafetyStateResult result;
      for (const auto& scene : safety_scenes::MovingScenes()) {
        // A fresh detector and tracker per scene: the detector is per-frame stateless by
        // contract, but the tracker is not, and carrying tracks across two unrelated scenes
        // would associate one scene's obstacles against another's.
        SegmentCircleDetector detector(detector_params);
        KfCircleTracker tracker(tracking_params);
        SafetyStateGenerator generator(shipped);
        for (const auto& frame : scene.frames) {
          detector.Detect(frame.scan, &detection);
          std::vector<CircleObservation> observations = detection.circles;
          for (CircleObservation& o : observations) o.frame = FrameId::kWorld;
          tracker.Update(observations, frame.stamp_s, &tracking);
          generator.Generate(tracking, frame.stamp_s, &result);
          floored += result.stats.radius_floored;
          emissions += result.stats.emitted;
        }
      }
    }

    // (ii) How large is the posterior radius sigma the k_sigma term multiplies, and how large is
    // the radius error it would have to cover?
    double worst_sigma_term = 0.0;
    double worst_radius_under = 0.0;
    for (const ContainmentSample& sample : samples) {
      worst_radius_under =
          std::max(worst_radius_under, sample.truth_radius - sample.radius_true);
    }
    // The competing candidate has to be measured on BOTH corpora, because the answer differs
    // between them and that difference is the finding. `k_sigma * sigma_r` in isolation:
    // regenerate with every other term off, then read it back as
    // (radius_inflated - radius_true - position_inflation).
    double worst_sigma_term_no_enlargement = 0.0;
    {
      SafetyParams sigma_only = shipped;
      sigma_only.radius_inflation_fixed_m = 0.0;
      sigma_only.latency_inflation_s = 0.0;
      sigma_only.min_radius_m = 1e-6;

      std::vector<ContainmentSample> sigma_samples;
      for (const auto& scene : safety_scenes::MovingScenes()) {
        RunMovingScene(scene, detector_params, tracking_params, sigma_only, &sigma_samples);
      }
      for (const ContainmentSample& sample : sigma_samples) {
        worst_sigma_term = std::max(
            worst_sigma_term,
            sample.radius_inflated - sample.radius_true - sample.position_inflation);
      }

      std::vector<ContainmentSample> raw_samples;
      for (const auto& stream : safety_scenes::RawTrackingStreams()) {
        RunStream(stream, tracking_params, sigma_only, &raw_samples);
      }
      for (const ContainmentSample& sample : raw_samples) {
        worst_sigma_term_no_enlargement = std::max(
            worst_sigma_term_no_enlargement,
            sample.radius_inflated - sample.radius_true - sample.position_inflation);
      }
    }

    std::printf("      floor bound the base radius on %d of %d emissions;"
                " worst radius under-estimate reaching safety %.4f m\n",
                floored, emissions, worst_radius_under);
    std::printf("      worst k_sigma*sigma_r term: %.4f m on the shipped pipeline, %.4f m on the "
                "same errors with the enlargement absent\n",
                worst_sigma_term, worst_sigma_term_no_enlargement);

    Check(floored == 0,
          "Q13: the 0.20 m floor NEVER binds on the shipped pipeline - the tracked radius carries "
          "the detector's 0.25 m enlargement, so it is 0.36-0.55 m where the floor is 0.20 m",
          std::to_string(floored) + " of " + std::to_string(emissions));
    Check(worst_radius_under <= 0.0,
          "Q13: and no emitted state under-estimates the true radius at all, so there is nothing "
          "for a floor to rescue",
          "worst " + F(worst_radius_under) + " m");

    // THE COMPETING CANDIDATE, AND WHY IT IS NOT AN ANSWER EITHER. The claim to test is that
    // `k_sigma * sigma_r` already scales up on short arcs via the detector's 1/(1 - cos alpha)
    // dilution, so no floor is needed. On the shipped pipeline the term IS large - but for a
    // reason that has nothing to do with the bias: the detector computes fit_residual_m against
    // the ENLARGED radius (segment_circle_detector.cpp), so the residual, and every sigma
    // derived from it, is dominated by the same 0.25 m constant. Remove the enlargement and the
    // term collapses by an order of magnitude while the bias it was supposed to cover does not
    // move at all - which is the definition of a term that is not tracking what it appears to.
    Check(worst_sigma_term_no_enlargement < 0.161,
          "Q13: with the enlargement absent, k_sigma*sigma_r collapses far below P8's worst "
          "short-arc radius bias (0.161 m) - a posterior variance cannot cover a systematic bias",
          F(worst_sigma_term_no_enlargement) + " m < 0.1613 m");
    // THE ARTEFACT, NOW FIXED - and this is the check that proves it, inverted from what it
    // originally asserted.
    //
    // P10 found sigma_r large and traced it here: `fit_residual_m` was the RMS of
    // |dist(p, centre) - radius_fitted_m| against the ENLARGED radius, so the residual and every
    // sigma derived from it carried `radius_enlargement_m` as an additive constant. This block
    // demonstrated it by running the SAME detector over the SAME scenes with only the
    // enlargement changed, and asserted the two sigmas differed by more than 3x (0.128 m vs
    // 0.024 m as measured then).
    //
    // The detector now measures the residual against the un-enlarged radius, so that dependence
    // is gone by construction. The same experiment is kept, with the assertion turned around:
    // sigma must now be INVARIANT under the enlargement. That is a stronger statement than the
    // old one and it is the property a future change could silently break.
    {
      auto mean_sigma = [&](double enlargement) {
        SegmentCircleDetectorParams probe = detector_params;
        probe.radius_enlargement_m = enlargement;
        SegmentCircleDetector detector(probe);
        Detection2DResult detection;
        double sum = 0.0;
        int count = 0;
        for (const auto& scene : safety_scenes::MovingScenes()) {
          for (const auto& frame : scene.frames) {
            if (detector.Detect(frame.scan, &detection) != nullptr) continue;
            for (const CircleObservation& circle : detection.circles) {
              sum += circle.sigma_radius_m;
              ++count;
            }
          }
        }
        return count > 0 ? sum / count : 0.0;
      };
      const double with_enlargement = mean_sigma(detector_params.radius_enlargement_m);
      const double without_enlargement = mean_sigma(0.0);
      std::printf("      same detector, same scenes, enlargement %.2f m -> mean sigma_radius_m "
                  "%.4f m; enlargement 0.00 m -> %.4f m\n",
                  detector_params.radius_enlargement_m, with_enlargement, without_enlargement);
      Check(Near(with_enlargement, without_enlargement, 1e-12),
            "Q13: sigma_radius_m is now INVARIANT under radius_enlargement_m - the detector "
            "measures its residual against the un-enlarged radius, so a safety-side uncertainty "
            "can no longer be moved by a detection-side padding knob",
            F(with_enlargement, 6) + " m vs " + F(without_enlargement, 6) + " m");
    }

    // Which leaves the floor's real job: a degenerate radius, which is what it is kept for.
    {
      SafetyStateGenerator generator(shipped);
      SafetyStateResult result;
      TrackedPair pair = MakePair(1, 5.0, 1.0, 8);
      pair.obstacle.radius_true_m = 0.01;  // A radius filter driven near zero.
      pair.track.radius_m = 0.01;
      generator.Generate(MakeTracking({pair}), 5.0, &result);
      Check(result.obstacles.size() == 1 && result.stats.radius_floored == 1 &&
                result.obstacles.front().radius_inflated_m > shipped.min_radius_m,
            "the floor does its actual job: a degenerate 0.01 m radius is lifted to the smallest "
            "object the arena contains, then inflated on top");
    }
  }

  // -------------------------------------------------------------------------------------
  Section("I. The seam-birth scenario - a NEW obstacle first seen inside the seam hole");
  // -------------------------------------------------------------------------------------
  {
    // P9 established that an EXISTING track survives a seam crossing (0 id switches, present in
    // 38 of 40 frames). The case P9 did not cover, and the one section 3 of this phase's brief
    // asks about, is a track that has to be BORN there: confirmation needs `confirm_hits`
    // CONSECUTIVE matched scans and a tentative track dies on its first miss, so a two-scan hole
    // that a confirmed track shrugs off resets a tentative one to zero.
    //
    // Geometry is P8's finding-4 geometry - a 0.25 m cylinder at 5 m astern - entering the scene
    // already within 0.06 rad of the +/-pi seam.
    const auto frames = safety_scenes::SeamBirth(5.0, 0.25, 1.2, 40, 0.06);

    SegmentCircleDetector detector(detector_params);
    KfCircleTracker tracker(tracking_params);
    SafetyStateGenerator generator(shipped);
    Detection2DResult detection;
    Tracking2DResult tracking;
    SafetyStateResult result;

    int detected_frames = 0;
    int confirmed_frames = 0;
    int emitted_frames = 0;
    int first_detection = -1;
    int first_emission = -1;
    int longest_detection_gap = 0;
    int gap = 0;
    int tentative_resets = 0;
    bool contained_whenever_emitted = true;
    double worst_margin = std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < frames.size(); ++i) {
      const auto& frame = frames[i];
      detector.Detect(frame.scan, &detection);
      std::vector<CircleObservation> observations = detection.circles;
      for (CircleObservation& o : observations) o.frame = FrameId::kWorld;

      tracker.Update(observations, frame.stamp_s, &tracking);
      generator.Generate(tracking, frame.stamp_s, &result);

      if (!observations.empty()) {
        ++detected_frames;
        if (first_detection < 0) first_detection = static_cast<int>(i);
        gap = 0;
      } else {
        ++gap;
        longest_detection_gap = std::max(longest_detection_gap, gap);
      }
      tentative_resets += tracking.stats.tracks_deleted_tentative_miss;
      if (!tracking.obstacles.empty()) ++confirmed_frames;
      if (!result.obstacles.empty()) {
        ++emitted_frames;
        if (first_emission < 0) first_emission = static_cast<int>(i);
        for (const SafetyObstacle& obstacle : result.obstacles) {
          const double margin = obstacle.radius_inflated_m -
                                ((obstacle.center - frame.truth_center).norm() +
                                 frame.truth_radius);
          worst_margin = std::min(worst_margin, margin);
          if (margin < 0.0) contained_whenever_emitted = false;
        }
      }
    }
    if (!std::isfinite(worst_margin)) worst_margin = 0.0;

    std::printf("      seam birth: detected in %d/%zu frames (longest gap %d), tracker confirmed "
                "in %d, safety emitted in %d\n",
                detected_frames, frames.size(), longest_detection_gap, confirmed_frames,
                emitted_frames);
    std::printf("      first detection frame %d, first safety emission frame %d"
                " (%.2f s of latency), tentative tracks killed by a miss: %d\n",
                first_detection, first_emission,
                first_emission >= 0 && first_detection >= 0
                    ? (first_emission - first_detection) * safety_scenes::kScanPeriodS
                    : -1.0,
                tentative_resets);

    Check(detected_frames > 0, "the object is detected at all during the crossing");
    Check(longest_detection_gap > 0,
          "the seam hole is genuinely present in this geometry - the scenario is not vacuous",
          std::to_string(longest_detection_gap) + " scans");
    Check(emitted_frames > 0,
          "a NEW obstacle born inside the seam hole does eventually reach the safety stage",
          std::to_string(emitted_frames) + " frames");
    Check(contained_whenever_emitted,
          "and whenever it is published, the safety circle contains it",
          "worst margin " + F(worst_margin) + " m");

    // THE COST, MEASURED. The gap between "the detector first saw it" and "the safety stage
    // first published it" is the confirmation delay plus min_track_age_s, and it is the price of
    // the decision not to emit tentative tracks. It is asserted against section 12's own
    // confirmation-delay budget so the price stays bounded rather than merely known.
    // Counted in SCANS, not in seconds: the budget is a scan count (confirm_hits) plus a
    // duration (min_track_age_s) plus the seam gap, and comparing the float product against a
    // float literal makes the assertion turn on a last-bit rounding rather than on the delay.
    const int emission_latency_scans =
        first_emission >= 0 && first_detection >= 0 ? first_emission - first_detection : 1 << 20;
    const int budget_scans = tracking_params.confirm_hits +
                             static_cast<int>(std::ceil(shipped.min_track_age_s /
                                                        safety_scenes::kScanPeriodS)) +
                             longest_detection_gap;
    Check(emission_latency_scans <= budget_scans,
          "the price of suppressing tentative tracks is bounded by confirm_hits + min_track_age "
          "+ the seam gap, and nothing else",
          std::to_string(emission_latency_scans) + " scans <= " + std::to_string(budget_scans) +
              " (" + std::to_string(tracking_params.confirm_hits) + " confirm + " +
              F(shipped.min_track_age_s, 2) + " s age + " +
              std::to_string(longest_detection_gap) + " gap)");

    // And the decision itself, asserted as behaviour rather than left as a comment: at no point
    // does a tentative track become a SafetyObstacle.
    Check(emitted_frames <= confirmed_frames,
          "no frame publishes a safety state without the tracker having confirmed a track first",
          std::to_string(emitted_frames) + " <= " + std::to_string(confirmed_frames));
  }

  // -------------------------------------------------------------------------------------
  Section("J. use_enclosing_radius is inert, and that is asserted rather than claimed");
  // -------------------------------------------------------------------------------------
  {
    // The doc's formula is max(fitted, enclosing). `radius_enclosing_m` is a DETECTION-stage
    // field that does not survive tracking, so the term has no operand at this seam. That is a
    // deliberate decision - over the whole P8 corpus the enclosing radius is SMALLER than the
    // fitted one in 20 cases out of 20, by 0.254-0.287 m, so plumbing a fourth filtered channel
    // through the tracker would carry a term that provably never binds. The config key is frozen
    // and retained; here is the proof it changes nothing.
    std::vector<ContainmentSample> with;
    std::vector<ContainmentSample> without;
    SafetyParams on = shipped;
    SafetyParams off = shipped;
    on.use_enclosing_radius = true;
    off.use_enclosing_radius = false;
    for (const auto& scene : safety_scenes::MovingScenes()) {
      RunMovingScene(scene, detector_params, tracking_params, on, &with);
      RunMovingScene(scene, detector_params, tracking_params, off, &without);
    }
    bool identical = with.size() == without.size();
    for (std::size_t i = 0; identical && i < with.size(); ++i) {
      identical = with[i].radius_inflated == without[i].radius_inflated &&
                  with[i].radius_true == without[i].radius_true;
    }
    Check(identical,
          "use_enclosing_radius on and off produce identical output on the whole corpus",
          std::to_string(with.size()) + " samples compared");
  }

  // -------------------------------------------------------------------------------------
  Section("K. Allocation-free steady state");
  // -------------------------------------------------------------------------------------
  {
    // Same assertion the detector and the tracker make: a per-frame std::vector on the
    // perception thread is exactly the hidden allocation risk R14 exists to forbid. Measured by
    // running a warm generator over a corpus and checking capacity never grows.
    SafetyStateGenerator generator(shipped);
    SafetyStateResult result;
    const double now = 10.0;
    std::vector<TrackedPair> pairs;
    for (std::uint32_t id = 1; id <= 8; ++id) pairs.push_back(MakePair(id, now, 1.0, 8));
    const Tracking2DResult tracking = MakeTracking(pairs);

    generator.Generate(tracking, now, &result);  // Warm the buffer.
    const std::size_t capacity = result.obstacles.capacity();
    const void* data = result.obstacles.data();
    for (int i = 0; i < 200; ++i) generator.Generate(tracking, now, &result);
    Check(result.obstacles.capacity() == capacity && result.obstacles.data() == data,
          "200 further frames on a warm buffer neither reallocate nor move the storage");
    Check(result.obstacles.size() == 8, "and every obstacle is still emitted");
  }

  return Report("perception_safety_test");
}
