// P11 acceptance, part 1: the SafetyObstacle -> dpcbf::ObstacleState conversion.
//
// Field mapping, the id policy, and the two places the id policy is NOT a property of this
// adapter at all - the tracker's fusion and fission passes. Plus the measurement the phase
// brief demands rather than assumes: what `dpcbf_adapter.max_obstacles` and `drop_invalid`
// actually do on the ESTIMATED path, whose obstacle counts have nothing to do with the
// oracle path's 90.
//
// argv[1] is perception/, for the shipped configs/perception.yaml.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "check.h"
#include "detection_scenes.h"
#include "safety_scenes.h"
#include "tracking_scenes.h"
#include "perception/adapters/dpcbf/safety_to_dpcbf.h"
#include "perception/core/detection/segment_circle_detector.h"
#include "perception/core/safety/safety_state_generator.h"
#include "perception/core/tracking/kf_circle_tracker.h"
#include "perception/integration/detection_params.h"
#include "perception/integration/dpcbf_adapter_params.h"
#include "perception/integration/perception_config.h"
#include "perception/integration/safety_params.h"
#include "perception/integration/tracking_params.h"

using perception::adapters::dpcbf::DpcbfAdapterParams;
using perception::adapters::dpcbf::DpcbfAdapterStats;
using perception::adapters::dpcbf::ToDpcbfObstacleState;
using perception::adapters::dpcbf::ToDpcbfObstacleStates;
using perception::core::CircleObservation;
using perception::core::Detection2DResult;
using perception::core::FrameId;
using perception::core::KfCircleTracker;
using perception::core::ObstacleSource;
using perception::core::SafetyObstacle;
using perception::core::SafetyParams;
using perception::core::SafetyStateGenerator;
using perception::core::SafetyStateResult;
using perception::core::SegmentCircleDetector;
using perception::core::SegmentCircleDetectorParams;
using perception::core::Tracking2DResult;
using perception::core::TrackingParams;
using perception_test::Check;
using perception_test::Report;
using perception_test::Section;

namespace {

std::string F(double value, int digits = 4) {
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.*f", digits, value);
  return buffer;
}

// A SafetyObstacle with every field distinct, so a mapping that crossed two of them would be
// visible rather than coincidentally right.
SafetyObstacle MakeSafety(std::uint32_t id) {
  SafetyObstacle obstacle;
  obstacle.id = id;
  obstacle.center = Eigen::Vector2d(2.25, -1.75);
  obstacle.velocity = Eigen::Vector2d(0.35, -0.45);
  obstacle.radius_true_m = 0.55;
  obstacle.radius_inflated_m = 0.95;
  obstacle.position_inflation_m = 0.22;
  obstacle.stamp_s = 3.5;
  obstacle.age_s = 0.05;
  obstacle.valid = true;
  obstacle.source = ObstacleSource::kEstimated;
  obstacle.frame = FrameId::kWorld;
  return obstacle;
}

// Drives one moving ray-cast scene all the way through the shipped stages to the adapter, and
// reports the per-frame estimated obstacle counts. This is the corpus the cap decision is made
// on; see the section-D comment for why the oracle-path count cannot stand in for it.
struct EstimatedCounts {
  std::vector<int> per_frame;
  int frames = 0;
  int max_count = 0;
  int total = 0;
};

void CountEstimated(const safety_scenes::MovingScene& scene,
                    const SegmentCircleDetectorParams& detector_params,
                    const TrackingParams& tracking_params, const SafetyParams& safety_params,
                    EstimatedCounts* counts) {
  SegmentCircleDetector detector(detector_params);
  KfCircleTracker tracker(tracking_params);
  SafetyStateGenerator safety(safety_params);
  Detection2DResult detection;
  Tracking2DResult tracking;
  SafetyStateResult result;

  for (const safety_scenes::MovingFrame& frame : scene.frames) {
    if (detector.Detect(frame.scan, &detection) != nullptr) return;
    std::vector<CircleObservation> observations = detection.circles;
    for (CircleObservation& observation : observations) observation.frame = FrameId::kWorld;
    if (tracker.Update(observations, frame.stamp_s, &tracking) != nullptr) return;
    if (safety.Generate(tracking, frame.stamp_s, &result) != nullptr) return;

    const int count = static_cast<int>(result.obstacles.size());
    counts->per_frame.push_back(count);
    ++counts->frames;
    counts->total += count;
    counts->max_count = std::max(counts->max_count, count);
  }
}

// The id trace of one observation stream, as the ADAPTER sees it: the set of dpcbf ids emitted
// per frame. This is the only vantage point from which "did the id survive" is a question about
// what DPCBF was told rather than about the tracker's internals.
struct IdTrace {
  std::vector<std::vector<int>> per_frame;
  std::vector<int> first_seen_frame;  // Parallel to `ids`.
  std::vector<int> ids;

  void Record(const std::vector<::dpcbf::ObstacleState>& states, int frame) {
    std::vector<int> frame_ids;
    for (const ::dpcbf::ObstacleState& state : states) {
      frame_ids.push_back(state.id);
      if (std::find(ids.begin(), ids.end(), state.id) == ids.end()) {
        ids.push_back(state.id);
        first_seen_frame.push_back(frame);
      }
    }
    std::sort(frame_ids.begin(), frame_ids.end());
    per_frame.push_back(frame_ids);
  }

  bool Present(int frame, int id) const {
    if (frame < 0 || frame >= static_cast<int>(per_frame.size())) return false;
    const std::vector<int>& frame_ids = per_frame[static_cast<std::size_t>(frame)];
    return std::find(frame_ids.begin(), frame_ids.end(), id) != frame_ids.end();
  }
};

// Runs a hand-built observation stream through the real tracker, the real safety stage and the
// real adapter. Used for fusion and fission, which are tracker DECISIONS: coaxing the detector
// into making them would test the detector's reachability instead of the id policy.
void RunObservationStream(const std::vector<std::vector<CircleObservation>>& frames,
                          double period_s, const TrackingParams& tracking_params,
                          const SafetyParams& safety_params, const DpcbfAdapterParams& adapter,
                          IdTrace* trace) {
  KfCircleTracker tracker(tracking_params);
  SafetyStateGenerator safety(safety_params);
  Tracking2DResult tracking;
  SafetyStateResult result;
  std::vector<::dpcbf::ObstacleState> states;

  for (std::size_t frame = 0; frame < frames.size(); ++frame) {
    const double stamp_s = static_cast<double>(frame) * period_s;
    std::vector<CircleObservation> observations = frames[frame];
    for (CircleObservation& observation : observations) observation.stamp_s = stamp_s;
    if (tracker.Update(observations, stamp_s, &tracking) != nullptr) return;
    if (safety.Generate(tracking, stamp_s, &result) != nullptr) return;
    ToDpcbfObstacleStates(result.obstacles, adapter, states);
    trace->Record(states, static_cast<int>(frame));
  }
}

CircleObservation Obs(const Eigen::Vector2d& center, double radius, int index) {
  return tracking_scenes::MakeObservation(center, radius, 0.0, index, 0.03, 0.015);
}

}  // namespace

int main(int argc, char** argv) {
  const std::string root = argc > 1 ? argv[1] : ".";

  // -------------------------------------------------------------------------------------
  Section("A. The shipped config reaches the adapter (integration/dpcbf_adapter_params.h)");
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
  }
  if (!config_loaded) return Report("perception_dpcbf_adapter_test");

  const DpcbfAdapterParams shipped =
      perception::integration::MakeDpcbfAdapterParams(config.dpcbf_adapter);
  Check(shipped.preserve_track_ids == config.dpcbf_adapter.preserve_track_ids,
        "preserve_track_ids reaches the adapter");
  Check(shipped.id_offset == config.dpcbf_adapter.id_offset, "id_offset reaches the adapter");
  Check(shipped.max_obstacles == config.dpcbf_adapter.max_obstacles,
        "max_obstacles reaches the adapter", std::to_string(shipped.max_obstacles));
  Check(shipped.drop_invalid == config.dpcbf_adapter.drop_invalid,
        "drop_invalid reaches the adapter");
  Check(shipped.Validate() == nullptr, "the shipped adapter params validate");

  // -------------------------------------------------------------------------------------
  Section("B. Params validation");
  // -------------------------------------------------------------------------------------
  {
    DpcbfAdapterParams params;
    params.max_obstacles = 0;
    Check(params.Validate() != nullptr, "max_obstacles = 0 is refused");
    params = DpcbfAdapterParams{};
    params.max_obstacles = -1;
    Check(params.Validate() != nullptr, "a negative max_obstacles is refused");
    params = DpcbfAdapterParams{};
    params.id_offset = -5;
    Check(params.Validate() != nullptr,
          "a negative id_offset is refused - it could drive a live id onto dpcbf's own -1 "
          "sentinel");
    Check(DpcbfAdapterParams{}.Validate() == nullptr, "the defaults validate");
  }

  // -------------------------------------------------------------------------------------
  Section("C. Field mapping");
  // -------------------------------------------------------------------------------------
  {
    const SafetyObstacle safety = MakeSafety(7);
    const ::dpcbf::ObstacleState state = ToDpcbfObstacleState(safety, DpcbfAdapterParams{});

    Check(state.x == safety.center.x(), "x <- center.x");
    Check(state.y == safety.center.y(), "y <- center.y");
    Check(state.velocity_x == safety.velocity.x(), "velocity_x <- velocity.x");
    Check(state.velocity_y == safety.velocity.y(), "velocity_y <- velocity.y");
    Check(state.id == 7, "id <- id");

    // THE ONE FIELD THAT MAKES THIS CONVERSION DIFFERENT FROM THE ORACLE ONE.
    Check(state.radius == safety.radius_inflated_m,
          "radius <- radius_inflated_m, NOT radius_true_m",
          "inflated " + F(safety.radius_inflated_m) + " vs true " + F(safety.radius_true_m));
    Check(state.radius != safety.radius_true_m,
          "the fixture actually distinguishes the two radii, so the check above can fail");

    // Bit-exactness: every field is double -> double or uint32 -> int, so the copy is exact by
    // construction rather than by luck. Asserted on a value chosen to be inexact in binary.
    SafetyObstacle awkward = MakeSafety(1);
    awkward.center = Eigen::Vector2d(1.0 / 3.0, -1.0 / 7.0);
    awkward.radius_inflated_m = 1.0 / 3.0;
    const ::dpcbf::ObstacleState exact = ToDpcbfObstacleState(awkward, DpcbfAdapterParams{});
    Check(exact.x == 1.0 / 3.0 && exact.y == -1.0 / 7.0 && exact.radius == 1.0 / 3.0,
          "the copy is bit-exact on values with no finite binary expansion");
  }

  // -------------------------------------------------------------------------------------
  Section("D. The id policy, at the conversion itself");
  // -------------------------------------------------------------------------------------
  {
    std::vector<SafetyObstacle> obstacles = {MakeSafety(3), MakeSafety(11), MakeSafety(42)};
    obstacles[1].center = Eigen::Vector2d(-1.0, 4.0);
    obstacles[2].center = Eigen::Vector2d(5.0, 0.5);

    std::vector<::dpcbf::ObstacleState> states;
    DpcbfAdapterStats stats;
    Check(ToDpcbfObstacleStates(obstacles, DpcbfAdapterParams{}, states, &stats) == nullptr,
          "the vector conversion succeeds");
    Check(states.size() == 3, "all three survive the shipped params");
    Check(states[0].id == 3 && states[1].id == 11 && states[2].id == 42,
          "track ids map 1:1 and in order (doc section 17)");
    Check(stats.IsBalanced() && stats.emitted == 3, "the census balances");

    DpcbfAdapterParams offset;
    offset.id_offset = 1000;
    ToDpcbfObstacleStates(obstacles, offset, states);
    Check(states[0].id == 1003 && states[1].id == 1011 && states[2].id == 1042,
          "id_offset shifts the whole id space and nothing else");

    DpcbfAdapterParams positional;
    positional.preserve_track_ids = false;
    ToDpcbfObstacleStates(obstacles, positional, states);
    Check(states[0].id == 0 && states[1].id == 1 && states[2].id == 2,
          "preserve_track_ids = false emits positional ids - implemented, not a no-op");

    // And that it is measurably the WRONG choice for the stated purpose: reorder the input and
    // the positional ids follow the position while the track ids follow the object.
    std::vector<SafetyObstacle> reordered = {obstacles[2], obstacles[0], obstacles[1]};
    ToDpcbfObstacleStates(reordered, positional, states);
    Check(states[0].id == 0 && states[1].id == 1 && states[2].id == 2,
          "a positional id follows the SLOT, so the same object changes id when the list moves "
          "- which is exactly the top-k hysteresis flicker the shipped default avoids");
    ToDpcbfObstacleStates(reordered, DpcbfAdapterParams{}, states);
    Check(states[0].id == 42 && states[1].id == 3 && states[2].id == 11,
          "with the shipped default the id follows the OBJECT through a reorder");

    // Order preservation among survivors, the property P3 preserved on the oracle path.
    Check(states[0].x == obstacles[2].center.x() && states[2].x == obstacles[1].center.x(),
          "the conversion is order-preserving");
  }

  // -------------------------------------------------------------------------------------
  Section("E. drop_invalid, and what it can find on this path");
  // -------------------------------------------------------------------------------------
  {
    std::vector<SafetyObstacle> obstacles = {MakeSafety(1), MakeSafety(2), MakeSafety(3)};
    obstacles[1].center = Eigen::Vector2d(-2.0, 3.0);
    obstacles[2].center = Eigen::Vector2d(4.0, 4.0);
    obstacles[1].id = 0;  // id == 0 is invalid by the contract.

    std::vector<::dpcbf::ObstacleState> states;
    DpcbfAdapterStats stats;
    DpcbfAdapterParams params;
    params.drop_invalid = true;
    ToDpcbfObstacleStates(obstacles, params, states, &stats);
    Check(states.size() == 2 && stats.dropped_invalid == 1,
          "drop_invalid omits an obstacle the contract rejects, and counts it");
    Check(states[0].id == 1 && states[1].id == 3, "the survivors keep their ids and their order");
    Check(stats.IsBalanced(), "the census balances across a drop");

    params.drop_invalid = false;
    ToDpcbfObstacleStates(obstacles, params, states, &stats);
    Check(states.size() == 3 && stats.dropped_invalid == 0,
          "drop_invalid = false passes it through - the knob is real, not decorative");
  }

  // -------------------------------------------------------------------------------------
  Section("F. Capacity");
  // -------------------------------------------------------------------------------------
  {
    std::vector<SafetyObstacle> obstacles;
    for (std::uint32_t id = 1; id <= 30; ++id) {
      SafetyObstacle obstacle = MakeSafety(id);
      obstacle.center = Eigen::Vector2d(static_cast<double>(id), 0.0);
      obstacles.push_back(obstacle);
    }
    std::vector<::dpcbf::ObstacleState> states;
    DpcbfAdapterStats stats;
    ToDpcbfObstacleStates(obstacles, shipped, states, &stats);
    Check(static_cast<int>(states.size()) == shipped.max_obstacles,
          "the cap binds at max_obstacles", std::to_string(states.size()));
    Check(stats.dropped_capacity == 30 - shipped.max_obstacles,
          "and every obstacle past it is counted, not silently lost",
          std::to_string(stats.dropped_capacity));
    Check(stats.IsBalanced(), "the census balances across a truncation");
    Check(states.front().id == 1 && states.back().id == static_cast<int>(shipped.max_obstacles),
          "the survivors are the LEADING max_obstacles in the safety stage's own emission "
          "order - the adapter does not re-rank what P10 already ranked");
  }

  // -------------------------------------------------------------------------------------
  Section("G. ID STABILITY THROUGH FUSION (N tracks -> 1 measurement)");
  // -------------------------------------------------------------------------------------
  // Two well-separated objects are tracked to confirmation, then a single observation arrives
  // between them. The tracker's fusion pass merges both tracks onto it.
  {
    TrackingParams tracking_params =
        perception::integration::MakeTrackingParams(config.tracking);
    SafetyParams safety_params =
        perception::integration::MakeSafetyParams(config.safety);

    const Eigen::Vector2d a(2.00, 0.00);
    const Eigen::Vector2d b(2.00, 0.26);  // Inside min_correspondence_cost of the midpoint.
    const Eigen::Vector2d midpoint = 0.5 * (a + b);

    std::vector<std::vector<CircleObservation>> frames;
    for (int frame = 0; frame < 8; ++frame) {
      frames.push_back({Obs(a, 0.50, 0), Obs(b, 0.50, 1)});
    }
    for (int frame = 0; frame < 6; ++frame) {
      frames.push_back({Obs(midpoint, 0.50, 0)});  // One measurement for two tracks.
    }

    IdTrace trace;
    RunObservationStream(frames, safety_scenes::kScanPeriodS, tracking_params, safety_params,
                         shipped, &trace);

    Check(trace.per_frame.size() == frames.size(), "the fusion stream ran to completion");

    const int last_two_frame = 7;
    const int after_fusion_frame = static_cast<int>(frames.size()) - 1;
    const std::size_t before = trace.per_frame[static_cast<std::size_t>(last_two_frame)].size();
    const std::size_t after = trace.per_frame[static_cast<std::size_t>(after_fusion_frame)].size();
    std::printf("      ids at the QP before the merge: %zu, after: %zu\n", before, after);

    Check(before == 2, "two ids reach DPCBF while both objects are separately observed");
    Check(after == 1, "one id reaches DPCBF after the fusion");

    // WHICH id survives, and what happens to the other. This is the finding, not an assertion
    // that ids are magically preserved.
    const std::vector<int>& pre = trace.per_frame[static_cast<std::size_t>(last_two_frame)];
    const std::vector<int>& post = trace.per_frame[static_cast<std::size_t>(after_fusion_frame)];
    const bool survivor_is_an_old_id = std::find(pre.begin(), pre.end(), post.front()) != pre.end();
    Check(survivor_is_an_old_id,
          "the surviving id is one of the two that already existed - fusion does NOT mint a new "
          "id, so a merge costs at most one disappearance and never a full re-identification",
          "survivor id " + std::to_string(post.front()));

    int vanished = -1;
    for (const int id : pre) {
      if (id != post.front()) vanished = id;
    }
    Check(vanished >= 0 && !trace.Present(after_fusion_frame, vanished),
          "the OTHER id simply stops appearing, with no event at the QP boundary",
          "vanished id " + std::to_string(vanished));
    std::printf(
        "      FINDING: fusion preserves the most-confident member's id and silently retires the\n"
        "      rest. DPCBF sees an obstacle cease to exist. That is correct for an over-\n"
        "      segmentation recovery and wrong-looking for anything else, and the two are\n"
        "      indistinguishable from the adapter's side. P12's diagnostics channel is where a\n"
        "      retirement event could be surfaced; there is nowhere to put one today.\n");
  }

  // -------------------------------------------------------------------------------------
  Section("H. ID STABILITY THROUGH FISSION (1 track -> N measurements)");
  // -------------------------------------------------------------------------------------
  {
    TrackingParams tracking_params =
        perception::integration::MakeTrackingParams(config.tracking);
    SafetyParams safety_params =
        perception::integration::MakeSafetyParams(config.safety);

    const Eigen::Vector2d whole(2.00, 0.00);
    const Eigen::Vector2d left(2.00, -0.12);
    const Eigen::Vector2d right(2.00, 0.12);

    std::vector<std::vector<CircleObservation>> frames;
    for (int frame = 0; frame < 8; ++frame) {
      frames.push_back({Obs(whole, 0.50, 0)});
    }
    for (int frame = 0; frame < 10; ++frame) {
      frames.push_back({Obs(left, 0.50, 0), Obs(right, 0.50, 1)});
    }

    IdTrace trace;
    RunObservationStream(frames, safety_scenes::kScanPeriodS, tracking_params, safety_params,
                         shipped, &trace);
    Check(trace.per_frame.size() == frames.size(), "the fission stream ran to completion");

    const int before_frame = 7;
    const int split_frame = 8;
    const int settled_frame = static_cast<int>(frames.size()) - 1;

    Check(trace.per_frame[static_cast<std::size_t>(before_frame)].size() == 1,
          "one id before the split");
    const int original = trace.per_frame[static_cast<std::size_t>(before_frame)].front();

    Check(trace.Present(split_frame, original),
          "the ORIGINAL id survives the split frame itself - the lowest-cost half continues the "
          "track, so a fission costs zero id switches for the continuing object",
          "id " + std::to_string(original));
    Check(trace.per_frame[static_cast<std::size_t>(split_frame)].size() == 1,
          "and the OTHER half is not published in the split frame: it is minted a fresh id but "
          "reset to tentative with hits = 0, so the safety stage's confirmation gate holds it "
          "back rather than handing DPCBF a split artefact");

    const std::size_t settled = trace.per_frame[static_cast<std::size_t>(settled_frame)].size();
    std::printf("      ids at the QP once the split has settled: %zu\n", settled);
    Check(settled == 2, "the heir appears later, on its own evidence");
    Check(trace.Present(settled_frame, original),
          "and the original id is still there beside it - one new id, no re-identification");

    // The delay is the confirmation gate, and it is exactly what it should be.
    int heir_frame = -1;
    for (std::size_t i = 0; i < trace.ids.size(); ++i) {
      if (trace.ids[i] != original && trace.first_seen_frame[i] > before_frame) {
        heir_frame = trace.first_seen_frame[i];
        break;
      }
    }
    const int gate = std::max(tracking_params.confirm_hits, safety_params.min_track_hits);
    Check(heir_frame >= split_frame + gate - 1,
          "the heir reaches the QP no sooner than the confirmation gate allows",
          "first seen at frame " + std::to_string(heir_frame) + ", split at " +
              std::to_string(split_frame) + ", gate " + std::to_string(gate));
    std::printf(
        "      FINDING: fission preserves the continuing object's id and delays the new one by\n"
        "      the confirmation gate. That is the safe direction on both counts - no flicker on\n"
        "      the object that was already a constraint, and no unvalidated constraint from a\n"
        "      split artefact - at the cost of a real second object arriving %d scans late.\n",
        gate);
  }

  // -------------------------------------------------------------------------------------
  Section("I. max_obstacles / drop_invalid on the ESTIMATED path - MEASURED, not inherited");
  // -------------------------------------------------------------------------------------
  // P3 measured the ORACLE path: 90 obstacles, a cap of 20 drops 70, and truncation changed
  // DpcbfSafetyFilter's output on 30/30 samples. That says nothing here. The estimated path's
  // count is set by how many objects the detector finds and the tracker confirms, which is
  // bounded by the detector's min_group_points and its range-dependent misses, not by
  // dpcbf_config.yaml's `count`.
  {
    const SegmentCircleDetectorParams detector_params =
        perception::integration::MakeSegmentCircleDetectorParams(config.detection);
    const TrackingParams tracking_params =
        perception::integration::MakeTrackingParams(config.tracking);
    const SafetyParams safety_params =
        perception::integration::MakeSafetyParams(config.safety);

    EstimatedCounts counts;
    std::printf("      scene                       frames  maxObstacles  meanObstacles\n");
    for (const safety_scenes::MovingScene& scene : safety_scenes::MovingScenes()) {
      EstimatedCounts scene_counts;
      CountEstimated(scene, detector_params, tracking_params, safety_params, &scene_counts);
      std::printf("      %-24s  %6d  %12d  %13s\n", scene.name.c_str(), scene_counts.frames,
                  scene_counts.max_count,
                  F(scene_counts.frames > 0 ? static_cast<double>(scene_counts.total) /
                                                  scene_counts.frames
                                            : 0.0, 3).c_str());
      counts.frames += scene_counts.frames;
      counts.total += scene_counts.total;
      counts.max_count = std::max(counts.max_count, scene_counts.max_count);
      counts.per_frame.insert(counts.per_frame.end(), scene_counts.per_frame.begin(),
                              scene_counts.per_frame.end());
    }

    std::printf("      corpus: %d frames, %d obstacle-frames, max %d in any single frame\n",
                counts.frames, counts.total, counts.max_count);
    Check(counts.frames > 0, "the estimated-path corpus produced frames");

    int frames_over_cap = 0;
    for (const int count : counts.per_frame) {
      if (count > shipped.max_obstacles) ++frames_over_cap;
    }
    Check(frames_over_cap == 0,
          "dpcbf_adapter.max_obstacles NEVER BINDS on the estimated path",
          "cap " + std::to_string(shipped.max_obstacles) + ", worst frame " +
              std::to_string(counts.max_count) + ", frames over cap " +
              std::to_string(frames_over_cap) + "/" + std::to_string(counts.frames));

    // And the same for drop_invalid: P10 drops rather than marks, so there is nothing here for
    // the belt-and-braces pass to find. Asserted rather than assumed.
    int invalid_seen = 0;
    {
      SegmentCircleDetector detector(detector_params);
      KfCircleTracker tracker(tracking_params);
      SafetyStateGenerator safety(safety_params);
      Detection2DResult detection;
      Tracking2DResult tracking;
      SafetyStateResult result;
      for (const safety_scenes::MovingScene& scene : safety_scenes::MovingScenes()) {
        for (const safety_scenes::MovingFrame& frame : scene.frames) {
          if (detector.Detect(frame.scan, &detection) != nullptr) break;
          std::vector<CircleObservation> observations = detection.circles;
          for (CircleObservation& observation : observations) observation.frame = FrameId::kWorld;
          if (tracker.Update(observations, frame.stamp_s, &tracking) != nullptr) break;
          if (safety.Generate(tracking, frame.stamp_s, &result) != nullptr) break;
          for (const SafetyObstacle& obstacle : result.obstacles) {
            if (!obstacle.IsValid()) ++invalid_seen;
          }
        }
      }
    }
    Check(invalid_seen == 0,
          "dpcbf_adapter.drop_invalid finds nothing on the estimated path - P10 drops rather "
          "than marks, so `valid` is true on everything the safety stage emits",
          std::to_string(invalid_seen) + " invalid obstacles seen");

    std::printf(
        "      DECISION: keep BOTH knobs at their shipped values (max_obstacles %d,\n"
        "      drop_invalid %s). Neither binds on this corpus, so neither is currently\n"
        "      changing what the QP sees, and both are cheap boundary checks worth keeping\n"
        "      against a future corpus that does reach them. This is the OPPOSITE conclusion\n"
        "      to the oracle path's for the opposite reason: there the cap would have thrown\n"
        "      away 70 real obstacles, here it has nothing to throw away.\n"
        "      CAVEAT for P12/P15: this corpus carries at most 2 movers per scene. A populated\n"
        "      90-cylinder arena driven through the real detector could plausibly confirm more\n"
        "      than %d simultaneous tracks, and safety.max_obstacles (%d) would bind first.\n",
        shipped.max_obstacles, shipped.drop_invalid ? "true" : "false", shipped.max_obstacles,
        safety_params.max_obstacles);
  }

  // -------------------------------------------------------------------------------------
  Section("J. Allocation-free steady state");
  // -------------------------------------------------------------------------------------
  {
    std::vector<SafetyObstacle> obstacles;
    for (std::uint32_t id = 1; id <= 8; ++id) {
      SafetyObstacle obstacle = MakeSafety(id);
      obstacle.center = Eigen::Vector2d(static_cast<double>(id), 1.0);
      obstacles.push_back(obstacle);
    }
    std::vector<::dpcbf::ObstacleState> states;
    ToDpcbfObstacleStates(obstacles, shipped, states);
    const std::size_t capacity = states.capacity();
    const ::dpcbf::ObstacleState* data = states.data();
    for (int iteration = 0; iteration < 200; ++iteration) {
      ToDpcbfObstacleStates(obstacles, shipped, states);
    }
    Check(states.capacity() == capacity && states.data() == data,
          "a reused output buffer does not reallocate in steady state (risk R14)");
  }

  return Report("perception_dpcbf_adapter_test");
}
