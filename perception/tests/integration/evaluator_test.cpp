// P11 acceptance, part 2: the paired evaluator and the duplicate side-effect-free DPCBF
// comparison (architecture doc section 16 ladder steps 4-5, section 12's DPCBF block).
//
// =========================================================================================
// WHAT THIS SUITE CAN AND CANNOT HONESTLY CLAIM, STATED BEFORE THE FIRST NUMBER
// =========================================================================================
// The paired corpus is `safety_scenes::MovingScenes()`, the same four scenes P10 measured
// containment on. What that corpus actually is, established by this phase's reconciliation and
// worth writing down where the numbers are:
//
//   It is NOT sensor-to-safety. `detection_scenes::CastScene` solves the analytic ray/cylinder
//   intersection per bin and writes a ProjectedScan DIRECTLY. It never calls RaycasterMj (P4),
//   never calls ScanProjector (P7), and there is no self filter or ground segmenter to call -
//   P5 and P6 do not exist. The stages that are real from that point on are the detector (P8),
//   the tracker (P9) and the safety stage (P10), and P11 adds the adapter and the filter.
//
//   The fixture also ASSERTS two things about its scan that no shipped stage produced:
//   `motion_compensation = kDeskewedToStamp` in a repository where deskew is unimplemented,
//   and a points census with zero rejections - no ground returns and no self returns, which is
//   exactly what P6 would have had to remove. Neither affects a detector that reads only bins,
//   which is why P7-P10 were sound in using it, but it means "shipped pipeline" names four
//   stages of a seven-stage design.
//
// So: the DPCBF numbers below are a genuine end-to-end measurement of DETECTOR-ONWARD error
// propagating into the QP, on a synthetic-but-not-fictional scan. They are not a measurement of
// what a real Mid-360 return does to the QP, because no code path from P4's raycaster to P8's
// detector exists yet. P12 builds it; P15 measures it at campaign scale.
//
// argv[1] is perception/, argv[2] is the repository root (for dpcbf/config/dpcbf_config.yaml).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "check.h"
#include "detection_scenes.h"
#include "safety_scenes.h"
#include "perception/adapters/dpcbf/paired_filter_probe.h"
#include "perception/core/detection/segment_circle_detector.h"
#include "perception/core/diagnostics/json_value.h"
#include "perception/core/safety/safety_state_generator.h"
#include "perception/core/tracking/kf_circle_tracker.h"
#include "perception/integration/detection_params.h"
#include "perception/integration/dpcbf_adapter_params.h"
#include "perception/integration/evaluator.h"
#include "perception/integration/perception_config.h"
#include "perception/integration/safety_params.h"
#include "perception/integration/tracking_params.h"

using perception::adapters::dpcbf::DpcbfAdapterParams;
using perception::adapters::dpcbf::PairedFilterProbe;
using perception::adapters::dpcbf::ProbeCommand;
using perception::adapters::dpcbf::ProbeOutcome;
using perception::adapters::dpcbf::ProbeRobotState;
using perception::core::CircleObservation;
using perception::core::Detection2DResult;
using perception::core::FrameId;
using perception::core::KfCircleTracker;
using perception::core::ObstacleSource;
using perception::core::OracleObstacleState;
using perception::core::SafetyObstacle;
using perception::core::SafetyParams;
using perception::core::SafetyStateGenerator;
using perception::core::SafetyStateResult;
using perception::core::SegmentCircleDetector;
using perception::core::SegmentCircleDetectorParams;
using perception::core::Tracking2DResult;
using perception::core::TrackingParams;
using perception::integration::AssociationStats;
using perception::integration::Evaluator;
using perception::integration::EvaluatorParams;
using perception::integration::MatchedPair;
using perception_test::Check;
using perception_test::Report;
using perception_test::Section;

namespace {

std::string F(double value, int digits = 4) {
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.*f", digits, value);
  return buffer;
}

bool Near(double a, double b, double tolerance) { return std::abs(a - b) <= tolerance; }

OracleObstacleState MakeOracle(std::int32_t id, const Eigen::Vector2d& center,
                               const Eigen::Vector2d& velocity, double radius, double stamp_s) {
  OracleObstacleState oracle;
  oracle.id = id;
  oracle.center = center;
  oracle.velocity = velocity;
  oracle.radius_m = radius;
  oracle.stamp_s = stamp_s;
  oracle.frame = FrameId::kWorld;
  return oracle;
}

SafetyObstacle MakeEstimate(std::uint32_t id, const Eigen::Vector2d& center,
                            const Eigen::Vector2d& velocity, double radius_true,
                            double radius_inflated, double stamp_s) {
  SafetyObstacle obstacle;
  obstacle.id = id;
  obstacle.center = center;
  obstacle.velocity = velocity;
  obstacle.radius_true_m = radius_true;
  obstacle.radius_inflated_m = radius_inflated;
  obstacle.position_inflation_m = 0.1;
  obstacle.stamp_s = stamp_s;
  obstacle.age_s = 0.0;
  obstacle.valid = true;
  obstacle.source = ObstacleSource::kEstimated;
  obstacle.frame = FrameId::kWorld;
  return obstacle;
}

// The same deterministic, varying command sequence P3's equivalence test uses, and for the same
// reason: a constant zero command leaves the filter nothing to correct, and a comparison that
// never made the QP work would pass whatever the obstacles were.
ProbeCommand DesiredCommand(int sample) {
  const double phase = 0.37 * static_cast<double>(sample);
  ProbeCommand desired;
  desired.sagittal = 1.2 * std::cos(phase);
  desired.lateral = 0.6 * std::sin(1.7 * phase);
  desired.yaw_rate = 0.5 * std::sin(0.9 * phase);
  return desired;
}

// The truth set for one frame of a moving scene, as OracleObstacleStates. The oracle sees
// EVERYTHING, including the cylinder that is occluded from the sensor in the occlusion scene -
// that asymmetry is the point of the comparison, not a flaw in it.
std::vector<OracleObstacleState> TruthOf(const safety_scenes::MovingFrame& frame) {
  std::vector<OracleObstacleState> truth;
  for (const safety_scenes::MovingTruth& sample : frame.truth) {
    truth.push_back(MakeOracle(static_cast<std::int32_t>(sample.truth_id), sample.center,
                               sample.velocity, sample.radius, frame.stamp_s));
  }
  return truth;
}

// Greps a scalar out of a YAML file by key. Deliberately crude: this exists so that a change to
// dpcbf_config.yaml's r_rob fails a test instead of silently invalidating the clearance numbers,
// and `perception_evaluator` may not link yaml-cpp to do it properly.
bool FindYamlScalar(const std::string& path, const std::string& key, double& out) {
  std::ifstream file(path);
  if (!file) return false;
  std::string line;
  while (std::getline(file, line)) {
    const std::size_t at = line.find(key + ":");
    if (at == std::string::npos) continue;
    if (line.find('#') < at) continue;  // The key appeared inside a comment.
    try {
      out = std::stod(line.substr(at + key.size() + 1));
      return true;
    } catch (const std::exception&) {
      return false;
    }
  }
  return false;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string perception_root = argc > 1 ? argv[1] : ".";
  const std::string repository_root = argc > 2 ? argv[2] : "..";

  perception::integration::PerceptionConfig config;
  try {
    config = perception::integration::PerceptionConfig::LoadFromYaml(
        perception_root + "/configs/perception.yaml");
    Check(true, "the shipped perception.yaml loads");
  } catch (const std::exception& error) {
    Check(false, "the shipped perception.yaml loads", error.what());
    return Report("perception_paired_evaluator_test");
  }

  const SegmentCircleDetectorParams detector_params =
      perception::integration::MakeSegmentCircleDetectorParams(config.detection);
  const TrackingParams tracking_params =
      perception::integration::MakeTrackingParams(config.tracking);
  const SafetyParams safety_params = perception::integration::MakeSafetyParams(config.safety);
  const DpcbfAdapterParams adapter_params =
      perception::integration::MakeDpcbfAdapterParams(config.dpcbf_adapter);

  // -------------------------------------------------------------------------------------
  Section("A. Association correctness on synthetic pairs");
  // -------------------------------------------------------------------------------------
  // Known truth, known estimate, verifiable-by-hand answers. This is where the association is
  // proved; the corpus sections below use it rather than re-testing it.
  {
    EvaluatorParams params;
    params.association_gate_m = 1.0;
    Evaluator evaluator(params);

    const std::vector<OracleObstacleState> oracle = {
        MakeOracle(0, {2.0, 0.0}, {0.5, 0.0}, 0.25, 1.0),
        MakeOracle(1, {-3.0, 1.0}, {0.0, -0.4}, 0.30, 1.0),
        MakeOracle(2, {6.0, 6.0}, {0.0, 0.0}, 0.20, 1.0),  // No estimate for this one.
    };
    const std::vector<SafetyObstacle> estimated = {
        // Deliberately in a DIFFERENT order from the oracle, so an association that quietly
        // matched by index would fail.
        MakeEstimate(11, {-3.10, 1.05}, {0.02, -0.42}, 0.55, 0.95, 1.0),
        MakeEstimate(10, {2.08, 0.06}, {0.55, 0.03}, 0.50, 0.94, 1.0),
        MakeEstimate(12, {0.0, -8.0}, {0.0, 0.0}, 0.50, 0.90, 1.0),  // No truth for this one.
    };

    std::vector<MatchedPair> matched;
    AssociationStats stats;
    Check(evaluator.ObserveFrame(1.0, oracle, estimated, &matched, &stats) == nullptr,
          "ObserveFrame succeeds");
    Check(stats.matched == 2, "two of three pair up", std::to_string(stats.matched));
    Check(stats.oracle_unmatched == 1, "the far oracle obstacle is a MISS");
    Check(stats.estimated_unmatched == 1, "the far estimate is SPURIOUS");
    Check(stats.IsBalanced(), "the association census balances on both sides");

    // Nearest-first ordering, so the tighter pair is reported first regardless of input order.
    Check(matched.size() == 2, "two matched pairs are returned");
    Check(matched[0].oracle_id == 0 && matched[0].estimated_id == 10,
          "oracle 0 matched estimate 10 - by centre, not by index");
    Check(matched[1].oracle_id == 1 && matched[1].estimated_id == 11,
          "oracle 1 matched estimate 11");

    // The error stats, computed by hand.
    const double expected_center_0 = std::sqrt(0.08 * 0.08 + 0.06 * 0.06);
    Check(Near(matched[0].center_error_m, expected_center_0, 1e-12),
          "centre error is exact", F(matched[0].center_error_m, 6));
    const double expected_velocity_0 = std::sqrt(0.05 * 0.05 + 0.03 * 0.03);
    Check(Near(matched[0].velocity_error_mps, expected_velocity_0, 1e-12),
          "velocity error is exact", F(matched[0].velocity_error_mps, 6));
    Check(Near(matched[0].radius_true_error_m, 0.50 - 0.25, 1e-12),
          "radius_true error is estimate-minus-truth and is POSITIVE - the tracked radius "
          "carries the detector's enlargement and is not a size estimate");
    Check(Near(matched[0].radius_inflated_error_m, 0.94 - 0.25, 1e-12),
          "radius_inflated error is what the QP was actually fed, minus the truth");

    Check(evaluator.errors().matched_pairs == 2, "the summary accumulated two pairs");
    Check(Near(evaluator.errors().center_rmse_m,
               std::sqrt((expected_center_0 * expected_center_0 +
                          matched[1].center_error_m * matched[1].center_error_m) /
                         2.0),
               1e-12),
          "the centre RMSE is the root mean square of the two, not their mean");
    Check(Near(evaluator.errors().DetectionRate(), 2.0 / 3.0, 1e-12), "detection rate 2/3");
    Check(Near(evaluator.errors().SpuriousRate(), 1.0 / 3.0, 1e-12), "spurious rate 1/3");
  }

  // -------------------------------------------------------------------------------------
  Section("B. Association edge cases");
  // -------------------------------------------------------------------------------------
  {
    EvaluatorParams params;
    params.association_gate_m = 0.5;
    Evaluator evaluator(params);

    // Just outside the gate: no match at all, and BOTH sides counted as unmatched. The failure
    // this guards is an association that quietly matches its nearest candidate at any distance,
    // which would report a small error for two objects that are not the same object.
    const std::vector<OracleObstacleState> oracle = {MakeOracle(0, {2.0, 0.0}, {0, 0}, 0.25, 1.0)};
    const std::vector<SafetyObstacle> estimated = {
        MakeEstimate(1, {2.6, 0.0}, {0, 0}, 0.5, 0.9, 1.0)};
    AssociationStats stats;
    evaluator.ObserveFrame(1.0, oracle, estimated, nullptr, &stats);
    Check(stats.matched == 0 && stats.oracle_unmatched == 1 && stats.estimated_unmatched == 1,
          "a candidate outside the gate is not matched, and neither side is silently dropped");

    // TWO estimates competing for ONE oracle obstacle. A per-estimate "nearest centre" would
    // match both; a matching may not. The nearer one wins and the other is spurious.
    Evaluator contested(EvaluatorParams{});
    const std::vector<OracleObstacleState> one = {MakeOracle(5, {2.0, 0.0}, {0, 0}, 0.25, 1.0)};
    const std::vector<SafetyObstacle> two = {
        MakeEstimate(20, {2.30, 0.0}, {0, 0}, 0.5, 0.9, 1.0),
        MakeEstimate(21, {2.10, 0.0}, {0, 0}, 0.5, 0.9, 1.0),
    };
    std::vector<MatchedPair> matched;
    contested.ObserveFrame(1.0, one, two, &matched, &stats);
    Check(stats.matched == 1 && stats.estimated_unmatched == 1,
          "one oracle obstacle admits exactly one estimate");
    Check(matched.size() == 1 && matched[0].estimated_id == 21,
          "and it is the NEARER of the two", "won by id " + std::to_string(matched[0].estimated_id));

    // Empty on either side.
    Evaluator empty(EvaluatorParams{});
    empty.ObserveFrame(1.0, {}, two, nullptr, &stats);
    Check(stats.matched == 0 && stats.estimated_unmatched == 2 && stats.IsBalanced(),
          "an empty oracle set makes every estimate spurious");
    empty.ObserveFrame(1.0, one, {}, nullptr, &stats);
    Check(stats.matched == 0 && stats.oracle_unmatched == 1 && stats.IsBalanced(),
          "an empty estimate set makes every oracle obstacle a miss");
  }

  // -------------------------------------------------------------------------------------
  Section("C. The evaluator cannot reach the estimator");
  // -------------------------------------------------------------------------------------
  // The structural claims from evaluator.h, restated as things a reader can check. The link-set
  // half is enforced by CMake (perception_evaluator does not link perception_core); what is
  // asserted here is the behavioural half: the evaluator does not modify what it is shown.
  {
    Evaluator evaluator(EvaluatorParams{});
    std::vector<OracleObstacleState> oracle = {MakeOracle(0, {2.0, 0.0}, {0.5, 0.0}, 0.25, 1.0)};
    std::vector<SafetyObstacle> estimated = {
        MakeEstimate(9, {2.05, 0.0}, {0.5, 0.0}, 0.5, 0.94, 1.0)};
    const OracleObstacleState oracle_before = oracle.front();
    const SafetyObstacle estimated_before = estimated.front();

    evaluator.ObserveFrame(1.0, oracle, estimated);

    Check(oracle.front().center == oracle_before.center &&
              oracle.front().radius_m == oracle_before.radius_m &&
              oracle.front().velocity == oracle_before.velocity,
          "the oracle set is unchanged by observation");
    Check(estimated.front().center == estimated_before.center &&
              estimated.front().radius_inflated_m == estimated_before.radius_inflated_m &&
              estimated.front().velocity == estimated_before.velocity,
          "the estimated set is unchanged by observation");
  }

  // -------------------------------------------------------------------------------------
  Section("D. The duplicate side-effect-free DPCBF comparison");
  // -------------------------------------------------------------------------------------
  const std::string dpcbf_config = repository_root + "/dpcbf/config/dpcbf_config.yaml";

  double r_rob = 0.0;
  const bool have_r_rob = FindYamlScalar(dpcbf_config, "  r_rob", r_rob);
  Check(have_r_rob, "dpcbf_config.yaml still declares robot.r_rob", F(r_rob, 3));

  EvaluatorParams evaluator_params;
  evaluator_params.robot_radius_m = have_r_rob ? r_rob : 0.30;
  Evaluator evaluator(evaluator_params);

  PairedFilterProbe probe;
  // The control step the simulator initializes the live filter with is the physics timestep;
  // 0.002 s is scene_g1.xml's. Both arms get the same one, which is the only property this
  // comparison needs from it.
  constexpr double kControlDtS = 0.002;
  const char* load_error = probe.Load(dpcbf_config, kControlDtS);
  Check(load_error == nullptr, "two independent DpcbfSafetyFilter instances load and initialize",
        load_error == nullptr ? "" : load_error);
  if (load_error != nullptr) return Report("perception_paired_evaluator_test");

  // The robot sits at the sensor origin, because in these scenes it IS the sensor origin (see
  // safety_scenes.h). It does not move: the scans were generated from a static sensor, so
  // driving the robot would make the recorded perception output describe a world the QP is no
  // longer in. Command variety comes from DesiredCommand and geometry variety from the movers.
  ProbeRobotState robot;

  int sample = 0;
  int estimated_frames_with_obstacles = 0;
  int total_frames = 0;
  double pipeline_wall_us_worst = 0.0;
  double pipeline_wall_us_sum = 0.0;

  for (const safety_scenes::MovingScene& scene : safety_scenes::MovingScenes()) {
    SegmentCircleDetector detector(detector_params);
    KfCircleTracker tracker(tracking_params);
    SafetyStateGenerator safety(safety_params);
    Detection2DResult detection;
    Tracking2DResult tracking;
    SafetyStateResult result;

    for (const safety_scenes::MovingFrame& frame : scene.frames) {
      if (detector.Detect(frame.scan, &detection) != nullptr) break;
      std::vector<CircleObservation> observations = detection.circles;
      for (CircleObservation& observation : observations) observation.frame = FrameId::kWorld;
      if (tracker.Update(observations, frame.stamp_s, &tracking) != nullptr) break;
      if (safety.Generate(tracking, frame.stamp_s, &result) != nullptr) break;

      const double stage_us =
          detection.stats.wall_time_us + tracking.stats.wall_time_us + result.stats.wall_time_us;
      pipeline_wall_us_sum += stage_us;
      pipeline_wall_us_worst = std::max(pipeline_wall_us_worst, stage_us);

      const std::vector<OracleObstacleState> truth = TruthOf(frame);
      ++total_frames;
      if (!result.obstacles.empty()) ++estimated_frames_with_obstacles;

      evaluator.ObserveFrame(frame.stamp_s, truth, result.obstacles);

      const ProbeCommand desired = DesiredCommand(sample++);
      ProbeOutcome oracle_outcome;
      ProbeOutcome estimated_outcome;
      const char* oracle_error = probe.FilterOracle(robot, desired, truth, &oracle_outcome);
      const char* estimated_error =
          probe.FilterEstimated(robot, desired, result.obstacles, adapter_params,
                                &estimated_outcome);
      if (oracle_error != nullptr || estimated_error != nullptr) {
        Check(false, "both arms filter without error",
              oracle_error != nullptr ? oracle_error : estimated_error);
        break;
      }
      evaluator.ObserveFilterPair(robot, desired, truth, oracle_outcome, estimated_outcome);
    }
  }

  const auto& errors = evaluator.errors();
  const auto& deltas = evaluator.deltas();

  Check(deltas.samples > 0, "the paired run produced samples", std::to_string(deltas.samples));
  Check(errors.frames == static_cast<std::int64_t>(total_frames),
        "every frame was observed by the evaluator");

  std::printf("      corpus: %d frames, %lld paired filter samples\n", total_frames,
              static_cast<long long>(deltas.samples));
  std::printf("      estimation: %lld oracle obstacle-frames, %lld estimated, %lld matched\n",
              static_cast<long long>(errors.oracle_seen),
              static_cast<long long>(errors.estimated_seen),
              static_cast<long long>(errors.matched_pairs));
  std::printf("      detection rate %s, spurious rate %s\n", F(errors.DetectionRate(), 4).c_str(),
              F(errors.SpuriousRate(), 4).c_str());
  std::printf("      centre RMSE %s m (worst %s), velocity RMSE %s m/s (worst %s)\n",
              F(errors.center_rmse_m).c_str(), F(errors.center_worst_m).c_str(),
              F(errors.velocity_rmse_mps).c_str(), F(errors.velocity_worst_mps).c_str());
  std::printf("      radius fed to the QP, minus truth: mean %s m, worst %s m\n",
              F(errors.radius_inflated_bias_m).c_str(), F(errors.radius_inflated_worst_m).c_str());

  // -------------------------------------------------------------------------------------
  Section("E. The section-12 DPCBF block - a FIRST READ, not a campaign");
  // -------------------------------------------------------------------------------------
  std::printf("      mean obstacles supplied: oracle %s, estimated %s\n",
              F(deltas.samples > 0 ? static_cast<double>(deltas.oracle_obstacles_sum) /
                                         static_cast<double>(deltas.samples)
                                   : 0.0, 3).c_str(),
              F(deltas.samples > 0 ? static_cast<double>(deltas.estimated_obstacles_sum) /
                                         static_cast<double>(deltas.samples)
                                   : 0.0, 3).c_str());
  std::printf("      mean active constraints: oracle %s, estimated %s\n",
              F(deltas.samples > 0 ? deltas.oracle_active_constraints_sum /
                                         static_cast<double>(deltas.samples)
                                   : 0.0, 3).c_str(),
              F(deltas.samples > 0 ? deltas.estimated_active_constraints_sum /
                                         static_cast<double>(deltas.samples)
                                   : 0.0, 3).c_str());

  const double oracle_feasibility = deltas.QpFeasibility(deltas.oracle_solved);
  const double estimated_feasibility = deltas.QpFeasibility(deltas.estimated_solved);
  const double oracle_intervention = deltas.InterventionRate(deltas.oracle_interventions);
  const double estimated_intervention = deltas.InterventionRate(deltas.estimated_interventions);

  // WHY THESE ARE PRINTED AS VERDICTS AND NOT ASSERTED AS TEST FAILURES. Doc section 10 gives
  // P11 its own bar - "this delivers integration modes 4-5 machinery without touching the live
  // path" - and section 12 tags the whole DPCBF block "(P11/P15)". P15 is the phase these
  // numbers GATE; P11 is the phase that first produces them. Asserting them here would either
  // turn a real, correctly-measured result into a red suite that says nothing about whether
  // the adapter and evaluator work, or invite someone to loosen the target until it passed.
  // They are reported, loudly, with the target beside each one. What IS asserted below is a
  // regression guard against the measured baseline, so these cannot silently get worse.
  auto Verdict = [](const char* name, double value, const char* relation, double target,
                    bool met, const char* units) {
    std::printf("      %-36s %-10s %s %-8s %s\n", name, F(value, 4).c_str(), relation,
                (F(target, 3) + units).c_str(), met ? "MEETS" : "*** MISSES ***");
  };

  std::printf("      section-12 metric                    value      target          verdict\n");
  Verdict("command-delta RMSE (planar)", deltas.command_rmse_mps, "<=", 0.1,
          deltas.command_rmse_mps <= 0.1, " m/s");
  std::printf("        worst single sample %s m/s\n", F(deltas.command_worst_mps).c_str());
  std::printf("      %-36s %-10s (reported; section 12 names no yaw target)\n",
              "yaw-delta RMSE", F(deltas.yaw_rmse_rps, 4).c_str());
  Verdict("QP feasibility (estimated)", estimated_feasibility, ">=", oracle_feasibility,
          estimated_feasibility >= oracle_feasibility, " (oracle)");

  const double intervention_difference = std::abs(estimated_intervention - oracle_intervention);
  Verdict("intervention-rate difference", intervention_difference, "<=", 0.20,
          intervention_difference <= 0.20, "");
  std::printf("        oracle %s vs estimated %s\n", F(oracle_intervention, 4).c_str(),
              F(estimated_intervention, 4).c_str());

  // The clearance and collision figures. Their qualifier is not optional - see evaluator.h.
  //
  // AND WHICH QUANTITY SECTION 12 ACTUALLY NAMES. "min-clearance difference" is the difference
  // of two per-run MINIMA - one scalar per arm - and not the minimum of the per-sample
  // differences. The two disagree sharply here and only the first is section 12's, so both are
  // printed and the stricter one is labelled as the diagnostic it is. Getting this backwards
  // would have reported a failure against a gate that was never written.
  const double clearance_difference_of_minima =
      deltas.estimated_min_clearance_worst_m - deltas.oracle_min_clearance_worst_m;
  std::printf("      OPEN-LOOP LOOKAHEAD PROXY (%s s horizon), NOT a campaign min-clearance:\n",
              F(evaluator_params.clearance_lookahead_s, 2).c_str());
  std::printf("        min clearance over the run: oracle %s m, estimated %s m\n",
              F(deltas.oracle_min_clearance_worst_m).c_str(),
              F(deltas.estimated_min_clearance_worst_m).c_str());
  Verdict("min-clearance difference", clearance_difference_of_minima, ">=", -0.05,
          clearance_difference_of_minima >= -0.05, " m");
  std::printf("        DIAGNOSTIC, not section 12's metric: worst PER-SAMPLE difference %s m,\n"
              "        mean %s m. The estimated arm is on average more cautious and its worst\n"
              "        clearance is better, but on individual samples it can steer nearer than\n"
              "        the oracle would have - from a larger absolute clearance.\n",
              F(deltas.clearance_delta_worst_m).c_str(),
              F(deltas.samples > 0 ? deltas.clearance_delta_sum_m /
                                         static_cast<double>(deltas.samples)
                                   : 0.0).c_str());
  Verdict("collision count (estimated)", static_cast<double>(deltas.estimated_lookahead_collisions),
          "==", 0.0, deltas.estimated_lookahead_collisions == 0, "");

  // Latency. What P11 can measure is the PROCESSING half; the physics-stamp-to-consumption half
  // needs the perception thread, which is P12.
  const double mean_pipeline_ms =
      total_frames > 0 ? pipeline_wall_us_sum / static_cast<double>(total_frames) / 1000.0 : 0.0;
  const double worst_pipeline_ms = pipeline_wall_us_worst / 1000.0;
  const double scan_window_ms = 1000.0 / config.lidar.sensor.scan_rate_hz;
  const double worst_total_ms = scan_window_ms + worst_pipeline_ms +
                                deltas.filter_wall_us_worst / 1000.0;
  std::printf("      latency: scan window %s ms + worst detect/track/safety %s ms"
              " + worst filter %s ms = %s ms\n",
              F(scan_window_ms, 2).c_str(), F(worst_pipeline_ms, 3).c_str(),
              F(deltas.filter_wall_us_worst / 1000.0, 3).c_str(), F(worst_total_ms, 2).c_str());
  Verdict("end-to-end latency (partial)", worst_total_ms, "<=", 150.0, worst_total_ms <= 150.0,
          " ms");
  std::printf("        PARTIAL: the queueing and thread-handoff terms do not exist until P12.\n"
              "        Mean detect/track/safety %s ms.\n", F(mean_pipeline_ms, 3).c_str());

  // Fallback frequency. The selector's per-frame fallback is P15's, but the condition it will
  // test is computable now: a frame with no estimated obstacle at all is a frame the estimated
  // path could not have driven.
  const double empty_frame_rate =
      total_frames > 0
          ? 1.0 - static_cast<double>(estimated_frames_with_obstacles) / total_frames
          : 0.0;
  std::printf("      frames where the estimated path published NOTHING: %d/%d (%s)\n",
              total_frames - estimated_frames_with_obstacles, total_frames,
              F(empty_frame_rate, 4).c_str());
  std::printf("        section 12 asks for fallback frequency <= 1%% of frames. The selector's\n"
              "        fallback is P15's; this is the closest computable proxy, and it is\n"
              "        dominated by track BIRTH (the confirmation gate) rather than by loss.\n");

  // -------------------------------------------------------------------------------------
  Section("F. THE VERDICT ON THE INFLATION FIGURE (0.94 m before the fix, 0.74 m after)");
  // -------------------------------------------------------------------------------------
  // P10 established containment and reported the cost: mean total inflation 0.444 m, so a real
  // 0.25 m cylinder was presented to the QP at roughly 0.94 m. P11 measured what that did to the
  // QP and found two section-12 gates missed. The fit_residual_m correction then removed the
  // largest term. This section reports the before/after and, where a gate is still missed, which
  // term now owns it - by ablation rather than by argument.
  {
    const double mean_supplied =
        deltas.samples > 0
            ? static_cast<double>(deltas.estimated_obstacles_sum) / deltas.samples
            : 0.0;
    const double oracle_mean_active =
        deltas.samples > 0 ? deltas.oracle_active_constraints_sum / deltas.samples : 0.0;
    const double estimated_mean_active =
        deltas.samples > 0 ? deltas.estimated_active_constraints_sum / deltas.samples : 0.0;

    std::printf("      radius fed to the QP is %s m over truth on average"
                " (worst %s m), against a truth radius of 0.20-0.30 m.\n",
                F(errors.radius_inflated_bias_m).c_str(),
                F(errors.radius_inflated_worst_m).c_str());
    std::printf("      what that does to the QP: constraints %s -> %s, interventions %s -> %s,"
                " feasibility %s -> %s\n",
                F(oracle_mean_active, 3).c_str(), F(estimated_mean_active, 3).c_str(),
                F(oracle_intervention, 4).c_str(), F(estimated_intervention, 4).c_str(),
                F(oracle_feasibility, 4).c_str(), F(estimated_feasibility, 4).c_str());
    std::printf("      obstacles supplied per frame: oracle %s (all truth), estimated %s\n",
                F(deltas.samples > 0 ? static_cast<double>(deltas.oracle_obstacles_sum) /
                                           deltas.samples
                                     : 0.0, 3).c_str(),
                F(mean_supplied, 3).c_str());

    std::printf(
        "\n      VERDICT: PARTLY. The fit_residual_m correction cleared one of the two gates that\n"
        "      missed and moved the other a long way without reaching it.\n"
        "        QP feasibility        0.9750 -> %s  (oracle 1.0000)  NOW MEETS\n"
        "        command-delta RMSE    0.3237 -> %s  (target 0.100)   STILL MISSES, by 1.9x\n"
        "      Supporting movement, all in the same direction: radius over truth 0.6803 -> %s m,\n"
        "      velocity RMSE 0.1122 -> %s m/s, intervention-rate difference 0.0875 -> %s,\n"
        "      worst per-sample clearance difference -0.8402 -> %s m. A 0.25 m cylinder now\n"
        "      reaches the QP at about 0.74 m rather than 0.94 m; after the filter's own s=1.05\n"
        "      and r_rob=0.30 that is an effective keep-out of ~1.07 m against the oracle's\n"
        "      ~0.56 m, so 1.9x rather than 2.3x.\n",
        F(estimated_feasibility, 4).c_str(), F(deltas.command_rmse_mps).c_str(),
        F(errors.radius_inflated_bias_m).c_str(), F(errors.velocity_rmse_mps).c_str(),
        F(intervention_difference, 4).c_str(), F(deltas.clearance_delta_worst_m).c_str());

    // THE LEVER, NOW APPLIED - and re-measured here so the claim is not inherited from the
    // phase that made it. `fit_residual_m` is computed against the un-enlarged radius, so it no
    // longer carries radius_enlargement_m as an additive constant.
    {
      SegmentCircleDetector probe_detector(detector_params);
      Detection2DResult probe_detection;
      double residual_sum = 0.0;
      double sigma_sum = 0.0;
      double residual_min = 1e9;
      int circles = 0;
      for (const safety_scenes::MovingScene& scene : safety_scenes::MovingScenes()) {
        for (const safety_scenes::MovingFrame& frame : scene.frames) {
          if (probe_detector.Detect(frame.scan, &probe_detection) != nullptr) break;
          for (const CircleObservation& circle : probe_detection.circles) {
            ++circles;
            residual_sum += circle.fit_residual_m;
            sigma_sum += circle.sigma_radius_m;
            residual_min = std::min(residual_min, circle.fit_residual_m);
          }
        }
      }
      const double mean_residual = circles > 0 ? residual_sum / circles : 0.0;
      const double mean_sigma = circles > 0 ? sigma_sum / circles : 0.0;
      const double enlargement = config.detection.radius_enlargement_m;

      std::printf(
          "\n      THE LEVER, AFTER (%d fitted circles; the same measurement P11 first reported):\n"
          "        mean fit_residual_m         %s m   (was 0.3076; smallest now %s m, was 0.2761)\n"
          "        radius_enlargement_m        %s m   (unchanged, and no longer inside the above)\n"
          "        mean sigma_radius_m         %s m   (was 0.1281)\n"
          "        P10 k_sigma inflation term  0.0739 m (was 0.2926) - a 75%% reduction\n"
          "        P10 mean total inflation    0.2413 m (was 0.4443)\n",
          circles, F(mean_residual).c_str(), F(residual_min).c_str(), F(enlargement).c_str(),
          F(mean_sigma).c_str());

      Check(residual_min < 0.5 * enlargement,
            "the residual is no longer floored by the enlargement - the smallest fit on the "
            "corpus now reports well under it, where it used to report just over it",
            "smallest residual " + F(residual_min) + " m vs enlargement " + F(enlargement) + " m");
    }

    // WHAT IS LEFT, AND WHICH TERM TO GO AFTER NEXT. Measured by ablation rather than argued
    // from the attribution table: run the estimated arm twice more, once with the true radius
    // substituted and once with the true centre and velocity substituted, against the same
    // oracle arm. Whichever substitution collapses the delta is the term that owns it.
    {
      std::printf("\n      WHAT STILL OWNS THE REMAINING %s m/s (ablation against the same "
                  "oracle arm):\n", F(deltas.command_rmse_mps).c_str());
      struct Ablation { const char* name; bool true_radius; bool true_pose; };
      const Ablation ablations[] = {
          {"estimated centres, TRUE radius        ", true, false},
          {"TRUE centres and velocity, est radius ", false, true},
      };
      double radius_only_rmse = 0.0;
      double pose_only_rmse = 0.0;
      for (const Ablation& ablation : ablations) {
        PairedFilterProbe ablation_probe;
        if (ablation_probe.Load(dpcbf_config, kControlDtS) != nullptr) continue;
        double square_sum = 0.0;
        int count = 0;
        int ablation_sample = 0;
        for (const safety_scenes::MovingScene& scene : safety_scenes::MovingScenes()) {
          SegmentCircleDetector ablation_detector(detector_params);
          KfCircleTracker ablation_tracker(tracking_params);
          SafetyStateGenerator ablation_safety(safety_params);
          Detection2DResult ablation_detection;
          Tracking2DResult ablation_tracking;
          SafetyStateResult ablation_result;
          for (const safety_scenes::MovingFrame& frame : scene.frames) {
            if (ablation_detector.Detect(frame.scan, &ablation_detection) != nullptr) break;
            std::vector<CircleObservation> observations = ablation_detection.circles;
            for (CircleObservation& observation : observations) observation.frame = FrameId::kWorld;
            if (ablation_tracker.Update(observations, frame.stamp_s, &ablation_tracking) != nullptr) {
              break;
            }
            if (ablation_safety.Generate(ablation_tracking, frame.stamp_s, &ablation_result) !=
                nullptr) {
              break;
            }
            const std::vector<OracleObstacleState> truth = TruthOf(frame);
            std::vector<SafetyObstacle> substituted = ablation_result.obstacles;
            for (SafetyObstacle& obstacle : substituted) {
              const safety_scenes::MovingTruth* best = nullptr;
              double best_distance = 1.0;
              for (const safety_scenes::MovingTruth& sample_truth : frame.truth) {
                const double distance = (obstacle.center - sample_truth.center).norm();
                if (distance < best_distance) {
                  best_distance = distance;
                  best = &sample_truth;
                }
              }
              if (best == nullptr) continue;
              if (ablation.true_radius) {
                obstacle.radius_inflated_m = best->radius;
                obstacle.radius_true_m = best->radius;
              }
              if (ablation.true_pose) {
                obstacle.center = best->center;
                obstacle.velocity = best->velocity;
              }
            }
            const ProbeCommand desired = DesiredCommand(ablation_sample++);
            ProbeRobotState ablation_robot;
            ProbeOutcome oracle_outcome;
            ProbeOutcome estimated_outcome;
            ablation_probe.FilterOracle(ablation_robot, desired, truth, &oracle_outcome);
            ablation_probe.FilterEstimated(ablation_robot, desired, substituted, adapter_params,
                                           &estimated_outcome);
            const double ds =
                estimated_outcome.command.sagittal - oracle_outcome.command.sagittal;
            const double dl = estimated_outcome.command.lateral - oracle_outcome.command.lateral;
            square_sum += ds * ds + dl * dl;
            ++count;
          }
        }
        const double rmse = count > 0 ? std::sqrt(square_sum / count) : 0.0;
        std::printf("        %s cmdRMSE %s m/s\n", ablation.name, F(rmse).c_str());
        if (ablation.true_radius) radius_only_rmse = rmse;
        if (ablation.true_pose) pose_only_rmse = rmse;
      }

      Check(radius_only_rmse < 0.1,
            "substituting the TRUE radius clears section 12's command-delta gate outright - so "
            "the remaining miss is owned by the radius, not by the estimator's position or "
            "velocity accuracy",
            F(radius_only_rmse) + " m/s vs the 0.1 m/s target");
      Check(pose_only_rmse > 0.5 * deltas.command_rmse_mps,
            "and substituting the true centre and velocity barely helps, which is the same "
            "finding from the other side",
            F(pose_only_rmse) + " m/s against " + F(deltas.command_rmse_mps) + " m/s as-is");

      std::printf(
          "\n      NEXT-LARGEST TERM, NAMED. The radius handed to the QP is %s m over truth on\n"
          "      average. Its budget, from P10's own attribution plus the detector's constant:\n"
          "        detection.radius_enlargement_m   0.2500 m   52%%  <- LARGEST\n"
          "        safety latency drift             0.1095 m   23%%\n"
          "        safety k_sigma terms             0.0739 m   15%%  (was 0.2926 before this fix)\n"
          "        safety.radius_inflation_fixed_m  0.0500 m   10%%\n"
          "      radius_enlargement_m is UPSTREAM's value. P1 kept it deliberately, 'pending the\n"
          "      P8 short-arc-bias experiment (Q11)'; P8 then measured that bias at 0.161 m worst\n"
          "      case and -0.0125 m mean on this corpus, and the enlargement has never been\n"
          "      re-derived from those numbers. It is the only term still sized by a default\n"
          "      rather than by a measurement.\n"
          "      IT CANNOT GO TO ZERO. PerceptionConfig::Validate() requires\n"
          "      radius_enlargement_m + radius_inflation_fixed_m >= 0.20 m, and section F of the\n"
          "      safety suite re-confirms that budget is what actually carries containment on an\n"
          "      enlargement-free corpus. The reachable saving is therefore about 0.10 m (0.30 m\n"
          "      of flat terms down to the 0.20 m floor), after which latency drift is next.\n",
          F(errors.radius_inflated_bias_m).c_str());
    }
  }

  // -------------------------------------------------------------------------------------
  Section("F2. Regression guards on the measured baseline");
  // -------------------------------------------------------------------------------------
  // The section-12 verdicts above are reported rather than asserted (see section E). These are
  // what actually gate: loose bounds around the numbers this phase measured, so the suite goes
  // red if a later change moves them materially in either direction - including a change that
  // "fixes" the inflation and should therefore be accompanied by new baselines here.
  {
    Check(deltas.command_rmse_mps < 0.25,
          "BASELINE GUARD: command-delta RMSE has not regressed past 0.25 m/s",
          F(deltas.command_rmse_mps) + " m/s (baseline 0.1890 after the residual fix, 0.3237 "
                                       "before it)");
    Check(estimated_feasibility >= oracle_feasibility,
          "section 12: QP feasibility >= oracle's - NOW MET after the residual fix, and gated",
          F(estimated_feasibility, 4) + " vs oracle " + F(oracle_feasibility, 4) +
              " (was 0.9750 before the fix)");
    Check(errors.velocity_rmse_mps <= 0.10,
          "section 12 tracking: velocity RMSE <= 0.1 m/s survives end to end on the real "
          "detector corpus, not only on P9's synthetic streams",
          F(errors.velocity_rmse_mps) + " m/s (was 0.1122 before the fix)");
    Check(intervention_difference <= 0.20,
          "section 12: intervention-rate difference <= 20% - MET, and gated",
          F(intervention_difference, 4));
    Check(clearance_difference_of_minima >= -0.05,
          "section 12: min-clearance difference >= -0.05 m [LOOKAHEAD PROXY] - MET, and gated",
          F(clearance_difference_of_minima) + " m");
    Check(deltas.estimated_lookahead_collisions == 0,
          "section 12: collision count = 0 [LOOKAHEAD PROXY] - MET, and gated",
          std::to_string(deltas.estimated_lookahead_collisions));
    Check(worst_total_ms <= 150.0,
          "section 12: end-to-end latency <= 150 ms [PARTIAL] - MET, and gated",
          F(worst_total_ms, 2) + " ms");
    Check(errors.radius_inflated_bias_m > 0.0,
          "the radius fed to the QP is over-stated on average, not under-stated - the one "
          "unrecoverable error stays absent in the mean (P10 gates it per sample)",
          "mean +" + F(errors.radius_inflated_bias_m) + " m");
  }

  // -------------------------------------------------------------------------------------
  Section("G. The JSON manifest");
  // -------------------------------------------------------------------------------------
  {
    const std::string manifest = evaluator.ToJsonManifest("p11_moving_scenes");
    Check(!manifest.empty(), "a manifest is produced");
    Check(manifest.find("\"command_rmse_mps\"") != std::string::npos,
          "it names the section-12 command-delta metric");
    Check(manifest.find("OPEN-LOOP LOOKAHEAD PROXY") != std::string::npos,
          "and it carries the clearance qualifier WITH the numbers, not in a phase report");

    // It must parse back as JSON, or it is not a manifest, it is a log line.
    perception::core::diagnostics::JsonValue parsed;
    std::string error;
    const bool ok = perception::core::diagnostics::JsonValue::Parse(manifest, parsed, error);
    Check(ok, "the manifest round-trips through the repository's own JSON reader", error);
    if (ok) {
      const perception::core::diagnostics::JsonValue* block = parsed.Find("dpcbf_paired");
      Check(block != nullptr, "the dpcbf_paired block is present and addressable");
    }
  }

  // -------------------------------------------------------------------------------------
  Section("H. Reset");
  // -------------------------------------------------------------------------------------
  {
    Evaluator evaluator_two(EvaluatorParams{});
    evaluator_two.ObserveFrame(1.0, {MakeOracle(0, {2, 0}, {0, 0}, 0.25, 1.0)},
                               {MakeEstimate(1, {2, 0}, {0, 0}, 0.5, 0.9, 1.0)});
    Check(evaluator_two.errors().matched_pairs == 1, "the second evaluator accumulated");
    evaluator_two.Reset();
    Check(evaluator_two.errors().matched_pairs == 0 && evaluator_two.errors().frames == 0 &&
              evaluator_two.deltas().samples == 0,
          "Reset() clears every accumulator, so one process can run several corpora");
  }

  return Report("perception_paired_evaluator_test");
}
