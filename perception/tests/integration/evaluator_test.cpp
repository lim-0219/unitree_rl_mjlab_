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
  Section("F. THE VERDICT ON THE INFLATION FIGURE (0.94 m at P10, 0.74 m, now 0.69 m)");
  // -------------------------------------------------------------------------------------
  // P10 established containment and reported the cost: mean total inflation 0.444 m, so a real
  // 0.25 m cylinder was presented to the QP at roughly 0.94 m. P11 measured what that did to the
  // QP and found two section-12 gates missed. Two corrections have been applied since, both to
  // CONSTANTS rather than to algorithms, and this section reports the before/after and - where a
  // gate is still missed - which term now owns it, by ablation rather than by argument.
  //
  //   1. `fit_residual_m` was computed against the ENLARGED radius, so every sigma carried
  //      `radius_enlargement_m` as an additive constant. Fixed: 0.94 m -> 0.74 m.
  //   2. `radius_enlargement_m` was upstream's 0.25 m default, the last term still sized by a
  //      default rather than a measurement. Re-derived from P8's short-arc bias to 0.17 m, with
  //      `safety.radius_inflation_fixed_m` taking 0.03 m of that back so the flat total lands at
  //      0.25 m: 0.74 m -> 0.69 m.
  //
  // THE COMMAND-DELTA GATE IS STILL MISSED AND THIS SECTION DOES NOT PRETEND OTHERWISE. It went
  // 0.3237 -> 0.1890 -> 0.1732 m/s against a 0.100 m/s target. Both of the levers that were
  // obviously available have now been pulled, and neither closed the gap.
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
        "\n      VERDICT: PARTLY, AND THE REMAINING GAP IS NOT CLOSING BY THIS ROUTE.\n"
        "        QP feasibility        0.9750 -> 1.0000 -> %s  (oracle 1.0000)  MEETS\n"
        "        command-delta RMSE    0.3237 -> 0.1890 -> %s  (target 0.100)   STILL MISSES\n"
        "      The three columns are P11 as measured, after the fit_residual_m correction, and\n"
        "      after radius_enlargement_m was re-derived from P8's short-arc bias measurement.\n"
        "      Supporting movement, all in the same direction: radius over truth\n"
        "      0.6803 -> 0.4776 -> %s m, velocity RMSE 0.1122 -> 0.0479 -> %s m/s,\n"
        "      intervention-rate difference 0.0875 -> 0.0437 -> %s, worst per-sample clearance\n"
        "      difference -0.8402 -> -0.0000 -> %s m. A 0.25 m cylinder now reaches the QP at\n"
        "      about 0.69 m rather than 0.94 m.\n"
        "\n"
        "      WHAT THE SECOND CORRECTION BOUGHT, STATED PLAINLY: 0.0158 m/s of the 0.0732 m/s\n"
        "      still outstanding. The first correction was worth 0.1347 m/s. Two constants have\n"
        "      now been re-derived from measurements and the gate is still missed by 1.73x, so\n"
        "      the remaining distance is not another mis-sized flat term - see the attribution\n"
        "      below, and the open finding attached to it.\n",
        F(estimated_feasibility, 4).c_str(), F(deltas.command_rmse_mps).c_str(),
        F(errors.radius_inflated_bias_m).c_str(), F(errors.velocity_rmse_mps).c_str(),
        F(intervention_difference, 4).c_str(), F(deltas.clearance_delta_worst_m).c_str());

    // THE LEVER, NOW APPLIED - and re-measured here so the claim is not inherited from the
    // phase that made it. `fit_residual_m` is computed against the un-enlarged radius, so it no
    // longer carries radius_enlargement_m as an additive constant.
    //
    // The same sweep also measures the detector's own MEAN de-enlarged radius error on this
    // corpus - P8's finding-2 quantity, on P11's scenes. It is the one row of the attribution
    // table below that is not a configured constant, and taking it by subtraction from the other
    // rows would be arithmetic on two harnesses' means rather than a measurement.
    double mean_deenlarged_radius_error = 0.0;
    {
      SegmentCircleDetector probe_detector(detector_params);
      Detection2DResult probe_detection;
      double residual_sum = 0.0;
      double sigma_sum = 0.0;
      double residual_min = 1e9;
      int circles = 0;
      double deenlarged_error_sum = 0.0;
      int matched_circles = 0;
      for (const safety_scenes::MovingScene& scene : safety_scenes::MovingScenes()) {
        for (const safety_scenes::MovingFrame& frame : scene.frames) {
          if (probe_detector.Detect(frame.scan, &probe_detection) != nullptr) break;
          for (const CircleObservation& circle : probe_detection.circles) {
            ++circles;
            residual_sum += circle.fit_residual_m;
            sigma_sum += circle.sigma_radius_m;
            residual_min = std::min(residual_min, circle.fit_residual_m);

            // Nearest truth centre inside the same generous gate the other harnesses use; a
            // tight gate would make the association part of the measurement.
            const safety_scenes::MovingTruth* best = nullptr;
            double best_distance = 1.0;
            for (const safety_scenes::MovingTruth& truth : frame.truth) {
              const double distance = (circle.center - truth.center).norm();
              if (distance < best_distance) {
                best_distance = distance;
                best = &truth;
              }
            }
            if (best == nullptr) continue;
            ++matched_circles;
            deenlarged_error_sum +=
                (circle.radius_fitted_m - detector_params.radius_enlargement_m) - best->radius;
          }
        }
      }
      mean_deenlarged_radius_error =
          matched_circles > 0 ? deenlarged_error_sum / matched_circles : 0.0;
      const double mean_residual = circles > 0 ? residual_sum / circles : 0.0;
      const double mean_sigma = circles > 0 ? sigma_sum / circles : 0.0;
      const double enlargement = config.detection.radius_enlargement_m;

      std::printf(
          "\n      THE LEVER, AFTER (%d fitted circles; the same measurement P11 first reported):\n"
          "        mean fit_residual_m         %s m   (was 0.3076; smallest now %s m, was 0.2761)\n"
          "        radius_enlargement_m        %s m   (re-derived from 0.25; NOT inside the above,\n"
          "                                            which is why the residual did not move with it)\n"
          "        mean sigma_radius_m         %s m   (was 0.1281)\n"
          "        safety k_sigma inflation    0.0739 m (was 0.2926) - a 75%% reduction\n"
          "        safety-side mean inflation  0.2719 m (was 0.4443; up from 0.2413 only because\n"
          "                                    radius_inflation_fixed_m absorbed 0.03 m of the\n"
          "                                    enlargement - the SUM of the two fell by 0.05 m)\n",
          circles, F(mean_residual).c_str(), F(residual_min).c_str(), F(enlargement).c_str(),
          F(mean_sigma).c_str());

      Check(residual_min < 0.5 * enlargement,
            "the residual is no longer floored by the enlargement - the smallest fit on the "
            "corpus now reports well under it, where it used to report just over it",
            "smallest residual " + F(residual_min) + " m vs enlargement " + F(enlargement) + " m");

      // THE RE-DERIVED ENLARGEMENT, CHECKED AGAINST THE THING IT WAS DERIVED FROM. P8 measured
      // the worst de-enlarged under-estimate at 0.16127 m and P10's corpus C measured 0.1668 m
      // of it surviving the filter; the enlargement is sized to cover the larger of those. This
      // asserts the relation rather than the number, so a future re-derivation that moved the
      // enlargement below what the measurement demands fails here instead of in the field.
      Check(enlargement >= 0.1668,
            "radius_enlargement_m covers the worst short-arc radius under-estimate that reaches "
            "the safety stage (P10 corpus C, 0.1668 m) on its own, without help from "
            "safety.radius_inflation_fixed_m",
            F(enlargement) + " m >= 0.1668 m");
      Check(mean_deenlarged_radius_error < 0.0 &&
                mean_deenlarged_radius_error > -enlargement,
            "and P8's finding 2 still holds on THIS corpus - the de-enlarged fit under-states "
            "the truth on average, by less than the enlargement covers",
            F(mean_deenlarged_radius_error) + " m against a " + F(enlargement) + " m enlargement");
    }

    // WHAT IS LEFT, AND WHICH TERM TO GO AFTER NEXT. Measured by ablation rather than argued
    // from the attribution table: run the estimated arm twice more, once with the true radius
    // substituted and once with the true centre and velocity substituted, against the same
    // oracle arm. Whichever substitution collapses the delta is the term that owns it.
    {
      std::printf("\n      WHAT STILL OWNS THE REMAINING %s m/s (ablation against the same "
                  "oracle arm):\n", F(deltas.command_rmse_mps).c_str());
      // THE THIRD AND FOURTH ROWS ARE NOT LEVERS - THEY ARE THE FLOOR OF THIS PATH.
      //
      // Two flat constants have now been re-derived from measurement and the gate is still
      // missed, so the question worth answering is not "which constant next" but "is 0.100 m/s
      // reachable by shrinking the safety radius at all". These two rows answer it by running
      // configurations that are deliberately NOT shippable:
      //
      //   * "flat at the strict-form minimum, latency zeroed" - the flat budget pushed down to
      //     the 0.2395 m that section G's strict-form containment gate needs, and
      //     `latency_inflation_s` set to zero, i.e. a perfect latency measurement returning
      //     nothing. This is the best a re-derivation of the remaining constants could do while
      //     still containing the obstacle.
      //   * "...and k_sigma zeroed too" - additionally discards the uncertainty inflation, which
      //     provably breaks containment on corpus C (safety section F). Included only as an
      //     unreachable lower bound.
      const double strict_form_flat_minimum = 0.2395;
      SafetyParams floor_params = safety_params;
      floor_params.latency_inflation_s = 0.0;
      floor_params.radius_inflation_fixed_m =
          strict_form_flat_minimum - detector_params.radius_enlargement_m;
      SafetyParams floor_no_sigma = floor_params;
      floor_no_sigma.radius_inflation_k_sigma = 0.0;

      struct Ablation {
        const char* name;
        bool true_radius;
        bool true_pose;
        const SafetyParams* safety_override;
      };
      const Ablation ablations[] = {
          {"estimated centres, TRUE radius             ", true, false, nullptr},
          {"TRUE centres and velocity, est radius      ", false, true, nullptr},
          {"flat at strict-form minimum, latency zeroed", false, false, &floor_params},
          {"  ...and k_sigma zeroed too (UNCONTAINED)  ", false, false, &floor_no_sigma},
      };
      double radius_only_rmse = 0.0;
      double pose_only_rmse = 0.0;
      double floor_rmse = 0.0;
      double unreachable_floor_rmse = 0.0;
      for (const Ablation& ablation : ablations) {
        PairedFilterProbe ablation_probe;
        if (ablation_probe.Load(dpcbf_config, kControlDtS) != nullptr) continue;
        const SafetyParams& ablation_params =
            ablation.safety_override != nullptr ? *ablation.safety_override : safety_params;
        double square_sum = 0.0;
        int count = 0;
        int ablation_sample = 0;
        for (const safety_scenes::MovingScene& scene : safety_scenes::MovingScenes()) {
          SegmentCircleDetector ablation_detector(detector_params);
          KfCircleTracker ablation_tracker(tracking_params);
          SafetyStateGenerator ablation_safety(ablation_params);
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
        if (ablation.safety_override == &floor_params) floor_rmse = rmse;
        if (ablation.safety_override == &floor_no_sigma) unreachable_floor_rmse = rmse;
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

      // THE DECISIVE ONE. Both floor rows still miss the target, so no re-derivation of any
      // remaining inflation constant can reach it - including a perfect latency measurement that
      // returned zero, and including a configuration that abandons containment altogether. This
      // is asserted rather than printed because it is the finding that says where NOT to look
      // next; if a future change made either row pass, the conclusion has to be revisited rather
      // than left standing as a stale "unreachable".
      std::printf("\n      NEITHER FLOOR ROW REACHES 0.100 m/s. The safety-legal floor of this\n"
                  "      path - every remaining inflation constant driven to the smallest value\n"
                  "      that still contains the obstacle - is %s m/s, and abandoning containment\n"
                  "      as well only reaches %s m/s. The gap is not a mis-sized constant.\n",
                  F(floor_rmse).c_str(), F(unreachable_floor_rmse).c_str());
      Check(floor_rmse > 0.1,
            "RECORDED: section 12's 0.100 m/s command-delta target is NOT reachable by shrinking "
            "the safety radius - the floor of the whole path, at the smallest inflation that "
            "still contains the obstacle, still misses it",
            F(floor_rmse) + " m/s against the 0.1 m/s target");
      Check(unreachable_floor_rmse > 0.1,
            "and it is not reachable even by a configuration that abandons containment, which is "
            "what makes this a property of the conservative-radius approach rather than of any "
            "one constant's value",
            F(unreachable_floor_rmse) + " m/s");
      Check(floor_rmse < deltas.command_rmse_mps,
            "the floor rows really are looser than the shipped config, so the two checks above "
            "are measuring a floor and not a broken ablation",
            F(floor_rmse) + " m/s vs " + F(deltas.command_rmse_mps) + " m/s shipped");

      std::printf(
          "\n      NEXT-LARGEST TERM, NAMED. The radius handed to the QP is %s m over truth on\n"
          "      average. Its budget, from the safety suite's own attribution plus the detector's\n"
          "      constant:\n"
          "        detection.radius_enlargement_m   0.1700 m   40%%  <- LARGEST\n"
          "        safety latency drift             0.1095 m   26%%\n"
          "        safety.radius_inflation_fixed_m  0.0800 m   19%%\n"
          "        safety k_sigma terms             0.0739 m   17%%\n"
          "        safety elapsed (age) drift       0.0090 m    2%%\n"
          "        detector's own mean radius error %s m  (MEASURED above, not taken by\n"
          "                                                   subtraction; the de-enlarged fit\n"
          "                                                   sits BELOW truth on average here)\n"
          "      The five configured terms sum to 0.4424 m; with the last row that is 0.4299 m\n"
          "      against a measured %s m, leaving 0.0017 m (0.4%%) unaccounted for - the four\n"
          "      safety terms are means over the safety suite's 171 containment samples while the\n"
          "      QP figure is a mean over this harness's associated samples, and the two sample\n"
          "      sets are close but not identical. Stated rather than rounded away.\n"
          "      The last row is the quantity P8 reports per population (-0.0061 m full-arc,\n"
          "      -0.1446 m half-arc). It was ATTRIBUTED TO P8 by the previous version of this\n"
          "      block, which is wrong - P8 never pools its two populations, and pooling them\n"
          "      over P8's own corpus gives -0.0523 m, not this. The value is a property of THIS\n"
          "      corpus, whose scenes are mostly full-arc. The number the previous version quoted\n"
          "      (-0.0125 m) is confirmed by the live measurement above; only its provenance was\n"
          "      wrong.\n"
          "\n"
          "      radius_enlargement_m IS NO LONGER A DEFAULT. P1 kept upstream's 0.25 m 'pending\n"
          "      the P8 short-arc-bias experiment (Q11)'; that experiment measured the bias at\n"
          "      0.16127 m worst case over 18 matched cylinders, P10's corpus C measured 0.1668 m\n"
          "      of it surviving the filter to the safety stage, and 0.17 m is the smallest\n"
          "      0.01 m-granular value covering both. See configs/perception.yaml.\n"
          "\n"
          "      THE FLAT BUDGET IS NOW AT ITS MEASURED FLOOR, WHICH IS NOT THE CONFIG FLOOR.\n"
          "      The prior pass projected a reachable saving of about 0.10 m, reasoning that the\n"
          "      0.30 m of flat terms could fall to the 0.20 m cross-field constraint. It cannot.\n"
          "      Section G of the safety suite measures same-instant containment with the latency\n"
          "      term zeroed, and that gate needs a flat total of 0.2395 m - its worst margin is\n"
          "      exactly 'total - 0.2395' m across the swept range. At the 0.20 m config floor it\n"
          "      reports 90.64%%. So the saving realised was 0.05 m (0.30 -> 0.25), and the flat\n"
          "      terms are done.\n"
          "\n"
          "      OPEN FINDING, NOT A NEXT LEVER. What is left at the top of the table after the\n"
          "      enlargement is safety.latency_inflation_s (0.15 s, 0.1095 m, 26%%), and it is\n"
          "      still a JUDGEMENT value - one scan period plus an estimate of processing. It\n"
          "      cannot be re-derived here, because no end-to-end latency observation exists in\n"
          "      this repository yet: the figure the section-12 latency row reports above is a\n"
          "      100 ms scan window plus sub-millisecond compute, with the queueing and\n"
          "      thread-handoff terms explicitly absent until P12. Tuning it now would be fitting\n"
          "      a constant to a number that has not been measured, which is the exact failure\n"
          "      mode the enlargement re-derivation just corrected.\n"
          "      Worse, it is no longer inert: P10 could zero it and keep 100%% containment with\n"
          "      +0.0605 m to spare, and that headroom is now +0.0105 m.\n"
          "      AND IT WOULD NOT BE ENOUGH ANYWAY - see the two floor rows in the ablation\n"
          "      above. The two obvious levers on the command-delta gate have both been pulled,\n"
          "      the gate is still missed by 1.73x, and the floor of the path is above the\n"
          "      target. That, and not a third constant, is this pass's result.\n",
          F(errors.radius_inflated_bias_m).c_str(), F(mean_deenlarged_radius_error).c_str(),
          F(errors.radius_inflated_bias_m).c_str());
      Check(std::abs(mean_deenlarged_radius_error + 0.0125) < 5e-4,
            "the attribution table's one non-configured row is the measured value, and it agrees "
            "with the figure the previous pass reported for it",
            F(mean_deenlarged_radius_error) + " m vs -0.0125 m");
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
    // The bound is tightened alongside each improvement rather than left at its original width,
    // so it keeps meaning "this has not regressed" instead of drifting into "this is under a
    // number nobody has looked at since P11". 0.20 m/s is the measured 0.1732 with the same
    // relative slack the 0.25 bound had over 0.1890.
    Check(deltas.command_rmse_mps < 0.20,
          "BASELINE GUARD: command-delta RMSE has not regressed past 0.20 m/s",
          F(deltas.command_rmse_mps) + " m/s (baseline 0.1732 after the enlargement was "
                                       "re-derived, 0.1890 after the residual fix, 0.3237 "
                                       "before either)");
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
