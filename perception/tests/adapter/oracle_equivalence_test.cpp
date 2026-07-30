// P3 acceptance: the oracle obstacle path is provably unchanged by the refactor.
//
// The claim under test is narrow and absolute - after routing DPCBF's obstacle input through
// OracleProviderMj -> adapters/dpcbf -> ObstacleSourceSelector instead of the inline
// conversion that used to sit in simulate/src/main.cc's axis-filter lambda, the numbers DPCBF
// receives, and the commands it produces from them, are bit-for-bit identical. Not close.
// Identical. Any difference is a bug in this phase, not a rounding artefact.
//
// The test attacks that claim from four independent directions, because a single comparison
// written after the refactor proves less than it looks like it does:
//
//   A. NEW PATH vs VERBATIM OLD CODE, same run, same snapshot. The old conversion is
//      transcribed into this file unchanged and run side by side with the new one.
//   B. OracleObstacleState leg vs a COMMITTED GOLDEN DUMP captured before the new modules
//      existed (tests/fixtures/oracle_golden/oracle_obstacles.jsonl, record type 20 - the
//      mechanism P2 built for exactly this). This is what stops a future edit to the
//      transcription in A from moving the goalposts along with the code.
//   C. dpcbf::ObstacleState leg vs a COMMITTED GOLDEN LOG from the same pre-change capture,
//      field by field, id for id, IN ORDER.
//   D. END TO END through the real DpcbfSafetyFilter: two filter instances, identical robot
//      and command sequence, obstacles from the old path and the new path respectively,
//      SafetyFilterResult compared field by field including the selected-constraint set.
//
// Plus the negative and structural checks that stop the above from passing vacuously:
// order-sensitivity of the comparison AND of DPCBF itself, the unfiltered-count guarantee,
// and the selector's refusal to run an unimplemented mode.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "dpcbf/dpcbf_safety_filter.h"
#include "dpcbf/dynamic_obstacles.h"

#include "check.h"
#include "oracle_golden_run.h"
#include "perception/core/diagnostics/dump_file.h"
#include "perception/integration/obstacle_source_selector.h"

using perception_test::Check;
using perception_test::Section;

namespace diagnostics = perception::core::diagnostics;
namespace integration = perception::integration;
namespace mj_adapters = perception::adapters::mujoco;
namespace dpcbf_adapter = perception::adapters::dpcbf;

namespace {

// Compares representations, not values: 0.0 vs -0.0 answer "different" and NaN vs NaN answers
// "same", which is what an exact-equality tier requires and what operator== does not give.
bool BitEqual(double a, double b) { return std::memcmp(&a, &b, sizeof(double)) == 0; }

// ===== BEGIN verbatim pre-change code, transcribed from simulate/src/main.cc ==============
// These two functions are the old implementation. They are here to be compared against, not
// to be improved. Any edit to them invalidates the comparison in section A - which is why
// sections B and C also check against fixtures captured before the new code was written.

std::vector<dpcbf::ObstacleState> InlineObstacleConversion(
    const dpcbf::DynamicObstacleManager& dynamic_obstacles) {
  std::vector<dpcbf::ObstacleState> obstacle_states;
  const auto obstacle_snapshot = dynamic_obstacles.Snapshot();
  obstacle_states.reserve(obstacle_snapshot.size());
  for (std::size_t obstacle_id = 0; obstacle_id < obstacle_snapshot.size(); ++obstacle_id) {
    const auto& obstacle = obstacle_snapshot[obstacle_id];
    obstacle_states.push_back({obstacle.position[0], obstacle.position[1], obstacle.radius,
                               obstacle.velocity[0], obstacle.velocity[1],
                               static_cast<int>(obstacle_id)});
  }
  return obstacle_states;
}

dpcbf::RobotState InlineReadRobotGroundTruth(const mjModel* model, const mjData* data,
                                             int body_id) {
  dpcbf::RobotState state;
  if (!model || !data || body_id < 0) {
    return state;
  }
  const mjtNum* position = data->xpos + 3 * body_id;
  const mjtNum* rotation = data->xmat + 9 * body_id;
  state.x = position[0];
  state.y = position[1];
  state.phi = std::atan2(rotation[3], rotation[0]);
  mjtNum body_velocity[6] = {};
  mj_objectVelocity(model, data, mjOBJ_BODY, body_id, body_velocity, 1);
  state.sagittal_velocity = body_velocity[3];
  state.lateral_velocity = body_velocity[4];
  return state;
}
// ====== END verbatim pre-change code ======================================================

bool ObstacleStateIdentical(const dpcbf::ObstacleState& a, const dpcbf::ObstacleState& b) {
  return a.id == b.id && BitEqual(a.x, b.x) && BitEqual(a.y, b.y) &&
         BitEqual(a.radius, b.radius) && BitEqual(a.velocity_x, b.velocity_x) &&
         BitEqual(a.velocity_y, b.velocity_y);
}

// Order-sensitive by construction: index i is compared against index i, never searched for.
bool VectorsIdentical(const std::vector<dpcbf::ObstacleState>& a,
                      const std::vector<dpcbf::ObstacleState>& b, std::string& detail) {
  if (a.size() != b.size()) {
    detail = "size " + std::to_string(a.size()) + " vs " + std::to_string(b.size());
    return false;
  }
  for (std::size_t index = 0; index < a.size(); ++index) {
    if (!ObstacleStateIdentical(a[index], b[index])) {
      detail = "slot " + std::to_string(index) + ": id " + std::to_string(a[index].id) +
               " vs " + std::to_string(b[index].id);
      return false;
    }
  }
  detail.clear();
  return true;
}

bool ConstraintIdentical(const dpcbf::DpcbfConstraint& a, const dpcbf::DpcbfConstraint& b) {
  if (!BitEqual(a.h, b.h) || !BitEqual(a.lf_h, b.lf_h)) return false;
  for (int index = 0; index < 3; ++index) {
    if (!BitEqual(a.lg_h[index], b.lg_h[index])) return false;
  }
  return true;
}

// Every field SafetyFilterResult exposes. Deliberately exhaustive rather than "the command is
// close enough": the selected-constraint set is what a reordering bug would show up in first,
// and the command could well be identical while the constraint set was not.
bool ResultIdentical(const dpcbf::SafetyFilterResult& a, const dpcbf::SafetyFilterResult& b,
                     std::string& detail) {
  detail.clear();
  if (!BitEqual(a.command.sagittal, b.command.sagittal) ||
      !BitEqual(a.command.lateral, b.command.lateral) ||
      !BitEqual(a.command.yaw_rate, b.command.yaw_rate)) {
    detail = "command differs";
    return false;
  }
  for (int index = 0; index < 3; ++index) {
    if (!BitEqual(a.acceleration[index], b.acceleration[index])) {
      detail = "acceleration[" + std::to_string(index) + "] differs";
      return false;
    }
  }
  if (a.active_constraints != b.active_constraints ||
      a.active_dpcbf_constraints != b.active_dpcbf_constraints ||
      a.active_ecbf_constraints != b.active_ecbf_constraints || a.solved != b.solved) {
    detail = "constraint counts / solved flag differ";
    return false;
  }
  if (a.decay_variables.size() != b.decay_variables.size() ||
      a.slack_variables.size() != b.slack_variables.size() ||
      a.selected_obstacles.size() != b.selected_obstacles.size()) {
    detail = "vector sizes differ";
    return false;
  }
  for (std::size_t index = 0; index < a.decay_variables.size(); ++index) {
    if (!BitEqual(a.decay_variables[index], b.decay_variables[index])) {
      detail = "decay_variables[" + std::to_string(index) + "] differs";
      return false;
    }
  }
  for (std::size_t index = 0; index < a.slack_variables.size(); ++index) {
    if (!BitEqual(a.slack_variables[index], b.slack_variables[index])) {
      detail = "slack_variables[" + std::to_string(index) + "] differs";
      return false;
    }
  }
  for (std::size_t index = 0; index < a.selected_obstacles.size(); ++index) {
    const dpcbf::DpcbfVisualizationObstacle& left = a.selected_obstacles[index];
    const dpcbf::DpcbfVisualizationObstacle& right = b.selected_obstacles[index];
    if (!ObstacleStateIdentical(left.obstacle, right.obstacle) ||
        !ConstraintIdentical(left.constraint, right.constraint) ||
        !BitEqual(left.distance, right.distance) ||
        !BitEqual(left.boundary_vertex_x, right.boundary_vertex_x) ||
        !BitEqual(left.boundary_curvature, right.boundary_curvature)) {
      detail = "selected_obstacles[" + std::to_string(index) + "] differs";
      return false;
    }
    for (int axis = 0; axis < 2; ++axis) {
      if (!BitEqual(left.relative_velocity_world[axis], right.relative_velocity_world[axis]) ||
          !BitEqual(left.relative_velocity_los[axis], right.relative_velocity_los[axis])) {
        detail = "selected_obstacles[" + std::to_string(index) + "] relative velocity differs";
        return false;
      }
    }
  }
  return true;
}

// A deterministic, varying command sequence. The point is to make the QP work: a constant
// zero command would leave the filter with nothing to correct, and the end-to-end comparison
// would pass whatever the obstacles were.
dpcbf::VelocityCommand DesiredCommand(int sample) {
  const double phase = 0.37 * static_cast<double>(sample);
  dpcbf::VelocityCommand desired;
  desired.sagittal = 1.2 * std::cos(phase);
  desired.lateral = 0.6 * std::sin(1.7 * phase);
  desired.yaw_rate = 0.5 * std::sin(0.9 * phase);
  return desired;
}

// The golden dpcbf::ObstacleState log: one vector per sample, parsed back from the exact hex
// doubles the pre-change capture wrote.
struct GoldenSample {
  double stamp_s = 0.0;
  std::vector<dpcbf::ObstacleState> states;
};

bool ReadGoldenStates(const std::filesystem::path& path, std::vector<GoldenSample>& out,
                      std::string& error) {
  out.clear();
  std::FILE* file = std::fopen(path.string().c_str(), "r");
  if (file == nullptr) {
    error = "could not open " + path.string();
    return false;
  }
  char line[512];
  while (std::fgets(line, sizeof(line), file) != nullptr) {
    if (line[0] == '#' || line[0] == '\n') continue;
    if (std::strncmp(line, "sample ", 7) == 0) {
      int index = 0;
      char stamp[64] = "";
      unsigned long count = 0;
      if (std::sscanf(line, "sample %d %63s %lu", &index, stamp, &count) != 3) {
        error = std::string("malformed sample line: ") + line;
        std::fclose(file);
        return false;
      }
      GoldenSample sample;
      sample.stamp_s = std::strtod(stamp, nullptr);
      sample.states.reserve(count);
      out.push_back(std::move(sample));
      continue;
    }
    if (std::strncmp(line, "state ", 6) == 0) {
      if (out.empty()) {
        error = "a state line appeared before any sample line";
        std::fclose(file);
        return false;
      }
      unsigned long slot = 0;
      int id = 0;
      char x[64], y[64], radius[64], vx[64], vy[64];
      if (std::sscanf(line, "state %lu %d %63s %63s %63s %63s %63s", &slot, &id, x, y, radius,
                      vx, vy) != 7) {
        error = std::string("malformed state line: ") + line;
        std::fclose(file);
        return false;
      }
      dpcbf::ObstacleState state;
      state.id = id;
      state.x = std::strtod(x, nullptr);
      state.y = std::strtod(y, nullptr);
      state.radius = std::strtod(radius, nullptr);
      state.velocity_x = std::strtod(vx, nullptr);
      state.velocity_y = std::strtod(vy, nullptr);
      if (out.back().states.size() != slot) {
        error = "state slots are out of order in the golden log";
        std::fclose(file);
        return false;
      }
      out.back().states.push_back(state);
      continue;
    }
    error = std::string("unrecognised line in the golden log: ") + line;
    std::fclose(file);
    return false;
  }
  std::fclose(file);
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <repository-root>\n", argv[0]);
    return 2;
  }
  const std::filesystem::path repository_root = argv[1];
  const std::filesystem::path fixtures =
      repository_root / "perception/tests/fixtures/oracle_golden";

  std::printf("perception_oracle_equivalence_test\n");
  std::printf("  repository root: %s\n", repository_root.string().c_str());

  // ---------------------------------------------------------------------------------------
  Section("Golden fixtures load (captured from the PRE-CHANGE inline path)");

  std::vector<perception::core::OracleObstacleState> golden_oracle;
  {
    diagnostics::DumpReader reader;
    std::string error;
    const bool opened = reader.Open(fixtures / "oracle_obstacles.jsonl", error);
    Check(opened, "oracle_obstacles.jsonl opens", opened ? "" : error);
    if (!opened) return perception_test::Report("perception_oracle_equivalence_test");
    const bool read = reader.ReadAll(golden_oracle, error);
    Check(read, "oracle_obstacles.jsonl reads with a complete footer", read ? "" : error);
    Check(reader.header().record_type == diagnostics::RecordType::kOracleObstacleState,
          "fixture is dump record type 20 (oracle_obstacle_state)");
  }

  std::vector<GoldenSample> golden_states;
  {
    std::string error;
    const bool read = ReadGoldenStates(fixtures / "dpcbf_obstacle_states.txt", golden_states,
                                       error);
    Check(read, "dpcbf_obstacle_states.txt parses", read ? "" : error);
    if (!read) return perception_test::Report("perception_oracle_equivalence_test");
  }

  const std::size_t expected_samples =
      static_cast<std::size_t>((oracle_golden::kSteps + oracle_golden::kSampleEvery - 1) /
                               oracle_golden::kSampleEvery);
  Check(golden_states.size() == expected_samples, "golden log sample count",
        std::to_string(golden_states.size()) + " samples");
  Check(!golden_states.empty() && !golden_states.front().states.empty(),
        "golden log is non-empty");

  // ---------------------------------------------------------------------------------------
  Section("Shipped config: the truncation knobs that must NOT be applied in oracle mode");

  integration::PerceptionConfig config;
  int configured_max_obstacles = 0;
  bool configured_drop_invalid = false;
  try {
    config = integration::PerceptionConfig::LoadFromYaml(
        repository_root / "perception/configs/perception.yaml");
    configured_max_obstacles = config.dpcbf_adapter.max_obstacles;
    configured_drop_invalid = config.dpcbf_adapter.drop_invalid;
    Check(true, "shipped perception.yaml loads");
  } catch (const std::exception& thrown) {
    Check(false, "shipped perception.yaml loads", thrown.what());
    return perception_test::Report("perception_oracle_equivalence_test");
  }
  Check(config.mode.name == "oracle", "shipped mode.name is oracle", config.mode.name);
  // If these two ever stop being "a cap smaller than the population" and "dropping is on",
  // the guarantee below stops being a meaningful test and someone must look again.
  Check(configured_max_obstacles > 0 && configured_drop_invalid,
        "dpcbf_adapter knobs are set (so ignoring them is a real, testable choice)",
        "max_obstacles=" + std::to_string(configured_max_obstacles) + " drop_invalid=true");

  // ---------------------------------------------------------------------------------------
  Section("Scene: G1 + the full dpcbf_config.yaml obstacle population");

  dpcbf::DynamicObstacleManager obstacles;
  mjModel* model = nullptr;
  mjData* data = nullptr;
  {
    std::string error;
    const bool built = oracle_golden::BuildScene(repository_root, obstacles, model, data, error);
    Check(built, "scene builds (parseXML -> AddToSpec -> compile)", built ? "" : error);
    if (!built) return perception_test::Report("perception_oracle_equivalence_test");
  }

  const std::size_t population = obstacles.Snapshot().size();
  Check(population > static_cast<std::size_t>(configured_max_obstacles),
        "obstacle population exceeds dpcbf_adapter.max_obstacles (so a cap would be visible)",
        std::to_string(population) + " obstacles vs cap " +
            std::to_string(configured_max_obstacles));

  // ---------------------------------------------------------------------------------------
  Section("Selector binds in oracle mode");

  integration::ObstacleSourceSelector selector;
  {
    std::string error;
    const bool bound = selector.Bind(config, &obstacles, error);
    Check(bound, "Bind(oracle) succeeds", bound ? "" : error);
    if (!bound) return perception_test::Report("perception_oracle_equivalence_test");
    Check(selector.mode() == integration::ObstacleSourceMode::kOracle, "mode() is kOracle");

    std::string model_error;
    const bool model_bound = selector.BindModel(model, model_error);
    Check(model_bound, "BindModel resolves frames.base_body",
          model_bound ? config.frames.base_body : model_error);
  }

  // A second, independent provider, driven directly rather than through the selector. Proves
  // the adapter pair is correct on its own and not only in the selector's composition.
  mj_adapters::OracleProviderMj direct_provider;
  direct_provider.BindObstacleSource(&obstacles);
  {
    std::string error;
    Check(direct_provider.BindModel(model, config.frames, error),
          "direct OracleProviderMj binds the model", error);
  }

  const int base_body_id = mj_name2id(model, mjOBJ_BODY, config.frames.base_body.c_str());
  Check(base_body_id >= 0 && base_body_id == direct_provider.base_body_id(),
        "provider resolved the same base body id the inline reader uses");

  // ---------------------------------------------------------------------------------------
  Section("The golden run: 600 steps, moving obstacles, 30 sampled comparisons");

  dpcbf::DpcbfSafetyFilter filter_old;
  dpcbf::DpcbfSafetyFilter filter_new;
  bool filters_ready = false;
  try {
    const auto dpcbf_config = repository_root / "dpcbf/config/dpcbf_config.yaml";
    filter_old.LoadConfig(dpcbf_config);
    filter_new.LoadConfig(dpcbf_config);
    filter_old.Initialize(model->opt.timestep);
    filter_new.Initialize(model->opt.timestep);
    filters_ready = true;
  } catch (const std::exception& thrown) {
    Check(false, "two independent DpcbfSafetyFilter instances initialize", thrown.what());
  }
  Check(filters_ready, "two independent DpcbfSafetyFilter instances initialize");

  int samples = 0;
  int mismatch_new_vs_inline = 0;
  int mismatch_selector_vs_direct = 0;
  int mismatch_vs_golden_log = 0;
  int mismatch_oracle_leg = 0;
  int mismatch_robot_pose = 0;
  int mismatch_filter_result = 0;
  int count_not_full = 0;
  int solved_samples = 0;
  int samples_with_active_constraints = 0;
  std::size_t golden_oracle_cursor = 0;
  std::string first_failure_detail;

  // Two probes run alongside the comparison, on every sample, each answering a question the
  // equivalence check itself cannot:
  //
  //   order      - the comparator is order-strict, so a reordering could not have slipped
  //                through section A or C. Separately: does DPCBF's own output actually
  //                depend on the order of an otherwise identical list? Measured rather than
  //                assumed, and reported either way - the refactor preserves order regardless.
  //   truncation - if the oracle path HAD honoured dpcbf_adapter.max_obstacles, would DPCBF
  //                behave differently? This is what turns "we deliberately did not apply the
  //                cap" from a stylistic note into a measured behavioural fact.
  int order_probe_samples = 0;
  int order_changed_result = 0;
  int comparator_caught_reorder = 0;
  int truncation_probe_samples = 0;
  int truncation_changed_result = 0;

  for (int step = 0; step < oracle_golden::kSteps; ++step) {
    obstacles.Step(model, data, model->opt.timestep);
    mj_step(model, data);
    if (step % oracle_golden::kSampleEvery != 0) continue;

    const double now_s = data->time;

    // --- A. the two paths, same snapshot, same instant --------------------------------
    const std::vector<dpcbf::ObstacleState> from_inline = InlineObstacleConversion(obstacles);
    const std::vector<dpcbf::ObstacleState> from_selector = selector.GetObstacleStates(now_s);

    std::vector<perception::core::OracleObstacleState> direct_oracle;
    direct_provider.SampleObstacles(now_s, direct_oracle);
    std::vector<dpcbf::ObstacleState> from_direct;
    dpcbf_adapter::ToDpcbfObstacleStates(direct_oracle, from_direct);

    std::string detail;
    if (!VectorsIdentical(from_inline, from_selector, detail)) {
      ++mismatch_new_vs_inline;
      if (first_failure_detail.empty()) first_failure_detail = "A: " + detail;
    }
    if (!VectorsIdentical(from_selector, from_direct, detail)) {
      ++mismatch_selector_vs_direct;
      if (first_failure_detail.empty()) first_failure_detail = "A': " + detail;
    }

    // --- unfiltered, uncapped pass-through -------------------------------------------
    if (from_selector.size() != population || selector.last_source_count() != population ||
        selector.last_emitted_count() != population) {
      ++count_not_full;
    }

    // --- C. against the committed pre-change dpcbf::ObstacleState log ------------------
    if (static_cast<std::size_t>(samples) < golden_states.size()) {
      const GoldenSample& golden = golden_states[samples];
      if (!BitEqual(golden.stamp_s, now_s) ||
          !VectorsIdentical(golden.states, from_selector, detail)) {
        ++mismatch_vs_golden_log;
        if (first_failure_detail.empty()) {
          first_failure_detail = "C: " + (detail.empty() ? std::string("stamp differs") : detail);
        }
      }
    }

    // --- B. against the committed pre-change OracleObstacleState dump ------------------
    for (const perception::core::OracleObstacleState& produced : direct_oracle) {
      if (golden_oracle_cursor >= golden_oracle.size()) {
        ++mismatch_oracle_leg;
        break;
      }
      const perception::core::OracleObstacleState& expected =
          golden_oracle[golden_oracle_cursor++];
      const bool same = expected.id == produced.id && expected.frame == produced.frame &&
                        BitEqual(expected.radius_m, produced.radius_m) &&
                        BitEqual(expected.stamp_s, produced.stamp_s) &&
                        BitEqual(expected.center.x(), produced.center.x()) &&
                        BitEqual(expected.center.y(), produced.center.y()) &&
                        BitEqual(expected.velocity.x(), produced.velocity.x()) &&
                        BitEqual(expected.velocity.y(), produced.velocity.y());
      if (!same) {
        ++mismatch_oracle_leg;
        if (first_failure_detail.empty()) {
          first_failure_detail = "B: record " + std::to_string(golden_oracle_cursor - 1);
        }
      }
    }

    // --- H. the robot-pose leg --------------------------------------------------------
    const dpcbf::RobotState robot_inline =
        InlineReadRobotGroundTruth(model, data, base_body_id);
    const mj_adapters::OracleRobotPose robot_new = direct_provider.SampleRobot(model, data);
    if (!robot_new.valid || !BitEqual(robot_inline.x, robot_new.x_m) ||
        !BitEqual(robot_inline.y, robot_new.y_m) ||
        !BitEqual(robot_inline.phi, robot_new.yaw_rad) ||
        !BitEqual(robot_inline.sagittal_velocity, robot_new.sagittal_velocity_mps) ||
        !BitEqual(robot_inline.lateral_velocity, robot_new.lateral_velocity_mps) ||
        !BitEqual(robot_new.stamp_s, now_s)) {
      ++mismatch_robot_pose;
    }

    // --- D. end to end through two independent real filters ---------------------------
    if (filters_ready) {
      const dpcbf::VelocityCommand desired = DesiredCommand(samples);
      const dpcbf::SafetyFilterResult result_old =
          filter_old.Filter(robot_inline, desired, from_inline);
      const dpcbf::SafetyFilterResult result_new =
          filter_new.Filter(robot_inline, desired, from_selector);
      if (!ResultIdentical(result_old, result_new, detail)) {
        ++mismatch_filter_result;
        if (first_failure_detail.empty()) first_failure_detail = "D: " + detail;
      }
      if (result_old.solved) ++solved_samples;
      if (result_old.active_constraints > 0) ++samples_with_active_constraints;

      // The probes. Each perturbed list is filtered by a FRESH instance and compared against
      // a fresh instance fed the unperturbed list, so no warm-solver history can be mistaken
      // for a difference caused by the perturbation itself.
      if (result_old.active_constraints > 0 && from_selector.size() > 4) {
        std::vector<dpcbf::ObstacleState> reversed(from_selector.rbegin(),
                                                   from_selector.rend());
        std::vector<dpcbf::ObstacleState> truncated(
            from_selector.begin(),
            from_selector.begin() + std::min<std::size_t>(from_selector.size(),
                                                          configured_max_obstacles));
        std::string probe_detail;
        if (!VectorsIdentical(from_selector, reversed, probe_detail)) ++comparator_caught_reorder;

        const auto fresh_result = [&](const std::vector<dpcbf::ObstacleState>& states,
                                      dpcbf::SafetyFilterResult& out) {
          dpcbf::DpcbfSafetyFilter probe;
          probe.LoadConfig(repository_root / "dpcbf/config/dpcbf_config.yaml");
          probe.Initialize(model->opt.timestep);
          out = probe.Filter(robot_inline, desired, states);
        };
        try {
          dpcbf::SafetyFilterResult baseline;
          dpcbf::SafetyFilterResult perturbed;
          std::string ignored;

          fresh_result(from_selector, baseline);
          fresh_result(reversed, perturbed);
          ++order_probe_samples;
          if (!ResultIdentical(baseline, perturbed, ignored)) ++order_changed_result;

          if (truncated.size() < from_selector.size()) {
            fresh_result(truncated, perturbed);
            ++truncation_probe_samples;
            if (!ResultIdentical(baseline, perturbed, ignored)) ++truncation_changed_result;
          }
        } catch (const std::exception&) {
          // A probe that cannot be built is reported as an un-run probe below, not silently
          // counted as agreement.
        }
      }
    }

    ++samples;
  }

  Check(samples == static_cast<int>(expected_samples), "sampled the expected number of times",
        std::to_string(samples) + " samples");

  Section("A. New path vs the verbatim pre-change inline conversion (bit-exact, in order)");
  Check(mismatch_new_vs_inline == 0,
        "selector output == inline conversion, every field, every slot, every sample",
        mismatch_new_vs_inline == 0
            ? std::to_string(samples) + "/" + std::to_string(samples) + " samples identical"
            : std::to_string(mismatch_new_vs_inline) + " mismatched samples; " +
                  first_failure_detail);
  Check(mismatch_selector_vs_direct == 0,
        "selector output == provider+adapter driven directly",
        std::to_string(mismatch_selector_vs_direct) + " mismatches");

  Section("B. OracleObstacleState leg vs the committed golden dump (record type 20)");
  Check(mismatch_oracle_leg == 0, "every OracleObstacleState matches the fixture bit-exactly",
        std::to_string(mismatch_oracle_leg) + " mismatches");
  Check(golden_oracle_cursor == golden_oracle.size(),
        "consumed the whole fixture - no records left over on either side",
        std::to_string(golden_oracle_cursor) + " of " + std::to_string(golden_oracle.size()) +
            " records");
  Check(golden_oracle.size() == expected_samples * population,
        "fixture holds samples x population records",
        std::to_string(golden_oracle.size()) + " records");

  Section("C. dpcbf::ObstacleState leg vs the committed golden log (field by field, in order)");
  Check(mismatch_vs_golden_log == 0,
        "every dpcbf::ObstacleState matches the pre-change log bit-exactly",
        std::to_string(mismatch_vs_golden_log) + " mismatched samples");

  Section("D. End to end: two real DpcbfSafetyFilter instances, identical results");
  Check(filters_ready && mismatch_filter_result == 0,
        "SafetyFilterResult identical - command, acceleration, constraint counts, decay, "
        "slack, and the whole selected-obstacle set",
        mismatch_filter_result == 0 ? std::to_string(samples) + " samples identical"
                                    : first_failure_detail);
  // Guards against a vacuous pass: an unconstrained or never-solving QP would compare equal
  // for uninteresting reasons.
  //
  // NOT "solved on every sample", deliberately. The robot in this run is uncontrolled and
  // falls into a 90-cylinder arena, so a couple of samples drive OSQP to its iteration limit
  // and DpcbfSafetyFilter holds its last feasible command. That is pre-existing filter
  // behaviour under a pathological state, not something this phase introduced or can fix -
  // and the equivalence above holds THROUGH those samples, on both instances, which is
  // stronger evidence than a run that never stressed the solver would have given.
  Check(solved_samples >= samples - samples / 10,
        "the QP solved on the large majority of samples (the comparison is not vacuous)",
        std::to_string(solved_samples) + "/" + std::to_string(samples) + " solved; the " +
            std::to_string(samples - solved_samples) +
            " unsolved sample(s) compared identical too");
  Check(samples_with_active_constraints > 0,
        "the filter was genuinely constrained by obstacles on at least one sample",
        std::to_string(samples_with_active_constraints) + "/" + std::to_string(samples) +
            " samples had active constraints");

  Section("H. Robot ground-truth pose leg");
  Check(mismatch_robot_pose == 0,
        "OracleProviderMj::SampleRobot == the inline ReadRobotGroundTruth, bit-exact",
        std::to_string(mismatch_robot_pose) + " mismatches");

  Section("Order sensitivity");
  Check(order_probe_samples > 0, "order probe ran on constrained samples",
        std::to_string(order_probe_samples) + " samples probed");
  Check(comparator_caught_reorder == order_probe_samples && order_probe_samples > 0,
        "the comparator rejects a reordered-but-element-wise-identical list on every probed "
        "sample - so sections A and C could not have passed on a silent reordering",
        std::to_string(comparator_caught_reorder) + "/" + std::to_string(order_probe_samples));
  // Reported, not asserted: whether DPCBF is order-sensitive is a property of dpcbf/, not a
  // requirement of this phase. The refactor preserves order either way, which is what the
  // check above pins down. The measurement is here because "order matters" was an assumption
  // worth converting into a number.
  std::printf(
      "  NOTE  reversing the obstacle list changed DpcbfSafetyFilter's output on %d/%d "
      "probed samples.\n        %s\n",
      order_changed_result, order_probe_samples,
      order_changed_result == 0
          ? "DPCBF's top-k selection is empirically order-invariant on this corpus (it sorts "
            "by\n        priority with distance as a tie-break, so a permutation of the input "
            "does not move\n        the selected set). Order preservation is therefore "
            "defence-in-depth here rather than\n        presently load-bearing - but it is "
            "still guaranteed, and still checked."
          : "DPCBF's output DOES depend on input order, so order preservation is directly "
            "load-bearing.");

  Section("Truncation impact: what applying max_obstacles would have cost");
  Check(truncation_probe_samples > 0, "truncation probe ran",
        std::to_string(truncation_probe_samples) + " samples probed");
  Check(truncation_changed_result > 0,
        "truncating the oracle list to dpcbf_adapter.max_obstacles CHANGES DpcbfSafetyFilter's "
        "output - so declining to apply that knob is a behavioural decision, not a cosmetic "
        "one",
        std::to_string(truncation_changed_result) + "/" +
            std::to_string(truncation_probe_samples) + " probed samples differed when capped "
            "at " + std::to_string(configured_max_obstacles));

  Section("dpcbf_adapter.max_obstacles / drop_invalid are NOT applied in oracle mode");
  Check(count_not_full == 0,
        "the complete, uncapped snapshot passed through on every sample",
        std::to_string(population) + " obstacles per sample, cap of " +
            std::to_string(configured_max_obstacles) + " deliberately ignored");
  {
    // The drop_invalid half, proven directly rather than inferred: an obstacle that fails
    // OracleObstacleState::Validate() must still be converted and emitted.
    perception::core::OracleObstacleState invalid;
    invalid.id = 7;
    invalid.center = Eigen::Vector2d(1.5, -2.5);
    invalid.velocity = Eigen::Vector2d(0.25, -0.75);
    invalid.radius_m = 0.0;  // Validate() rejects this.
    invalid.stamp_s = 1.0;
    Check(invalid.Validate() != nullptr, "the probe obstacle really is invalid",
          invalid.Validate() == nullptr ? "" : invalid.Validate());

    std::vector<perception::core::OracleObstacleState> input = {invalid};
    std::vector<dpcbf::ObstacleState> converted;
    dpcbf_adapter::ToDpcbfObstacleStates(input, converted);
    Check(converted.size() == 1, "an invalid oracle obstacle is still emitted, not dropped");
    Check(converted.size() == 1 && converted[0].id == 7 && BitEqual(converted[0].x, 1.5) &&
              BitEqual(converted[0].y, -2.5) && BitEqual(converted[0].radius, 0.0) &&
              BitEqual(converted[0].velocity_x, 0.25) &&
              BitEqual(converted[0].velocity_y, -0.75),
          "and it is emitted unmodified - no clamping, no substitution");
  }
  {
    // Truncation, proven directly too: 40 obstacles in, 40 out, with a cap of 20 configured.
    std::vector<perception::core::OracleObstacleState> many(40);
    for (std::size_t index = 0; index < many.size(); ++index) {
      many[index].id = static_cast<int32_t>(index);
      many[index].radius_m = 0.25;
      many[index].center = Eigen::Vector2d(static_cast<double>(index), 0.0);
    }
    std::vector<dpcbf::ObstacleState> converted;
    dpcbf_adapter::ToDpcbfObstacleStates(many, converted);
    bool ids_in_order = converted.size() == many.size();
    for (std::size_t index = 0; ids_in_order && index < converted.size(); ++index) {
      ids_in_order = converted[index].id == static_cast<int>(index);
    }
    Check(ids_in_order, "40 in -> 40 out, ids in input order, with max_obstacles=" +
                            std::to_string(configured_max_obstacles) + " configured");
  }

  Section("Mode gating: unimplemented modes refuse to start");
  for (const char* name : {"shadow", "compare", "estimated_fallback", "estimated"}) {
    integration::PerceptionConfig other = config;
    other.mode.name = name;
    integration::ObstacleSourceSelector rejected;
    std::string error;
    const bool bound = rejected.Bind(other, &obstacles, error);
    Check(!bound && error.find(name) != std::string::npos,
          std::string("Bind refuses mode '") + name + "' with a message naming it",
          bound ? "it bound anyway" : error.substr(0, 60) + "...");
    Check(!rejected.bound(), std::string("selector reports not bound after refusing ") + name);
  }
  {
    integration::ObstacleSourceMode parsed = integration::ObstacleSourceMode::kOracle;
    std::string error;
    Check(!integration::ParseObstacleSourceMode("oracle_but_faster", parsed, error),
          "an unknown mode name is rejected rather than defaulted to oracle");
    integration::PerceptionConfig no_manager = config;
    integration::ObstacleSourceSelector unbound;
    Check(!unbound.Bind(no_manager, nullptr, error),
          "oracle mode refuses a null DynamicObstacleManager");
    bool threw = false;
    try {
      integration::ObstacleSourceSelector never_bound;
      never_bound.GetObstacleStates(0.0);
    } catch (const std::runtime_error&) {
      threw = true;
    }
    Check(threw,
          "GetObstacleStates before Bind throws rather than returning an empty arena");
  }

  mj_deleteData(data);
  mj_deleteModel(model);
  return perception_test::Report("perception_oracle_equivalence_test");
}
