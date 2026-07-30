// oracle_golden_capture - the PRE-CHANGE tap that produces this phase's golden fixture.
//
// WHY THIS EXISTS AS ITS OWN EXECUTABLE. The P3 acceptance bar is "oracle-only behaviour
// provably unchanged". Proving that needs a record of what the OLD code produced, captured
// by code written against the OLD code, before the new modules existed. A comparison written
// after the refactor - even a careful one - can only ever show that the new path agrees with
// a transcription of the old path made by someone who had already read the new path.
//
// So this tool was written and run FIRST, with `perception/src/adapters/mujoco/
// oracle_provider_mj.*`, `perception/src/adapters/dpcbf/oracle_to_dpcbf.*` and
// `perception/src/integration/obstacle_source_selector.*` not yet in existence, and its two
// outputs were committed as fixtures:
//
//   perception/tests/fixtures/oracle_golden/oracle_obstacles.jsonl
//       The OracleObstacleState stream (dump record type 20, the P2 mechanism built for
//       exactly this) that the inline conversion derived its dpcbf::ObstacleState values
//       from. Compared bit-for-bit by perception_oracle_equivalence_test.
//
//   perception/tests/fixtures/oracle_golden/dpcbf_obstacle_states.txt
//       The final dpcbf::ObstacleState vectors the inline code actually handed to
//       safety_filter.Filter(), field by field, in order. Deliberately a plain text log and
//       NOT a dump record: dpcbf::ObstacleState may not enter core/diagnostics (dependency
//       rules, architecture doc section 8), so the second leg of the proof is a test-side
//       format, hexadecimal so it is exact rather than printf-rounded.
//
// THE INLINE CONVERSION BELOW IS A VERBATIM TRANSCRIPTION of what stood in the axis-filter
// lambda in simulate/src/main.cc before this phase. Do not "improve" it - its only job is to
// still be the old code. It is duplicated in the test binary, and the two committed fixtures
// are what keep both copies honest.
//
// REPRODUCIBILITY. The obstacle stream is a pure function of (dpcbf_config.yaml, physics
// timestep, number of Step calls): DynamicObstacleManager::Step integrates position and
// reflects at the arena walls without reading mjData at all, so the fixture does not depend
// on the robot, the contacts, or the solver. That is what makes it safe to commit. It does
// depend on libstdc++'s mt19937/uniform_real_distribution stream for the initial sampling; if
// a toolchain change ever moves that, the fixture must be regenerated deliberately, not
// silently loosened.

#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "dpcbf/dpcbf_safety_filter.h"
#include "dpcbf/dynamic_obstacles.h"
#include "perception/core/contracts/obstacles.h"
#include "perception/core/diagnostics/dump_file.h"

#include "oracle_golden_run.h"

namespace diagnostics = perception::core::diagnostics;

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: %s <repository-root> <output-directory>\n", argv[0]);
    return 2;
  }
  const std::filesystem::path repository_root = argv[1];
  const std::filesystem::path output_directory = argv[2];

  dpcbf::DynamicObstacleManager obstacles;
  mjModel* model = nullptr;
  mjData* data = nullptr;
  std::string error;
  if (!oracle_golden::BuildScene(repository_root, obstacles, model, data, error)) {
    std::fprintf(stderr, "scene setup failed: %s\n", error.c_str());
    return 1;
  }

  std::filesystem::create_directories(output_directory);

  // --- The OracleObstacleState leg: the P2 dump writer, record type 20. ------------------
  diagnostics::RunProvenance provenance;
  provenance.producer = "oracle_golden_capture";
  provenance.config_source = "dpcbf/config/dpcbf_config.yaml";

  diagnostics::DumpWriter writer;
  const std::filesystem::path oracle_path = output_directory / "oracle_obstacles.jsonl";
  if (!writer.Open(oracle_path, diagnostics::DumpFormat::kJsonl,
                   diagnostics::RecordType::kOracleObstacleState, provenance,
                   /*retained_mask=*/0, error)) {
    std::fprintf(stderr, "DumpWriter::Open failed: %s\n", error.c_str());
    return 1;
  }

  // --- The dpcbf::ObstacleState leg: a plain, exact, test-side text log. -----------------
  const std::filesystem::path states_path = output_directory / "dpcbf_obstacle_states.txt";
  std::FILE* states = std::fopen(states_path.string().c_str(), "w");
  if (states == nullptr) {
    std::fprintf(stderr, "could not open %s\n", states_path.string().c_str());
    return 1;
  }
  std::fprintf(states,
               "# dpcbf::ObstacleState vectors produced by the PRE-CHANGE inline conversion\n"
               "# in simulate/src/main.cc's axis-filter lambda. Hex doubles: exact, not\n"
               "# rounded. Order is the order handed to safety_filter.Filter().\n"
               "# sample <index> <sim_time_hex> <count>\n"
               "# state <slot> <id> <x> <y> <radius> <velocity_x> <velocity_y>\n");

  uint64_t sequence = 0;
  int samples = 0;
  for (int step = 0; step < oracle_golden::kSteps; ++step) {
    // Mirrors PhysicsLoop: obstacles are integrated, then physics advances.
    obstacles.Step(model, data, model->opt.timestep);
    mj_step(model, data);

    if (step % oracle_golden::kSampleEvery != 0) continue;

    // ===== BEGIN verbatim pre-change inline conversion (main.cc axis-filter lambda) =====
    std::vector<dpcbf::ObstacleState> obstacle_states;
    const auto obstacle_snapshot = obstacles.Snapshot();
    obstacle_states.reserve(obstacle_snapshot.size());
    for (std::size_t obstacle_id = 0; obstacle_id < obstacle_snapshot.size(); ++obstacle_id) {
      const auto& obstacle = obstacle_snapshot[obstacle_id];
      obstacle_states.push_back({obstacle.position[0], obstacle.position[1], obstacle.radius,
                                 obstacle.velocity[0], obstacle.velocity[1],
                                 static_cast<int>(obstacle_id)});
    }
    // ====== END verbatim pre-change inline conversion ===================================

    std::fprintf(states, "sample %d %s %zu\n", samples,
                 oracle_golden::HexDouble(data->time).c_str(), obstacle_states.size());
    for (std::size_t slot = 0; slot < obstacle_states.size(); ++slot) {
      const dpcbf::ObstacleState& state = obstacle_states[slot];
      std::fprintf(states, "state %zu %d %s %s %s %s %s\n", slot, state.id,
                   oracle_golden::HexDouble(state.x).c_str(),
                   oracle_golden::HexDouble(state.y).c_str(),
                   oracle_golden::HexDouble(state.radius).c_str(),
                   oracle_golden::HexDouble(state.velocity_x).c_str(),
                   oracle_golden::HexDouble(state.velocity_y).c_str());
    }

    // The OracleObstacleState view of the SAME snapshot: the intermediate the new path will
    // route through, written here from the old path's own data so the fixture is genuinely
    // pre-change.
    for (std::size_t obstacle_id = 0; obstacle_id < obstacle_snapshot.size(); ++obstacle_id) {
      const auto& obstacle = obstacle_snapshot[obstacle_id];
      perception::core::OracleObstacleState record;
      record.id = static_cast<int32_t>(obstacle_id);
      record.center = Eigen::Vector2d(obstacle.position[0], obstacle.position[1]);
      record.velocity = Eigen::Vector2d(obstacle.velocity[0], obstacle.velocity[1]);
      record.radius_m = obstacle.radius;
      record.stamp_s = data->time;
      record.frame = perception::core::FrameId::kWorld;
      if (!writer.Write(record, sequence++, record.stamp_s, error)) {
        std::fprintf(stderr, "DumpWriter::Write failed: %s\n", error.c_str());
        return 1;
      }
    }
    ++samples;
  }

  if (!writer.Close(error)) {
    std::fprintf(stderr, "DumpWriter::Close failed: %s\n", error.c_str());
    return 1;
  }
  std::fclose(states);
  mj_deleteData(data);
  mj_deleteModel(model);

  std::printf("captured %d samples, %llu OracleObstacleState records\n", samples,
              static_cast<unsigned long long>(sequence));
  std::printf("  %s\n  %s\n", oracle_path.string().c_str(), states_path.string().c_str());
  return 0;
}
