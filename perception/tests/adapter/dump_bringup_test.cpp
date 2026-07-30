// End-to-end: dump REAL data from the P4 bring-up path, replay it through the actual
// perception_replay binary, and check the printed summary against what the live scan reported.
//
// WHY THIS TEST AND NOT ANOTHER ROUND-TRIP. The round-trip test proves the codec is exact on
// hand-authored fixtures. It cannot prove that the thing wired into the live path dumps what
// the live path actually produced - a recorder that wrote a stale buffer, or that dropped the
// self-hit count, would pass every synthetic test. So this one runs the real raycaster against
// the real G1 scene, keeps the ScanStats and FrameTransformSnapshots the live code reported,
// and requires the replayed values to be bit-identical to them.
//
// It also runs the REAL perception_replay executable (path passed by CMake) rather than only
// the library function behind it, so the app's own argument handling, exit status and printed
// text are covered too.
//
// Two runs, because the interesting configuration is not one setting:
//   run A - retain_stage_clouds=true,  decimation=1, format=both  -> everything recorded
//   run B - retain_stage_clouds=false, decimation=2, format=jsonl -> the raw cloud is NOT
//           recorded and half the frames are dropped, which is exactly the case a replay must
//           distinguish from "the stage ran and produced nothing".

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "check.h"
#include "perception/core/diagnostics/replay_report.h"
#include "perception/integration/mid360_bringup.h"

using perception_test::Check;
using perception_test::Section;
namespace diagnostics = perception::core::diagnostics;

namespace {

constexpr int kScans = 4;

// Bit-exact, not epsilon-close: this is the section-11 exact-equality tier. Compares the
// representations so that -0.0 vs 0.0 and NaN vs NaN both answer correctly.
bool BitEqual(double a, double b) { return std::memcmp(&a, &b, sizeof(double)) == 0; }

bool ScanStatsIdentical(const perception::core::ScanStats& a,
                        const perception::core::ScanStats& b) {
  return a.rays_cast == b.rays_cast && a.raw_hits == b.raw_hits && a.no_hit == b.no_hit &&
         a.self_rejected == b.self_rejected && a.range_rejected == b.range_rejected &&
         a.accepted == b.accepted && a.aperture_transmitted == b.aperture_transmitted &&
         a.aperture_diagnostic_valid == b.aperture_diagnostic_valid &&
         BitEqual(a.scan_wall_time_us, b.scan_wall_time_us);
}

bool SnapshotIdentical(const perception::core::FrameTransformSnapshot& a,
                       const perception::core::FrameTransformSnapshot& b) {
  if (!BitEqual(a.stamp_s, b.stamp_s) || a.valid != b.valid) return false;
  const Eigen::Isometry3d* left[3] = {&a.world_from_base, &a.world_from_sensor,
                                      &a.base_from_sensor};
  const Eigen::Isometry3d* right[3] = {&b.world_from_base, &b.world_from_sensor,
                                       &b.base_from_sensor};
  for (int pose = 0; pose < 3; ++pose) {
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 4; ++column) {
        if (!BitEqual(left[pose]->matrix()(row, column), right[pose]->matrix()(row, column))) {
          return false;
        }
      }
    }
  }
  for (int index = 0; index < 3; ++index) {
    if (!BitEqual(a.gravity_in_base[index], b.gravity_in_base[index])) return false;
  }
  return true;
}

std::string RunCommand(const std::string& command, int& exit_status) {
  std::string output;
  FILE* pipe = popen(command.c_str(), "r");
  if (pipe == nullptr) {
    exit_status = -1;
    return output;
  }
  char buffer[4096];
  while (std::fgets(buffer, sizeof(buffer), pipe) != nullptr) output += buffer;
  const int closed = pclose(pipe);
  exit_status = closed == -1 ? -1 : WEXITSTATUS(closed);
  return output;
}

struct LiveRun {
  std::vector<perception::core::ScanStats> stats;
  std::vector<perception::core::FrameTransformSnapshot> snapshots;
  std::vector<std::size_t> point_counts;
  std::vector<double> stamps;
  bool ok = false;
  std::string error;
};

// Runs the real bring-up against the real scene with dumping on.
LiveRun Scan(const std::filesystem::path& scene, const std::filesystem::path& dump_directory,
             const std::string& format, bool retain_clouds, int decimation) {
  LiveRun run;

  perception::integration::Mid360BringUp bringup;
  perception::integration::PerceptionConfig config;
  config.dumps.enabled = true;
  config.dumps.format = format;
  config.dumps.retain_stage_clouds = retain_clouds;
  config.dumps.decimation = decimation;
  // Off: the OpenCV window needs a display and this test is headless by design.
  config.visualization.enabled = false;
  bringup.SetConfig(config);

  char load_error[1024] = "";
  mjModel* model =
      mj_loadXML(scene.string().c_str(), nullptr, load_error, sizeof(load_error));
  if (model == nullptr) {
    run.error = std::string("could not load ") + scene.string() + ": " + load_error;
    return run;
  }
  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  std::string error;
  if (!bringup.BindModel(model, data, error)) {
    run.error = "BindModel failed: " + error;
    mj_deleteData(data);
    mj_deleteModel(model);
    return run;
  }
  if (!bringup.StartDumps("perception_dump_bringup_test", dump_directory, error)) {
    run.error = "StartDumps failed: " + error;
    mj_deleteData(data);
    mj_deleteModel(model);
    return run;
  }

  const double period_s = 1.0 / bringup.config().lidar.sensor.scan_rate_hz;
  for (int scan = 0; scan < kScans; ++scan) {
    // Static scene, clock advanced by hand: the same pacing lidar_bench uses without --step,
    // so every scan is let through by the rate limiter and the geometry is identical each
    // time. Identical geometry is what makes a divergence in the dumped census attributable
    // to the recorder rather than to the physics.
    data->time = scan * period_s;
    mj_forward(model, data);
    if (!bringup.MaybeScan(data->time, /*force=*/false)) {
      run.error = "MaybeScan declined to scan at t=" + std::to_string(data->time);
      mj_deleteData(data);
      mj_deleteModel(model);
      return run;
    }
    run.stats.push_back(bringup.latest_stats());
    run.snapshots.push_back(bringup.latest_snapshot());
    run.point_counts.push_back(bringup.latest_cloud().size());
    run.stamps.push_back(bringup.latest_cloud().stamp_s);
  }

  if (!bringup.FinishDumps(error)) {
    run.error = "FinishDumps failed: " + error;
    mj_deleteData(data);
    mj_deleteModel(model);
    return run;
  }

  mj_deleteData(data);
  mj_deleteModel(model);
  run.ok = true;
  return run;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) {
    std::printf(
        "usage: perception_dump_bringup_test <repo-root> <scratch-dir> <perception_replay>\n");
    return 2;
  }
  const std::filesystem::path repository_root = argv[1];
  const std::filesystem::path scratch = argv[2];
  const std::string replay_binary = argv[3];
  const std::filesystem::path scene =
      repository_root / "src/assets/robots/unitree_g1/xmls/scene_g1.xml";

  std::error_code filesystem_error;
  std::filesystem::remove_all(scratch, filesystem_error);
  std::filesystem::create_directories(scratch, filesystem_error);

  // =====================================================================================
  // Run A: everything retained, every frame, both formats.
  // =====================================================================================
  Section("run A - real bring-up scan with retain_stage_clouds=true, decimation=1, both formats");

  const std::filesystem::path directory_a = scratch / "run_a";
  const LiveRun live_a = Scan(scene, directory_a, "both", /*retain_clouds=*/true,
                              /*decimation=*/1);
  Check(live_a.ok, "run A: the live bring-up produced four scans and finished its dumps",
        live_a.error);
  if (!live_a.ok) return perception_test::Report("perception_dump_bringup_test");

  // Sanity on the live data itself, so a later comparison against zeros cannot pass.
  long long live_rays = 0;
  long long live_accepted = 0;
  long long live_self = 0;
  bool all_balanced = true;
  for (const auto& stats : live_a.stats) {
    live_rays += stats.rays_cast;
    live_accepted += stats.accepted;
    live_self += stats.self_rejected;
    if (!stats.IsBalanced()) all_balanced = false;
  }
  Check(live_rays > 0 && live_accepted > 0,
        "run A: the live scan actually returned points (rays=" + std::to_string(live_rays) +
            ", accepted=" + std::to_string(live_accepted) + ")");
  Check(all_balanced, "run A: every live scan census balances");

  diagnostics::ReplaySummary summary_a;
  std::string error;
  const bool loaded_a = diagnostics::LoadReplaySummary(directory_a, summary_a, error);
  Check(loaded_a, "run A: the dump directory replays", error);
  if (!loaded_a) return perception_test::Report("perception_dump_bringup_test");

  Check(summary_a.truncated_files.empty(),
        "run A: no truncated or inconsistent dump files",
        summary_a.truncated_files.empty() ? "" : summary_a.truncated_files.front());
  Check(summary_a.config_hash_verified,
        "run A: resolved_config.txt hashes to the config_hash in every header",
        summary_a.config_hash_note);
  Check(summary_a.provenance.config_source == "<built-in defaults>",
        "run A: provenance names the built-in defaults, not a YAML that was never read",
        summary_a.provenance.config_source);
  Check(summary_a.provenance.producer == "perception_dump_bringup_test",
        "run A: provenance names the producing executable");
  Check(summary_a.provenance.config_schema_version == 1 &&
            summary_a.provenance.dump_schema_version == diagnostics::kDumpSchemaVersion,
        "run A: both schema versions are recorded, separately");
  Check(summary_a.provenance.git_sha.size() == 40 &&
            summary_a.provenance.git_sha != std::string(40, '0'),
        "run A: a real 40-character git sha was captured",
        summary_a.provenance.git_sha);

  Check(summary_a.manifest.frames_seen == static_cast<uint64_t>(kScans) &&
            summary_a.manifest.frames_recorded == static_cast<uint64_t>(kScans),
        "run A: the manifest accounts for all four frames as recorded");

  // THE comparison this test exists for: the replayed census must be bit-identical to what
  // the live scan reported, per record, not merely close in aggregate.
  Check(summary_a.scan_stats.size() == live_a.stats.size(),
        "run A: replay decoded one ScanStats per live scan");
  bool stats_identical = summary_a.scan_stats.size() == live_a.stats.size();
  for (std::size_t index = 0; stats_identical && index < live_a.stats.size(); ++index) {
    stats_identical = ScanStatsIdentical(live_a.stats[index], summary_a.scan_stats[index]);
  }
  Check(stats_identical,
        "run A: every replayed ScanStats field is exactly what the live scan reported "
        "(ints exact, wall time bit-exact)");

  bool self_fraction_identical = stats_identical;
  for (std::size_t index = 0; self_fraction_identical && index < live_a.stats.size(); ++index) {
    self_fraction_identical = BitEqual(live_a.stats[index].SelfHitFraction(),
                                       summary_a.scan_stats[index].SelfHitFraction());
  }
  Check(self_fraction_identical,
        "run A: the replayed self-hit fraction is bit-identical (denominator: rays_cast)");

  Check(summary_a.transforms.size() == live_a.snapshots.size(),
        "run A: replay decoded one FrameTransformSnapshot per live scan");
  bool snapshots_identical = summary_a.transforms.size() == live_a.snapshots.size();
  for (std::size_t index = 0; snapshots_identical && index < live_a.snapshots.size(); ++index) {
    snapshots_identical =
        SnapshotIdentical(live_a.snapshots[index], summary_a.transforms[index]);
  }
  Check(snapshots_identical,
        "run A: every replayed sensor pose is bit-identical to the live one, including the "
        "composition invariant");

  Check(summary_a.raw_cloud_point_counts.size() == live_a.point_counts.size(),
        "run A: replay decoded one raw cloud per live scan");
  bool clouds_identical = summary_a.raw_cloud_point_counts.size() == live_a.point_counts.size();
  for (std::size_t index = 0; clouds_identical && index < live_a.point_counts.size(); ++index) {
    clouds_identical =
        summary_a.raw_cloud_point_counts[index] == live_a.point_counts[index] &&
        BitEqual(summary_a.raw_cloud_stamps_s[index], live_a.stamps[index]);
  }
  Check(clouds_identical,
        "run A: replayed raw-cloud point counts and stamps match the live scan exactly");

  // The accepted count and the cloud size are two independent reports of the same thing;
  // if the recorder had written a stale buffer they would drift apart.
  bool accepted_matches_cloud = true;
  for (std::size_t index = 0; index < live_a.stats.size(); ++index) {
    if (static_cast<std::size_t>(live_a.stats[index].accepted) != live_a.point_counts[index]) {
      accepted_matches_cloud = false;
    }
  }
  Check(accepted_matches_cloud,
        "run A: ScanStats::accepted equals the dumped cloud's point count");

  // format: both -> a .bin AND a .jsonl for each of the three record types.
  int binary_files = 0;
  int jsonl_files = 0;
  for (const auto& census : summary_a.types) {
    if (!census.file_present) continue;
    if (census.format == diagnostics::DumpFormat::kBinary) ++binary_files;
    if (census.format == diagnostics::DumpFormat::kJsonl) ++jsonl_files;
  }
  Check(binary_files == 3 && jsonl_files == 3,
        "run A: format 'both' produced a binary and a JSONL file for each of the three "
        "record types (got " +
            std::to_string(binary_files) + " + " + std::to_string(jsonl_files) + ")");

  // Retained-stage verdicts. raw_cloud was retained AND recorded; the eight later stages do
  // not exist yet, so they must read as NEVER RECORDED - never as "produced nothing".
  int not_recorded = 0;
  int recorded = 0;
  int inconsistent = 0;
  for (const auto& stage : summary_a.stages) {
    switch (stage.verdict) {
      case diagnostics::StageAvailability::Verdict::kNotRecorded: ++not_recorded; break;
      case diagnostics::StageAvailability::Verdict::kRecordedNonEmpty: ++recorded; break;
      case diagnostics::StageAvailability::Verdict::kInconsistent: ++inconsistent; break;
      default: break;
    }
  }
  Check(recorded == 1 && not_recorded == 8 && inconsistent == 0,
        "run A: raw_cloud reads as RECORDED and the eight unbuilt stages as NOT RECORDED "
        "(recorded=" + std::to_string(recorded) + ", not_recorded=" +
            std::to_string(not_recorded) + ", inconsistent=" + std::to_string(inconsistent) +
            ")");

  // =====================================================================================
  // The real perception_replay binary, on the real dump.
  // =====================================================================================
  Section("perception_replay executable on the run A dump");

  int exit_status = 0;
  const std::string command =
      "\"" + replay_binary + "\" \"" + directory_a.string() + "\" --verbose 2>&1";
  const std::string output = RunCommand(command, exit_status);
  Check(exit_status == 0, "perception_replay exits 0 on a complete dump",
        "status=" + std::to_string(exit_status));
  Check(output.find("RESULT: PASS") != std::string::npos,
        "perception_replay reports PASS");

  // The printed numbers must be the LIVE numbers. Formats mirror replay_report.cpp's.
  const std::string accepted_needle = "accepted=" + std::to_string(live_accepted);
  Check(output.find(accepted_needle) != std::string::npos,
        "perception_replay prints the live accepted-point total (" + accepted_needle + ")");

  const std::string rays_needle = "rays_cast=" + std::to_string(live_rays);
  Check(output.find(rays_needle) != std::string::npos,
        "perception_replay prints the live rays_cast total (" + rays_needle + ")");

  char self_fraction_text[64];
  std::snprintf(self_fraction_text, sizeof(self_fraction_text), "self_hit_fraction=%.6f",
                live_rays > 0 ? static_cast<double>(live_self) / static_cast<double>(live_rays)
                              : 0.0);
  Check(output.find(self_fraction_text) != std::string::npos,
        std::string("perception_replay prints the live self-hit fraction (") +
            self_fraction_text + ")");
  Check(output.find("denominator: rays_cast") != std::string::npos,
        "perception_replay states the self-hit fraction's denominator rather than leaving it "
        "to be guessed");

  Check(output.find("NOT RECORDED") != std::string::npos &&
            output.find("RECORDED, EMPTY") == std::string::npos,
        "perception_replay's report distinguishes NOT RECORDED from RECORDED, EMPTY");
  Check(output.find(summary_a.provenance.git_sha) != std::string::npos,
        "perception_replay prints the git sha from the manifest");
  Check(output.find(summary_a.provenance.config_hash) != std::string::npos,
        "perception_replay prints the config hash from the manifest");

  // --expect-records is the machine-checkable form of the same assertion.
  int expect_status = 0;
  RunCommand("\"" + replay_binary + "\" \"" + directory_a.string() +
                 "\" --quiet --expect-records " + std::to_string(kScans),
             expect_status);
  Check(expect_status == 0, "perception_replay --expect-records 4 succeeds");
  int wrong_expect_status = 0;
  RunCommand("\"" + replay_binary + "\" \"" + directory_a.string() +
                 "\" --quiet --expect-records 99",
             wrong_expect_status);
  Check(wrong_expect_status != 0,
        "perception_replay --expect-records 99 fails (the assertion actually asserts)");

  // =====================================================================================
  // Run B: the raw cloud NOT retained, and half the frames dropped by decimation.
  // =====================================================================================
  Section("run B - retain_stage_clouds=false, decimation=2, jsonl only");

  const std::filesystem::path directory_b = scratch / "run_b";
  const LiveRun live_b = Scan(scene, directory_b, "jsonl", /*retain_clouds=*/false,
                              /*decimation=*/2);
  Check(live_b.ok, "run B: the live bring-up finished its dumps", live_b.error);
  if (live_b.ok) {
    diagnostics::ReplaySummary summary_b;
    const bool loaded_b = diagnostics::LoadReplaySummary(directory_b, summary_b, error);
    Check(loaded_b, "run B: the dump directory replays", error);
    if (loaded_b) {
      Check(summary_b.truncated_files.empty(), "run B: no truncated files");
      Check(summary_b.manifest.frames_seen == 4 && summary_b.manifest.frames_recorded == 2,
            "run B: decimation=2 recorded 2 of 4 frames, and the manifest says so");
      Check(summary_b.scan_stats.size() == 2,
            "run B: two ScanStats records, matching frames_recorded");

      // The decimated frames are the EVEN ones (index % decimation == 0), so the recorded
      // stamps must be the live scans 0 and 2 - not the first two.
      bool decimation_picked_expected_frames =
          summary_b.scan_stats.size() == 2 &&
          ScanStatsIdentical(live_b.stats[0], summary_b.scan_stats[0]) &&
          ScanStatsIdentical(live_b.stats[2], summary_b.scan_stats[1]);
      Check(decimation_picked_expected_frames,
            "run B: decimation kept live scans 0 and 2, bit-identically");

      Check(summary_b.raw_cloud_point_counts.empty(),
            "run B: no raw cloud was recorded (retain_stage_clouds=false)");

      const diagnostics::StageAvailability* raw_cloud = nullptr;
      for (const auto& stage : summary_b.stages) {
        if (stage.record_type == diagnostics::RecordType::kTimedPointCloud) raw_cloud = &stage;
      }
      Check(raw_cloud != nullptr && !raw_cloud->retained && !raw_cloud->file_present &&
                raw_cloud->verdict ==
                    diagnostics::StageAvailability::Verdict::kNotRecorded,
            "run B: the un-retained raw cloud reads as NOT RECORDED - the whole point of "
            "carrying RetainedStages in the header");

      int replay_status = 0;
      const std::string report_b = RunCommand(
          "\"" + replay_binary + "\" \"" + directory_b.string() + "\" 2>&1", replay_status);
      Check(replay_status == 0, "run B: perception_replay exits 0");
      Check(report_b.find("fewer than seen") != std::string::npos,
            "run B: the report says explicitly that frames were dropped by decimation");
      Check(report_b.find("raw_cloud") != std::string::npos &&
                report_b.find("NOT RECORDED") != std::string::npos,
            "run B: the report shows raw_cloud as NOT RECORDED");
    }
  }

  // =====================================================================================
  // Determinism: the same scene and config, scanned again, must dump identical bytes.
  // =====================================================================================
  Section("determinism - an identical run produces identical record bytes");

  const std::filesystem::path directory_c = scratch / "run_c";
  // The SAME config as run A, right down to dumps.format. That matters for the config_hash
  // assertion below: the hash covers the entire resolved tree including the `dumps:` section,
  // so a repeat run that differed only in dump format would legitimately hash differently.
  const LiveRun live_c = Scan(scene, directory_c, "both", /*retain_clouds=*/true,
                              /*decimation=*/1);
  Check(live_c.ok, "run C: the repeat run finished", live_c.error);
  if (live_c.ok) {
    bool identical = live_c.stats.size() == live_a.stats.size();
    for (std::size_t index = 0; identical && index < live_a.stats.size(); ++index) {
      // scan_wall_time_us is a wall-clock measurement and legitimately differs between runs,
      // so it is excluded here rather than being asserted and then explained away.
      const auto& a = live_a.stats[index];
      const auto& c = live_c.stats[index];
      identical = a.rays_cast == c.rays_cast && a.accepted == c.accepted &&
                  a.self_rejected == c.self_rejected && a.no_hit == c.no_hit &&
                  a.range_rejected == c.range_rejected;
    }
    Check(identical,
          "run C: the repeated scan census is identical to run A's (wall time excluded - it "
          "is a measurement, not a result)");

    diagnostics::ReplaySummary summary_c;
    if (diagnostics::LoadReplaySummary(directory_c, summary_c, error)) {
      Check(summary_c.provenance.config_hash == summary_a.provenance.config_hash,
            "run C: an identical config resolves to an identical config_hash");
      Check(summary_c.provenance.git_sha == summary_a.provenance.git_sha,
            "run C: the git sha is stable across runs of the same build");
    } else {
      Check(false, "run C: replay loads", error);
    }
  }

  // The hash's scope, asserted in both directions rather than described in a comment.
  {
    using perception::integration::PerceptionConfig;
    using perception::integration::ResolvedConfigText;

    const PerceptionConfig baseline;

    PerceptionConfig identical;
    Check(ResolvedConfigText(identical) == ResolvedConfigText(baseline),
          "two configs equal in every field resolve to identical text");

    PerceptionConfig different_pipeline;
    different_pipeline.projection.bins = 720;
    Check(ResolvedConfigText(different_pipeline) != ResolvedConfigText(baseline),
          "a changed pipeline field changes the resolved-config text");

    // Deliberate and documented: the hash covers the WHOLE resolved tree, so a dump-only
    // setting changes it too. The alternative - excluding the sections judged not to affect
    // behaviour - would make the hash identify a subset of the config chosen by an
    // unversioned judgement call.
    PerceptionConfig different_dumps;
    different_dumps.dumps.format = "binary";
    Check(ResolvedConfigText(different_dumps) != ResolvedConfigText(baseline),
          "a changed dumps-only field also changes the text (the hash covers the whole tree, "
          "by design)");
  }

  return perception_test::Report("perception_dump_bringup_test");
}
