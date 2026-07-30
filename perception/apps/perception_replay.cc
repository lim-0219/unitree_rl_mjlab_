// perception_replay - headless inspection of a dump directory or a single dump file.
//
// This is the "headless visualization" the architecture doc puts in place of RViz
// (section 13: "every stage is also dumpable headlessly, so RViz is never load-bearing").
// It reads what a run wrote and prints: the run manifest and its provenance, a per-record-type
// census, and - the part that needs a tool rather than a `cat` - the three-way verdict on each
// retained stage, so an empty section can be read as either "never recorded" or "ran and
// produced nothing" without guessing.
//
// It links core/diagnostics only: no MuJoCo, no yaml-cpp, no dpcbf. That is deliberate and
// checked by the link set - replaying a fixture must work on a machine that cannot run the
// simulator, which is what makes dumps usable as the regression format for later phases.
//
// Usage:
//   perception_replay <dump-dir | run_manifest.json | dumpfile.bin|.jsonl> [--verbose]
//                     [--expect-records N] [--quiet]

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>

#include "perception/core/diagnostics/replay_report.h"

namespace {

void Usage() {
  std::printf(
      "usage: perception_replay <dump-dir | run_manifest.json | dumpfile> [options]\n"
      "  --verbose            also print the per-scan census lines\n"
      "  --quiet              print nothing on success; exit status only\n"
      "  --expect-records N   fail unless every present dump file holds exactly N records\n"
      "                       (for use as a test assertion)\n");
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    Usage();
    return 2;
  }

  std::filesystem::path target;
  bool verbose = false;
  bool quiet = false;
  long long expect_records = -1;

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--verbose") {
      verbose = true;
    } else if (argument == "--quiet") {
      quiet = true;
    } else if (argument == "--expect-records" && index + 1 < argc) {
      expect_records = std::atoll(argv[++index]);
    } else if (argument == "--help" || argument == "-h") {
      Usage();
      return 0;
    } else if (argument.rfind("--", 0) == 0) {
      std::printf("unknown option: %s\n", argument.c_str());
      Usage();
      return 2;
    } else if (target.empty()) {
      target = argument;
    } else {
      std::printf("unexpected extra argument: %s\n", argument.c_str());
      Usage();
      return 2;
    }
  }

  if (target.empty()) {
    Usage();
    return 2;
  }

  perception::core::diagnostics::ReplaySummary summary;
  std::string error;
  if (!perception::core::diagnostics::LoadReplaySummary(target, summary, error)) {
    std::printf("FATAL: %s\n", error.c_str());
    return 1;
  }

  if (!quiet) {
    const std::string report =
        perception::core::diagnostics::RenderReplayReport(summary, verbose);
    std::fputs(report.c_str(), stdout);
  }

  int status = 0;

  // A dump with a missing footer or a manifest/file disagreement is reported as a failure,
  // not as a smaller successful run. Silently replaying a truncated fixture is the exact
  // failure this phase is meant to make impossible.
  if (!summary.truncated_files.empty()) {
    std::printf("RESULT: FAIL (%zu incomplete or inconsistent dump file(s))\n",
                summary.truncated_files.size());
    status = 1;
  }

  for (const auto& stage : summary.stages) {
    if (stage.verdict ==
        perception::core::diagnostics::StageAvailability::Verdict::kInconsistent) {
      std::printf(
          "RESULT: FAIL (stage '%s': the retained flag and the files on disk disagree)\n",
          stage.stage_name);
      status = 1;
    }
  }

  if (expect_records >= 0) {
    for (const auto& census : summary.types) {
      if (!census.file_present) continue;
      if (static_cast<long long>(census.record_count) != expect_records) {
        std::printf("RESULT: FAIL (%s holds %llu records, expected %lld)\n",
                    perception::core::diagnostics::ToString(census.record_type),
                    static_cast<unsigned long long>(census.record_count), expect_records);
        status = 1;
      }
    }
  }

  if (status == 0 && !quiet) std::printf("RESULT: PASS\n");
  return status;
}
