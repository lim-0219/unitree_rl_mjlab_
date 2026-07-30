// Reads a dump directory (or a single dump file) and produces a human-readable summary.
//
// THIS IS THE VISUALIZATION. Architecture doc section 13: "every stage is also dumpable
// headlessly, so RViz is never load-bearing." The corollary is that the headless channel has
// to actually answer the questions a viewer would - what came out of each stage, how much of
// it, over what time span - which is what this renders.
//
// THE THREE-WAY VERDICT is the part that matters most. An empty stage section has two
// completely different meanings, and conflating them turns a broken pipeline into an idle
// one:
//   NOT RECORDED       - the stage was not retained; the dump says nothing about it.
//   RECORDED, EMPTY    - the stage ran and genuinely produced nothing. A real finding.
//   RECORDED, N items  - the stage ran and produced N.
//   INCONSISTENT       - the retained flag and the files on disk disagree, i.e. the header
//                        is lying and neither reading can be trusted.
// The RetainedStages bitmask carried in every file header and in the manifest is what makes
// the first two separable at all.
//
// The renderer is a library function rather than code inside apps/perception_replay.cc so
// that the end-to-end test can assert on the same numbers a human reads, instead of on a
// re-implementation of them.
#ifndef PERCEPTION_CORE_DIAGNOSTICS_REPLAY_REPORT_H_
#define PERCEPTION_CORE_DIAGNOSTICS_REPLAY_REPORT_H_

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "perception/core/contracts/frame_transform_snapshot.h"
#include "perception/core/contracts/timed_point_cloud.h"
#include "perception/core/diagnostics/dump_file.h"
#include "perception/core/diagnostics/run_manifest.h"

namespace perception::core::diagnostics {

// Per-record-type census, for all 24 types, present or not.
struct RecordTypeCensus {
  RecordType record_type = RecordType::kUnknown;
  bool file_present = false;
  std::string path;
  DumpFormat format = DumpFormat::kBinary;

  uint64_t record_count = 0;
  uint64_t sequence_first = 0;
  uint64_t sequence_last = 0;
  double stamp_first_s = 0.0;
  double stamp_last_s = 0.0;

  // Populated only for types this build knows how to describe beyond a count; empty
  // otherwise. Never a fabricated placeholder.
  std::string detail;
};

struct StageAvailability {
  enum class Verdict {
    kNotRecorded,       // Retained flag false and no file: the dump is silent about it.
    kRecordedEmpty,     // Retained and present, zero records: the stage produced nothing.
    kRecordedNonEmpty,  // Retained and present with records.
    kInconsistent,      // The flag and the files on disk disagree.

    // Single-file mode only, and only for stages OTHER than the one in the file. Without a
    // manifest there is no directory view, so "no file for this stage" carries no
    // information - the file simply was not asked about. Reporting kNotRecorded here would
    // claim knowledge the input cannot support, and kInconsistent would be a false alarm.
    kUnknownNoDirectoryView,
  };

  uint32_t bit_index = 0;
  const char* stage_name = "";
  RecordType record_type = RecordType::kUnknown;
  bool retained = false;
  bool file_present = false;
  uint64_t record_count = 0;
  Verdict verdict = Verdict::kNotRecorded;
};

const char* ToString(StageAvailability::Verdict verdict);

struct ReplaySummary {
  bool manifest_present = false;
  RunManifest manifest;

  // True when the resolved-config sibling file exists AND its FNV-1a/64 matches the
  // config_hash in the provenance. False with `config_hash_note` explaining which of the two
  // failed - a mismatch means the dump and the config text next to it are from different
  // runs, which silently invalidates any reproduction attempt.
  bool config_hash_verified = false;
  std::string config_hash_note;

  // The provenance actually in force: the manifest's when there is one, otherwise the single
  // file's header.
  RunProvenance provenance;
  uint32_t retained_stages_mask = 0;

  std::vector<RecordTypeCensus> types;   // Only the types with a file, in manifest order.
  std::vector<StageAvailability> stages; // All nine, always.

  // Decoded payloads the end-to-end test asserts against. Read only when the corresponding
  // file is present; the point counts are stored instead of the clouds themselves so a
  // summary of a long run does not have to hold every point in memory.
  std::vector<ScanStats> scan_stats;
  std::vector<uint64_t> raw_cloud_point_counts;
  std::vector<double> raw_cloud_stamps_s;
  std::vector<FrameTransformSnapshot> transforms;

  // Every file whose footer was missing or whose count disagreed. Non-empty means the run
  // was interrupted; the report says so rather than presenting a short run as a whole one.
  std::vector<std::string> truncated_files;
};

// Loads whatever `target` points at. Accepts a dump directory, a run_manifest.json, or a
// single dump file. Returns false only when nothing could be read at all; per-file problems
// are reported inside the summary so one bad file does not hide the rest of a run.
bool LoadReplaySummary(const std::filesystem::path& target, ReplaySummary& out,
                       std::string& error);

// The printable report. `verbose` adds the per-record census lines for the scan-level types.
std::string RenderReplayReport(const ReplaySummary& summary, bool verbose);

}  // namespace perception::core::diagnostics

#endif  // PERCEPTION_CORE_DIAGNOSTICS_REPLAY_REPORT_H_
