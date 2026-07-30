// The run manifest: one JSON file that indexes a run's dump files and records how to
// reproduce it.
//
// STYLE. Deliberately the same shape as dpcbf/parameter_optimization's own output - a
// hand-written flat JSON object, two-space indent, 17 significant digits on every real
// (dpcbf_rollout_evaluator.cpp WriteJson does `setprecision(17)`; FormatJsonDouble's %.17g is
// the same width) - plus the seed/resolved-config reproducibility fields tune_dpcbf carries
// in its `summary.json`. The one thing added that no existing artefact in this repository
// records is the git identity that architecture doc section 14 asks for.
//
// HOW IT RELATES TO THE PER-FRAME DUMPS. The manifest is a sibling index, not a container:
// every dump file ALSO carries the full provenance block in its own header, so a single .bin
// or .jsonl found on its own is still self-describing. The duplication is intentional -
// fixtures get copied around individually, and a fixture that cannot say which config
// produced it is not a fixture. What the manifest adds on top is the cross-file view: which
// record types a run produced at all, the whole-run frame accounting (how many scans were
// offered vs written, after decimation), and the pointer to the resolved-config text whose
// hash appears in every header.
#ifndef PERCEPTION_CORE_DIAGNOSTICS_RUN_MANIFEST_H_
#define PERCEPTION_CORE_DIAGNOSTICS_RUN_MANIFEST_H_

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "perception/core/diagnostics/dump_file.h"
#include "perception/core/diagnostics/dump_format.h"

namespace perception::core::diagnostics {

// Conventional file names inside a dump directory.
inline constexpr const char* kRunManifestFileName = "run_manifest.json";
inline constexpr const char* kResolvedConfigFileName = "resolved_config.txt";

struct DumpFileEntry {
  RecordType record_type = RecordType::kUnknown;

  // Relative to the manifest's own directory, so a dump directory can be moved or copied
  // without rewriting it. An absolute path here would make every fixture machine-specific.
  std::string path;

  DumpFormat format = DumpFormat::kBinary;

  uint64_t record_count = 0;
  uint64_t sequence_first = 0;
  uint64_t sequence_last = 0;
  double stamp_first_s = 0.0;
  double stamp_last_s = 0.0;
};

struct RunManifest {
  uint32_t manifest_version = kManifestVersion;

  RunProvenance provenance;

  // The producing frame's RetainedStages, packed. Present here as well as in every file
  // header so that a run which produced NO file for a stage can still say whether that
  // stage was meant to be retained - the distinction a replay tool needs and the one an
  // absent file cannot make on its own.
  uint32_t retained_stages_mask = 0;

  // Echo of the `dumps:` config section that governed this run, so a reader can tell a
  // deliberately thinned dump from a lossy one.
  std::string dumps_format = "jsonl";  // "jsonl" | "binary" | "both"
  bool retain_stage_clouds = false;
  int32_t decimation = 1;
  int32_t max_frames = 0;

  // Whole-run accounting. `frames_seen` counts every scan the recorder was offered;
  // `frames_recorded` counts those actually written. They differ by exactly the frames
  // dropped by `decimation` and `max_frames`, which is what stops a decimated dump from
  // reading as a run that produced fewer scans than it did.
  uint64_t frames_seen = 0;
  uint64_t frames_recorded = 0;

  // Sibling file holding the canonical resolved-config text. Its FNV-1a/64 is
  // provenance.config_hash, so the pair is self-verifying.
  std::string resolved_config_path = kResolvedConfigFileName;

  std::vector<DumpFileEntry> files;

  const DumpFileEntry* FindFile(RecordType type) const;

  RetainedStages retained() const { return UnpackRetainedStages(retained_stages_mask); }

  bool Write(const std::filesystem::path& path, std::string& error) const;
  static bool Read(const std::filesystem::path& path, RunManifest& out, std::string& error);
};

}  // namespace perception::core::diagnostics

#endif  // PERCEPTION_CORE_DIAGNOSTICS_RUN_MANIFEST_H_
