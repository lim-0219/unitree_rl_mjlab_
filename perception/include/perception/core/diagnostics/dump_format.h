// The on-disk dump format: versions, record identity, file header/footer, provenance.
//
// WHY THIS EXISTS. Dumps are two things at once (architecture doc section 10 P2 and
// section 13): the regression-fixture format for every later phase, and the headless
// inspection channel that keeps RViz off the critical path. Both uses require that a file
// found on disk months later can say what it is, what produced it, and what was NOT in it.
//
// THE DUMP SCHEMA VERSION IS NOT THE CONFIG SCHEMA VERSION. `kDumpSchemaVersion` below
// versions the byte layout of these files. `PerceptionConfig::schema_version` versions the
// YAML key tree. They are different artefacts with different change triggers - adding a
// tracking parameter bumps the config schema and leaves every dump readable; adding a field
// to TrackState2D bumps the dump schema and leaves every YAML valid - so they are kept as
// two fields and both are recorded in every file. Conflating them would make one of the two
// numbers a lie the first time either changed alone.
//
// core/diagnostics is the only part of core that does file I/O, and it still depends on
// nothing but core/contracts and the standard library (architecture doc section 8).
#ifndef PERCEPTION_CORE_DIAGNOSTICS_DUMP_FORMAT_H_
#define PERCEPTION_CORE_DIAGNOSTICS_DUMP_FORMAT_H_

#include <cstdint>
#include <string>

#include "perception/core/contracts/perception_frame.h"

namespace perception::core::diagnostics {

// Byte layout of the dump files. BUMP THIS whenever a contract's serialized field list
// changes in any way that is not a pure append at the end of the outermost record.
inline constexpr uint32_t kDumpSchemaVersion = 1;

// Version of the run-manifest JSON key set. Separate again, because the manifest is a
// sibling index rather than part of the record stream and can gain keys without touching
// a single serialized byte.
inline constexpr uint32_t kManifestVersion = 1;

// 8 bytes. The trailing 0x1A is the traditional end-of-text byte: it stops a binary dump
// from scrolling a terminal to death if someone `cat`s it, and it makes a file that has
// been mangled by a text-mode transfer fail the magic check instead of the length check.
inline constexpr char kDumpMagic[8] = {'P', 'C', 'P', 'D', 'U', 'M', 'P', '\x1a'};
inline constexpr int kDumpMagicSize = 8;

// Marks the end of the record stream in both formats. Its absence is how a truncated dump
// is detected, which is what stops a half-written file from replaying as a short-but-valid
// one (the failure mode the P2 acceptance criteria call out by name).
inline constexpr uint32_t kDumpFooterSentinel = 0xF00DFACEu;

// Every dumpable contract. Values are EXPLICIT and permanent: a record type id is written
// into files, so renumbering one silently reinterprets every dump ever taken. Append new
// types with the next free number; never reuse a retired one.
enum class RecordType : uint32_t {
  kUnknown = 0,
  kRayPattern = 1,
  kRawTimedPoint = 2,
  kTimedPointCloud = 3,
  kScanStats = 4,
  kFrameTransformSnapshot = 5,
  kDeskewedPointCloud = 6,
  kGravityAlignedCloud = 7,
  kGroundPlane = 8,
  kGroundSegmentationResult = 9,
  kLabeledPointCloud = 10,
  kProjectionStats = 11,
  kProjectedScan = 12,
  kCluster2D = 13,
  kFittedPrimitive2D = 14,
  kCircleObservation = 15,
  kTrackState2D = 16,
  kTrackPrediction2D = 17,
  kPerceptionObstacle = 18,
  kSafetyObstacle = 19,
  kOracleObstacleState = 20,
  kStageTiming = 21,
  kMatchedPairError = 22,
  kEstimationDiagnostics = 23,
  kPerceptionFrame = 24,
};

// The number of ids currently defined, for iterating the inventory in tests and in the
// replay census. Not a valid record type itself.
inline constexpr uint32_t kRecordTypeCount = 25;

// Stable snake_case names. These appear in file names, in the JSONL header line and in the
// manifest, so they are part of the format just as much as the numeric ids are.
inline const char* ToString(RecordType type) {
  switch (type) {
    case RecordType::kRayPattern:              return "ray_pattern";
    case RecordType::kRawTimedPoint:           return "raw_timed_point";
    case RecordType::kTimedPointCloud:         return "timed_point_cloud";
    case RecordType::kScanStats:               return "scan_stats";
    case RecordType::kFrameTransformSnapshot:  return "frame_transform_snapshot";
    case RecordType::kDeskewedPointCloud:      return "deskewed_point_cloud";
    case RecordType::kGravityAlignedCloud:     return "gravity_aligned_cloud";
    case RecordType::kGroundPlane:             return "ground_plane";
    case RecordType::kGroundSegmentationResult:return "ground_segmentation_result";
    case RecordType::kLabeledPointCloud:       return "labeled_point_cloud";
    case RecordType::kProjectionStats:         return "projection_stats";
    case RecordType::kProjectedScan:           return "projected_scan";
    case RecordType::kCluster2D:               return "cluster_2d";
    case RecordType::kFittedPrimitive2D:       return "fitted_primitive_2d";
    case RecordType::kCircleObservation:       return "circle_observation";
    case RecordType::kTrackState2D:            return "track_state_2d";
    case RecordType::kTrackPrediction2D:       return "track_prediction_2d";
    case RecordType::kPerceptionObstacle:      return "perception_obstacle";
    case RecordType::kSafetyObstacle:          return "safety_obstacle";
    case RecordType::kOracleObstacleState:     return "oracle_obstacle_state";
    case RecordType::kStageTiming:             return "stage_timing";
    case RecordType::kMatchedPairError:        return "matched_pair_error";
    case RecordType::kEstimationDiagnostics:   return "estimation_diagnostics";
    case RecordType::kPerceptionFrame:         return "perception_frame";
    case RecordType::kUnknown:                 break;
  }
  return "unknown";
}

// Inverse of ToString. Returns kUnknown for anything unrecognised rather than guessing.
RecordType RecordTypeFromString(const std::string& name);

// True when `value` is a defined record type. Used by the reader: a record type id from a
// file is untrusted input, and casting an arbitrary uint32 to the enum and switching on it
// would be undefined behaviour dressed up as a parse.
bool IsKnownRecordType(uint32_t value);

// ---------------------------------------------------------------------------------------
// RetainedStages as a bitmask.
//
// This is THE field that disambiguates "the stage never ran / was never recorded" from
// "the stage ran and produced nothing". An empty dump section is otherwise identical in
// both cases, and a replay tool that cannot tell them apart will report a broken pipeline
// as an idle one (or worse, the reverse). PerceptionFrame::retained already carries the
// same nine flags; packing them into one integer keeps the file header a fixed size across
// schema versions.
// ---------------------------------------------------------------------------------------
enum RetainedStageBit : uint32_t {
  kRetainRawCloud            = 1u << 0,
  kRetainDeskewedCloud       = 1u << 1,
  kRetainGravityAlignedCloud = 1u << 2,
  kRetainLabeledCloud        = 1u << 3,
  kRetainProjectedScan       = 1u << 4,
  kRetainClusters            = 1u << 5,
  kRetainPrimitives          = 1u << 6,
  kRetainObservations        = 1u << 7,
  kRetainTracks              = 1u << 8,
};

inline constexpr uint32_t kRetainedStageBitCount = 9;
inline constexpr uint32_t kRetainedStageMask = (1u << kRetainedStageBitCount) - 1u;

uint32_t PackRetainedStages(const RetainedStages& stages);
RetainedStages UnpackRetainedStages(uint32_t mask);

// Name of each bit, for the replay report and the manifest. `bit_index` in [0, 9).
const char* RetainedStageBitName(uint32_t bit_index);

// Which RecordType each retained-stage bit gates, so the replay report can line up
// "retained?" against "a file for it exists?" without a hand-maintained second table.
RecordType RetainedStageBitRecordType(uint32_t bit_index);

// ---------------------------------------------------------------------------------------
// Provenance: enough to regenerate the run.
//
// Mirrors the reproducibility fields of dpcbf/parameter_optimization (the evaluator's
// setprecision(17) flat JSON and tune_dpcbf's seed/resolved-config summaries), with the
// git identity the architecture doc asks for in section 14 and which no existing artefact
// in this repository actually records yet.
// ---------------------------------------------------------------------------------------
struct RunProvenance {
  // The two versions, side by side and never merged. See the header comment.
  uint32_t dump_schema_version = kDumpSchemaVersion;
  int32_t config_schema_version = 0;

  // FNV-1a/64 of the canonical resolved-config text, as 16 lowercase hex digits. Hashing
  // the RESOLVED tree rather than the YAML file bytes is deliberate: two runs with
  // differently-formatted YAML that resolve to the same values are the same experiment,
  // and two runs whose YAML differs only in a comment must not look like different ones.
  std::string config_hash = "0000000000000000";

  // Where the config came from: a path, or "<built-in defaults>" when nothing was loaded.
  std::string config_source = "<built-in defaults>";

  // Captured by CMake from the working tree. "unknown" when git was unavailable, which is
  // an honest answer rather than a fabricated sha.
  std::string git_sha = "unknown";
  std::string git_describe = "unknown";

  // True when the working tree had uncommitted changes at build time. Without this a sha
  // is worse than no sha: it names a commit the binary was not built from.
  bool git_dirty = false;

  // Wall-clock ISO-8601 UTC. Provenance only - every timestamp INSIDE the records is
  // MuJoCo sim time, and the two must never be confused.
  std::string created_utc = "unknown";

  // PerceptionConfig::seed, the master run seed.
  uint64_t seed = 0;

  // Which executable wrote this, e.g. "perception_lidar_bench".
  std::string producer = "unknown";
};

// ---------------------------------------------------------------------------------------
// File header and footer.
//
// One dump file holds records of exactly ONE record type. A run therefore produces several
// files plus a manifest that indexes them; see run_manifest.h for why that split beats one
// interleaved multi-type stream.
//
// The count and the sequence/stamp ranges are CLOSING facts - a streaming writer cannot
// know them when it opens the file. They are written twice: into the footer, and back-
// patched into the header at Close() so that the binary header genuinely carries the range
// the P2 brief asks for. `range_valid` says which of those two things happened; a file whose
// writer was killed reads back with range_valid == false and no footer, and the reader
// reports it as truncated rather than as complete-but-short.
// ---------------------------------------------------------------------------------------
struct DumpFileHeader {
  uint32_t dump_schema_version = kDumpSchemaVersion;
  RecordType record_type = RecordType::kUnknown;
  uint32_t retained_stages_mask = 0;

  uint64_t record_count = 0;
  uint64_t sequence_first = 0;
  uint64_t sequence_last = 0;
  double stamp_first_s = 0.0;
  double stamp_last_s = 0.0;

  // False until Close() patches the four fields above. Not a corruption flag.
  bool range_valid = false;

  RunProvenance provenance;

  RetainedStages retained() const { return UnpackRetainedStages(retained_stages_mask); }
};

struct DumpFileFooter {
  uint64_t record_count = 0;
  uint64_t sequence_first = 0;
  uint64_t sequence_last = 0;
  double stamp_first_s = 0.0;
  double stamp_last_s = 0.0;
};

}  // namespace perception::core::diagnostics

#endif  // PERCEPTION_CORE_DIAGNOSTICS_DUMP_FORMAT_H_
