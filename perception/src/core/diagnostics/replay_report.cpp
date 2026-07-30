#include "perception/core/diagnostics/replay_report.h"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <type_traits>

#include "perception/core/contracts/detection_2d.h"
#include "perception/core/contracts/diagnostics.h"
#include "perception/core/contracts/labeled_point_cloud.h"
#include "perception/core/contracts/obstacles.h"
#include "perception/core/contracts/perception_frame.h"
#include "perception/core/contracts/point_clouds.h"
#include "perception/core/contracts/projected_scan.h"
#include "perception/core/contracts/ray_pattern.h"
#include "perception/core/contracts/tracking_2d.h"
#include "perception/core/diagnostics/config_hash.h"

namespace perception::core::diagnostics {
namespace {

// Small printf-into-string helper. std::ostringstream would work, but every number in this
// report has a chosen width and printf states that at the call site.
template <class... Args>
std::string Sprintf(const char* pattern, Args... args) {
  char buffer[512];
  const int written = std::snprintf(buffer, sizeof(buffer), pattern, args...);
  if (written < 0) return std::string();
  return std::string(buffer, static_cast<std::size_t>(
                                 std::min<std::size_t>(sizeof(buffer) - 1,
                                                       static_cast<std::size_t>(written))));
}

// ---------------------------------------------------------------------------------------
// Per-type description. The generic case returns nothing rather than inventing a summary:
// a made-up "detail" line for a contract nothing produces yet would be a claim about data
// that does not exist.
// ---------------------------------------------------------------------------------------
template <class T>
std::string Describe(const std::vector<T>& /*records*/) {
  return std::string();
}

template <>
std::string Describe(const std::vector<ScanStats>& records) {
  if (records.empty()) return std::string();
  long long rays = 0;
  long long accepted = 0;
  long long self_rejected = 0;
  long long no_hit = 0;
  long long range_rejected = 0;
  int unbalanced = 0;
  for (const ScanStats& stats : records) {
    rays += stats.rays_cast;
    accepted += stats.accepted;
    self_rejected += stats.self_rejected;
    no_hit += stats.no_hit;
    range_rejected += stats.range_rejected;
    if (!stats.IsBalanced()) ++unbalanced;
  }
  // Denominator named explicitly, per the contract's own discipline: self_rejected divided
  // by RAYS CAST, not by raw hits. The two differ materially (0.0415 vs 0.0553 on this
  // repository's own data) and the shipped definition is this one.
  const double self_hit_fraction =
      rays > 0 ? static_cast<double>(self_rejected) / static_cast<double>(rays) : 0.0;
  std::string detail = Sprintf(
      "rays_cast=%lld accepted=%lld no_hit=%lld self_rejected=%lld range_rejected=%lld "
      "self_hit_fraction=%.6f (denominator: rays_cast)",
      rays, accepted, no_hit, self_rejected, range_rejected, self_hit_fraction);
  if (unbalanced != 0) {
    detail += Sprintf("  WARNING: %d scan census(es) do not balance", unbalanced);
  }
  return detail;
}

template <>
std::string Describe(const std::vector<TimedPointCloud>& records) {
  if (records.empty()) return std::string();
  std::size_t total = 0;
  std::size_t smallest = records.front().size();
  std::size_t largest = records.front().size();
  for (const TimedPointCloud& cloud : records) {
    total += cloud.size();
    smallest = std::min(smallest, cloud.size());
    largest = std::max(largest, cloud.size());
  }
  return Sprintf("points total=%zu min/scan=%zu max/scan=%zu mean/scan=%.1f", total, smallest,
                 largest, static_cast<double>(total) / static_cast<double>(records.size()));
}

template <>
std::string Describe(const std::vector<FrameTransformSnapshot>& records) {
  if (records.empty()) return std::string();
  int valid = 0;
  int self_consistent = 0;
  double lowest_z = records.front().world_from_sensor.translation().z();
  double highest_z = lowest_z;
  for (const FrameTransformSnapshot& snapshot : records) {
    if (snapshot.valid) ++valid;
    if (snapshot.IsValid()) ++self_consistent;
    const double z = snapshot.world_from_sensor.translation().z();
    lowest_z = std::min(lowest_z, z);
    highest_z = std::max(highest_z, z);
  }
  return Sprintf("valid=%d/%zu self_consistent=%d/%zu sensor_world_z=[%.4f, %.4f] m", valid,
                 records.size(), self_consistent, records.size(), lowest_z, highest_z);
}

template <>
std::string Describe(const std::vector<ProjectedScan>& records) {
  if (records.empty()) return std::string();
  long long occupied = 0;
  long long bins = 0;
  for (const ProjectedScan& scan : records) {
    occupied += scan.stats.occupied_bins;
    bins += scan.stats.total_bins;
  }
  return Sprintf("bins=%lld occupied=%lld occupancy=%.4f", bins, occupied,
                 bins > 0 ? static_cast<double>(occupied) / static_cast<double>(bins) : 0.0);
}

template <>
std::string Describe(const std::vector<LabeledPointCloud>& records) {
  if (records.empty()) return std::string();
  long long ground = 0;
  long long non_ground = 0;
  long long self_points = 0;
  long long invalid = 0;
  for (const LabeledPointCloud& cloud : records) {
    ground += cloud.segmentation.ground_points;
    non_ground += cloud.segmentation.non_ground_points;
    self_points += cloud.segmentation.self_points;
    invalid += cloud.segmentation.invalid_points;
  }
  return Sprintf("ground=%lld non_ground=%lld self=%lld invalid=%lld", ground, non_ground,
                 self_points, invalid);
}

template <>
std::string Describe(const std::vector<PerceptionFrame>& records) {
  if (records.empty()) return std::string();
  int valid = 0;
  std::size_t safety_obstacles = 0;
  for (const PerceptionFrame& frame : records) {
    if (frame.valid) ++valid;
    safety_obstacles += frame.safety_obstacles.size();
  }
  return Sprintf("valid=%d/%zu safety_obstacles_total=%zu", valid, records.size(),
                 safety_obstacles);
}

template <>
std::string Describe(const std::vector<SafetyObstacle>& records) {
  if (records.empty()) return std::string();
  int valid = 0;
  for (const SafetyObstacle& obstacle : records) {
    if (obstacle.valid) ++valid;
  }
  return Sprintf("valid=%d/%zu", valid, records.size());
}

template <>
std::string Describe(const std::vector<RayPattern>& records) {
  if (records.empty()) return std::string();
  const RayPattern& pattern = records.front();
  return Sprintf("rays=%zu azimuth=%d elevation=%d period=%.4f s consistent=%s",
                 pattern.size(), pattern.azimuth_rays, pattern.elevation_rays,
                 pattern.period_s, pattern.IsConsistent() ? "yes" : "NO");
}

// ---------------------------------------------------------------------------------------
// Read one file's worth of records and census them.
// ---------------------------------------------------------------------------------------
template <class T>
bool CensusOf(DumpReader& reader, RecordTypeCensus& census, ReplaySummary& summary,
              std::string& file_error) {
  std::vector<T> records;
  if (!reader.ReadAll(records, file_error)) return false;

  census.record_count = records.size();
  census.sequence_first = reader.footer().sequence_first;
  census.sequence_last = reader.footer().sequence_last;
  census.stamp_first_s = reader.footer().stamp_first_s;
  census.stamp_last_s = reader.footer().stamp_last_s;
  census.detail = Describe(records);

  // Keep the payloads the end-to-end test needs to compare against the live run. Clouds are
  // reduced to counts and stamps here so summarising a long run stays cheap.
  if constexpr (std::is_same_v<T, ScanStats>) {
    summary.scan_stats = records;
  } else if constexpr (std::is_same_v<T, FrameTransformSnapshot>) {
    summary.transforms = records;
  } else if constexpr (std::is_same_v<T, TimedPointCloud>) {
    summary.raw_cloud_point_counts.clear();
    summary.raw_cloud_stamps_s.clear();
    for (const TimedPointCloud& cloud : records) {
      summary.raw_cloud_point_counts.push_back(static_cast<uint64_t>(cloud.size()));
      summary.raw_cloud_stamps_s.push_back(cloud.stamp_s);
    }
  }
  return true;
}

// The one place a runtime RecordType becomes a compile-time contract type. A switch rather
// than a registry of function pointers so that adding a record type without handling it here
// is a -Wswitch warning at the point the format changed.
bool CensusDispatch(DumpReader& reader, RecordTypeCensus& census, ReplaySummary& summary,
                    std::string& file_error) {
  switch (census.record_type) {
    case RecordType::kRayPattern:
      return CensusOf<RayPattern>(reader, census, summary, file_error);
    case RecordType::kRawTimedPoint:
      return CensusOf<RawTimedPoint>(reader, census, summary, file_error);
    case RecordType::kTimedPointCloud:
      return CensusOf<TimedPointCloud>(reader, census, summary, file_error);
    case RecordType::kScanStats:
      return CensusOf<ScanStats>(reader, census, summary, file_error);
    case RecordType::kFrameTransformSnapshot:
      return CensusOf<FrameTransformSnapshot>(reader, census, summary, file_error);
    case RecordType::kDeskewedPointCloud:
      return CensusOf<DeskewedPointCloud>(reader, census, summary, file_error);
    case RecordType::kGravityAlignedCloud:
      return CensusOf<GravityAlignedCloud>(reader, census, summary, file_error);
    case RecordType::kGroundPlane:
      return CensusOf<GroundPlane>(reader, census, summary, file_error);
    case RecordType::kGroundSegmentationResult:
      return CensusOf<GroundSegmentationResult>(reader, census, summary, file_error);
    case RecordType::kLabeledPointCloud:
      return CensusOf<LabeledPointCloud>(reader, census, summary, file_error);
    case RecordType::kProjectionStats:
      return CensusOf<ProjectionStats>(reader, census, summary, file_error);
    case RecordType::kProjectedScan:
      return CensusOf<ProjectedScan>(reader, census, summary, file_error);
    case RecordType::kCluster2D:
      return CensusOf<Cluster2D>(reader, census, summary, file_error);
    case RecordType::kFittedPrimitive2D:
      return CensusOf<FittedPrimitive2D>(reader, census, summary, file_error);
    case RecordType::kCircleObservation:
      return CensusOf<CircleObservation>(reader, census, summary, file_error);
    case RecordType::kTrackState2D:
      return CensusOf<TrackState2D>(reader, census, summary, file_error);
    case RecordType::kTrackPrediction2D:
      return CensusOf<TrackPrediction2D>(reader, census, summary, file_error);
    case RecordType::kPerceptionObstacle:
      return CensusOf<PerceptionObstacle>(reader, census, summary, file_error);
    case RecordType::kSafetyObstacle:
      return CensusOf<SafetyObstacle>(reader, census, summary, file_error);
    case RecordType::kOracleObstacleState:
      return CensusOf<OracleObstacleState>(reader, census, summary, file_error);
    case RecordType::kStageTiming:
      return CensusOf<StageTiming>(reader, census, summary, file_error);
    case RecordType::kMatchedPairError:
      return CensusOf<MatchedPairError>(reader, census, summary, file_error);
    case RecordType::kEstimationDiagnostics:
      return CensusOf<EstimationDiagnostics>(reader, census, summary, file_error);
    case RecordType::kPerceptionFrame:
      return CensusOf<PerceptionFrame>(reader, census, summary, file_error);
    case RecordType::kUnknown:
      break;
  }
  file_error = "unknown record type";
  return false;
}

// `directory_view` is true when a manifest told us the whole set of files a run produced. In
// single-file mode it is false, and the absence of a file for some other stage means nothing.
void BuildStageAvailability(ReplaySummary& summary, bool directory_view) {
  summary.stages.clear();
  const RetainedStages retained = UnpackRetainedStages(summary.retained_stages_mask);
  const bool retained_flags[kRetainedStageBitCount] = {
      retained.raw_cloud,      retained.deskewed_cloud, retained.gravity_aligned_cloud,
      retained.labeled_cloud,  retained.projected_scan, retained.clusters,
      retained.primitives,     retained.observations,   retained.tracks,
  };

  for (uint32_t bit = 0; bit < kRetainedStageBitCount; ++bit) {
    StageAvailability stage;
    stage.bit_index = bit;
    stage.stage_name = RetainedStageBitName(bit);
    stage.record_type = RetainedStageBitRecordType(bit);
    stage.retained = retained_flags[bit];

    for (const RecordTypeCensus& census : summary.types) {
      if (census.record_type == stage.record_type && census.file_present) {
        stage.file_present = true;
        stage.record_count = census.record_count;
        break;
      }
    }

    if (stage.retained && stage.file_present) {
      stage.verdict = stage.record_count > 0 ? StageAvailability::Verdict::kRecordedNonEmpty
                                             : StageAvailability::Verdict::kRecordedEmpty;
    } else if (!directory_view && !stage.file_present) {
      // No manifest, and this is not the stage the single file holds. Say so instead of
      // inventing a verdict from an absence that was never asserted.
      stage.verdict = StageAvailability::Verdict::kUnknownNoDirectoryView;
    } else if (!stage.retained && !stage.file_present) {
      stage.verdict = StageAvailability::Verdict::kNotRecorded;
    } else {
      // Retained but no file, or a file for a stage the header says was not retained. Either
      // way the header and the directory disagree and neither reading is trustworthy.
      stage.verdict = StageAvailability::Verdict::kInconsistent;
    }
    summary.stages.push_back(stage);
  }
}

void VerifyConfigHash(const std::filesystem::path& directory, ReplaySummary& summary) {
  if (summary.manifest.resolved_config_path.empty()) {
    summary.config_hash_note = "the manifest names no resolved-config file";
    return;
  }
  const std::filesystem::path config_path =
      directory / summary.manifest.resolved_config_path;
  std::ifstream stream(config_path, std::ios::binary);
  if (!stream) {
    summary.config_hash_note =
        "the resolved-config file is missing: " + config_path.string();
    return;
  }
  std::ostringstream contents;
  contents << stream.rdbuf();
  const std::string actual = HashConfigText(contents.str());
  if (actual == summary.provenance.config_hash) {
    summary.config_hash_verified = true;
    summary.config_hash_note = "resolved_config.txt hashes to the recorded config_hash";
    return;
  }
  summary.config_hash_note = "MISMATCH: resolved_config.txt hashes to " + actual +
                             " but the dump records " + summary.provenance.config_hash;
}

}  // namespace

const char* ToString(StageAvailability::Verdict verdict) {
  switch (verdict) {
    case StageAvailability::Verdict::kNotRecorded:      return "NOT RECORDED";
    case StageAvailability::Verdict::kRecordedEmpty:    return "RECORDED, EMPTY";
    case StageAvailability::Verdict::kRecordedNonEmpty: return "RECORDED";
    case StageAvailability::Verdict::kInconsistent:     return "INCONSISTENT";
    case StageAvailability::Verdict::kUnknownNoDirectoryView:
      return "UNKNOWN (no manifest: single-file mode cannot see other stages)";
  }
  return "unknown";
}

bool LoadReplaySummary(const std::filesystem::path& target, ReplaySummary& out,
                       std::string& error) {
  out = ReplaySummary{};

  std::error_code filesystem_error;
  if (!std::filesystem::exists(target, filesystem_error)) {
    error = "no such path: " + target.string();
    return false;
  }

  std::filesystem::path manifest_path;
  std::filesystem::path single_file;
  std::filesystem::path directory;

  if (std::filesystem::is_directory(target, filesystem_error)) {
    directory = target;
    manifest_path = target / kRunManifestFileName;
    if (!std::filesystem::exists(manifest_path, filesystem_error)) {
      error = "no " + std::string(kRunManifestFileName) + " in " + target.string();
      return false;
    }
  } else if (target.filename() == kRunManifestFileName) {
    manifest_path = target;
    directory = target.parent_path();
  } else {
    single_file = target;
    directory = target.parent_path();
  }

  if (!manifest_path.empty()) {
    if (!RunManifest::Read(manifest_path, out.manifest, error)) return false;
    out.manifest_present = true;
    out.provenance = out.manifest.provenance;
    out.retained_stages_mask = out.manifest.retained_stages_mask;
    VerifyConfigHash(directory, out);

    for (const DumpFileEntry& entry : out.manifest.files) {
      RecordTypeCensus census;
      census.record_type = entry.record_type;
      census.path = entry.path;
      census.format = entry.format;

      const std::filesystem::path file_path = directory / entry.path;
      DumpReader reader;
      std::string file_error;
      if (!reader.Open(file_path, file_error)) {
        out.truncated_files.push_back(entry.path + ": " + file_error);
        out.types.push_back(census);
        continue;
      }
      census.file_present = true;
      census.format = reader.format();
      if (!CensusDispatch(reader, census, out, file_error)) {
        out.truncated_files.push_back(entry.path + ": " + file_error);
      } else if (census.record_count != entry.record_count) {
        // The manifest and the file's own footer must agree. When they do not, the manifest
        // was written from a different run than the file it points at.
        out.truncated_files.push_back(
            entry.path + ": the manifest declares " + std::to_string(entry.record_count) +
            " records but the file contains " + std::to_string(census.record_count));
      }
      out.types.push_back(census);
    }
    BuildStageAvailability(out, /*directory_view=*/true);
    return true;
  }

  // Single-file mode: the file's own header is the only provenance available.
  DumpReader reader;
  if (!reader.Open(single_file, error)) return false;
  out.provenance = reader.header().provenance;
  out.retained_stages_mask = reader.header().retained_stages_mask;

  RecordTypeCensus census;
  census.record_type = reader.header().record_type;
  census.path = single_file.filename().string();
  census.format = reader.format();
  census.file_present = true;
  std::string file_error;
  if (!CensusDispatch(reader, census, out, file_error)) {
    out.truncated_files.push_back(census.path + ": " + file_error);
  }
  out.types.push_back(census);
  BuildStageAvailability(out, /*directory_view=*/false);
  return true;
}

std::string RenderReplayReport(const ReplaySummary& summary, bool verbose) {
  std::string report;

  report += "=== perception dump replay ===\n\n";

  report += "-- provenance\n";
  report += Sprintf("  dump_schema_version    %u   (versions the FILE FORMAT)\n",
                    summary.provenance.dump_schema_version);
  report += Sprintf("  config_schema_version  %d   (versions the YAML KEY TREE - a "
                    "different number, deliberately)\n",
                    summary.provenance.config_schema_version);
  report += "  config_hash            " + summary.provenance.config_hash + "  (FNV-1a/64 of "
            "the resolved config text)\n";
  report += "  config_source          " + summary.provenance.config_source + "\n";
  report += "  git_sha                " + summary.provenance.git_sha +
            (summary.provenance.git_dirty ? "  (WORKING TREE WAS DIRTY)\n" : "\n");
  report += "  git_describe           " + summary.provenance.git_describe + "\n";
  report += "  created_utc            " + summary.provenance.created_utc +
            "   (wall clock; every stamp INSIDE the records is MuJoCo sim time)\n";
  report += Sprintf("  seed                   %llu\n",
                    static_cast<unsigned long long>(summary.provenance.seed));
  report += "  producer               " + summary.provenance.producer + "\n";

  if (summary.manifest_present) {
    report += "\n-- run manifest\n";
    report += Sprintf("  manifest_version       %u\n", summary.manifest.manifest_version);
    report += "  dumps.format           " + summary.manifest.dumps_format + "\n";
    report += Sprintf("  retain_stage_clouds    %s\n",
                      summary.manifest.retain_stage_clouds ? "true" : "false");
    report += Sprintf("  decimation             %d\n", summary.manifest.decimation);
    report += Sprintf("  max_frames             %d%s\n", summary.manifest.max_frames,
                      summary.manifest.max_frames == 0 ? "   (0 = unlimited)" : "");
    report += Sprintf("  frames_seen            %llu\n",
                      static_cast<unsigned long long>(summary.manifest.frames_seen));
    report += Sprintf("  frames_recorded        %llu%s\n",
                      static_cast<unsigned long long>(summary.manifest.frames_recorded),
                      summary.manifest.frames_seen != summary.manifest.frames_recorded
                          ? "   (fewer than seen: decimation / max_frames)"
                          : "");
    report += "  resolved_config        " + summary.manifest.resolved_config_path + "  " +
              (summary.config_hash_verified ? "[hash OK]" : "[hash NOT verified]") + "\n";
    if (!summary.config_hash_note.empty()) {
      report += "    " + summary.config_hash_note + "\n";
    }
  } else {
    report += "\n-- run manifest\n  none (single-file mode; provenance above is the file "
              "header's own)\n";
  }

  report += "\n-- retained stages: never-recorded vs ran-and-produced-nothing\n";
  if (!summary.manifest_present) {
    report += "  (single-file mode: only the stage this file holds can be judged; the others\n"
              "   are unknown rather than absent - point this at the dump DIRECTORY for the\n"
              "   full verdict)\n";
  }
  for (const StageAvailability& stage : summary.stages) {
    report += Sprintf("  %-22s retained=%-5s file=%-5s records=%-8llu %s\n", stage.stage_name,
                      stage.retained ? "true" : "false", stage.file_present ? "yes" : "no",
                      static_cast<unsigned long long>(stage.record_count),
                      ToString(stage.verdict));
  }

  report += "\n-- record census\n";
  if (summary.types.empty()) {
    report += "  (no dump files)\n";
  }
  for (const RecordTypeCensus& census : summary.types) {
    report += Sprintf("  %-26s %-7s %6llu records", ToString(census.record_type),
                      census.file_present ? ToString(census.format) : "MISSING",
                      static_cast<unsigned long long>(census.record_count));
    if (census.record_count > 0) {
      report += Sprintf("  seq [%llu..%llu]  stamp [%.6f..%.6f] s",
                        static_cast<unsigned long long>(census.sequence_first),
                        static_cast<unsigned long long>(census.sequence_last),
                        census.stamp_first_s, census.stamp_last_s);
    }
    report += "\n";
    if (!census.detail.empty()) report += "      " + census.detail + "\n";
  }

  if (verbose && !summary.scan_stats.empty()) {
    report += "\n-- per-scan census (ScanStats)\n";
    for (std::size_t index = 0; index < summary.scan_stats.size(); ++index) {
      const ScanStats& stats = summary.scan_stats[index];
      report += Sprintf(
          "  scan %-3zu rays=%d accepted=%d no_hit=%d self=%d range=%d self_frac=%.6f "
          "balanced=%s\n",
          index, stats.rays_cast, stats.accepted, stats.no_hit, stats.self_rejected,
          stats.range_rejected, stats.SelfHitFraction(),
          stats.IsBalanced() ? "yes" : "NO");
    }
  }

  if (verbose && !summary.raw_cloud_point_counts.empty()) {
    report += "\n-- per-scan raw cloud\n";
    for (std::size_t index = 0; index < summary.raw_cloud_point_counts.size(); ++index) {
      report += Sprintf("  scan %-3zu points=%llu stamp=%.6f s\n", index,
                        static_cast<unsigned long long>(summary.raw_cloud_point_counts[index]),
                        summary.raw_cloud_stamps_s[index]);
    }
  }

  if (!summary.truncated_files.empty()) {
    report += "\n-- PROBLEMS (a dump with these is not a complete run)\n";
    for (const std::string& problem : summary.truncated_files) {
      report += "  " + problem + "\n";
    }
  }

  report += "\n";
  return report;
}

}  // namespace perception::core::diagnostics
