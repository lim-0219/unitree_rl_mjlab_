// DumpRecorder - the config-driven glue between `perception.yaml`'s `dumps:` section and
// core/diagnostics.
//
// WHY IT LIVES IN integration/. core/diagnostics knows how to serialize contracts and must
// not know what a PerceptionConfig is (architecture doc section 8: `core -> yaml-cpp` and
// config access from arbitrary modules are both forbidden). Everything config-shaped -
// reading `dumps.enabled`, honouring `decimation`, deciding which formats to write, computing
// the config hash, stamping the git provenance - happens here, and core receives plain
// structs and plain contracts.
//
// GENERIC BY CONSTRUCTION. `Emit<T>()` accepts any contract with a RecordTypeOf mapping, so
// the later phases wire their stage outputs in by calling it rather than by adding code here.
// The bring-up convenience below (RecordBringUpScan) is the only type-specific entry point,
// and it exists because the P4 slice is the one producer that exists today.
#ifndef PERCEPTION_INTEGRATION_DUMP_RECORDER_H_
#define PERCEPTION_INTEGRATION_DUMP_RECORDER_H_

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "perception/core/contracts/frame_transform_snapshot.h"
#include "perception/core/contracts/perception_frame.h"
#include "perception/core/contracts/timed_point_cloud.h"
#include "perception/core/diagnostics/dump_file.h"
#include "perception/core/diagnostics/run_manifest.h"
#include "perception/integration/perception_config.h"

namespace perception::integration {

// The canonical text form of a resolved config: every field, one per line, in a fixed order.
// Its FNV-1a/64 is the `config_hash` every dump header carries.
//
// Hashing this rather than the YAML file's bytes is the point: two runs whose YAML differs
// only in comments or key order are the same experiment and must hash the same, and a run
// that used built-in defaults with no file at all must still get a real hash rather than a
// placeholder.
std::string ResolvedConfigText(const PerceptionConfig& config);

// Fills a RunProvenance from the config plus this build's git identity (captured at build
// time - see cmake/GenerateBuildInfo.cmake) and the wall clock.
core::diagnostics::RunProvenance MakeRunProvenance(const PerceptionConfig& config,
                                                   const std::string& config_source,
                                                   const std::string& producer);

class DumpRecorder {
 public:
  DumpRecorder() = default;
  ~DumpRecorder();

  DumpRecorder(const DumpRecorder&) = delete;
  DumpRecorder& operator=(const DumpRecorder&) = delete;

  // Reads `config.dumps`. When `dumps.enabled` is false this is a no-op and every later call
  // is a cheap early return - the recorder is always constructed, never conditionally.
  // `directory_override` replaces `dumps.directory` when non-empty (the bench's --dump-dir).
  bool Configure(const PerceptionConfig& config, const std::string& config_source,
                 const std::string& producer, const std::filesystem::path& directory_override,
                 std::string& error);

  bool enabled() const { return enabled_; }
  const std::filesystem::path& directory() const { return directory_; }

  // Which stage outputs this run retains. Set by the producer; written into every file
  // header and into the manifest, and it is what lets a replay tell "never recorded" from
  // "ran and produced nothing".
  void set_retained_stages(const core::RetainedStages& stages);
  const core::RetainedStages& retained_stages() const { return retained_; }

  // Should this offered frame be written? Applies `decimation` and `max_frames`, and counts
  // the frame as seen either way so the manifest's frames_seen/frames_recorded pair stays
  // honest about what was dropped.
  bool ShouldRecordFrame();

  // Appends one record of any mapped contract type, to every configured format.
  template <class T>
  bool Emit(const T& record, uint64_t sequence, double stamp_s, std::string& error) {
    if (!enabled_) return true;
    constexpr core::diagnostics::RecordType kType = core::diagnostics::RecordTypeOf<T>::value;
    Sink* sink = SinkFor(kType, error);
    if (sink == nullptr) return false;
    if (sink->binary && !sink->binary->Write(record, sequence, stamp_s, error)) return false;
    if (sink->jsonl && !sink->jsonl->Write(record, sequence, stamp_s, error)) return false;
    return true;
  }

  // The P4 bring-up slice's three real products. `raw_cloud` is written only when
  // `dumps.retain_stage_clouds` is set - which is exactly the "ran but not retained" case the
  // header's RetainedStages exists to express, exercised here on live data rather than in a
  // unit test.
  bool RecordBringUpScan(const core::TimedPointCloud& cloud, const core::ScanStats& stats,
                         const core::FrameTransformSnapshot& snapshot, std::string& error);

  // Closes every file, writes resolved_config.txt and run_manifest.json. Safe to call twice;
  // the second call is a no-op. NOT called from the destructor: a manifest written during
  // stack unwinding would describe a run that failed as one that finished.
  bool Finish(std::string& error);

  uint64_t frames_seen() const { return frames_seen_; }
  uint64_t frames_recorded() const { return frames_recorded_; }

 private:
  struct Sink {
    core::diagnostics::RecordType type = core::diagnostics::RecordType::kUnknown;
    std::unique_ptr<core::diagnostics::DumpWriter> binary;
    std::unique_ptr<core::diagnostics::DumpWriter> jsonl;
  };

  Sink* SinkFor(core::diagnostics::RecordType type, std::string& error);

  bool enabled_ = false;
  bool want_binary_ = false;
  bool want_jsonl_ = false;
  bool finished_ = false;

  std::filesystem::path directory_;
  std::string dumps_format_;
  bool retain_stage_clouds_ = false;
  int32_t decimation_ = 1;
  int32_t max_frames_ = 0;

  core::RetainedStages retained_;
  uint32_t retained_mask_ = 0;
  core::diagnostics::RunProvenance provenance_;
  std::string resolved_config_text_;

  uint64_t frames_seen_ = 0;
  uint64_t frames_recorded_ = 0;

  std::vector<std::unique_ptr<Sink>> sinks_;
};

}  // namespace perception::integration

#endif  // PERCEPTION_INTEGRATION_DUMP_RECORDER_H_
