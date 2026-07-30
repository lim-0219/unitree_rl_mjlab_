// Dump files: one header, N records of ONE record type, one footer.
//
// WHY ONE TYPE PER FILE. The alternative - a single interleaved stream of tagged records -
// needs a dynamic dispatch over 24 types on both write and read, and makes "how many
// ProjectedScans are in here" a whole-file scan. Per-type files make the census a directory
// listing, let a consumer read only the stage it cares about, and make `wc -l` meaningful on
// a JSONL dump. The run manifest (run_manifest.h) is what ties the set back together.
//
// TRUNCATION IS AN ERROR, NOT A SHORT READ. Every file ends with a footer carrying the
// record count. A reader that reaches end-of-data without seeing the footer reports the file
// as truncated. Without this, a dump whose writer was killed mid-run would replay as a
// complete-but-shorter run, and a regression fixture would silently lose its tail.
//
// SEQUENCE AND STAMP ARE CALLER-DECLARED. Eight of the 24 record types (ScanStats,
// Cluster2D, StageTiming, ...) carry no timestamp of their own, so the file-level range
// cannot be derived from the records. The writer takes them per record from the caller, who
// knows which scan the record belongs to. Passing 0 for both is legitimate for a fixture
// that has no time base; the header then reports a degenerate range rather than a wrong one.
//
// WHOLE-FILE READ. DumpReader loads the file into memory at Open(). Replay is an offline,
// human-timescale activity and records are variable-length, so a streaming reader would need
// per-record length prefixes for no present benefit. Streaming is a real forward requirement
// for the later phases (a long campaign's PerceptionFrame dump will not fit comfortably) and
// is called out as such in the phase report.
#ifndef PERCEPTION_CORE_DIAGNOSTICS_DUMP_FILE_H_
#define PERCEPTION_CORE_DIAGNOSTICS_DUMP_FILE_H_

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <type_traits>
#include <vector>

#include "perception/core/diagnostics/binary_codec.h"
#include "perception/core/diagnostics/dump_format.h"
#include "perception/core/diagnostics/json_codec.h"
#include "perception/core/diagnostics/json_value.h"

namespace perception::core::diagnostics {

enum class DumpFormat { kBinary, kJsonl };

inline const char* ToString(DumpFormat format) {
  return format == DumpFormat::kBinary ? "binary" : "jsonl";
}

// Conventional extension, so a directory listing says what each file is.
inline const char* DumpFormatExtension(DumpFormat format) {
  return format == DumpFormat::kBinary ? ".bin" : ".jsonl";
}

// ---------------------------------------------------------------------------------------
// Which RecordType a C++ contract type maps to. One place, so a writer cannot label a file
// with a type its records are not.
// ---------------------------------------------------------------------------------------
template <class T>
struct RecordTypeOf;

#define PERCEPTION_MAP_RECORD_TYPE(Type, Value)             \
  template <>                                               \
  struct RecordTypeOf<Type> {                               \
    static constexpr RecordType value = RecordType::Value;  \
  }

PERCEPTION_MAP_RECORD_TYPE(RayPattern, kRayPattern);
PERCEPTION_MAP_RECORD_TYPE(RawTimedPoint, kRawTimedPoint);
PERCEPTION_MAP_RECORD_TYPE(TimedPointCloud, kTimedPointCloud);
PERCEPTION_MAP_RECORD_TYPE(ScanStats, kScanStats);
PERCEPTION_MAP_RECORD_TYPE(FrameTransformSnapshot, kFrameTransformSnapshot);
PERCEPTION_MAP_RECORD_TYPE(DeskewedPointCloud, kDeskewedPointCloud);
PERCEPTION_MAP_RECORD_TYPE(GravityAlignedCloud, kGravityAlignedCloud);
PERCEPTION_MAP_RECORD_TYPE(GroundPlane, kGroundPlane);
PERCEPTION_MAP_RECORD_TYPE(GroundSegmentationResult, kGroundSegmentationResult);
PERCEPTION_MAP_RECORD_TYPE(LabeledPointCloud, kLabeledPointCloud);
PERCEPTION_MAP_RECORD_TYPE(ProjectionStats, kProjectionStats);
PERCEPTION_MAP_RECORD_TYPE(ProjectedScan, kProjectedScan);
PERCEPTION_MAP_RECORD_TYPE(Cluster2D, kCluster2D);
PERCEPTION_MAP_RECORD_TYPE(FittedPrimitive2D, kFittedPrimitive2D);
PERCEPTION_MAP_RECORD_TYPE(CircleObservation, kCircleObservation);
PERCEPTION_MAP_RECORD_TYPE(TrackState2D, kTrackState2D);
PERCEPTION_MAP_RECORD_TYPE(TrackPrediction2D, kTrackPrediction2D);
PERCEPTION_MAP_RECORD_TYPE(PerceptionObstacle, kPerceptionObstacle);
PERCEPTION_MAP_RECORD_TYPE(SafetyObstacle, kSafetyObstacle);
PERCEPTION_MAP_RECORD_TYPE(OracleObstacleState, kOracleObstacleState);
PERCEPTION_MAP_RECORD_TYPE(StageTiming, kStageTiming);
PERCEPTION_MAP_RECORD_TYPE(MatchedPairError, kMatchedPairError);
PERCEPTION_MAP_RECORD_TYPE(EstimationDiagnostics, kEstimationDiagnostics);
PERCEPTION_MAP_RECORD_TYPE(PerceptionFrame, kPerceptionFrame);

#undef PERCEPTION_MAP_RECORD_TYPE

// ---------------------------------------------------------------------------------------
// Binary framing constants. Part of the format; changing one is a schema bump.
// ---------------------------------------------------------------------------------------

// Precedes every binary record. Catches a desynchronised stream at the record boundary
// instead of several fields later, where the error would read as bad data rather than as a
// bad offset.
inline constexpr uint32_t kBinaryRecordMarker = 0x52454344u;  // 'RECD', little-endian bytes.

// Byte offset and length of the closing facts inside the binary header, which Close() seeks
// back and overwrites. Layout: magic(8) + schema(4) + record_type(4) + retained_mask(4).
inline constexpr std::size_t kBinaryPatchOffset = 20;
// range_valid(1) + record_count(8) + seq_first(8) + seq_last(8) + stamp_first(8) + stamp_last(8)
inline constexpr std::size_t kBinaryPatchSize = 41;

// JSONL line markers. Records never contain a key beginning with '_', so these cannot
// collide with a record object.
inline constexpr const char* kJsonlHeaderKey = "__dump_header__";
inline constexpr const char* kJsonlFooterKey = "__dump_footer__";

// Serializes/parses the provenance block. Shared by dump files and the run manifest so the
// two can never disagree about a run's identity.
void WriteProvenanceBinary(ByteWriter& writer, const RunProvenance& provenance);
bool ReadProvenanceBinary(ByteReader& reader, RunProvenance& provenance);
void WriteProvenanceJson(JsonTextWriter& writer, const RunProvenance& provenance);
bool ReadProvenanceJson(const JsonValue& node, RunProvenance& provenance, std::string& error);

// ---------------------------------------------------------------------------------------
// Writer
// ---------------------------------------------------------------------------------------
class DumpWriter {
 public:
  DumpWriter() = default;
  ~DumpWriter();

  DumpWriter(const DumpWriter&) = delete;
  DumpWriter& operator=(const DumpWriter&) = delete;

  // Creates parent directories as needed. `retained_mask` is PackRetainedStages() of the
  // producing frame's RetainedStages - see dump_format.h on why every file carries it.
  bool Open(const std::filesystem::path& path, DumpFormat format, RecordType record_type,
            const RunProvenance& provenance, uint32_t retained_mask, std::string& error);

  // Appends one record. `sequence` and `stamp_s` are the file-level index keys; see the
  // header comment on why the caller supplies them.
  template <class T>
  bool Write(const T& record, uint64_t sequence, double stamp_s, std::string& error) {
    static_assert(RecordTypeOf<T>::value != RecordType::kUnknown,
                  "this type has no RecordType mapping");
    if (!open_) {
      error = "DumpWriter::Write called on a closed writer";
      return false;
    }
    if (RecordTypeOf<T>::value != record_type_) {
      error = std::string("DumpWriter: this file holds '") + ToString(record_type_) +
              "' records but was handed a '" + ToString(RecordTypeOf<T>::value) + "'";
      return false;
    }
    if (format_ == DumpFormat::kBinary) {
      ByteWriter body;
      body.U32(kBinaryRecordMarker);
      WriteBinaryRecord(body, record);
      stream_.write(body.buffer().data(), static_cast<std::streamsize>(body.size()));
    } else {
      JsonTextWriter body(/*pretty=*/false);
      WriteJsonRecord(body, record);
      stream_ << body.text() << '\n';
    }
    if (!stream_) {
      error = "DumpWriter: write failed on " + path_.string();
      return false;
    }
    NoteRecord(sequence, stamp_s);
    return true;
  }

  // Writes the footer and, for binary, back-patches the header's closing facts. A writer
  // that is destroyed without Close() leaves a file the reader reports as truncated, which
  // is the intended outcome: a crashed run must not produce a fixture that looks finished.
  bool Close(std::string& error);

  bool open() const { return open_; }
  uint64_t record_count() const { return footer_.record_count; }
  const std::filesystem::path& path() const { return path_; }
  DumpFormat format() const { return format_; }
  RecordType record_type() const { return record_type_; }
  const DumpFileFooter& footer() const { return footer_; }

 private:
  void NoteRecord(uint64_t sequence, double stamp_s);

  std::ofstream stream_;
  std::filesystem::path path_;
  DumpFormat format_ = DumpFormat::kBinary;
  RecordType record_type_ = RecordType::kUnknown;
  uint32_t retained_mask_ = 0;
  DumpFileFooter footer_;
  bool open_ = false;
};

// ---------------------------------------------------------------------------------------
// Reader
// ---------------------------------------------------------------------------------------
class DumpReader {
 public:
  // Loads and validates the header. Fails - with a reason naming both versions - when the
  // magic is wrong or the dump schema version is not the one this build speaks.
  bool Open(const std::filesystem::path& path, std::string& error);

  // Reads the next record. Sets `end_of_stream` when the footer has been reached; returns
  // false only on an actual error, so `while (ReadNext(...) && !end)` is the intended loop.
  template <class T>
  bool ReadNext(T& record, bool& end_of_stream, std::string& error) {
    static_assert(RecordTypeOf<T>::value != RecordType::kUnknown,
                  "this type has no RecordType mapping");
    end_of_stream = false;
    if (RecordTypeOf<T>::value != header_.record_type) {
      error = std::string("DumpReader: file holds '") + ToString(header_.record_type) +
              "' records, asked for '" + ToString(RecordTypeOf<T>::value) + "'";
      return false;
    }
    if (format_ == DumpFormat::kBinary) return ReadNextBinary(record, end_of_stream, error);
    return ReadNextJsonl(record, end_of_stream, error);
  }

  // Convenience: read every record into a vector, and require a well-formed footer whose
  // count matches. This is what the replay app and the tests use.
  template <class T>
  bool ReadAll(std::vector<T>& out, std::string& error) {
    out.clear();
    bool end_of_stream = false;
    while (true) {
      T record{};
      if (!ReadNext(record, end_of_stream, error)) return false;
      if (end_of_stream) break;
      out.push_back(std::move(record));
    }
    if (!footer_present_) {
      error = "dump is truncated: no footer (the writer did not complete)";
      return false;
    }
    if (footer_.record_count != out.size()) {
      error = "dump footer declares " + std::to_string(footer_.record_count) +
              " records but " + std::to_string(out.size()) + " were read";
      return false;
    }
    return true;
  }

  // The header, with the closing facts merged in from the footer once it has been reached.
  // Before that, `header().range_valid` is true only for binary files (whose header was
  // back-patched at Close) and false for JSONL, whose header line cannot be rewritten.
  const DumpFileHeader& header() const { return header_; }
  const DumpFileFooter& footer() const { return footer_; }
  bool footer_present() const { return footer_present_; }
  DumpFormat format() const { return format_; }
  const std::filesystem::path& path() const { return path_; }

  // Reads only the header, without consuming records. Used by the replay census.
  static bool PeekHeader(const std::filesystem::path& path, DumpFileHeader& header,
                         DumpFormat& format, std::string& error);

 private:
  template <class T>
  bool ReadNextBinary(T& record, bool& end_of_stream, std::string& error) {
    ByteReader reader(buffer_.data() + position_, buffer_.size() - position_);
    const uint32_t marker = reader.U32();
    if (!reader.ok()) {
      error = "dump is truncated: expected a record marker or a footer";
      return false;
    }
    if (marker == kDumpFooterSentinel) {
      if (!ReadFooterBinary(reader, error)) return false;
      position_ += reader.position();
      end_of_stream = true;
      return true;
    }
    if (marker != kBinaryRecordMarker) {
      error = "dump stream is desynchronised: bad record marker";
      return false;
    }
    if (!ReadBinaryRecord(reader, record)) {
      error = "failed to read a '" + std::string(ToString(header_.record_type)) +
              "' record: " + reader.error();
      return false;
    }
    position_ += reader.position();
    return true;
  }

  template <class T>
  bool ReadNextJsonl(T& record, bool& end_of_stream, std::string& error) {
    while (line_index_ < lines_.size()) {
      const std::string& line = lines_[line_index_];
      if (line.empty()) {
        ++line_index_;
        continue;
      }
      JsonValue node;
      std::string parse_error;
      if (!JsonValue::Parse(line, node, parse_error)) {
        error = "malformed JSONL on line " + std::to_string(line_index_ + 1) + ": " +
                parse_error;
        return false;
      }
      if (node.Find(kJsonlFooterKey) != nullptr) {
        if (!ReadFooterJson(*node.Find(kJsonlFooterKey), error)) return false;
        ++line_index_;
        end_of_stream = true;
        return true;
      }
      if (node.Find(kJsonlHeaderKey) != nullptr) {
        // A second header line means two dumps were concatenated - a real hazard with a
        // line-oriented format, and one that would otherwise read as a longer single run.
        error = "unexpected second header line on line " + std::to_string(line_index_ + 1) +
                " (were two dumps concatenated?)";
        return false;
      }
      if (!ReadJsonRecord(node, record, error)) {
        error = "line " + std::to_string(line_index_ + 1) + ": " + error;
        return false;
      }
      ++line_index_;
      return true;
    }
    error = "dump is truncated: JSONL ended without a footer line";
    return false;
  }

  bool ReadFooterBinary(ByteReader& reader, std::string& error);
  bool ReadFooterJson(const JsonValue& node, std::string& error);
  void MergeFooterIntoHeader();

  std::filesystem::path path_;
  DumpFormat format_ = DumpFormat::kBinary;
  DumpFileHeader header_;
  DumpFileFooter footer_;
  bool footer_present_ = false;

  // Binary payload, or the raw JSONL text split into lines.
  std::string buffer_;
  std::size_t position_ = 0;
  std::vector<std::string> lines_;
  std::size_t line_index_ = 0;
};

}  // namespace perception::core::diagnostics

#endif  // PERCEPTION_CORE_DIAGNOSTICS_DUMP_FILE_H_
