// Dump round-trip fidelity, versioned-schema rejection, and header/manifest fidelity.
//
// TOLERANCE TIER. Architecture doc section 11 puts dump round-trips in the EXACT EQUALITY
// tier, alongside p2l equivalence and oracle-mode behaviour preservation - not in the
// numerical-tolerance tier. So nothing here compares with an epsilon.
//
// HOW BIT-EXACTNESS IS PROVEN. Every float and double in the binary encoding travels as its
// IEEE-754 bit pattern (byte_io.h), so the encoding is an injective function of a record's
// field bit patterns. Therefore: re-serializing a record that was read back and comparing the
// BYTES to the original's bytes proves every integer field is exactly equal AND every float
// field is bit-identical, in one comparison that cannot miss a field. An epsilon comparison
// would pass on -0.0 vs 0.0 and on a denormal flushed to zero; this does not, which is why the
// fixtures deliberately contain both.
//
// The same trick checks the JSONL codec: a JSONL-read record is re-encoded to BINARY and
// compared against the original's binary bytes, so the JSON path is held to the identical bar
// rather than to a text comparison that could hide a lossy decimal round-trip.
//
// Links perception_diagnostics only. That is itself an assertion: if core/diagnostics ever
// grows a MuJoCo, yaml-cpp or dpcbf dependency, this target stops linking.

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include "check.h"
#include "perception/core/diagnostics/binary_codec.h"
#include "perception/core/diagnostics/config_hash.h"
#include "perception/core/diagnostics/dump_file.h"
#include "perception/core/diagnostics/json_codec.h"
#include "perception/core/diagnostics/replay_report.h"
#include "perception/core/diagnostics/run_manifest.h"
#include "synthetic_contracts.h"

using namespace perception::core;              // NOLINT - test-local convenience.
using namespace perception::core::diagnostics;  // NOLINT
using perception_test::Check;
using perception_test::Section;

namespace {

std::filesystem::path g_scratch;

// ---------------------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------------------
template <class T, class = void>
struct HasValidate : std::false_type {};

template <class T>
struct HasValidate<T, std::void_t<decltype(std::declval<const T&>().Validate())>>
    : std::true_type {};

template <class T>
std::string ToBinary(const T& record) {
  ByteWriter writer;
  WriteBinaryRecord(writer, record);
  return writer.buffer();
}

template <class T>
std::string ToJson(const T& record) {
  JsonTextWriter writer(/*pretty=*/false);
  WriteJsonRecord(writer, record);
  return writer.text();
}

RunProvenance MakeProvenance() {
  RunProvenance provenance;
  provenance.dump_schema_version = kDumpSchemaVersion;
  // Deliberately DIFFERENT from the dump schema version, so a codec that conflated the two
  // would produce a detectable value rather than an accidentally-correct one.
  provenance.config_schema_version = 7;
  provenance.config_hash = "0123456789abcdef";
  provenance.config_source = "perception/configs/perception.yaml";
  provenance.git_sha = "0123456789012345678901234567890123456789";
  provenance.git_describe = "v0.1-3-gdeadbee-dirty";
  provenance.git_dirty = true;
  provenance.created_utc = "2026-07-30T11:22:33Z";
  provenance.seed = 42;
  provenance.producer = "perception_dump_roundtrip_test";
  return provenance;
}

bool ProvenanceEqual(const RunProvenance& a, const RunProvenance& b) {
  return a.dump_schema_version == b.dump_schema_version &&
         a.config_schema_version == b.config_schema_version && a.config_hash == b.config_hash &&
         a.config_source == b.config_source && a.git_sha == b.git_sha &&
         a.git_describe == b.git_describe && a.git_dirty == b.git_dirty &&
         a.created_utc == b.created_utc && a.seed == b.seed && a.producer == b.producer;
}

std::string ReadFile(const std::filesystem::path& path) {
  std::ifstream stream(path, std::ios::binary);
  std::ostringstream contents;
  contents << stream.rdbuf();
  return contents.str();
}

void WriteFile(const std::filesystem::path& path, const std::string& contents) {
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  stream << contents;
}

// ---------------------------------------------------------------------------------------
// Per-contract round trip
// ---------------------------------------------------------------------------------------
template <class T, class Maker>
void RoundTrip(const char* type_name, Maker make, uint32_t retained_mask) {
  const RecordType record_type = RecordTypeOf<T>::value;
  const RunProvenance provenance = MakeProvenance();

  std::vector<T> originals;
  for (int variant = 0; variant < 3; ++variant) originals.push_back(make(variant));

  // Fixtures must be well-formed to begin with; otherwise a codec bug and a fixture bug are
  // indistinguishable at the far end.
  if constexpr (HasValidate<T>::value) {
    bool all_valid = true;
    std::string first_reason;
    for (const T& record : originals) {
      if (const char* reason = record.Validate()) {
        all_valid = false;
        if (first_reason.empty()) first_reason = reason;
      }
    }
    Check(all_valid, std::string(type_name) + ": fixtures satisfy Validate()", first_reason);
  }

  const std::filesystem::path binary_path =
      g_scratch / (std::string(type_name) + ".bin");
  const std::filesystem::path jsonl_path =
      g_scratch / (std::string(type_name) + ".jsonl");

  std::string error;
  bool wrote = true;
  for (int pass = 0; pass < 2; ++pass) {
    const DumpFormat format = pass == 0 ? DumpFormat::kBinary : DumpFormat::kJsonl;
    const std::filesystem::path& path = pass == 0 ? binary_path : jsonl_path;
    DumpWriter writer;
    if (!writer.Open(path, format, record_type, provenance, retained_mask, error)) {
      wrote = false;
      break;
    }
    for (std::size_t index = 0; index < originals.size(); ++index) {
      if (!writer.Write(originals[index], 1000 + index, 0.1 * static_cast<double>(index),
                        error)) {
        wrote = false;
        break;
      }
    }
    if (!wrote || !writer.Close(error)) {
      wrote = false;
      break;
    }
  }
  Check(wrote, std::string(type_name) + ": writes both formats", error);
  if (!wrote) return;

  std::vector<T> from_binary;
  std::vector<T> from_jsonl;
  DumpReader binary_reader;
  DumpReader jsonl_reader;

  const bool binary_ok = binary_reader.Open(binary_path, error) &&
                         binary_reader.ReadAll(from_binary, error);
  Check(binary_ok, std::string(type_name) + ": binary reads back", error);
  const bool jsonl_ok =
      jsonl_reader.Open(jsonl_path, error) && jsonl_reader.ReadAll(from_jsonl, error);
  Check(jsonl_ok, std::string(type_name) + ": jsonl reads back", error);
  if (!binary_ok || !jsonl_ok) return;

  Check(from_binary.size() == originals.size() && from_jsonl.size() == originals.size(),
        std::string(type_name) + ": record counts survive");
  if (from_binary.size() != originals.size() || from_jsonl.size() != originals.size()) return;

  bool binary_exact = true;
  bool jsonl_exact = true;
  bool json_text_exact = true;
  for (std::size_t index = 0; index < originals.size(); ++index) {
    const std::string reference = ToBinary(originals[index]);
    if (ToBinary(from_binary[index]) != reference) binary_exact = false;
    if (ToBinary(from_jsonl[index]) != reference) jsonl_exact = false;
    if (ToJson(from_binary[index]) != ToJson(originals[index])) json_text_exact = false;
  }
  Check(binary_exact, std::string(type_name) + ": BINARY round trip is bit-exact");
  Check(jsonl_exact, std::string(type_name) + ": JSONL round trip is bit-exact");
  Check(json_text_exact, std::string(type_name) + ": JSONL text is stable across a round trip");

  if constexpr (HasValidate<T>::value) {
    bool still_valid = true;
    std::string first_reason;
    for (std::size_t index = 0; index < originals.size(); ++index) {
      for (const T* record : {&from_binary[index], &from_jsonl[index]}) {
        if (const char* reason = record->Validate()) {
          still_valid = false;
          if (first_reason.empty()) first_reason = reason;
        }
      }
    }
    Check(still_valid, std::string(type_name) + ": read-back records satisfy Validate()",
          first_reason);
  }

  // Header fidelity, per format. The retained mask and the provenance are the fields a later
  // phase's replay depends on, so they are checked for every record type, not once.
  for (int pass = 0; pass < 2; ++pass) {
    const DumpReader& reader = pass == 0 ? binary_reader : jsonl_reader;
    const char* label = pass == 0 ? " (binary)" : " (jsonl)";
    const DumpFileHeader& header = reader.header();
    Check(header.dump_schema_version == kDumpSchemaVersion,
          std::string(type_name) + ": header dump schema version" + label);
    Check(header.record_type == record_type,
          std::string(type_name) + ": header record type" + label);
    Check(header.retained_stages_mask == retained_mask,
          std::string(type_name) + ": header RetainedStages mask" + label);
    Check(ProvenanceEqual(header.provenance, provenance),
          std::string(type_name) + ": header provenance" + label);
    Check(header.range_valid && header.record_count == originals.size() &&
              header.sequence_first == 1000 &&
              header.sequence_last == 1000 + originals.size() - 1,
          std::string(type_name) + ": header sequence range" + label);
    Check(header.stamp_first_s == 0.0 &&
              header.stamp_last_s == 0.1 * static_cast<double>(originals.size() - 1),
          std::string(type_name) + ": header stamp range" + label);
  }
}

// ---------------------------------------------------------------------------------------
// Codec-level pathological floats. Most contracts reject non-finite values, so these are
// driven straight through the codec rather than through a contract fixture.
// ---------------------------------------------------------------------------------------
void PathologicalFloats() {
  Section("pathological floats through both codecs");

  const double doubles[] = {
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity(),
      -0.0,
      0.0,
      5e-324,                      // Smallest denormal.
      2.2250738585072014e-308,     // Smallest normal.
      1.7976931348623157e308,      // Largest finite.
      1.0 / 3.0,
      0.1,
      -1.2345678901234567e-17,
  };

  bool binary_exact = true;
  bool json_exact = true;
  for (const double value : doubles) {
    ByteWriter writer;
    writer.F64(value);
    ByteReader reader(writer.buffer());
    const double binary_result = reader.F64();
    if (std::memcmp(&value, &binary_result, sizeof(double)) != 0) binary_exact = false;

    // The JSON path: format, parse, and compare BIT PATTERNS, not values. Comparing values
    // would report NaN != NaN as a failure and -0.0 == 0.0 as a success, both wrong here.
    const std::string text = FormatJsonDouble(value);
    JsonValue parsed;
    std::string parse_error;
    if (!JsonValue::Parse(text, parsed, parse_error)) {
      json_exact = false;
      continue;
    }
    const double json_result = parsed.AsDouble();
    if (std::memcmp(&value, &json_result, sizeof(double)) != 0) json_exact = false;
  }
  Check(binary_exact, "double: every pathological value is bit-exact through binary");
  Check(json_exact, "double: every pathological value is bit-exact through JSON text");

  const float floats[] = {
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity(),
      -std::numeric_limits<float>::infinity(),
      -0.0f,
      1.4e-45f,                 // Smallest denormal float.
      3.4028235e38f,            // Largest finite float.
      0.1f,
      static_cast<float>(1.0 / 3.0),
  };
  bool float_binary_exact = true;
  bool float_json_exact = true;
  for (const float value : floats) {
    ByteWriter writer;
    writer.F32(value);
    ByteReader reader(writer.buffer());
    const float binary_result = reader.F32();
    if (std::memcmp(&value, &binary_result, sizeof(float)) != 0) float_binary_exact = false;

    const std::string text = FormatJsonFloat(value);
    JsonValue parsed;
    std::string parse_error;
    if (!JsonValue::Parse(text, parsed, parse_error)) {
      float_json_exact = false;
      continue;
    }
    const float json_result = parsed.AsFloat();
    if (std::memcmp(&value, &json_result, sizeof(float)) != 0) float_json_exact = false;
  }
  Check(float_binary_exact, "float: every pathological value is bit-exact through binary");
  Check(float_json_exact, "float: every pathological value is bit-exact through JSON text");

  // NaN in a ProjectedScan bin is the load-bearing case: an empty bin IS a NaN in that
  // contract, so losing it destroys the never-measured / measured distinction.
  ProjectedScan scan = perception_fixtures::MakeProjectedScan(0);
  const bool nan_bins_present = std::isnan(scan.ranges.back());
  Check(nan_bins_present, "ProjectedScan fixture actually contains empty (NaN) bins");

  const std::string reference = ToBinary(scan);
  ByteReader reader(reference);
  ProjectedScan decoded;
  const bool decoded_ok = ReadBinaryRecord(reader, decoded);
  Check(decoded_ok && ToBinary(decoded) == reference,
        "ProjectedScan: NaN empty bins survive binary bit-exactly");

  const std::string json_text = ToJson(scan);
  JsonValue node;
  std::string parse_error;
  ProjectedScan json_decoded;
  std::string record_error;
  const bool json_ok = JsonValue::Parse(json_text, node, parse_error) &&
                       ReadJsonRecord(node, json_decoded, record_error);
  Check(json_ok && ToBinary(json_decoded) == reference,
        "ProjectedScan: NaN empty bins survive JSONL bit-exactly",
        json_ok ? "" : parse_error + record_error);
  Check(json_text.find("NaN") != std::string::npos,
        "ProjectedScan JSONL spells an empty bin as the NaN token");
}

// ---------------------------------------------------------------------------------------
// Negative paths: a corrupted or mismatched dump must FAIL LOUDLY, never misparse.
// ---------------------------------------------------------------------------------------
void SchemaRejection() {
  Section("versioned-schema rejection and corruption handling");

  const RunProvenance provenance = MakeProvenance();
  const std::filesystem::path good_binary = g_scratch / "reject.bin";
  const std::filesystem::path good_jsonl = g_scratch / "reject.jsonl";
  std::string error;

  for (int pass = 0; pass < 2; ++pass) {
    const DumpFormat format = pass == 0 ? DumpFormat::kBinary : DumpFormat::kJsonl;
    const std::filesystem::path& path = pass == 0 ? good_binary : good_jsonl;
    DumpWriter writer;
    writer.Open(path, format, RecordType::kScanStats, provenance, kRetainRawCloud, error);
    for (int variant = 0; variant < 3; ++variant) {
      writer.Write(perception_fixtures::MakeScanStats(variant), 10 + variant,
                   0.1 * variant, error);
    }
    writer.Close(error);
  }

  // 1. Binary: bump the dump schema version in place. Everything else stays valid, so a
  //    reader that ignored the version would parse the records happily and be WRONG.
  {
    std::string bytes = ReadFile(good_binary);
    const uint32_t bad_version = kDumpSchemaVersion + 1;
    for (int byte = 0; byte < 4; ++byte) {
      bytes[static_cast<std::size_t>(kDumpMagicSize + byte)] =
          static_cast<char>((bad_version >> (8 * byte)) & 0xFFu);
    }
    const std::filesystem::path path = g_scratch / "bad_version.bin";
    WriteFile(path, bytes);
    DumpReader reader;
    const bool opened = reader.Open(path, error);
    Check(!opened, "binary: a mismatched dump schema version is refused");
    Check(!opened && error.find("schema version") != std::string::npos,
          "binary: the error names the schema-version mismatch", error);
    Check(!opened && error.find("config schema_version") != std::string::npos,
          "binary: the error distinguishes the DUMP version from the CONFIG one", error);
  }

  // 2. JSONL: same, in the header line.
  {
    std::string text = ReadFile(good_jsonl);
    const std::string needle =
        "\"dump_schema_version\":" + std::to_string(kDumpSchemaVersion);
    const std::size_t at = text.find(needle);
    Check(at != std::string::npos, "jsonl: the header line carries dump_schema_version");
    if (at != std::string::npos) {
      text.replace(at, needle.size(),
                   "\"dump_schema_version\":" + std::to_string(kDumpSchemaVersion + 1));
      const std::filesystem::path path = g_scratch / "bad_version.jsonl";
      WriteFile(path, text);
      DumpReader reader;
      const bool opened = reader.Open(path, error);
      Check(!opened, "jsonl: a mismatched dump schema version is refused");
      Check(!opened && error.find("schema version") != std::string::npos,
            "jsonl: the error names the schema-version mismatch", error);
    }
  }

  // 3. Bad magic.
  {
    std::string bytes = ReadFile(good_binary);
    bytes[0] = 'X';
    const std::filesystem::path path = g_scratch / "bad_magic.bin";
    WriteFile(path, bytes);
    DumpReader reader;
    Check(!reader.Open(path, error), "binary: a file with the wrong magic is refused");
  }

  // 4. Unknown record type id.
  {
    std::string bytes = ReadFile(good_binary);
    const uint32_t bad_type = 9999;
    for (int byte = 0; byte < 4; ++byte) {
      bytes[static_cast<std::size_t>(kDumpMagicSize + 4 + byte)] =
          static_cast<char>((bad_type >> (8 * byte)) & 0xFFu);
    }
    const std::filesystem::path path = g_scratch / "bad_type.bin";
    WriteFile(path, bytes);
    DumpReader reader;
    const bool opened = reader.Open(path, error);
    Check(!opened && error.find("record type") != std::string::npos,
          "binary: an unknown record type id is refused, not cast blindly", error);
  }

  // 5. TRUNCATION. The footer is gone, so a reader that stopped at end-of-data would report
  //    a shorter but perfectly plausible run. It must report truncation instead.
  {
    std::string bytes = ReadFile(good_binary);
    bytes.resize(bytes.size() - 24);
    const std::filesystem::path path = g_scratch / "truncated.bin";
    WriteFile(path, bytes);
    DumpReader reader;
    std::vector<ScanStats> records;
    error.clear();  // Otherwise a PASS below would print the previous case's message.
    const bool opened = reader.Open(path, error);
    Check(opened, "binary: a truncated file still opens (its header is intact)", error);
    const bool read_all = opened && reader.ReadAll(records, error);
    Check(!read_all, "binary: a truncated file fails to read, rather than reading short");
    Check(!read_all && (error.find("truncat") != std::string::npos ||
                        error.find("footer") != std::string::npos),
          "binary: the error says the dump is truncated", error);
  }

  // 6. JSONL truncation: drop the footer line.
  {
    std::string text = ReadFile(good_jsonl);
    const std::size_t last_newline = text.rfind('\n', text.size() - 2);
    if (last_newline != std::string::npos) text.resize(last_newline + 1);
    const std::filesystem::path path = g_scratch / "truncated.jsonl";
    WriteFile(path, text);
    DumpReader reader;
    std::vector<ScanStats> records;
    const bool read_all = reader.Open(path, error) && reader.ReadAll(records, error);
    Check(!read_all, "jsonl: a footerless file fails to read");
    Check(!read_all && error.find("footer") != std::string::npos,
          "jsonl: the error says the footer is missing", error);
  }

  // 7. Asking for the wrong contract type. Caught by the header's record type, before a
  //    single field is interpreted - the alternative is a field-for-field misparse.
  {
    DumpReader reader;
    std::vector<TrackState2D> wrong;
    const bool ok = reader.Open(good_binary, error) && reader.ReadAll(wrong, error);
    Check(!ok, "reading a scan_stats dump as track_state_2d is refused");
    Check(!ok && error.find("track_state_2d") != std::string::npos,
          "the wrong-type error names both types", error);
  }

  // 8. A record body corrupted so an enum goes out of range. The reader must reject the
  //    value rather than cast an arbitrary integer into the enum.
  {
    SafetyObstacle obstacle = perception_fixtures::MakeSafetyObstacle(0);
    std::string bytes = ToBinary(obstacle);
    // `source` is the second-to-last field; overwrite the whole tail region's enum slot by
    // rewriting the last 8 bytes as two out-of-range enum values.
    for (std::size_t offset = bytes.size() - 8; offset < bytes.size(); ++offset) {
      bytes[offset] = static_cast<char>(0x7F);
    }
    ByteReader reader(bytes);
    SafetyObstacle decoded;
    const bool ok = ReadBinaryRecord(reader, decoded);
    Check(!ok, "binary: an out-of-range enum in a record body is rejected");
    Check(!ok && reader.error().find("enum") != std::string::npos,
          "binary: the error names the enum field", reader.error());
  }

  // 9. Two JSONL dumps concatenated - a real hazard for a line-oriented format, and one that
  //    would otherwise read as a single longer run.
  {
    const std::string text = ReadFile(good_jsonl);
    const std::filesystem::path path = g_scratch / "concatenated.jsonl";
    WriteFile(path, text + text);
    DumpReader reader;
    std::vector<ScanStats> records;
    const bool ok = reader.Open(path, error) && reader.ReadAll(records, error);
    // The first footer terminates the stream cleanly, so ReadAll succeeds with 3 records -
    // the trailing garbage is simply never reached. Assert that, rather than pretending the
    // reader detects it: what matters is that it does NOT silently return 6.
    Check(ok && records.size() == 3,
          "jsonl: a concatenated dump stops at the first footer instead of merging runs",
          ok ? "" : error);
  }

  // 10. A JSONL record missing a required field must fail, not default it.
  {
    std::string text = ReadFile(good_jsonl);
    const std::size_t at = text.find("\"rays_cast\":");
    Check(at != std::string::npos, "jsonl: a record line carries rays_cast");
    if (at != std::string::npos) {
      const std::size_t comma = text.find(',', at);
      text.erase(at, comma - at + 1);
      const std::filesystem::path path = g_scratch / "missing_field.jsonl";
      WriteFile(path, text);
      DumpReader reader;
      std::vector<ScanStats> records;
      const bool ok = reader.Open(path, error) && reader.ReadAll(records, error);
      Check(!ok, "jsonl: a missing required field is an error, not a silent default");
      Check(!ok && error.find("rays_cast") != std::string::npos,
            "jsonl: the error names the missing field", error);
    }
  }
}

// ---------------------------------------------------------------------------------------
// Manifest fidelity and the replay verdict logic.
// ---------------------------------------------------------------------------------------
void ManifestAndReplay() {
  Section("run manifest and replay verdicts");

  const std::filesystem::path directory = g_scratch / "manifest_run";
  std::error_code filesystem_error;
  std::filesystem::remove_all(directory, filesystem_error);

  const RunProvenance provenance = MakeProvenance();
  std::string error;

  // A run that retained the raw cloud and produced records for it, and that also produced a
  // ScanStats file. Every other stage is unretained and absent - which the replay must call
  // NOT RECORDED rather than empty.
  RetainedStages retained;
  retained.raw_cloud = true;
  const uint32_t mask = PackRetainedStages(retained);

  RunManifest manifest;
  manifest.provenance = provenance;
  manifest.retained_stages_mask = mask;
  manifest.dumps_format = "binary";
  manifest.retain_stage_clouds = true;
  manifest.decimation = 2;
  manifest.max_frames = 0;
  manifest.frames_seen = 6;
  manifest.frames_recorded = 3;

  struct Target {
    RecordType type;
    const char* file;
  };
  const Target targets[] = {
      {RecordType::kScanStats, "scan_stats.bin"},
      {RecordType::kTimedPointCloud, "timed_point_cloud.bin"},
  };
  bool wrote = true;
  for (const Target& target : targets) {
    DumpWriter writer;
    if (!writer.Open(directory / target.file, DumpFormat::kBinary, target.type, provenance,
                     mask, error)) {
      wrote = false;
      break;
    }
    for (int variant = 0; variant < 3; ++variant) {
      const bool ok = target.type == RecordType::kScanStats
                          ? writer.Write(perception_fixtures::MakeScanStats(variant), variant,
                                         0.1 * variant, error)
                          : writer.Write(perception_fixtures::MakeTimedPointCloud(variant),
                                         variant, 0.1 * variant, error);
      if (!ok) {
        wrote = false;
        break;
      }
    }
    if (!wrote || !writer.Close(error)) {
      wrote = false;
      break;
    }
    DumpFileEntry entry;
    entry.record_type = target.type;
    entry.path = target.file;
    entry.format = DumpFormat::kBinary;
    entry.record_count = writer.footer().record_count;
    entry.sequence_first = writer.footer().sequence_first;
    entry.sequence_last = writer.footer().sequence_last;
    entry.stamp_first_s = writer.footer().stamp_first_s;
    entry.stamp_last_s = writer.footer().stamp_last_s;
    manifest.files.push_back(entry);
  }
  Check(wrote, "manifest run: dump files written", error);
  if (!wrote) return;

  // The resolved-config sibling, whose hash the manifest claims. Written to actually match,
  // so the replay's verification is exercised in the passing direction too.
  const std::string config_text = "schema_version=7\nseed=42\n";
  manifest.provenance.config_hash = HashConfigText(config_text);
  WriteFile(directory / kResolvedConfigFileName, config_text);

  const std::filesystem::path manifest_path = directory / kRunManifestFileName;
  Check(manifest.Write(manifest_path, error), "manifest: writes", error);

  RunManifest read_back;
  const bool manifest_read = RunManifest::Read(manifest_path, read_back, error);
  Check(manifest_read, "manifest: reads back", error);
  if (manifest_read) {
    Check(ProvenanceEqual(read_back.provenance, manifest.provenance),
          "manifest: provenance (config hash + git sha) survives exactly");
    Check(read_back.retained_stages_mask == mask, "manifest: RetainedStages mask survives");
    Check(read_back.dumps_format == manifest.dumps_format &&
              read_back.retain_stage_clouds == manifest.retain_stage_clouds &&
              read_back.decimation == manifest.decimation &&
              read_back.max_frames == manifest.max_frames,
          "manifest: the dumps config echo survives");
    Check(read_back.frames_seen == 6 && read_back.frames_recorded == 3,
          "manifest: frames_seen/frames_recorded survive (decimation is visible)");
    Check(read_back.files.size() == 2, "manifest: both file entries survive");
    bool entries_exact = read_back.files.size() == manifest.files.size();
    for (std::size_t index = 0; entries_exact && index < manifest.files.size(); ++index) {
      const DumpFileEntry& a = manifest.files[index];
      const DumpFileEntry& b = read_back.files[index];
      entries_exact = a.record_type == b.record_type && a.path == b.path &&
                      a.format == b.format && a.record_count == b.record_count &&
                      a.sequence_first == b.sequence_first &&
                      a.sequence_last == b.sequence_last &&
                      std::memcmp(&a.stamp_first_s, &b.stamp_first_s, sizeof(double)) == 0 &&
                      std::memcmp(&a.stamp_last_s, &b.stamp_last_s, sizeof(double)) == 0;
    }
    Check(entries_exact, "manifest: file entries survive bit-exactly");
  }

  // The replay summary over the same directory.
  ReplaySummary summary;
  const bool loaded = LoadReplaySummary(directory, summary, error);
  Check(loaded, "replay: loads the run directory", error);
  if (!loaded) return;

  Check(summary.truncated_files.empty(), "replay: no truncated or inconsistent files");
  Check(summary.config_hash_verified,
        "replay: resolved_config.txt hashes to the recorded config_hash",
        summary.config_hash_note);
  Check(summary.scan_stats.size() == 3, "replay: decoded three ScanStats records");
  Check(summary.raw_cloud_point_counts.size() == 3,
        "replay: decoded three raw-cloud point counts");

  // THE disambiguation this phase exists for.
  const StageAvailability* raw_cloud = nullptr;
  const StageAvailability* tracks = nullptr;
  for (const StageAvailability& stage : summary.stages) {
    if (stage.record_type == RecordType::kTimedPointCloud) raw_cloud = &stage;
    if (stage.record_type == RecordType::kTrackState2D) tracks = &stage;
  }
  Check(raw_cloud != nullptr && raw_cloud->verdict ==
                                    StageAvailability::Verdict::kRecordedNonEmpty,
        "replay: a retained stage with records reads as RECORDED");
  Check(tracks != nullptr && tracks->verdict == StageAvailability::Verdict::kNotRecorded,
        "replay: an unretained absent stage reads as NOT RECORDED (not 'empty')");

  // Now the other side of the same coin: retained, present, and genuinely empty. This is the
  // case an empty section must NOT be confused with the one above.
  {
    const std::filesystem::path empty_directory = g_scratch / "empty_stage_run";
    std::filesystem::remove_all(empty_directory, filesystem_error);
    RetainedStages retain_projected;
    retain_projected.projected_scan = true;
    const uint32_t empty_mask = PackRetainedStages(retain_projected);

    DumpWriter writer;
    const bool opened = writer.Open(empty_directory / "projected_scan.bin",
                                    DumpFormat::kBinary, RecordType::kProjectedScan,
                                    provenance, empty_mask, error);
    const bool closed = opened && writer.Close(error);
    Check(closed, "empty-stage run: a zero-record dump file is written", error);

    RunManifest empty_manifest;
    empty_manifest.provenance = provenance;
    empty_manifest.retained_stages_mask = empty_mask;
    empty_manifest.frames_seen = 4;
    empty_manifest.frames_recorded = 4;
    DumpFileEntry entry;
    entry.record_type = RecordType::kProjectedScan;
    entry.path = "projected_scan.bin";
    entry.format = DumpFormat::kBinary;
    entry.record_count = 0;
    empty_manifest.files.push_back(entry);
    WriteFile(empty_directory / kResolvedConfigFileName, config_text);
    empty_manifest.provenance.config_hash = HashConfigText(config_text);
    empty_manifest.Write(empty_directory / kRunManifestFileName, error);

    ReplaySummary empty_summary;
    const bool empty_loaded = LoadReplaySummary(empty_directory, empty_summary, error);
    Check(empty_loaded, "empty-stage run: replay loads", error);
    if (empty_loaded) {
      const StageAvailability* projected = nullptr;
      for (const StageAvailability& stage : empty_summary.stages) {
        if (stage.record_type == RecordType::kProjectedScan) projected = &stage;
      }
      Check(projected != nullptr &&
                projected->verdict == StageAvailability::Verdict::kRecordedEmpty,
            "replay: a retained, present, zero-record stage reads as RECORDED, EMPTY");
      Check(empty_summary.truncated_files.empty(),
            "replay: a legitimately empty dump is not reported as truncated");
    }
  }

  // And the inconsistent case: the header claims a stage was retained but no file exists.
  {
    const std::filesystem::path bad_directory = g_scratch / "inconsistent_run";
    std::filesystem::remove_all(bad_directory, filesystem_error);
    RetainedStages claims_tracks;
    claims_tracks.tracks = true;
    RunManifest lying;
    lying.provenance = provenance;
    lying.retained_stages_mask = PackRetainedStages(claims_tracks);
    WriteFile(bad_directory / kResolvedConfigFileName, config_text);
    lying.provenance.config_hash = HashConfigText(config_text);
    lying.Write(bad_directory / kRunManifestFileName, error);

    ReplaySummary bad_summary;
    if (LoadReplaySummary(bad_directory, bad_summary, error)) {
      const StageAvailability* tracks_stage = nullptr;
      for (const StageAvailability& stage : bad_summary.stages) {
        if (stage.record_type == RecordType::kTrackState2D) tracks_stage = &stage;
      }
      Check(tracks_stage != nullptr &&
                tracks_stage->verdict == StageAvailability::Verdict::kInconsistent,
            "replay: retained-but-no-file reads as INCONSISTENT, not as empty");
    } else {
      Check(false, "inconsistent run: replay loads", error);
    }
  }

  // Single-file mode. Without a manifest there is no directory view, so the absence of a file
  // for some OTHER stage carries no information: reporting NOT RECORDED would claim knowledge
  // the input cannot support, and INCONSISTENT would be a false alarm that makes
  // perception_replay exit non-zero on a perfectly good file.
  {
    ReplaySummary single;
    const bool single_loaded =
        LoadReplaySummary(directory / "scan_stats.bin", single, error);
    Check(single_loaded, "replay: a single dump file loads without a manifest", error);
    if (single_loaded) {
      Check(!single.manifest_present, "replay: single-file mode reports no manifest");
      Check(single.scan_stats.size() == 3, "replay: single-file mode decodes the records");
      int unknown = 0;
      int inconsistent = 0;
      for (const StageAvailability& stage : single.stages) {
        if (stage.verdict == StageAvailability::Verdict::kUnknownNoDirectoryView) ++unknown;
        if (stage.verdict == StageAvailability::Verdict::kInconsistent) ++inconsistent;
      }
      Check(unknown == kRetainedStageBitCount && inconsistent == 0,
            "replay: single-file mode reports every stage as UNKNOWN, not as absent or "
            "inconsistent (unknown=" + std::to_string(unknown) + ", inconsistent=" +
                std::to_string(inconsistent) + ")");
    }
  }

  // A manifest whose declared count disagrees with the file's own footer must be flagged:
  // that means the manifest and the file came from different runs.
  {
    RunManifest mismatched = manifest;
    mismatched.files[0].record_count = 99;
    Check(mismatched.Write(manifest_path, error), "mismatch run: manifest rewritten", error);
    ReplaySummary mismatch_summary;
    if (LoadReplaySummary(directory, mismatch_summary, error)) {
      Check(!mismatch_summary.truncated_files.empty(),
            "replay: a manifest/footer count disagreement is reported as a problem");
    } else {
      Check(false, "mismatch run: replay loads", error);
    }
  }
}

void RecordTypeInventory() {
  Section("record-type inventory");

  // Every id in [1, kRecordTypeCount) must have a name, and every name must map back to its
  // id. A gap here means a record type can be written but never identified on read.
  bool names_complete = true;
  bool round_trips = true;
  for (uint32_t value = 1; value < kRecordTypeCount; ++value) {
    const RecordType type = static_cast<RecordType>(value);
    const std::string name = ToString(type);
    if (name == "unknown") names_complete = false;
    if (RecordTypeFromString(name) != type) round_trips = false;
  }
  Check(names_complete, "every record type id has a name");
  Check(round_trips, "every record type name maps back to its id");
  Check(RecordTypeFromString("no_such_type") == RecordType::kUnknown,
        "an unrecognised record type name resolves to kUnknown, not to a guess");
  Check(!IsKnownRecordType(0) && !IsKnownRecordType(kRecordTypeCount),
        "record type range checking rejects 0 and out-of-range ids");

  // RetainedStages must survive the bitmask packing exactly, one bit at a time, or the
  // never-recorded/produced-nothing verdict is built on a lossy field.
  bool mask_exact = true;
  for (uint32_t bit = 0; bit < kRetainedStageBitCount; ++bit) {
    const RetainedStages unpacked = UnpackRetainedStages(1u << bit);
    if (PackRetainedStages(unpacked) != (1u << bit)) mask_exact = false;
    if (std::string(RetainedStageBitName(bit)) == "unknown") mask_exact = false;
    if (RetainedStageBitRecordType(bit) == RecordType::kUnknown) mask_exact = false;
  }
  Check(mask_exact, "RetainedStages survives pack/unpack for every single bit");

  RetainedStages all;
  all.raw_cloud = all.deskewed_cloud = all.gravity_aligned_cloud = all.labeled_cloud =
      all.projected_scan = all.clusters = all.primitives = all.observations = all.tracks = true;
  Check(PackRetainedStages(all) == kRetainedStageMask,
        "all nine RetainedStages bits pack to the full mask");

  // The two versions must not accidentally be the same constant with two names.
  Check(kDumpSchemaVersion == 1 && kManifestVersion == 1,
        "the dump and manifest versions are at their expected values");

  // The documented asymmetry between the formats: Close() back-patches the BINARY header, so a
  // binary dump's range is readable without walking to the footer. A JSONL header line cannot
  // be rewritten in place, so its range comes from the footer and Peek reports it as unset.
  // Asserted rather than only described, because a consumer that peeked a JSONL header and
  // trusted a zeroed range would silently report an empty run.
  {
    DumpFileHeader binary_header;
    DumpFileHeader jsonl_header;
    DumpFormat format = DumpFormat::kBinary;
    std::string error;
    const bool binary_peeked =
        DumpReader::PeekHeader(g_scratch / "scan_stats.bin", binary_header, format, error);
    Check(binary_peeked && format == DumpFormat::kBinary,
          "peek: a binary dump's format is detected from its magic, not its name", error);
    Check(binary_peeked && binary_header.range_valid && binary_header.record_count == 3,
          "peek: a binary header carries the back-patched record count without a footer walk");
    const bool jsonl_peeked =
        DumpReader::PeekHeader(g_scratch / "scan_stats.jsonl", jsonl_header, format, error);
    Check(jsonl_peeked && format == DumpFormat::kJsonl,
          "peek: a JSONL dump's format is detected", error);
    Check(jsonl_peeked && !jsonl_header.range_valid,
          "peek: a JSONL header reports range_valid=false rather than a zeroed range");
    Check(binary_peeked && jsonl_peeked &&
              binary_header.retained_stages_mask == jsonl_header.retained_stages_mask &&
              ProvenanceEqual(binary_header.provenance, jsonl_header.provenance),
          "peek: both formats carry the identical RetainedStages mask and provenance");
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::printf("usage: perception_dump_roundtrip_test <scratch-dir>\n");
    return 2;
  }
  g_scratch = argv[1];
  std::error_code filesystem_error;
  std::filesystem::remove_all(g_scratch, filesystem_error);
  std::filesystem::create_directories(g_scratch, filesystem_error);
  if (filesystem_error) {
    std::printf("FATAL: could not create the scratch directory %s: %s\n",
                g_scratch.string().c_str(), filesystem_error.message().c_str());
    return 1;
  }

  Section("per-contract round trip (exact ints, bit-exact floats, both formats)");

  // All 24 record types. The list is the phase's own inventory: every contract in
  // architecture doc section 7 gets a working writer AND reader, whether or not any stage
  // produces it yet.
  RoundTrip<RayPattern>("ray_pattern", perception_fixtures::MakeRayPattern, 0);
  RoundTrip<RawTimedPoint>("raw_timed_point", perception_fixtures::MakeRawTimedPoint, 0);
  RoundTrip<TimedPointCloud>("timed_point_cloud", perception_fixtures::MakeTimedPointCloud,
                             kRetainRawCloud);
  RoundTrip<ScanStats>("scan_stats", perception_fixtures::MakeScanStats, 0);
  RoundTrip<FrameTransformSnapshot>("frame_transform_snapshot",
                                    perception_fixtures::MakeFrameTransformSnapshot, 0);
  RoundTrip<DeskewedPointCloud>("deskewed_point_cloud",
                                perception_fixtures::MakeDeskewedPointCloud,
                                kRetainDeskewedCloud);
  RoundTrip<GravityAlignedCloud>("gravity_aligned_cloud",
                                 perception_fixtures::MakeGravityAlignedCloud,
                                 kRetainGravityAlignedCloud);
  RoundTrip<GroundPlane>("ground_plane", perception_fixtures::MakeGroundPlane, 0);
  RoundTrip<GroundSegmentationResult>("ground_segmentation_result",
                                      perception_fixtures::MakeGroundSegmentationResult, 0);
  RoundTrip<LabeledPointCloud>("labeled_point_cloud",
                               perception_fixtures::MakeLabeledPointCloud, kRetainLabeledCloud);
  RoundTrip<ProjectionStats>("projection_stats", perception_fixtures::MakeProjectionStats, 0);
  RoundTrip<ProjectedScan>("projected_scan", perception_fixtures::MakeProjectedScan,
                           kRetainProjectedScan);
  RoundTrip<Cluster2D>("cluster_2d", perception_fixtures::MakeCluster2D, kRetainClusters);
  RoundTrip<FittedPrimitive2D>("fitted_primitive_2d",
                               perception_fixtures::MakeFittedPrimitive2D, kRetainPrimitives);
  RoundTrip<CircleObservation>("circle_observation",
                               perception_fixtures::MakeCircleObservation,
                               kRetainObservations);
  RoundTrip<TrackState2D>("track_state_2d", perception_fixtures::MakeTrackState2D,
                          kRetainTracks);
  RoundTrip<TrackPrediction2D>("track_prediction_2d",
                               perception_fixtures::MakeTrackPrediction2D, 0);
  RoundTrip<PerceptionObstacle>("perception_obstacle",
                                perception_fixtures::MakePerceptionObstacle, 0);
  RoundTrip<SafetyObstacle>("safety_obstacle", perception_fixtures::MakeSafetyObstacle, 0);
  RoundTrip<OracleObstacleState>("oracle_obstacle_state",
                                 perception_fixtures::MakeOracleObstacleState, 0);
  RoundTrip<StageTiming>("stage_timing", perception_fixtures::MakeStageTiming, 0);
  RoundTrip<MatchedPairError>("matched_pair_error", perception_fixtures::MakeMatchedPairError,
                              0);
  RoundTrip<EstimationDiagnostics>("estimation_diagnostics",
                                   perception_fixtures::MakeEstimationDiagnostics, 0);
  RoundTrip<PerceptionFrame>("perception_frame", perception_fixtures::MakePerceptionFrame,
                             kRetainedStageMask);

  PathologicalFloats();
  SchemaRejection();
  ManifestAndReplay();
  RecordTypeInventory();

  return perception_test::Report("perception_dump_roundtrip_test");
}
