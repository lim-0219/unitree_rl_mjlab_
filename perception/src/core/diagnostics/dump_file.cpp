#include "perception/core/diagnostics/dump_file.h"

#include <cstring>
#include <limits>
#include <sstream>

namespace perception::core::diagnostics {
namespace {

// Sentinel written into the header's range fields before Close() patches them, and left
// there permanently in JSONL. Distinguishable from a real count of 0.
constexpr uint64_t kUnknownCount = std::numeric_limits<uint64_t>::max();

bool ReadWholeFile(const std::filesystem::path& path, std::string& out, std::string& error) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    error = "could not open dump file: " + path.string();
    return false;
  }
  std::ostringstream contents;
  contents << stream.rdbuf();
  out = contents.str();
  return true;
}

void SplitLines(const std::string& text, std::vector<std::string>& out) {
  out.clear();
  std::size_t start = 0;
  while (start <= text.size()) {
    const std::size_t newline = text.find('\n', start);
    if (newline == std::string::npos) {
      if (start < text.size()) out.push_back(text.substr(start));
      break;
    }
    out.push_back(text.substr(start, newline - start));
    start = newline + 1;
  }
}

}  // namespace

// ---------------------------------------------------------------------------------------
// Provenance serialization. Note the deliberate absence of dump_schema_version: it lives in
// the file header, and storing it twice would create two sources of truth for the one number
// the reader uses to decide whether it can read the file at all.
// ---------------------------------------------------------------------------------------
void WriteProvenanceBinary(ByteWriter& writer, const RunProvenance& provenance) {
  writer.I32(provenance.config_schema_version);
  writer.Str(provenance.config_hash);
  writer.Str(provenance.config_source);
  writer.Str(provenance.git_sha);
  writer.Str(provenance.git_describe);
  writer.Bool(provenance.git_dirty);
  writer.Str(provenance.created_utc);
  writer.U64(provenance.seed);
  writer.Str(provenance.producer);
}

bool ReadProvenanceBinary(ByteReader& reader, RunProvenance& provenance) {
  provenance.config_schema_version = reader.I32();
  provenance.config_hash = reader.Str();
  provenance.config_source = reader.Str();
  provenance.git_sha = reader.Str();
  provenance.git_describe = reader.Str();
  provenance.git_dirty = reader.Bool();
  provenance.created_utc = reader.Str();
  provenance.seed = reader.U64();
  provenance.producer = reader.Str();
  return reader.ok();
}

void WriteProvenanceJson(JsonTextWriter& writer, const RunProvenance& provenance) {
  writer.Key("dump_schema_version");
  writer.Uint64(provenance.dump_schema_version);
  writer.Key("config_schema_version");
  writer.Int64(provenance.config_schema_version);
  writer.Key("config_hash");
  writer.String(provenance.config_hash);
  writer.Key("config_source");
  writer.String(provenance.config_source);
  writer.Key("git_sha");
  writer.String(provenance.git_sha);
  writer.Key("git_describe");
  writer.String(provenance.git_describe);
  writer.Key("git_dirty");
  writer.Bool(provenance.git_dirty);
  writer.Key("created_utc");
  writer.String(provenance.created_utc);
  writer.Key("seed");
  writer.Uint64(provenance.seed);
  writer.Key("producer");
  writer.String(provenance.producer);
}

bool ReadProvenanceJson(const JsonValue& node, RunProvenance& provenance, std::string& error) {
  struct Required {
    const char* name;
    JsonValue::Kind kind;
  };
  static const Required kRequired[] = {
      {"config_schema_version", JsonValue::Kind::kNumber},
      {"config_hash", JsonValue::Kind::kString},
      {"config_source", JsonValue::Kind::kString},
      {"git_sha", JsonValue::Kind::kString},
      {"git_describe", JsonValue::Kind::kString},
      {"git_dirty", JsonValue::Kind::kBool},
      {"created_utc", JsonValue::Kind::kString},
      {"seed", JsonValue::Kind::kNumber},
      {"producer", JsonValue::Kind::kString},
  };
  for (const Required& field : kRequired) {
    const JsonValue* value = node.Find(field.name);
    if (value == nullptr) {
      error = std::string("provenance is missing '") + field.name + "'";
      return false;
    }
    if (value->kind() != field.kind) {
      error = std::string("provenance field '") + field.name + "' has the wrong type";
      return false;
    }
  }
  provenance.config_schema_version =
      static_cast<int32_t>(node.Find("config_schema_version")->AsInt64());
  provenance.config_hash = node.Find("config_hash")->text();
  provenance.config_source = node.Find("config_source")->text();
  provenance.git_sha = node.Find("git_sha")->text();
  provenance.git_describe = node.Find("git_describe")->text();
  provenance.git_dirty = node.Find("git_dirty")->boolean();
  provenance.created_utc = node.Find("created_utc")->text();
  provenance.seed = node.Find("seed")->AsUint64();
  provenance.producer = node.Find("producer")->text();
  // dump_schema_version is optional here because the file header is authoritative; when
  // present (manifests write it) it is used, so a manifest read on its own is complete.
  if (const JsonValue* version = node.Find("dump_schema_version");
      version != nullptr && version->is_number()) {
    provenance.dump_schema_version = static_cast<uint32_t>(version->AsUint64());
  }
  return true;
}

// ---------------------------------------------------------------------------------------
// DumpWriter
// ---------------------------------------------------------------------------------------
DumpWriter::~DumpWriter() {
  // No implicit Close(): a file finalised by a destructor during stack unwinding would
  // claim to be a complete dump of a run that threw. Leaving it footerless is the signal.
  if (stream_.is_open()) stream_.close();
}

bool DumpWriter::Open(const std::filesystem::path& path, DumpFormat format,
                      RecordType record_type, const RunProvenance& provenance,
                      uint32_t retained_mask, std::string& error) {
  if (open_) {
    error = "DumpWriter::Open called twice";
    return false;
  }
  if (!IsKnownRecordType(static_cast<uint32_t>(record_type))) {
    error = "DumpWriter::Open with an unknown record type";
    return false;
  }
  if ((retained_mask & ~kRetainedStageMask) != 0) {
    error = "DumpWriter::Open with reserved bits set in the retained-stages mask";
    return false;
  }

  std::error_code filesystem_error;
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path(), filesystem_error);
    if (filesystem_error) {
      error = "could not create dump directory " + path.parent_path().string() + ": " +
              filesystem_error.message();
      return false;
    }
  }

  stream_.open(path, std::ios::binary | std::ios::trunc);
  if (!stream_) {
    error = "could not open dump file for writing: " + path.string();
    return false;
  }

  path_ = path;
  format_ = format;
  record_type_ = record_type;
  retained_mask_ = retained_mask;
  footer_ = DumpFileFooter{};
  open_ = true;

  if (format_ == DumpFormat::kBinary) {
    ByteWriter header;
    header.Raw(kDumpMagic, kDumpMagicSize);
    header.U32(kDumpSchemaVersion);
    header.U32(static_cast<uint32_t>(record_type));
    header.U32(retained_mask);
    // The patchable block. Its size must match kBinaryPatchSize exactly, so the assertion
    // below is a real guard rather than documentation: a field added here without updating
    // the constant would patch over the start of the provenance strings.
    const std::size_t patch_start = header.size();
    header.Bool(false);          // range_valid
    header.U64(kUnknownCount);   // record_count
    header.U64(0);               // sequence_first
    header.U64(0);               // sequence_last
    header.F64(0.0);             // stamp_first_s
    header.F64(0.0);             // stamp_last_s
    if (patch_start != kBinaryPatchOffset || header.size() - patch_start != kBinaryPatchSize) {
      error = "internal: binary dump header patch geometry does not match the constants";
      stream_.close();
      open_ = false;
      return false;
    }
    WriteProvenanceBinary(header, provenance);
    stream_.write(header.buffer().data(), static_cast<std::streamsize>(header.size()));
  } else {
    JsonTextWriter header(/*pretty=*/false);
    header.BeginObject();
    header.Key(kJsonlHeaderKey);
    header.BeginObject();
    header.Key("magic");
    header.String(std::string(kDumpMagic, kDumpMagicSize - 1));  // Printable prefix only.
    header.Key("record_type");
    header.String(ToString(record_type));
    header.Key("record_type_id");
    header.Uint64(static_cast<uint64_t>(record_type));
    header.Key("retained_stages_mask");
    header.Uint64(retained_mask);
    header.Key("retained_stages");
    header.BeginObject();
    for (uint32_t bit = 0; bit < kRetainedStageBitCount; ++bit) {
      header.Key(RetainedStageBitName(bit));
      header.Bool((retained_mask & (1u << bit)) != 0);
    }
    header.EndObject();
    // Left unset on purpose: a JSONL header line cannot be back-patched at Close() without
    // fixed-width padding, so the closing facts live in the footer line and the reader
    // merges them. See DumpReader::header().
    header.Key("range_valid");
    header.Bool(false);
    header.Key("jsonl_number_note");
    header.String("non-finite values use the bare tokens NaN / Infinity / -Infinity");
    WriteProvenanceJson(header, provenance);
    header.EndObject();
    header.EndObject();
    stream_ << header.text() << '\n';
  }

  if (!stream_) {
    error = "could not write the dump header to " + path.string();
    stream_.close();
    open_ = false;
    return false;
  }
  return true;
}

void DumpWriter::NoteRecord(uint64_t sequence, double stamp_s) {
  if (footer_.record_count == 0) {
    footer_.sequence_first = sequence;
    footer_.stamp_first_s = stamp_s;
  }
  footer_.sequence_last = sequence;
  footer_.stamp_last_s = stamp_s;
  ++footer_.record_count;
}

bool DumpWriter::Close(std::string& error) {
  if (!open_) {
    error = "DumpWriter::Close called on a closed writer";
    return false;
  }

  if (format_ == DumpFormat::kBinary) {
    ByteWriter tail;
    tail.U32(kDumpFooterSentinel);
    tail.U64(footer_.record_count);
    tail.U64(footer_.sequence_first);
    tail.U64(footer_.sequence_last);
    tail.F64(footer_.stamp_first_s);
    tail.F64(footer_.stamp_last_s);
    stream_.write(tail.buffer().data(), static_cast<std::streamsize>(tail.size()));

    // Back-patch the header so a binary dump's header alone carries the range the phase
    // brief asks for, without a consumer having to seek to the end first.
    ByteWriter patch;
    patch.Bool(true);
    patch.U64(footer_.record_count);
    patch.U64(footer_.sequence_first);
    patch.U64(footer_.sequence_last);
    patch.F64(footer_.stamp_first_s);
    patch.F64(footer_.stamp_last_s);
    if (patch.size() != kBinaryPatchSize) {
      error = "internal: binary header patch block is the wrong size";
      stream_.close();
      open_ = false;
      return false;
    }
    stream_.seekp(static_cast<std::streamoff>(kBinaryPatchOffset), std::ios::beg);
    stream_.write(patch.buffer().data(), static_cast<std::streamsize>(patch.size()));
  } else {
    JsonTextWriter tail(/*pretty=*/false);
    tail.BeginObject();
    tail.Key(kJsonlFooterKey);
    tail.BeginObject();
    tail.Key("record_count");
    tail.Uint64(footer_.record_count);
    tail.Key("sequence_first");
    tail.Uint64(footer_.sequence_first);
    tail.Key("sequence_last");
    tail.Uint64(footer_.sequence_last);
    tail.Key("stamp_first_s");
    tail.Double(footer_.stamp_first_s);
    tail.Key("stamp_last_s");
    tail.Double(footer_.stamp_last_s);
    tail.EndObject();
    tail.EndObject();
    stream_ << tail.text() << '\n';
  }

  stream_.flush();
  const bool good = static_cast<bool>(stream_);
  stream_.close();
  open_ = false;
  if (!good) {
    error = "failed to finalise the dump file " + path_.string();
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------------------
// DumpReader
// ---------------------------------------------------------------------------------------
namespace {

// Decides the format from the first bytes rather than from the file name, so a renamed dump
// still reads and a .jsonl full of binary fails on the magic instead of on the JSON grammar.
bool DetectFormat(const std::string& contents, DumpFormat& format, std::string& error) {
  if (contents.size() >= static_cast<std::size_t>(kBinaryPatchOffset) &&
      std::memcmp(contents.data(), kDumpMagic, kDumpMagicSize) == 0) {
    format = DumpFormat::kBinary;
    return true;
  }
  if (!contents.empty() && contents[0] == '{') {
    format = DumpFormat::kJsonl;
    return true;
  }
  error = "not a perception dump: the file starts with neither the binary magic nor '{'";
  return false;
}

bool ParseBinaryHeader(const std::string& contents, DumpFileHeader& header,
                       std::size_t& body_offset, std::string& error) {
  ByteReader reader(contents);
  char magic[kDumpMagicSize] = {};
  if (!reader.Raw(magic, kDumpMagicSize) ||
      std::memcmp(magic, kDumpMagic, kDumpMagicSize) != 0) {
    error = "not a perception dump: bad magic";
    return false;
  }
  header.dump_schema_version = reader.U32();
  const uint32_t record_type_id = reader.U32();
  header.retained_stages_mask = reader.U32();
  header.range_valid = reader.Bool();
  header.record_count = reader.U64();
  header.sequence_first = reader.U64();
  header.sequence_last = reader.U64();
  header.stamp_first_s = reader.F64();
  header.stamp_last_s = reader.F64();
  if (!reader.ok()) {
    error = "dump header is truncated: " + reader.error();
    return false;
  }

  // Version check BEFORE anything version-dependent is interpreted. A mismatched schema is
  // refused outright rather than read optimistically: a field-for-field misparse of a
  // plausible-looking record is exactly the failure mode this check exists to prevent.
  if (header.dump_schema_version != kDumpSchemaVersion) {
    error = "dump schema version mismatch: file declares " +
            std::to_string(header.dump_schema_version) + ", this build reads " +
            std::to_string(kDumpSchemaVersion) +
            " (note: this is the DUMP schema version, not the config schema_version)";
    return false;
  }
  if (!IsKnownRecordType(record_type_id)) {
    error = "dump declares record type id " + std::to_string(record_type_id) +
            ", which this build does not know";
    return false;
  }
  header.record_type = static_cast<RecordType>(record_type_id);
  if ((header.retained_stages_mask & ~kRetainedStageMask) != 0) {
    error = "dump header has reserved bits set in the retained-stages mask";
    return false;
  }
  if (header.record_count == kUnknownCount) {
    header.record_count = 0;
    header.range_valid = false;
  }
  if (!ReadProvenanceBinary(reader, header.provenance)) {
    error = "dump provenance block is truncated: " + reader.error();
    return false;
  }
  header.provenance.dump_schema_version = header.dump_schema_version;
  body_offset = reader.position();
  return true;
}

bool ParseJsonlHeader(const std::string& line, DumpFileHeader& header, std::string& error) {
  JsonValue root;
  if (!JsonValue::Parse(line, root, error)) {
    error = "the JSONL header line is malformed: " + error;
    return false;
  }
  const JsonValue* node = root.Find(kJsonlHeaderKey);
  if (node == nullptr || !node->is_object()) {
    error = "the first JSONL line is not a dump header";
    return false;
  }
  const JsonValue* version = node->Find("dump_schema_version");
  if (version == nullptr || !version->is_number()) {
    error = "the JSONL header has no dump_schema_version";
    return false;
  }
  header.dump_schema_version = static_cast<uint32_t>(version->AsUint64());
  if (header.dump_schema_version != kDumpSchemaVersion) {
    error = "dump schema version mismatch: file declares " +
            std::to_string(header.dump_schema_version) + ", this build reads " +
            std::to_string(kDumpSchemaVersion) +
            " (note: this is the DUMP schema version, not the config schema_version)";
    return false;
  }
  const JsonValue* record_type_id = node->Find("record_type_id");
  if (record_type_id == nullptr || !record_type_id->is_number()) {
    error = "the JSONL header has no record_type_id";
    return false;
  }
  const uint64_t raw_type = record_type_id->AsUint64();
  if (!IsKnownRecordType(static_cast<uint32_t>(raw_type))) {
    error = "dump declares record type id " + std::to_string(raw_type) +
            ", which this build does not know";
    return false;
  }
  header.record_type = static_cast<RecordType>(raw_type);

  // Cross-check the human-readable name against the numeric id. They are written from one
  // source, so a disagreement means the file was hand-edited - worth refusing rather than
  // resolving silently in favour of either one.
  const JsonValue* record_type_name = node->Find("record_type");
  if (record_type_name != nullptr && record_type_name->is_string() &&
      record_type_name->text() != ToString(header.record_type)) {
    error = "the JSONL header's record_type name and record_type_id disagree";
    return false;
  }

  const JsonValue* mask = node->Find("retained_stages_mask");
  if (mask == nullptr || !mask->is_number()) {
    error = "the JSONL header has no retained_stages_mask";
    return false;
  }
  header.retained_stages_mask = static_cast<uint32_t>(mask->AsUint64());
  if ((header.retained_stages_mask & ~kRetainedStageMask) != 0) {
    error = "dump header has reserved bits set in the retained-stages mask";
    return false;
  }
  header.range_valid = false;
  header.record_count = 0;
  if (!ReadProvenanceJson(*node, header.provenance, error)) return false;
  header.provenance.dump_schema_version = header.dump_schema_version;
  return true;
}

}  // namespace

bool DumpReader::Open(const std::filesystem::path& path, std::string& error) {
  std::string contents;
  if (!ReadWholeFile(path, contents, error)) return false;
  if (!DetectFormat(contents, format_, error)) return false;

  path_ = path;
  header_ = DumpFileHeader{};
  footer_ = DumpFileFooter{};
  footer_present_ = false;
  position_ = 0;
  line_index_ = 0;
  lines_.clear();
  buffer_.clear();

  if (format_ == DumpFormat::kBinary) {
    std::size_t body_offset = 0;
    if (!ParseBinaryHeader(contents, header_, body_offset, error)) return false;
    buffer_ = std::move(contents);
    position_ = body_offset;
    return true;
  }

  SplitLines(contents, lines_);
  if (lines_.empty()) {
    error = "the dump file is empty";
    return false;
  }
  if (!ParseJsonlHeader(lines_[0], header_, error)) return false;
  line_index_ = 1;
  return true;
}

bool DumpReader::PeekHeader(const std::filesystem::path& path, DumpFileHeader& header,
                            DumpFormat& format, std::string& error) {
  DumpReader reader;
  if (!reader.Open(path, error)) return false;
  header = reader.header();
  format = reader.format();
  return true;
}

bool DumpReader::ReadFooterBinary(ByteReader& reader, std::string& error) {
  footer_.record_count = reader.U64();
  footer_.sequence_first = reader.U64();
  footer_.sequence_last = reader.U64();
  footer_.stamp_first_s = reader.F64();
  footer_.stamp_last_s = reader.F64();
  if (!reader.ok()) {
    error = "dump footer is truncated: " + reader.error();
    return false;
  }
  footer_present_ = true;
  MergeFooterIntoHeader();
  return true;
}

bool DumpReader::ReadFooterJson(const JsonValue& node, std::string& error) {
  struct Required {
    const char* name;
    uint64_t* integer;
    double* real;
  };
  const Required fields[] = {
      {"record_count", &footer_.record_count, nullptr},
      {"sequence_first", &footer_.sequence_first, nullptr},
      {"sequence_last", &footer_.sequence_last, nullptr},
      {"stamp_first_s", nullptr, &footer_.stamp_first_s},
      {"stamp_last_s", nullptr, &footer_.stamp_last_s},
  };
  for (const Required& field : fields) {
    const JsonValue* value = node.Find(field.name);
    if (value == nullptr || !value->is_number()) {
      error = std::string("the JSONL footer is missing '") + field.name + "'";
      return false;
    }
    if (field.integer != nullptr) {
      *field.integer = value->AsUint64();
    } else {
      *field.real = value->AsDouble();
    }
  }
  footer_present_ = true;
  MergeFooterIntoHeader();
  return true;
}

void DumpReader::MergeFooterIntoHeader() {
  // For binary these values are already in the header (back-patched at Close), and this is
  // a no-op that also serves as a consistency check opportunity; for JSONL it is how the
  // header gets its range at all.
  header_.record_count = footer_.record_count;
  header_.sequence_first = footer_.sequence_first;
  header_.sequence_last = footer_.sequence_last;
  header_.stamp_first_s = footer_.stamp_first_s;
  header_.stamp_last_s = footer_.stamp_last_s;
  header_.range_valid = true;
}

}  // namespace perception::core::diagnostics
