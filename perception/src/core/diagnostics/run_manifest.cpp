#include "perception/core/diagnostics/run_manifest.h"

#include <fstream>
#include <sstream>

#include "perception/core/diagnostics/json_value.h"

namespace perception::core::diagnostics {

const DumpFileEntry* RunManifest::FindFile(RecordType type) const {
  for (const DumpFileEntry& entry : files) {
    if (entry.record_type == type) return &entry;
  }
  return nullptr;
}

bool RunManifest::Write(const std::filesystem::path& path, std::string& error) const {
  std::error_code filesystem_error;
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path(), filesystem_error);
    if (filesystem_error) {
      error = "could not create the manifest directory: " + filesystem_error.message();
      return false;
    }
  }

  JsonTextWriter writer(/*pretty=*/true);
  writer.BeginObject();

  writer.Key("manifest_version");
  writer.Uint64(manifest_version);

  // The two schema versions side by side, at the top, so nobody has to be told twice that
  // they are different numbers.
  WriteProvenanceJson(writer, provenance);

  writer.Key("retained_stages_mask");
  writer.Uint64(retained_stages_mask);
  writer.Key("retained_stages");
  writer.BeginObject();
  for (uint32_t bit = 0; bit < kRetainedStageBitCount; ++bit) {
    writer.Key(RetainedStageBitName(bit));
    writer.Bool((retained_stages_mask & (1u << bit)) != 0);
  }
  writer.EndObject();

  writer.Key("dumps_format");
  writer.String(dumps_format);
  writer.Key("retain_stage_clouds");
  writer.Bool(retain_stage_clouds);
  writer.Key("decimation");
  writer.Int64(decimation);
  writer.Key("max_frames");
  writer.Int64(max_frames);

  writer.Key("frames_seen");
  writer.Uint64(frames_seen);
  writer.Key("frames_recorded");
  writer.Uint64(frames_recorded);

  writer.Key("resolved_config_path");
  writer.String(resolved_config_path);

  writer.Key("files");
  writer.BeginArray();
  for (const DumpFileEntry& entry : files) {
    writer.BeginObject();
    writer.Key("record_type");
    writer.String(ToString(entry.record_type));
    writer.Key("record_type_id");
    writer.Uint64(static_cast<uint64_t>(entry.record_type));
    writer.Key("path");
    writer.String(entry.path);
    writer.Key("format");
    writer.String(ToString(entry.format));
    writer.Key("record_count");
    writer.Uint64(entry.record_count);
    writer.Key("sequence_first");
    writer.Uint64(entry.sequence_first);
    writer.Key("sequence_last");
    writer.Uint64(entry.sequence_last);
    writer.Key("stamp_first_s");
    writer.Double(entry.stamp_first_s);
    writer.Key("stamp_last_s");
    writer.Double(entry.stamp_last_s);
    writer.EndObject();
  }
  writer.EndArray();

  writer.EndObject();

  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  if (!stream) {
    error = "could not open the manifest for writing: " + path.string();
    return false;
  }
  stream << writer.text() << '\n';
  stream.flush();
  if (!stream) {
    error = "failed to write the manifest: " + path.string();
    return false;
  }
  return true;
}

bool RunManifest::Read(const std::filesystem::path& path, RunManifest& out,
                       std::string& error) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    error = "could not open the manifest: " + path.string();
    return false;
  }
  std::ostringstream contents;
  contents << stream.rdbuf();

  JsonValue root;
  if (!JsonValue::Parse(contents.str(), root, error)) {
    error = "the manifest is not valid JSON: " + error;
    return false;
  }
  if (!root.is_object()) {
    error = "the manifest must be a JSON object";
    return false;
  }

  out = RunManifest{};

  const JsonValue* version = root.Find("manifest_version");
  if (version == nullptr || !version->is_number()) {
    error = "the manifest has no manifest_version";
    return false;
  }
  out.manifest_version = static_cast<uint32_t>(version->AsUint64());
  if (out.manifest_version != kManifestVersion) {
    error = "manifest version mismatch: file declares " +
            std::to_string(out.manifest_version) + ", this build reads " +
            std::to_string(kManifestVersion);
    return false;
  }

  if (!ReadProvenanceJson(root, out.provenance, error)) return false;
  if (out.provenance.dump_schema_version != kDumpSchemaVersion) {
    error = "manifest declares dump schema version " +
            std::to_string(out.provenance.dump_schema_version) + ", this build reads " +
            std::to_string(kDumpSchemaVersion);
    return false;
  }

  const JsonValue* mask = root.Find("retained_stages_mask");
  if (mask == nullptr || !mask->is_number()) {
    error = "the manifest has no retained_stages_mask";
    return false;
  }
  out.retained_stages_mask = static_cast<uint32_t>(mask->AsUint64());
  if ((out.retained_stages_mask & ~kRetainedStageMask) != 0) {
    error = "the manifest's retained_stages_mask has reserved bits set";
    return false;
  }

  if (const JsonValue* value = root.Find("dumps_format");
      value != nullptr && value->is_string()) {
    out.dumps_format = value->text();
  }
  if (const JsonValue* value = root.Find("retain_stage_clouds");
      value != nullptr && value->is_bool()) {
    out.retain_stage_clouds = value->boolean();
  }
  if (const JsonValue* value = root.Find("decimation");
      value != nullptr && value->is_number()) {
    out.decimation = static_cast<int32_t>(value->AsInt64());
  }
  if (const JsonValue* value = root.Find("max_frames");
      value != nullptr && value->is_number()) {
    out.max_frames = static_cast<int32_t>(value->AsInt64());
  }
  if (const JsonValue* value = root.Find("frames_seen");
      value != nullptr && value->is_number()) {
    out.frames_seen = value->AsUint64();
  }
  if (const JsonValue* value = root.Find("frames_recorded");
      value != nullptr && value->is_number()) {
    out.frames_recorded = value->AsUint64();
  }
  if (const JsonValue* value = root.Find("resolved_config_path");
      value != nullptr && value->is_string()) {
    out.resolved_config_path = value->text();
  }

  const JsonValue* files = root.Find("files");
  if (files == nullptr || !files->is_array()) {
    error = "the manifest has no files array";
    return false;
  }
  for (std::size_t index = 0; index < files->size(); ++index) {
    const JsonValue& node = files->at(index);
    if (!node.is_object()) {
      error = "manifest files[] entries must be objects";
      return false;
    }
    DumpFileEntry entry;

    const JsonValue* type_id = node.Find("record_type_id");
    if (type_id == nullptr || !type_id->is_number()) {
      error = "a manifest file entry has no record_type_id";
      return false;
    }
    const uint64_t raw_type = type_id->AsUint64();
    if (!IsKnownRecordType(static_cast<uint32_t>(raw_type))) {
      error = "a manifest file entry names record type id " + std::to_string(raw_type) +
              ", which this build does not know";
      return false;
    }
    entry.record_type = static_cast<RecordType>(raw_type);

    const JsonValue* entry_path = node.Find("path");
    if (entry_path == nullptr || !entry_path->is_string()) {
      error = "a manifest file entry has no path";
      return false;
    }
    entry.path = entry_path->text();

    const JsonValue* format = node.Find("format");
    if (format == nullptr || !format->is_string()) {
      error = "a manifest file entry has no format";
      return false;
    }
    if (format->text() == "binary") {
      entry.format = DumpFormat::kBinary;
    } else if (format->text() == "jsonl") {
      entry.format = DumpFormat::kJsonl;
    } else {
      error = "a manifest file entry has an unknown format: " + format->text();
      return false;
    }

    struct Numeric {
      const char* name;
      uint64_t* integer;
      double* real;
    };
    const Numeric numerics[] = {
        {"record_count", &entry.record_count, nullptr},
        {"sequence_first", &entry.sequence_first, nullptr},
        {"sequence_last", &entry.sequence_last, nullptr},
        {"stamp_first_s", nullptr, &entry.stamp_first_s},
        {"stamp_last_s", nullptr, &entry.stamp_last_s},
    };
    for (const Numeric& field : numerics) {
      const JsonValue* value = node.Find(field.name);
      if (value == nullptr || !value->is_number()) {
        error = std::string("a manifest file entry is missing '") + field.name + "'";
        return false;
      }
      if (field.integer != nullptr) {
        *field.integer = value->AsUint64();
      } else {
        *field.real = value->AsDouble();
      }
    }
    out.files.push_back(entry);
  }
  return true;
}

}  // namespace perception::core::diagnostics
