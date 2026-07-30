// The JSONL codec: two more visitors over the same contract_fields.h field lists.
//
// ROLE. JSONL is the human-readable sibling of the binary format - one record per line, so
// `grep`, `head` and `wc -l` all work on a dump, and so a reviewer can read a frame without
// a tool. It is a full peer of the binary format, not a lossy preview: it round-trips
// bit-exactly (see json_value.h on digit widths and on the NaN/Infinity tokens), and the
// replay app reads either. That matters because `dumps.format: "jsonl"` is the shipped
// default in perception.yaml, so a write-only JSONL would have made the DEFAULT dump format
// unreplayable.
//
// ENUMS TRAVEL AS NUMBERS. Their symbolic names live in the contracts' ToString() functions
// and are rendered by the replay report, not stored. Storing names would need a
// name-to-enum table per enum whose only user is the reader, and a typo in it would be a
// silent misparse of exactly the kind this phase exists to prevent. The numbers are still
// range-checked on read.
//
// STRICT ON READ. A missing field is an error, not a default. A dump is a fixture; a fixture
// that silently defaults a field it did not contain is how a regression test starts passing
// for the wrong reason.
#ifndef PERCEPTION_CORE_DIAGNOSTICS_JSON_CODEC_H_
#define PERCEPTION_CORE_DIAGNOSTICS_JSON_CODEC_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "perception/core/diagnostics/contract_fields.h"
#include "perception/core/diagnostics/json_value.h"

namespace perception::core::diagnostics {

// ---------------------------------------------------------------------------------------
// Write
// ---------------------------------------------------------------------------------------
class JsonWriteVisitor {
 public:
  // `element_mode` true means "the value being written IS an array element", so no key is
  // emitted. It is how one visitor serves both members and elements.
  explicit JsonWriteVisitor(JsonTextWriter& writer, bool element_mode = false)
      : writer_(writer), element_mode_(element_mode) {}

  template <class T>
  void Prim(const char* name, T& value) {
    MaybeKey(name);
    if constexpr (std::is_same_v<T, bool>) {
      writer_.Bool(value);
    } else if constexpr (std::is_same_v<T, float>) {
      writer_.Float(value);
    } else if constexpr (std::is_same_v<T, double>) {
      writer_.Double(value);
    } else if constexpr (std::is_same_v<T, std::string>) {
      writer_.String(value);
    } else if constexpr (std::is_integral_v<T>) {
      if constexpr (std::is_signed_v<T>) {
        writer_.Int64(static_cast<int64_t>(value));
      } else {
        writer_.Uint64(static_cast<uint64_t>(value));
      }
    } else {
      static_assert(sizeof(T) == 0, "JsonWriteVisitor::Prim: unsupported field type");
    }
  }

  template <class E>
  void Enm(const char* name, E& value) {
    static_assert(std::is_enum_v<E>, "Enm requires an enum");
    MaybeKey(name);
    writer_.Uint64(static_cast<uint64_t>(value));
  }

  void Eig(const char* name, Eigen::Vector2d& value) {
    WriteDoubles(name, value.data(), 2);
  }

  void Eig(const char* name, Eigen::Vector3d& value) {
    WriteDoubles(name, value.data(), 3);
  }

  void Eig(const char* name, Eigen::Vector4d& value) {
    WriteDoubles(name, value.data(), 4);
  }

  void Eig(const char* name, Eigen::Vector3f& value) {
    MaybeKey(name);
    writer_.BeginArray();
    for (int index = 0; index < 3; ++index) writer_.Float(value[index]);
    writer_.EndArray();
  }

  // Row-major, to match the binary codec and to read the way the matrix is written on paper.
  void Eig(const char* name, Eigen::Matrix3d& value) {
    MaybeKey(name);
    writer_.BeginArray();
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) writer_.Double(value(row, column));
    }
    writer_.EndArray();
  }

  // 12 numbers: the 3x3 linear block row-major, then the translation. Same as binary.
  void Eig(const char* name, Eigen::Isometry3d& value) {
    MaybeKey(name);
    writer_.BeginArray();
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) writer_.Double(value.linear()(row, column));
    }
    for (int row = 0; row < 3; ++row) writer_.Double(value.translation()[row]);
    writer_.EndArray();
  }

  template <class S>
  void Sub(const char* name, S& value) {
    MaybeKey(name);
    writer_.BeginObject();
    JsonWriteVisitor nested(writer_, /*element_mode=*/false);
    VisitFields(nested, value);
    writer_.EndObject();
  }

  template <class T>
  void Arr(const char* name, std::vector<T>& value) {
    MaybeKey(name);
    writer_.BeginArray();
    JsonWriteVisitor element(writer_, /*element_mode=*/true);
    for (T& item : value) VisitArrayElement(element, name, item);
    writer_.EndArray();
  }

  template <std::size_t N>
  void FixedArr(const char* name, std::array<double, N>& value) {
    MaybeKey(name);
    writer_.BeginArray();
    for (double item : value) writer_.Double(item);
    writer_.EndArray();
  }

 private:
  void MaybeKey(const char* name) {
    if (!element_mode_) writer_.Key(name);
  }

  void WriteDoubles(const char* name, const double* data, int count) {
    MaybeKey(name);
    writer_.BeginArray();
    for (int index = 0; index < count; ++index) writer_.Double(data[index]);
    writer_.EndArray();
  }

  JsonTextWriter& writer_;
  bool element_mode_ = false;
};

// ---------------------------------------------------------------------------------------
// Read
// ---------------------------------------------------------------------------------------

// Shared failure state. The visitor interface has void returns, so the status has to be
// out-of-band; it is fail-once, like ByteReader, for the same reason.
struct JsonReadStatus {
  bool ok = true;
  std::string error;

  void Fail(const std::string& reason) {
    if (ok) {
      ok = false;
      error = reason;
    }
  }
};

class JsonReadVisitor {
 public:
  JsonReadVisitor(const JsonValue* node, JsonReadStatus& status, bool element_mode = false)
      : node_(node), status_(status), element_mode_(element_mode) {}

  template <class T>
  void Prim(const char* name, T& value) {
    const JsonValue* field = Member(name);
    if (field == nullptr) return;
    if constexpr (std::is_same_v<T, bool>) {
      if (!field->is_bool()) return Reject(name, "boolean");
      value = field->boolean();
    } else if constexpr (std::is_same_v<T, float>) {
      if (!field->is_number()) return Reject(name, "number");
      value = field->AsFloat();
    } else if constexpr (std::is_same_v<T, double>) {
      if (!field->is_number()) return Reject(name, "number");
      value = field->AsDouble();
    } else if constexpr (std::is_same_v<T, std::string>) {
      if (!field->is_string()) return Reject(name, "string");
      value = field->text();
    } else if constexpr (std::is_integral_v<T>) {
      if (!field->is_number()) return Reject(name, "number");
      if constexpr (std::is_signed_v<T>) {
        value = static_cast<T>(field->AsInt64());
      } else {
        value = static_cast<T>(field->AsUint64());
      }
    } else {
      static_assert(sizeof(T) == 0, "JsonReadVisitor::Prim: unsupported field type");
    }
  }

  template <class E>
  void Enm(const char* name, E& value) {
    static_assert(std::is_enum_v<E>, "Enm requires an enum");
    const JsonValue* field = Member(name);
    if (field == nullptr) return;
    if (!field->is_number()) return Reject(name, "number");
    const uint64_t raw = field->AsUint64();
    if (raw > EnumUpperBound<E>()) {
      status_.Fail(std::string("enum field '") + name + "' is out of range");
      return;
    }
    value = static_cast<E>(raw);
  }

  void Eig(const char* name, Eigen::Vector2d& value) { ReadDoubles(name, value.data(), 2); }
  void Eig(const char* name, Eigen::Vector3d& value) { ReadDoubles(name, value.data(), 3); }
  void Eig(const char* name, Eigen::Vector4d& value) { ReadDoubles(name, value.data(), 4); }

  void Eig(const char* name, Eigen::Vector3f& value) {
    const JsonValue* field = Array(name, 3);
    if (field == nullptr) return;
    for (int index = 0; index < 3; ++index) value[index] = field->at(index).AsFloat();
  }

  void Eig(const char* name, Eigen::Matrix3d& value) {
    const JsonValue* field = Array(name, 9);
    if (field == nullptr) return;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        value(row, column) = field->at(static_cast<std::size_t>(row * 3 + column)).AsDouble();
      }
    }
  }

  void Eig(const char* name, Eigen::Isometry3d& value) {
    const JsonValue* field = Array(name, 12);
    if (field == nullptr) return;
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        rotation(row, column) = field->at(static_cast<std::size_t>(row * 3 + column)).AsDouble();
      }
    }
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    for (int row = 0; row < 3; ++row) {
      translation[row] = field->at(static_cast<std::size_t>(9 + row)).AsDouble();
    }
    value.setIdentity();
    value.linear() = rotation;
    value.translation() = translation;
  }

  template <class S>
  void Sub(const char* name, S& value) {
    const JsonValue* field = Member(name);
    if (field == nullptr) return;
    if (!field->is_object()) return Reject(name, "object");
    JsonReadVisitor nested(field, status_, /*element_mode=*/false);
    VisitFields(nested, value);
  }

  template <class T>
  void Arr(const char* name, std::vector<T>& value) {
    const JsonValue* field = Member(name);
    if (field == nullptr) return;
    if (!field->is_array()) return Reject(name, "array");
    value.clear();
    value.resize(field->size());
    for (std::size_t index = 0; index < field->size(); ++index) {
      JsonReadVisitor element(&field->at(index), status_, /*element_mode=*/true);
      VisitArrayElement(element, name, value[index]);
      if (!status_.ok) return;
    }
  }

  template <std::size_t N>
  void FixedArr(const char* name, std::array<double, N>& value) {
    const JsonValue* field = Array(name, static_cast<int>(N));
    if (field == nullptr) return;
    for (std::size_t index = 0; index < N; ++index) {
      value[index] = field->at(index).AsDouble();
    }
  }

 private:
  const JsonValue* Member(const char* name) {
    if (!status_.ok) return nullptr;
    if (node_ == nullptr) {
      status_.Fail("internal: JSON read visitor has no node");
      return nullptr;
    }
    // In element mode the node IS the value; there is no member to look up.
    if (element_mode_) return node_;
    if (!node_->is_object()) {
      status_.Fail(std::string("expected an object while reading field '") + name + "'");
      return nullptr;
    }
    const JsonValue* field = node_->Find(name);
    if (field == nullptr) {
      status_.Fail(std::string("missing required field '") + name + "'");
    }
    return field;
  }

  const JsonValue* Array(const char* name, int expected_size) {
    const JsonValue* field = Member(name);
    if (field == nullptr) return nullptr;
    if (!field->is_array()) {
      Reject(name, "array");
      return nullptr;
    }
    if (field->size() != static_cast<std::size_t>(expected_size)) {
      status_.Fail(std::string("field '") + name + "' has the wrong array length");
      return nullptr;
    }
    return field;
  }

  void Reject(const char* name, const char* expected) {
    status_.Fail(std::string("field '") + name + "' is not a " + expected);
  }

  // Same table as the binary reader's, and it must stay in step with it. Both are here
  // rather than in the contracts on purpose: the range of an enum ON DISK is a property of
  // the format, and the format is what versions.
  template <class E>
  static uint64_t EnumUpperBound() {
    if constexpr (std::is_same_v<E, FrameId>) {
      return static_cast<uint64_t>(FrameId::kWorld);
    } else if constexpr (std::is_same_v<E, MotionCompensation>) {
      return static_cast<uint64_t>(MotionCompensation::kDeskewedToStamp);
    } else if constexpr (std::is_same_v<E, PointLabel>) {
      return static_cast<uint64_t>(PointLabel::kSelf);
    } else if constexpr (std::is_same_v<E, PrimitiveKind>) {
      return static_cast<uint64_t>(PrimitiveKind::kCircle);
    } else if constexpr (std::is_same_v<E, TrackStatus>) {
      return static_cast<uint64_t>(TrackStatus::kCoasting);
    } else if constexpr (std::is_same_v<E, ObstacleSource>) {
      return static_cast<uint64_t>(ObstacleSource::kOracle);
    } else if constexpr (std::is_same_v<E, FrameInvalidReason>) {
      return static_cast<uint64_t>(FrameInvalidReason::kStale);
    } else {
      static_assert(sizeof(E) == 0,
                    "JsonReadVisitor: this enum has no declared upper bound - add it");
      return 0;
    }
  }

  void ReadDoubles(const char* name, double* data, int count) {
    const JsonValue* field = Array(name, count);
    if (field == nullptr) return;
    for (int index = 0; index < count; ++index) {
      data[index] = field->at(static_cast<std::size_t>(index)).AsDouble();
    }
  }

  const JsonValue* node_ = nullptr;
  JsonReadStatus& status_;
  bool element_mode_ = false;
};

// ---------------------------------------------------------------------------------------
// Record-level entry points. See binary_codec.h on the const_cast.
// ---------------------------------------------------------------------------------------
template <class T>
void WriteJsonRecord(JsonTextWriter& writer, const T& record) {
  writer.BeginObject();
  JsonWriteVisitor visitor(writer, /*element_mode=*/false);
  VisitFields(visitor, const_cast<T&>(record));
  writer.EndObject();
}

template <class T>
bool ReadJsonRecord(const JsonValue& node, T& record, std::string& error) {
  if (!node.is_object()) {
    error = "a JSONL record must be a JSON object";
    return false;
  }
  JsonReadStatus status;
  JsonReadVisitor visitor(&node, status, /*element_mode=*/false);
  VisitFields(visitor, record);
  if (!status.ok) error = status.error;
  return status.ok;
}

}  // namespace perception::core::diagnostics

#endif  // PERCEPTION_CORE_DIAGNOSTICS_JSON_CODEC_H_
