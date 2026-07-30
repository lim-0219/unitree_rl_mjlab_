// The binary codec: two visitors over the contract_fields.h field lists.
//
// Binary is the PRIMARY dump format. It is exact by construction (every float travels as
// its bit pattern), compact enough that retaining stage clouds is affordable, and its
// reader is the one the regression fixtures of later phases will depend on. JSONL is the
// human-readable sibling; see json_codec.h.
//
// The write and read visitors are deliberately adjacent in one file: they are the two
// halves of one format, and keeping them apart is how such pairs drift.
#ifndef PERCEPTION_CORE_DIAGNOSTICS_BINARY_CODEC_H_
#define PERCEPTION_CORE_DIAGNOSTICS_BINARY_CODEC_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "perception/core/diagnostics/byte_io.h"
#include "perception/core/diagnostics/contract_fields.h"

namespace perception::core::diagnostics {

// ---------------------------------------------------------------------------------------
// Write
// ---------------------------------------------------------------------------------------
class BinaryWriteVisitor {
 public:
  explicit BinaryWriteVisitor(ByteWriter& writer) : writer_(writer) {}

  // Field names are structural in the JSON codec and pure documentation here: the binary
  // format is positional, which is what makes it compact and what makes the dump schema
  // version load-bearing.
  template <class T>
  void Prim(const char* /*name*/, T& value) {
    if constexpr (std::is_same_v<T, bool>) {
      writer_.Bool(value);
    } else if constexpr (std::is_same_v<T, float>) {
      writer_.F32(value);
    } else if constexpr (std::is_same_v<T, double>) {
      writer_.F64(value);
    } else if constexpr (std::is_same_v<T, std::string>) {
      writer_.Str(value);
    } else if constexpr (std::is_integral_v<T>) {
      if constexpr (sizeof(T) <= 4) {
        writer_.U32(static_cast<uint32_t>(value));
      } else {
        writer_.U64(static_cast<uint64_t>(value));
      }
    } else {
      static_assert(sizeof(T) == 0, "BinaryWriteVisitor::Prim: unsupported field type");
    }
  }

  // Enums travel as uint32 whatever their declared underlying type. A uint8_t enum would
  // otherwise make adding a tenth enumerator a silent format change the moment someone
  // widened the underlying type.
  template <class E>
  void Enm(const char* /*name*/, E& value) {
    static_assert(std::is_enum_v<E>, "Enm requires an enum");
    writer_.U32(static_cast<uint32_t>(value));
  }

  void Eig(const char* /*name*/, Eigen::Vector2d& value) {
    writer_.F64(value.x());
    writer_.F64(value.y());
  }

  void Eig(const char* /*name*/, Eigen::Vector3d& value) {
    writer_.F64(value.x());
    writer_.F64(value.y());
    writer_.F64(value.z());
  }

  void Eig(const char* /*name*/, Eigen::Vector3f& value) {
    writer_.F32(value.x());
    writer_.F32(value.y());
    writer_.F32(value.z());
  }

  void Eig(const char* /*name*/, Eigen::Vector4d& value) {
    for (int index = 0; index < 4; ++index) writer_.F64(value[index]);
  }

  void Eig(const char* /*name*/, Eigen::Matrix3d& value) {
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) writer_.F64(value(row, column));
    }
  }

  // 3x4 affine, not the padded 4x4: the bottom row of an Isometry3d is [0 0 0 1] by the
  // type's own invariant, so writing it would store four bytes-worth of tautology per pose
  // and invite a reader to trust a value it should be asserting instead.
  void Eig(const char* /*name*/, Eigen::Isometry3d& value) {
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) writer_.F64(value.linear()(row, column));
    }
    for (int row = 0; row < 3; ++row) writer_.F64(value.translation()[row]);
  }

  template <class S>
  void Sub(const char* /*name*/, S& value) {
    VisitFields(*this, value);
  }

  template <class T>
  void Arr(const char* name, std::vector<T>& value) {
    writer_.U64(static_cast<uint64_t>(value.size()));
    for (T& element : value) VisitArrayElement(*this, name, element);
  }

  template <std::size_t N>
  void FixedArr(const char* /*name*/, std::array<double, N>& value) {
    // Length is written even though it is fixed, so a schema change that resizes the array
    // is caught by the reader's length check instead of silently shifting every later field.
    writer_.U64(static_cast<uint64_t>(N));
    for (double element : value) writer_.F64(element);
  }

 private:
  ByteWriter& writer_;
};

// ---------------------------------------------------------------------------------------
// Read
// ---------------------------------------------------------------------------------------
class BinaryReadVisitor {
 public:
  explicit BinaryReadVisitor(ByteReader& reader) : reader_(reader) {}

  template <class T>
  void Prim(const char* /*name*/, T& value) {
    if constexpr (std::is_same_v<T, bool>) {
      value = reader_.Bool();
    } else if constexpr (std::is_same_v<T, float>) {
      value = reader_.F32();
    } else if constexpr (std::is_same_v<T, double>) {
      value = reader_.F64();
    } else if constexpr (std::is_same_v<T, std::string>) {
      value = reader_.Str();
    } else if constexpr (std::is_integral_v<T>) {
      if constexpr (sizeof(T) <= 4) {
        value = static_cast<T>(reader_.U32());
      } else {
        value = static_cast<T>(reader_.U64());
      }
    } else {
      static_assert(sizeof(T) == 0, "BinaryReadVisitor::Prim: unsupported field type");
    }
  }

  template <class E>
  void Enm(const char* name, E& value) {
    static_assert(std::is_enum_v<E>, "Enm requires an enum");
    const uint32_t raw = reader_.U32();
    if (!reader_.ok()) return;
    // Enum values from a file are untrusted. Range-checking against the declared
    // enumerator count keeps a corrupted byte from becoming an out-of-range enum that
    // every later switch handles as "default" - the archetypal silent misparse.
    if (raw > EnumUpperBound<E>()) {
      reader_.Fail(std::string("enum field '") + name + "' is out of range");
      return;
    }
    value = static_cast<E>(raw);
  }

  void Eig(const char* /*name*/, Eigen::Vector2d& value) {
    value.x() = reader_.F64();
    value.y() = reader_.F64();
  }

  void Eig(const char* /*name*/, Eigen::Vector3d& value) {
    value.x() = reader_.F64();
    value.y() = reader_.F64();
    value.z() = reader_.F64();
  }

  void Eig(const char* /*name*/, Eigen::Vector3f& value) {
    value.x() = reader_.F32();
    value.y() = reader_.F32();
    value.z() = reader_.F32();
  }

  void Eig(const char* /*name*/, Eigen::Vector4d& value) {
    for (int index = 0; index < 4; ++index) value[index] = reader_.F64();
  }

  void Eig(const char* /*name*/, Eigen::Matrix3d& value) {
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) value(row, column) = reader_.F64();
    }
  }

  void Eig(const char* /*name*/, Eigen::Isometry3d& value) {
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) rotation(row, column) = reader_.F64();
    }
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    for (int row = 0; row < 3; ++row) translation[row] = reader_.F64();
    // setIdentity first so the implicit bottom row is [0 0 0 1] and not whatever this
    // object held before being reused.
    value.setIdentity();
    value.linear() = rotation;
    value.translation() = translation;
  }

  template <class S>
  void Sub(const char* /*name*/, S& value) {
    VisitFields(*this, value);
  }

  template <class T>
  void Arr(const char* name, std::vector<T>& value) {
    const uint64_t count = reader_.Count();
    if (!reader_.ok()) return;
    value.clear();
    value.resize(static_cast<std::size_t>(count));
    for (T& element : value) {
      VisitArrayElement(*this, name, element);
      if (!reader_.ok()) return;
    }
  }

  template <std::size_t N>
  void FixedArr(const char* name, std::array<double, N>& value) {
    const uint64_t count = reader_.Count();
    if (!reader_.ok()) return;
    if (count != N) {
      reader_.Fail(std::string("fixed array '") + name + "' has the wrong length");
      return;
    }
    for (double& element : value) element = reader_.F64();
  }

 private:
  // The largest valid value of each enum used in a contract. Kept here rather than in the
  // contracts so that adding an enumerator does not require touching a header every
  // consumer includes; the static_asserts below make forgetting to update it a build break
  // in the codec, which is where the format lives.
  template <class E>
  static uint32_t EnumUpperBound() {
    if constexpr (std::is_same_v<E, FrameId>) {
      return static_cast<uint32_t>(FrameId::kWorld);
    } else if constexpr (std::is_same_v<E, MotionCompensation>) {
      return static_cast<uint32_t>(MotionCompensation::kDeskewedToStamp);
    } else if constexpr (std::is_same_v<E, PointLabel>) {
      return static_cast<uint32_t>(PointLabel::kSelf);
    } else if constexpr (std::is_same_v<E, PrimitiveKind>) {
      return static_cast<uint32_t>(PrimitiveKind::kCircle);
    } else if constexpr (std::is_same_v<E, TrackStatus>) {
      return static_cast<uint32_t>(TrackStatus::kCoasting);
    } else if constexpr (std::is_same_v<E, ObstacleSource>) {
      return static_cast<uint32_t>(ObstacleSource::kOracle);
    } else if constexpr (std::is_same_v<E, FrameInvalidReason>) {
      return static_cast<uint32_t>(FrameInvalidReason::kStale);
    } else {
      static_assert(sizeof(E) == 0,
                    "BinaryReadVisitor: this enum has no declared upper bound - add it");
      return 0;
    }
  }

  ByteReader& reader_;
};

// ---------------------------------------------------------------------------------------
// Record-level entry points.
// ---------------------------------------------------------------------------------------

// The const_cast is the one place the shared-field-list design costs anything: VisitFields
// takes a mutable reference so that one list can serve both directions, and the write
// visitor provably never assigns through it (every Prim/Eig/Arr above only reads).
template <class T>
void WriteBinaryRecord(ByteWriter& writer, const T& record) {
  BinaryWriteVisitor visitor(writer);
  VisitFields(visitor, const_cast<T&>(record));
}

template <class T>
bool ReadBinaryRecord(ByteReader& reader, T& record) {
  BinaryReadVisitor visitor(reader);
  VisitFields(visitor, record);
  return reader.ok();
}

}  // namespace perception::core::diagnostics

#endif  // PERCEPTION_CORE_DIAGNOSTICS_BINARY_CODEC_H_
