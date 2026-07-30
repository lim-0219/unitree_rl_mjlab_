// Fixed-width little-endian primitive I/O - the bottom layer of the dump format.
//
// EXPLICIT ENCODING, not memcpy of structs. Writing a contract by memcpy would bake this
// compiler's padding, enum width and Eigen alignment into the file, so a dump taken by one
// build could not be read by another - and the dumps are supposed to be the regression
// fixtures that OUTLIVE builds. Every value therefore goes out as a declared width in a
// declared byte order.
//
// BIT-EXACTNESS. Floats and doubles are punned to uint32/uint64 through memcpy (not a
// reinterpret_cast, which would be strict-aliasing UB) and emitted as integers. That makes
// the round-trip exact for every value including NaN, subnormals and signed zero, which is
// what the section-11 "exact equality" tier for dump round-trips actually requires - an
// approximate decimal round-trip would quietly turn -0.0 into 0.0 and a signalling NaN into
// a quiet one.
#ifndef PERCEPTION_CORE_DIAGNOSTICS_BYTE_IO_H_
#define PERCEPTION_CORE_DIAGNOSTICS_BYTE_IO_H_

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

namespace perception::core::diagnostics {

// Upper bound on any single serialized string or array length. Untrusted input: without a
// cap, a corrupted length field makes the reader try to allocate whatever 64 bits happened
// to be on disk, which is a crash rather than the clear parse error we owe the caller.
inline constexpr uint64_t kMaxSerializedElements = 1ull << 28;  // 268 million.

class ByteWriter {
 public:
  ByteWriter() = default;

  void U8(uint8_t value) { buffer_.push_back(static_cast<char>(value)); }

  void U32(uint32_t value) {
    for (int shift = 0; shift < 32; shift += 8) {
      buffer_.push_back(static_cast<char>((value >> shift) & 0xFFu));
    }
  }

  void U64(uint64_t value) {
    for (int shift = 0; shift < 64; shift += 8) {
      buffer_.push_back(static_cast<char>((value >> shift) & 0xFFull));
    }
  }

  void I32(int32_t value) { U32(static_cast<uint32_t>(value)); }
  void I64(int64_t value) { U64(static_cast<uint64_t>(value)); }
  void Bool(bool value) { U8(value ? 1u : 0u); }

  void F32(float value) {
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    U32(bits);
  }

  void F64(double value) {
    uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    U64(bits);
  }

  void Str(const std::string& value) {
    U64(static_cast<uint64_t>(value.size()));
    buffer_.append(value);
  }

  void Raw(const char* data, std::size_t size) { buffer_.append(data, size); }

  const std::string& buffer() const { return buffer_; }
  std::size_t size() const { return buffer_.size(); }
  void Clear() { buffer_.clear(); }

  // Overwrite `size` bytes at `offset`. Used only to back-patch the file header's closing
  // facts (record count, sequence/stamp range); it is never a way to revise a record.
  bool Patch(std::size_t offset, const char* data, std::size_t size) {
    if (offset + size > buffer_.size()) return false;
    std::memcpy(&buffer_[offset], data, size);
    return true;
  }

 private:
  std::string buffer_;
};

// Bounds-checked, fail-once reader. After the first failure `ok()` stays false and every
// subsequent read returns a zero value, so a caller that forgets one check still cannot
// build a plausible-looking contract out of garbage.
class ByteReader {
 public:
  ByteReader(const char* data, std::size_t size) : data_(data), size_(size) {}

  explicit ByteReader(const std::string& buffer)
      : data_(buffer.data()), size_(buffer.size()) {}

  uint8_t U8() {
    if (!Require(1)) return 0;
    return static_cast<uint8_t>(static_cast<unsigned char>(data_[position_++]));
  }

  uint32_t U32() {
    if (!Require(4)) return 0;
    uint32_t value = 0;
    for (int index = 0; index < 4; ++index) {
      value |= static_cast<uint32_t>(static_cast<unsigned char>(data_[position_ + index]))
               << (8 * index);
    }
    position_ += 4;
    return value;
  }

  uint64_t U64() {
    if (!Require(8)) return 0;
    uint64_t value = 0;
    for (int index = 0; index < 8; ++index) {
      value |= static_cast<uint64_t>(static_cast<unsigned char>(data_[position_ + index]))
               << (8 * index);
    }
    position_ += 8;
    return value;
  }

  int32_t I32() { return static_cast<int32_t>(U32()); }
  int64_t I64() { return static_cast<int64_t>(U64()); }

  // Any byte other than 0 or 1 is a corrupt stream, not a truthy value.
  bool Bool() {
    const uint8_t raw = U8();
    if (raw > 1u) {
      Fail("boolean field is neither 0 nor 1");
      return false;
    }
    return raw == 1u;
  }

  float F32() {
    const uint32_t bits = U32();
    float value = 0.0f;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }

  double F64() {
    const uint64_t bits = U64();
    double value = 0.0;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }

  std::string Str() {
    const uint64_t length = U64();
    if (!ok_) return std::string();
    if (length > kMaxSerializedElements) {
      Fail("string length exceeds the serialization cap");
      return std::string();
    }
    if (!Require(static_cast<std::size_t>(length))) return std::string();
    std::string value(data_ + position_, static_cast<std::size_t>(length));
    position_ += static_cast<std::size_t>(length);
    return value;
  }

  // Reads a container length and rejects anything absurd before the caller resizes.
  uint64_t Count() {
    const uint64_t count = U64();
    if (!ok_) return 0;
    if (count > kMaxSerializedElements) {
      Fail("container length exceeds the serialization cap");
      return 0;
    }
    return count;
  }

  bool Raw(char* out, std::size_t size) {
    if (!Require(size)) return false;
    std::memcpy(out, data_ + position_, size);
    position_ += size;
    return true;
  }

  bool ok() const { return ok_; }
  const std::string& error() const { return error_; }
  std::size_t position() const { return position_; }
  std::size_t remaining() const { return ok_ ? size_ - position_ : 0; }
  bool at_end() const { return position_ >= size_; }

  void Fail(const std::string& reason) {
    if (ok_) {
      ok_ = false;
      error_ = reason;
    }
  }

 private:
  bool Require(std::size_t bytes) {
    if (!ok_) return false;
    if (position_ + bytes > size_) {
      Fail("unexpected end of dump data (truncated file?)");
      return false;
    }
    return true;
  }

  const char* data_ = nullptr;
  std::size_t size_ = 0;
  std::size_t position_ = 0;
  bool ok_ = true;
  std::string error_;
};

}  // namespace perception::core::diagnostics

#endif  // PERCEPTION_CORE_DIAGNOSTICS_BYTE_IO_H_
