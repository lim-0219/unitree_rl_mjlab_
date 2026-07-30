// A minimal JSON reader/writer, sized exactly to the dump format's needs.
//
// WHY NOT A LIBRARY. core/diagnostics must depend on nothing but core/contracts and the
// standard library (architecture doc section 8), and this repository has no JSON dependency
// to borrow. The grammar we have to parse is also one we emit ourselves, so the parser can
// be strict rather than permissive - which is what we want, because a permissive parser is
// exactly how a corrupted fixture becomes a plausible one.
//
// EXACT NUMBERS. Doubles are emitted with 17 significant digits and floats with 9, which
// are the shortest widths that round-trip every IEEE-754 value of those types exactly
// (DBL_DECIMAL_DIG / FLT_DECIMAL_DIG). Parsed numbers keep their original LEXEME and are
// converted with strtod/strtof on demand, so the round-trip never passes through a lossy
// intermediate. That is what lets the JSONL format meet the same bit-exact round-trip bar
// as the binary one (architecture doc section 11, "exact equality" tier).
//
// NON-STANDARD LITERALS, deliberately. NaN and infinities have no JSON spelling, and this
// format has to carry them: an empty ProjectedScan bin IS a NaN, and dropping that
// distinction would make a never-measured bin indistinguishable from a zero-range one. They
// are written as the bare tokens `NaN`, `Infinity` and `-Infinity`, the same spelling
// Python's json module produces and accepts. Any strict consumer must be told; that is
// documented in the run manifest's `jsonl_number_note` field.
#ifndef PERCEPTION_CORE_DIAGNOSTICS_JSON_VALUE_H_
#define PERCEPTION_CORE_DIAGNOSTICS_JSON_VALUE_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace perception::core::diagnostics {

// ---------------------------------------------------------------------------------------
// Number formatting. Free functions so both the DOM and the streaming writer share them.
// ---------------------------------------------------------------------------------------
std::string FormatJsonDouble(double value);
std::string FormatJsonFloat(float value);
std::string EscapeJsonString(const std::string& value);

// ---------------------------------------------------------------------------------------
// Streaming writer. Used for records (compact, one line each) and for the run manifest
// (pretty, two-space indent, matching dpcbf_rollout_evaluator's hand-written JSON style).
// ---------------------------------------------------------------------------------------
class JsonTextWriter {
 public:
  explicit JsonTextWriter(bool pretty) : pretty_(pretty) {}

  void BeginObject();
  void EndObject();
  void BeginArray();
  void EndArray();

  // Names a member of the current object. Must be followed by exactly one value.
  void Key(const char* name);

  void Bool(bool value);
  void Int64(int64_t value);
  void Uint64(uint64_t value);
  void Double(double value);
  void Float(float value);
  void String(const std::string& value);
  void Null();

  const std::string& text() const { return text_; }
  void Clear();

 private:
  void Separator();
  void NewlineIndent();

  std::string text_;
  bool pretty_ = false;
  int depth_ = 0;
  // Per-depth "has this container emitted an element yet" flags, for comma placement.
  std::vector<bool> populated_;
  bool expect_value_ = false;  // A Key() was just emitted.
};

// ---------------------------------------------------------------------------------------
// DOM, for reading.
// ---------------------------------------------------------------------------------------
class JsonValue {
 public:
  enum class Kind { kNull, kBool, kNumber, kString, kArray, kObject };

  Kind kind() const { return kind_; }
  bool is_null() const { return kind_ == Kind::kNull; }
  bool is_bool() const { return kind_ == Kind::kBool; }
  bool is_number() const { return kind_ == Kind::kNumber; }
  bool is_string() const { return kind_ == Kind::kString; }
  bool is_array() const { return kind_ == Kind::kArray; }
  bool is_object() const { return kind_ == Kind::kObject; }

  bool boolean() const { return boolean_; }
  const std::string& text() const { return text_; }

  // Converted from the stored lexeme, so no precision is lost on the way in.
  double AsDouble() const;
  float AsFloat() const;
  int64_t AsInt64() const;
  uint64_t AsUint64() const;

  std::size_t size() const { return items_.size(); }
  const JsonValue& at(std::size_t index) const { return items_[index]; }

  // nullptr when absent. Linear scan: records have tens of fields, not thousands, and
  // preserving declaration order matters more than lookup speed.
  const JsonValue* Find(const char* name) const;

  // Parses one complete JSON value. `error` is set and false returned on any malformed
  // input, including trailing content after the value.
  static bool Parse(const std::string& input, JsonValue& out, std::string& error);

 private:
  friend class JsonParser;

  Kind kind_ = Kind::kNull;
  bool boolean_ = false;
  std::string text_;                  // Number lexeme, or string contents.
  std::vector<JsonValue> items_;      // Array elements, or object values.
  std::vector<std::string> keys_;     // Object keys, parallel to items_.
};

}  // namespace perception::core::diagnostics

#endif  // PERCEPTION_CORE_DIAGNOSTICS_JSON_VALUE_H_
