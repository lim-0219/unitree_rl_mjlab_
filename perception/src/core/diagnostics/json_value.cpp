#include "perception/core/diagnostics/json_value.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace perception::core::diagnostics {
namespace {

// The three non-JSON tokens this format needs. See json_value.h for why.
constexpr const char* kNanToken = "NaN";
constexpr const char* kInfToken = "Infinity";
constexpr const char* kNegInfToken = "-Infinity";

}  // namespace

std::string FormatJsonDouble(double value) {
  if (std::isnan(value)) return kNanToken;
  if (std::isinf(value)) return value > 0.0 ? kInfToken : kNegInfToken;
  char buffer[64];
  // 17 significant digits: the shortest width that round-trips every double exactly.
  std::snprintf(buffer, sizeof(buffer), "%.17g", value);
  return buffer;
}

std::string FormatJsonFloat(float value) {
  if (std::isnan(value)) return kNanToken;
  if (std::isinf(value)) return value > 0.0f ? kInfToken : kNegInfToken;
  char buffer[64];
  // 9 significant digits does the same job for float.
  std::snprintf(buffer, sizeof(buffer), "%.9g", static_cast<double>(value));
  return buffer;
}

std::string EscapeJsonString(const std::string& value) {
  std::string out;
  out.reserve(value.size() + 2);
  for (const char raw : value) {
    const unsigned char character = static_cast<unsigned char>(raw);
    switch (character) {
      case '"':  out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n";  break;
      case '\r': out += "\\r";  break;
      case '\t': out += "\\t";  break;
      case '\b': out += "\\b";  break;
      case '\f': out += "\\f";  break;
      default:
        if (character < 0x20) {
          char buffer[8];
          std::snprintf(buffer, sizeof(buffer), "\\u%04x", character);
          out += buffer;
        } else {
          out += raw;
        }
    }
  }
  return out;
}

// ---------------------------------------------------------------------------------------
// JsonTextWriter
// ---------------------------------------------------------------------------------------
void JsonTextWriter::Clear() {
  text_.clear();
  depth_ = 0;
  populated_.clear();
  expect_value_ = false;
}

void JsonTextWriter::Separator() {
  if (expect_value_) {
    // The key already emitted its colon; a value never needs a comma of its own.
    expect_value_ = false;
    return;
  }
  if (depth_ > 0) {
    if (populated_[static_cast<std::size_t>(depth_) - 1]) text_ += ',';
    populated_[static_cast<std::size_t>(depth_) - 1] = true;
    NewlineIndent();
  }
}

void JsonTextWriter::NewlineIndent() {
  if (!pretty_) return;
  text_ += '\n';
  text_.append(static_cast<std::size_t>(depth_) * 2, ' ');
}

void JsonTextWriter::BeginObject() {
  Separator();
  text_ += '{';
  ++depth_;
  populated_.push_back(false);
}

void JsonTextWriter::EndObject() {
  const bool had_members = populated_.back();
  populated_.pop_back();
  --depth_;
  if (had_members) NewlineIndent();
  text_ += '}';
}

void JsonTextWriter::BeginArray() {
  Separator();
  text_ += '[';
  ++depth_;
  populated_.push_back(false);
}

void JsonTextWriter::EndArray() {
  const bool had_members = populated_.back();
  populated_.pop_back();
  --depth_;
  if (had_members) NewlineIndent();
  text_ += ']';
}

void JsonTextWriter::Key(const char* name) {
  Separator();
  text_ += '"';
  text_ += EscapeJsonString(name);
  text_ += pretty_ ? "\": " : "\":";
  expect_value_ = true;
}

void JsonTextWriter::Bool(bool value) {
  Separator();
  text_ += value ? "true" : "false";
}

void JsonTextWriter::Int64(int64_t value) {
  Separator();
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%lld", static_cast<long long>(value));
  text_ += buffer;
}

void JsonTextWriter::Uint64(uint64_t value) {
  Separator();
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%llu", static_cast<unsigned long long>(value));
  text_ += buffer;
}

void JsonTextWriter::Double(double value) {
  Separator();
  text_ += FormatJsonDouble(value);
}

void JsonTextWriter::Float(float value) {
  Separator();
  text_ += FormatJsonFloat(value);
}

void JsonTextWriter::String(const std::string& value) {
  Separator();
  text_ += '"';
  text_ += EscapeJsonString(value);
  text_ += '"';
}

void JsonTextWriter::Null() {
  Separator();
  text_ += "null";
}

// ---------------------------------------------------------------------------------------
// JsonValue accessors
// ---------------------------------------------------------------------------------------
double JsonValue::AsDouble() const {
  if (kind_ != Kind::kNumber) return 0.0;
  if (text_ == kNanToken) return std::nan("");
  if (text_ == kInfToken) return HUGE_VAL;
  if (text_ == kNegInfToken) return -HUGE_VAL;
  return std::strtod(text_.c_str(), nullptr);
}

float JsonValue::AsFloat() const {
  if (kind_ != Kind::kNumber) return 0.0f;
  if (text_ == kNanToken) return std::nanf("");
  if (text_ == kInfToken) return HUGE_VALF;
  if (text_ == kNegInfToken) return -HUGE_VALF;
  return std::strtof(text_.c_str(), nullptr);
}

int64_t JsonValue::AsInt64() const {
  if (kind_ != Kind::kNumber) return 0;
  return static_cast<int64_t>(std::strtoll(text_.c_str(), nullptr, 10));
}

uint64_t JsonValue::AsUint64() const {
  if (kind_ != Kind::kNumber) return 0;
  // Negative lexemes must not wrap silently into a huge unsigned value.
  if (!text_.empty() && text_[0] == '-') return 0;
  return static_cast<uint64_t>(std::strtoull(text_.c_str(), nullptr, 10));
}

const JsonValue* JsonValue::Find(const char* name) const {
  if (kind_ != Kind::kObject) return nullptr;
  for (std::size_t index = 0; index < keys_.size(); ++index) {
    if (keys_[index] == name) return &items_[index];
  }
  return nullptr;
}

// ---------------------------------------------------------------------------------------
// Parser
// ---------------------------------------------------------------------------------------
class JsonParser {
 public:
  JsonParser(const std::string& input, std::string& error) : input_(input), error_(error) {}

  bool ParseDocument(JsonValue& out) {
    SkipWhitespace();
    if (!ParseValue(out, 0)) return false;
    SkipWhitespace();
    if (position_ != input_.size()) {
      return Fail("trailing content after the JSON value");
    }
    return true;
  }

 private:
  // Bounds recursion so a pathological or hostile file cannot blow the stack. Dump records
  // nest at most about four levels (PerceptionFrame -> diagnostics -> segmentation -> plane).
  static constexpr int kMaxDepth = 32;

  bool Fail(const std::string& reason) {
    if (error_.empty()) {
      error_ = reason + " at offset " + std::to_string(position_);
    }
    return false;
  }

  void SkipWhitespace() {
    while (position_ < input_.size()) {
      const char character = input_[position_];
      if (character == ' ' || character == '\t' || character == '\n' || character == '\r') {
        ++position_;
      } else {
        break;
      }
    }
  }

  bool Literal(const char* token) {
    const std::size_t length = std::strlen(token);
    if (input_.compare(position_, length, token) != 0) return false;
    position_ += length;
    return true;
  }

  bool ParseValue(JsonValue& out, int depth) {
    if (depth > kMaxDepth) return Fail("JSON nesting is too deep");
    if (position_ >= input_.size()) return Fail("unexpected end of JSON input");
    const char character = input_[position_];
    switch (character) {
      case '{': return ParseObject(out, depth);
      case '[': return ParseArray(out, depth);
      case '"': {
        out.kind_ = JsonValue::Kind::kString;
        return ParseString(out.text_);
      }
      case 't':
        if (!Literal("true")) return Fail("malformed literal");
        out.kind_ = JsonValue::Kind::kBool;
        out.boolean_ = true;
        return true;
      case 'f':
        if (!Literal("false")) return Fail("malformed literal");
        out.kind_ = JsonValue::Kind::kBool;
        out.boolean_ = false;
        return true;
      case 'n':
        if (Literal("null")) {
          out.kind_ = JsonValue::Kind::kNull;
          return true;
        }
        return Fail("malformed literal");
      case 'N':
        if (Literal(kNanToken)) {
          out.kind_ = JsonValue::Kind::kNumber;
          out.text_ = kNanToken;
          return true;
        }
        return Fail("malformed literal");
      case 'I':
        if (Literal(kInfToken)) {
          out.kind_ = JsonValue::Kind::kNumber;
          out.text_ = kInfToken;
          return true;
        }
        return Fail("malformed literal");
      default:
        break;
    }
    if (character == '-' && Literal(kNegInfToken)) {
      out.kind_ = JsonValue::Kind::kNumber;
      out.text_ = kNegInfToken;
      return true;
    }
    return ParseNumber(out);
  }

  bool ParseString(std::string& out) {
    if (input_[position_] != '"') return Fail("expected a string");
    ++position_;
    out.clear();
    while (true) {
      if (position_ >= input_.size()) return Fail("unterminated string");
      const char character = input_[position_++];
      if (character == '"') return true;
      if (character != '\\') {
        out += character;
        continue;
      }
      if (position_ >= input_.size()) return Fail("unterminated escape");
      const char escape = input_[position_++];
      switch (escape) {
        case '"':  out += '"';  break;
        case '\\': out += '\\'; break;
        case '/':  out += '/';  break;
        case 'n':  out += '\n'; break;
        case 'r':  out += '\r'; break;
        case 't':  out += '\t'; break;
        case 'b':  out += '\b'; break;
        case 'f':  out += '\f'; break;
        case 'u': {
          if (position_ + 4 > input_.size()) return Fail("truncated \\u escape");
          unsigned code = 0;
          for (int digit = 0; digit < 4; ++digit) {
            const char hex = input_[position_ + static_cast<std::size_t>(digit)];
            code <<= 4;
            if (hex >= '0' && hex <= '9') {
              code |= static_cast<unsigned>(hex - '0');
            } else if (hex >= 'a' && hex <= 'f') {
              code |= static_cast<unsigned>(hex - 'a' + 10);
            } else if (hex >= 'A' && hex <= 'F') {
              code |= static_cast<unsigned>(hex - 'A' + 10);
            } else {
              return Fail("bad hex digit in \\u escape");
            }
          }
          position_ += 4;
          // This format only ever emits \u for control characters, so a one-byte
          // reconstruction is exact for everything we write. Anything above 0x7F in an
          // escape is rejected rather than mangled.
          if (code > 0x7F) return Fail("\\u escapes above U+007F are not supported");
          out += static_cast<char>(code);
          break;
        }
        default:
          return Fail("unknown escape sequence");
      }
    }
  }

  bool ParseNumber(JsonValue& out) {
    const std::size_t start = position_;
    if (position_ < input_.size() && (input_[position_] == '-' || input_[position_] == '+')) {
      ++position_;
    }
    bool any_digit = false;
    while (position_ < input_.size() && input_[position_] >= '0' && input_[position_] <= '9') {
      ++position_;
      any_digit = true;
    }
    if (position_ < input_.size() && input_[position_] == '.') {
      ++position_;
      while (position_ < input_.size() && input_[position_] >= '0' && input_[position_] <= '9') {
        ++position_;
        any_digit = true;
      }
    }
    if (!any_digit) return Fail("expected a number");
    if (position_ < input_.size() && (input_[position_] == 'e' || input_[position_] == 'E')) {
      ++position_;
      if (position_ < input_.size() && (input_[position_] == '-' || input_[position_] == '+')) {
        ++position_;
      }
      bool exponent_digit = false;
      while (position_ < input_.size() && input_[position_] >= '0' && input_[position_] <= '9') {
        ++position_;
        exponent_digit = true;
      }
      if (!exponent_digit) return Fail("malformed exponent");
    }
    out.kind_ = JsonValue::Kind::kNumber;
    out.text_ = input_.substr(start, position_ - start);
    return true;
  }

  bool ParseObject(JsonValue& out, int depth) {
    ++position_;  // '{'
    out.kind_ = JsonValue::Kind::kObject;
    SkipWhitespace();
    if (position_ < input_.size() && input_[position_] == '}') {
      ++position_;
      return true;
    }
    while (true) {
      SkipWhitespace();
      std::string key;
      if (!ParseString(key)) return false;
      SkipWhitespace();
      if (position_ >= input_.size() || input_[position_] != ':') {
        return Fail("expected ':' after an object key");
      }
      ++position_;
      SkipWhitespace();
      out.keys_.push_back(key);
      out.items_.emplace_back();
      if (!ParseValue(out.items_.back(), depth + 1)) return false;
      SkipWhitespace();
      if (position_ >= input_.size()) return Fail("unterminated object");
      if (input_[position_] == ',') {
        ++position_;
        continue;
      }
      if (input_[position_] == '}') {
        ++position_;
        return true;
      }
      return Fail("expected ',' or '}' in an object");
    }
  }

  bool ParseArray(JsonValue& out, int depth) {
    ++position_;  // '['
    out.kind_ = JsonValue::Kind::kArray;
    SkipWhitespace();
    if (position_ < input_.size() && input_[position_] == ']') {
      ++position_;
      return true;
    }
    while (true) {
      SkipWhitespace();
      out.items_.emplace_back();
      if (!ParseValue(out.items_.back(), depth + 1)) return false;
      SkipWhitespace();
      if (position_ >= input_.size()) return Fail("unterminated array");
      if (input_[position_] == ',') {
        ++position_;
        continue;
      }
      if (input_[position_] == ']') {
        ++position_;
        return true;
      }
      return Fail("expected ',' or ']' in an array");
    }
  }

  const std::string& input_;
  std::string& error_;
  std::size_t position_ = 0;
};

bool JsonValue::Parse(const std::string& input, JsonValue& out, std::string& error) {
  out = JsonValue{};
  error.clear();
  JsonParser parser(input, error);
  if (!parser.ParseDocument(out)) {
    if (error.empty()) error = "malformed JSON";
    return false;
  }
  return true;
}

}  // namespace perception::core::diagnostics
