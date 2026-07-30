// FNV-1a/64 over text, rendered as 16 lowercase hex digits.
//
// WHAT IT IS FOR. Identifying a run's resolved configuration in a manifest, so that two
// dumps can be compared and a fixture can be traced to the settings that produced it. It is
// a fingerprint, NOT a security primitive: FNV-1a is trivially collidable on purpose-built
// input. Nothing in this subsystem authenticates anything with it, and nothing should.
//
// Chosen over a cryptographic hash because core/diagnostics may not take dependencies and
// hand-rolling SHA-256 for a provenance label would be a much larger surface for a much
// weaker reason. If this ever needs to resist an adversary rather than a typo, replace it
// deliberately and bump kManifestVersion.
#ifndef PERCEPTION_CORE_DIAGNOSTICS_CONFIG_HASH_H_
#define PERCEPTION_CORE_DIAGNOSTICS_CONFIG_HASH_H_

#include <cstdint>
#include <string>

namespace perception::core::diagnostics {

inline constexpr uint64_t kFnvOffsetBasis = 1469598103934665603ull;
inline constexpr uint64_t kFnvPrime = 1099511628211ull;

inline uint64_t Fnv1a64(const std::string& text) {
  uint64_t hash = kFnvOffsetBasis;
  for (const char raw : text) {
    hash ^= static_cast<uint64_t>(static_cast<unsigned char>(raw));
    hash *= kFnvPrime;
  }
  return hash;
}

// Zero-padded to a fixed 16 characters, so hashes line up in a report and a truncated one
// is visibly wrong rather than plausibly short.
inline std::string ToHex64(uint64_t value) {
  static constexpr char kDigits[] = "0123456789abcdef";
  std::string out(16, '0');
  for (int index = 15; index >= 0; --index) {
    out[static_cast<std::size_t>(index)] = kDigits[value & 0xFull];
    value >>= 4;
  }
  return out;
}

inline std::string HashConfigText(const std::string& text) { return ToHex64(Fnv1a64(text)); }

}  // namespace perception::core::diagnostics

#endif  // PERCEPTION_CORE_DIAGNOSTICS_CONFIG_HASH_H_
