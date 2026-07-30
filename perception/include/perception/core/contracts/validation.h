// Shared validation conventions for the perception contracts.
//
// CONVENTION, applied by every contract in this directory:
//
//   const char* Validate() const;   // nullptr when the value is well-formed,
//                                   // otherwise a STATIC reason string.
//   bool IsValid() const;           // Validate() == nullptr.
//
// The reason is a static string rather than a std::string so that validating a contract
// never allocates - these run inside the perception thread, which the architecture
// requires to be allocation-free in steady state
// (perception/docs/architecture.md, "Internal data contracts").
//
// The three contracts that shipped with the Mid-360 bring-up slice predate this
// convention and keep their own named predicates (`RayPattern::IsConsistent`,
// `ScanStats::IsBalanced`); they are deliberately NOT renamed, and the contract tests
// exercise them under their existing names.
//
// IMMUTABILITY. Every contract here is a plain aggregate, and immutability is a
// discipline rather than a language guarantee: a stage fills a buffer it owns and then
// publishes it as `const&`. Aggregates were chosen over const members because the
// buffers must be reusable across frames - a contract with const members could not be
// refilled in place, which would force exactly the per-frame heap traffic the
// architecture forbids. `Reserve()` sizes the buffers once, `Clear()` empties them
// without releasing capacity.
#ifndef PERCEPTION_CORE_CONTRACTS_VALIDATION_H_
#define PERCEPTION_CORE_CONTRACTS_VALIDATION_H_

#include <cmath>
#include <cstddef>

#include <Eigen/Core>

namespace perception::core {

// Pi as a constant rather than M_PI, which is POSIX rather than standard C++.
inline constexpr double kPi = 3.14159265358979323846;

// A range whose upper bound is exclusive of NaN: any NaN comparison is false, so an
// explicit finiteness test has to come first everywhere.
inline bool IsFinite(double value) { return std::isfinite(value); }

inline bool IsFinite(const Eigen::Vector2d& v) {
  return std::isfinite(v.x()) && std::isfinite(v.y());
}

inline bool IsFinite(const Eigen::Vector3d& v) {
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

inline bool IsFinite(const Eigen::Vector3f& v) {
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

// Non-negative AND finite. `value >= 0.0` alone admits +inf, which every buffer-sizing
// and inflation term in this subsystem must reject.
inline bool IsFiniteNonNegative(double value) { return std::isfinite(value) && value >= 0.0; }

inline bool IsFinitePositive(double value) { return std::isfinite(value) && value > 0.0; }

}  // namespace perception::core

#endif  // PERCEPTION_CORE_CONTRACTS_VALIDATION_H_
